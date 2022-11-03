#include "Solver.h"
#include "openMVG/cameras/Camera_Spherical.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/svg_features.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/image_concat.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/multiview/conditioning.hpp"
#include "openMVG/multiview/essential.hpp"
#include "openMVG/multiview/motion_from_essential.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/multiview/solver_essential_eight_point.hpp"
#include "openMVG/multiview/solver_essential_five_point.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"

#include "gurobi_c++.h"
#include <imgui/imgui.h>
#include <regex>  // regex_match, smatch

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;

static std::vector<std::string> split(std::string s) {
    std::vector<std::string> ret;
    size_t pos = 0;
    while (true) {
        size_t newpos = s.find_first_of(" \t\v\f\n\r\\/", pos);
        if (newpos == s.npos) {
            ret.push_back(s.substr(pos, newpos));
            break;
        }
        ret.push_back(s.substr(pos, newpos - pos));
        pos = s.find_first_not_of(" \t\v\f\n\r\\/", newpos + 1);
    }
    return ret;
}

float WeightedRMSE(const std::vector<glm::vec3>& pos_gt, const std::vector<glm::vec3>& pos_gen, const std::vector<uint32_t>& weights)
{
    float sum = 0.0f;
    int weightSum = 0;
    for (int i = 0; i < pos_gt.size(); ++i)
    {
        float d = glm::distance(pos_gt[i], pos_gen[i]);
        float weightedSquareError = weights[i] * d * d;
        sum += weightedSquareError;
        weightSum += weights[i];
    }
    return std::sqrt(sum / weightSum);
}

static Pose3 ParseStrToPose(std::string& str)
{
    // example: pano_R90_T(0,0_5,0) => rotate(90 degree) and Translation (0, 0.5, 0)
    // Step 1: Separate rotation angle and translation
    std::replace(str.begin(), str.end(), '_', '.');
    std::regex pattern{ R"([+-]?[0-9]*[.]?[0-9]+)" };
    std::vector<std::string> matches;
    std::copy(std::sregex_token_iterator(str.begin(), str.end(), pattern),
        std::sregex_token_iterator(), // Default constructor. Constructs the end-of-sequence iterator.
        std::back_inserter(matches));

    // Step 2: Get rotation matrix and translation vector
    double degree = std::stod(matches[0]);
    double x = std::stod(matches[1]);
    double y = std::stod(matches[2]);
    double z = std::stod(matches[3]);

    // Transition to OpenMVG coordinate system
    Eigen::Matrix3d transition;
    transition << 0.0, -1.0,  0.0,
                  0.0,  0.0, -1.0,
                  1.0,  0.0,  0.0;
    Vec3 rotaAxis = transition * Vec3(0.0, 0.0, 1.0);
    Eigen::AngleAxisd aa(glm::radians(degree), rotaAxis);
    Mat3 R = aa.toRotationMatrix();

    Vec3 t{ x, y, z };
    t = transition * t;
    t.normalize();
}

void RelativePoseSolver::Solve(const char* jpg_filenameL, const char* jpg_filenameR, 
    const MatchPoints& match_points, int method)
{
    Image<unsigned char> imageL, imageR;
    ReadImage(jpg_filenameL, &imageL);
    ReadImage(jpg_filenameR, &imageR);


    // Setup 2 camera intrinsics
    cameras::Intrinsic_Spherical
        cameraL(imageL.Width(), imageL.Height()),
        cameraR(imageR.Width(), imageR.Height());

    using namespace openMVG::features;
    PointFeatures featureL, featureR;

    auto& L = match_points.left_pixels;
    auto& R = match_points.right_pixels;

    for (size_t i = 0; i < L.size(); ++i)
    {
        featureL.push_back(PointFeature(L[i].x, L[i].y));
        featureR.push_back(PointFeature(R[i].x, R[i].y));

    }

    std::cout
        << "Left image count: " << featureL.size() << std::endl
        << "Right image count: " << featureR.size() << std::endl;

    //- Draw features on the two image (side by side)
    {
        Features2SVG
        (
            jpg_filenameL,
            { imageL.Width(), imageL.Height() },
            featureL,
            jpg_filenameR,
            { imageR.Width(), imageR.Height() },
            featureR,
            "02_features.svg"
        );
    }
    std::vector<IndMatch> vec_PutativeMatches;
    for (int i = 0; i < L.size(); ++i)
    {
        vec_PutativeMatches.push_back(IndMatch(i, i));
    }
    const bool bVertical = true;
    Matches2SVG
    (
        jpg_filenameL,
        { imageL.Width(), imageL.Height() },
        featureL,
        jpg_filenameR,
        { imageR.Width(), imageR.Height() },
        featureR,
        vec_PutativeMatches,
        "03_Matches.svg",
        bVertical
    );

    {
        // Essential geometry filtering of putative matches
        Mat2X
            xL(2, vec_PutativeMatches.size()),
            xR(2, vec_PutativeMatches.size());

        for (size_t k = 0; k < vec_PutativeMatches.size(); ++k) {
            const PointFeature& imaL = featureL[vec_PutativeMatches[k].i_];
            const PointFeature& imaR = featureR[vec_PutativeMatches[k].j_];
            xL.col(k) = imaL.coords().cast<double>();
            xR.col(k) = imaR.coords().cast<double>();
        }

        const Mat xL_spherical = cameraL(xL),
            xR_spherical = cameraR(xR);
        
        std::vector<uint32_t> vec_inliers(L.size());
        std::iota(vec_inliers.begin(), vec_inliers.end(), 0);

        //solve essential matrix E:
        std::vector<Mat3> Es;
        if (method == 0)  //Pure Eigth Point Algorithm            
        {
            openMVG::EightPointRelativePoseSolver::Solve(xL_spherical, xR_spherical, &Es);
        }
        else if (method == 1)  //solve essential matrix by Gurobi?
        {            
            SolveEssentialMatrixGurobi(xL_spherical, xR_spherical, &Es);
        }

        std::cout << "Solved essential matrix:" << std::endl << Es[0] << std::endl;
        //report essential matrix equation errors
        std::cout << "Essential matrix equation residuals:" << std::endl;
        for (int i = 0; i < xL_spherical.cols(); i++)
        {
            Eigen::Vector3d x1 = xL_spherical.col(i);
            Eigen::Vector3d x2 = xR_spherical.col(i);
            std::cout << x2.transpose() * Es[0] * x1 << std::endl;
        }

        // Decompose the essential matrix and keep the best solution (if any)
        geometry::Pose3 relative_pose;
        std::vector<uint32_t> inliers_indexes;
        std::vector<Vec3> inliers_X;
        RelativePoseFromEssential(xL_spherical,
            xR_spherical,
            Es[0], vec_inliers,
            &relative_pose,
            &inliers_indexes,
            &inliers_X);

        if (true)
        {
            // Lets make a BA on the scene to check if it relative pose and structure can be refined

            // Setup a SfM scene with two view corresponding the pictures
            SfM_Data tiny_scene;
            tiny_scene.views[0].reset(new View("", 0, 0, 0, imageL.Width(), imageL.Height()));
            tiny_scene.views[1].reset(new View("", 1, 0, 1, imageR.Width(), imageR.Height()));
            // Setup shared intrinsics camera data
            tiny_scene.intrinsics[0].reset(new Intrinsic_Spherical(imageR.Width(), imageR.Height()));

            // Setup poses camera data
            const Pose3 pose0 = tiny_scene.poses[tiny_scene.views[0]->id_pose] = Pose3(Mat3::Identity(), Vec3::Zero());
            const Pose3 pose1 = tiny_scene.poses[tiny_scene.views[1]->id_pose] = relative_pose;

            std::cout << "[                R              |     T     ]" << std::endl;
            std::cout << relative_pose.asMatrix() << std::endl << std::endl;

            
            // calculate rotation angle and axis from rotation martix
            Eigen::AngleAxisd angleAxis(relative_pose.rotation());
            Eigen::Vector3d& axis = angleAxis.axis();
            std::cout << "Rotation Axis: [" << axis.x() << ", " << axis.y() << ", " << axis.z() << "]\t" << "Angle: " << angleAxis.angle() * (180.0 / M_PI) << std::endl << std::endl;

            // ==Ground truth Pose==
            // get folder name : pano_Rxx_T(x,x,x)
            std::string s(jpg_filenameR);
            std::vector<std::string> s_split = split(s);
            
            //Eigen::Mat34 relative_pose_gt = 
            Pose3 pose_gt = ParseStrToPose(s_split[s_split.size() - 2]);
            std::cout << "[                R              |     T     ]" << std::endl;
            std::cout << pose_gt.asMatrix() << std::endl << std::endl;
            Eigen::AngleAxisd angleAxis_gt(pose_gt.rotation());
            Eigen::Vector3d& axis_gt = angleAxis_gt.axis();
            std::cout << "Rotation Axis: [" << axis_gt.x() << ", " << axis_gt.y() << ", " << axis_gt.z() << "]\t" << "Angle: " << angleAxis_gt.angle() * (180.0 / M_PI) << std::endl << std::endl;


            // TODO: scale and compare with ground truth.
            float ratio = 0.5f / relative_pose.translation().x();
            int cnt = 0;
            const float camera_height = 1.58f;
            float center_to_floor = 0.0f;

            std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;
            for (auto i : inliers_indexes) std::cout << i << " ";
            std::cout << std::endl;
            auto& pos_gt = match_points.positions;
            std::vector<glm::vec3> pos_gen;
            
            // Add a new landmark (3D point with its image observations)
            for (int i = 0; i < inliers_indexes.size(); ++i)
            {
                Landmark landmark;
                landmark.X = inliers_X[i];
                landmark.obs[tiny_scene.views[0]->id_view] = Observation(xL.col(inliers_indexes[i]), 0);
                landmark.obs[tiny_scene.views[1]->id_view] = Observation(xR.col(inliers_indexes[i]), 0);
                tiny_scene.structure.insert({ tiny_scene.structure.size(), landmark });
                
                glm::vec3 point(inliers_X[i].z(), -inliers_X[i].x(), -inliers_X[i].y());
                point = point * ratio;
                //std::cout << "3D point:     " << inliers_X[i].x() << " " << inliers_X[i].y() << " " << inliers_X[i].z() << std::endl;
                /*std::cout << "Match " << i << std::endl;
                std::cout << "3D point:     " << std::setw(10) << point.x << " " << std::setw(10) << point.y << " " << std::setw(10) << point.z << std::endl;
                std::cout << "3D point(gt): " << std::setw(10) << pos_gt[i].x << " " << std::setw(10) << pos_gt[i].y << " " << std::setw(10) << pos_gt[i].z << std::endl;
                pos_gen.push_back(point);
                std::cout << "Distance: " << glm::distance(pos_gt[i], point) << std::endl;
                std::cout << std::endl;*/
            }

            //// RMSE
            //auto SquareError = [](const glm::vec3& a, const glm::vec3& b) { float d = glm::distance(a, b); return d * d; };
            //auto sum = std::transform_reduce(pos_gt.begin(), pos_gt.end(), pos_gen.begin(), 0.0f, std::plus<>(), SquareError);
            //auto rmse = std::sqrt(sum / pos_gt.size());
            //auto w_rmse = WeightedRMSE(pos_gt, pos_gen, match_points.weights);
            //std::cout << "RMSE : " << rmse << std::endl;
            //std::cout << "Weighted RMSE : " << w_rmse << std::endl;
            

            
            // Perform Bundle Adjustment of the scene
            Bundle_Adjustment_Ceres bundle_adjustment_obj;
            if (bundle_adjustment_obj.Adjust(tiny_scene,
                Optimize_Options(
                    Intrinsic_Parameter_Type::NONE,
                    Extrinsic_Parameter_Type::ADJUST_ALL,
                    Structure_Parameter_Type::ADJUST_ALL)))
            {
                std::vector<double> residuals;
                // Compute reprojection error
                const Pose3 pose0 = tiny_scene.poses[tiny_scene.views[0]->id_pose];
                const Pose3 pose1 = tiny_scene.poses[tiny_scene.views[1]->id_pose];

                for (const auto& landmark_it : tiny_scene.GetLandmarks())
                {
                    const Landmark& landmark = landmark_it.second;
                    const Observations& obs = landmark.obs;
                    Observations::const_iterator iterObs_xI = obs.find(tiny_scene.views[0]->id_view);
                    Observations::const_iterator iterObs_xJ = obs.find(tiny_scene.views[1]->id_view);

                    const Observation& ob_x0 = iterObs_xI->second;
                    const Observation& ob_x1 = iterObs_xJ->second;

                    const Vec2 residual_I = cameraL.residual(pose0(landmark.X), ob_x0.x); // pose0() °µ rotation ©M translation
                    const Vec2 residual_J = cameraR.residual(pose1(landmark.X), ob_x1.x);
                    residuals.emplace_back(residual_I.norm());
                    residuals.emplace_back(residual_J.norm());

                }

                std::cout << "Residual statistics (pixels):" << std::endl;
                minMaxMeanMedian<double>(residuals.cbegin(), residuals.cend(), std::cout);

                //Save(tiny_scene, "EssentialGeometry_refined.ply", ESfM_Data(ALL));
            }

        }
    }
}

bool RelativePoseSolver::SolveEssentialMatrixGurobi(
    const Mat3X& x1,
    const Mat3X& x2,
    std::vector<Mat3>* pvec_E)
{
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    try
    {
        //variables: 9 entries in E, row major
        std::vector<GRBVar> vars;
        for (int i = 0; i < 9; i++)
        {
            vars.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS));
        }

        //boolean flags for relaxing certain ("outlier") matchings?

        model.update();

        //obj: least squares of Ae = 0  (A is rows of size-9 cols, e is vector form of E)
        GRBQuadExpr obj;
        for (int i = 0; i < x1.cols(); i++)
        {
            std::vector<float> row;
            row.push_back(x1(0, i) * x2(0, i));
            row.push_back(x1(1, i) * x2(0, i));
            row.push_back(x1(2, i) * x2(0, i));
            row.push_back(x1(0, i) * x2(1, i));
            row.push_back(x1(1, i) * x2(1, i));
            row.push_back(x1(2, i) * x2(1, i));
            row.push_back(x1(0, i) * x2(2, i));
            row.push_back(x1(1, i) * x2(2, i));
            row.push_back(x1(2, i) * x2(2, i));

            GRBLinExpr term;
            for (int j = 0; j < 9; j++)  //w.r.t. a row of A
            {
                term += row[j] * vars[j];
            }

            obj += term * term;
        }
        model.setObjective(obj);  //to minimize

        //constraint: e vector is unit vector
        {
            GRBQuadExpr term;
            for (int j = 0; j < 9; j++)
            {
                term += vars[j] * vars[j];
            }

            model.addQConstr(term == 1);
        }

        //solve!
        model.set(GRB_IntParam_NonConvex, 2);

        model.optimize();
        int status = model.get(GRB_IntAttr_Status);
        if (status == 3)
        {
            //infeasible
            std::cout << "[SolveEssentialMatrixGurobi] the problem is infeasible." << std::endl;
            return false;
        }
        else if (status != 9/*time-out*/ && status != 2 && status != 11 && status != 13)
        {
            //some other failure
            std::cout << "[SolveEssentialMatrixGurobi] optimize failed! status:" << status << std::endl;
            return false;
        }

        //get results
        Mat3 E;
        for (int i = 0; i < 9; i++)
        {
            E(i / 3, i % 3) = vars[i].get(GRB_DoubleAttr_X);
        }

        pvec_E->emplace_back(E);
    }
    catch (GRBException e)
    {
        std::cout << "Gurobi excception:" << e.getMessage() << std::endl;
    }
}

void SIFTSolver::Solve(const char* jpg_filenameL, const char* jpg_filenameR, const std::vector<glm::vec3>& pos_gt, MatchPoints& match_points)
{
    Image<unsigned char> imageL, imageR;
    ReadImage(jpg_filenameL, &imageL);
    ReadImage(jpg_filenameR, &imageR);

    // Setup 2 camera intrinsics
    cameras::Intrinsic_Spherical
        cameraL(imageL.Width(), imageL.Height()),
        cameraR(imageR.Width(), imageR.Height());

    //--
    // Detect regions thanks to an image_describer
    //--
    using namespace openMVG::features;
    std::unique_ptr<Image_describer> image_describer
    (new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params(-1)));
    std::map<IndexT, std::unique_ptr<features::Regions>> regions_perImage;
    image_describer->Describe(imageL, regions_perImage[0]);
    image_describer->Describe(imageR, regions_perImage[1]);

    const SIFT_Regions
        * regionsL = dynamic_cast<const SIFT_Regions*>(regions_perImage.at(0).get()),
        * regionsR = dynamic_cast<const SIFT_Regions*>(regions_perImage.at(1).get());

    const PointFeatures
        featsL = regions_perImage.at(0)->GetRegionsPositions(),
        featsR = regions_perImage.at(1)->GetRegionsPositions();

    std::cout
        << "Left image SIFT count: " << featsL.size() << std::endl
        << "Right image SIFT count: " << featsR.size() << std::endl;

    // Show both images side by side
    {
        Image<unsigned char> concat;
        ConcatH(imageL, imageR, concat);
        std::string out_filename = "01_concat.jpg";
        WriteImage(out_filename.c_str(), concat);
    }

    //- Draw features on the two image (side by side)
    {
        Features2SVG
        (
            jpg_filenameL,
            { imageL.Width(), imageL.Height() },
            regionsL->GetRegionsPositions(),
            jpg_filenameR,
            { imageR.Width(), imageR.Height() },
            regionsR->GetRegionsPositions(),
            "02_features.svg"
        );
    }

    std::vector<IndMatch> vec_PutativeMatches;
    //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
    {
        // Find corresponding points
        matching::DistanceRatioMatch(
            0.8, matching::BRUTE_FORCE_L2,
            *regions_perImage.at(0).get(),
            *regions_perImage.at(1).get(),
            vec_PutativeMatches);

        IndMatchDecorator<float> matchDeduplicator(vec_PutativeMatches, featsL, featsR);
        matchDeduplicator.getDeduplicated(vec_PutativeMatches);

        // Draw correspondences after Nearest Neighbor ratio filter
        const bool bVertical = true;
        Matches2SVG
        (
            jpg_filenameL,
            { imageL.Width(), imageL.Height() },
            regionsL->GetRegionsPositions(),
            jpg_filenameR,
            { imageR.Width(), imageR.Height() },
            regionsR->GetRegionsPositions(),
            vec_PutativeMatches,
            "03_Matches.svg",
            bVertical
        );
    }

    // Essential geometry filtering of putative matches
    {
        //A. get back interest point and send it to the robust estimation framework
        Mat2X
            xL(2, vec_PutativeMatches.size()),
            xR(2, vec_PutativeMatches.size());

        for (size_t k = 0; k < vec_PutativeMatches.size(); ++k) {
            const PointFeature& imaL = featsL[vec_PutativeMatches[k].i_];
            const PointFeature& imaR = featsR[vec_PutativeMatches[k].j_];
            xL.col(k) = imaL.coords().cast<double>();
            xR.col(k) = imaR.coords().cast<double>();
        }

        //-- Convert planar to spherical coordinates
        const Mat xL_spherical = cameraL(xL),
            xR_spherical = cameraR(xR);

        //-- Essential matrix robust estimation from spherical bearing vectors
        {
            std::vector<uint32_t> vec_inliers;

            // Define the AContrario angular error adaptor
            using KernelType =
                openMVG::robust::ACKernelAdaptor_AngularRadianError<
                openMVG::EightPointRelativePoseSolver, // Use the 8 point solver in order to estimate E
                //openMVG::essential::kernel::FivePointSolver, // Use the 5 point solver in order to estimate E
                openMVG::AngularError,
                Mat3>;

            KernelType kernel(xL_spherical, xR_spherical);

            // Robust estimation of the Essential matrix and its precision
            Mat3 E;
            const double precision = D2R(4.0);  //std::numeric_limits<double>::infinity(); //
            const std::pair<double, double> ACRansacOut =
                ACRANSAC(kernel, vec_inliers, 1024, &E, precision, true);
            const double& threshold = ACRansacOut.first;

            std::cout << "\n Angular threshold found: " << R2D(threshold) << "(Degree)" << std::endl;
            std::cout << "\n #Putatives/#inliers : " << xL_spherical.cols() << "/" << vec_inliers.size() << "\n" << std::endl;

            const bool bVertical = true;
            InlierMatches2SVG
            (
                jpg_filenameL,
                { imageL.Width(), imageL.Height() },
                regionsL->GetRegionsPositions(),
                jpg_filenameR,
                { imageR.Width(), imageR.Height() },
                regionsR->GetRegionsPositions(),
                vec_PutativeMatches,
                vec_inliers,
                "04_inliers.svg",
                bVertical
            );

            if (vec_inliers.size() > 60) // 60 is used to filter solution with few common geometric matches (unstable solution)
            {
                // Decompose the essential matrix and keep the best solution (if any)
                geometry::Pose3 relative_pose;
                std::vector<uint32_t> inliers_indexes;
                std::vector<Vec3> inliers_X;

                if (RelativePoseFromEssential(xL_spherical,
                    xR_spherical,
                    E, vec_inliers,
                    &relative_pose,
                    &inliers_indexes,
                    &inliers_X))
                {
                    // Lets make a BA on the scene to check if it relative pose and structure can be refined

                    // Setup a SfM scene with two view corresponding the pictures
                    SfM_Data tiny_scene;
                    tiny_scene.views[0].reset(new View("", 0, 0, 0, imageL.Width(), imageL.Height()));
                    tiny_scene.views[1].reset(new View("", 1, 0, 1, imageR.Width(), imageR.Height()));
                    // Setup shared intrinsics camera data
                    tiny_scene.intrinsics[0].reset(new Intrinsic_Spherical(imageR.Width(), imageR.Height()));

                    // Setup poses camera data
                    const Pose3 pose0 = tiny_scene.poses[tiny_scene.views[0]->id_pose] = Pose3(Mat3::Identity(), Vec3::Zero());
                    const Pose3 pose1 = tiny_scene.poses[tiny_scene.views[1]->id_pose] = relative_pose;

                    std::cout << "[                R               |     T     ]" << std::endl;
                    std::cout << relative_pose.asMatrix() << std::endl;
                    std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;

                    // TODO: scale and compare with ground truth.
                    float ratio = 0.5f / relative_pose.translation().x();
                    int cnt = 0;
                    const float camera_height = 1.58f;
                    float center_to_floor = 0.0f;

                    std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;
                    for (auto i : inliers_indexes) std::cout << i << " ";
                    std::cout << std::endl;
                    std::vector<glm::vec3> pos_gen;


                    // Add a new landmark (3D point with its image observations)
                    for (int i = 0; i < inliers_indexes.size(); ++i)
                    {
                        Landmark landmark;
                        landmark.X = inliers_X[i];
                        landmark.obs[tiny_scene.views[0]->id_view] = Observation(xL.col(inliers_indexes[i]), 0);
                        landmark.obs[tiny_scene.views[1]->id_view] = Observation(xR.col(inliers_indexes[i]), 0);
                        tiny_scene.structure.insert({ tiny_scene.structure.size(), landmark });

                        glm::vec3 point(inliers_X[i].z(), -inliers_X[i].x(), -inliers_X[i].y());
                        point = point * ratio;

                        int ind = glm::round(xL.col(inliers_indexes[i]).y()) * 1024 + glm::round(xL.col(inliers_indexes[i]).x());
                        //std::cout << "X: " << xL.col(inliers_indexes[i]).x() << " Y: " << xL.col(inliers_indexes[i]).y() << " ind: " << ind << std::endl;
                        //std::cout << "3D point:     " << inliers_X[i].x() << " " << inliers_X[i].y() << " " << inliers_X[i].z() << std::endl;
                        //std::cout << "Match " << i << std::endl;
                        //std::cout << "3D point:     " << std::setw(10) << point.x << " " << std::setw(10) << point.y << " " << std::setw(10) << point.z << std::endl;
                        //std::cout << "3D point(gt): " << std::setw(10) << pos_gt[ind].x << " " << std::setw(10) << pos_gt[ind].y << " " << std::setw(10) << pos_gt[ind].z << std::endl;
                        pos_gen.push_back(point);
                        const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
                        match_points.AddPoint(glm::vec2(xL.col(inliers_indexes[i]).x(), xL.col(inliers_indexes[i]).y()), glm::vec2(xR.col(inliers_indexes[i]).x(), xR.col(inliers_indexes[i]).y()), col, 10, pos_gt[ind]);
                        //std::cout << "Distance: " << glm::distance(pos_gt[ind], point) << std::endl;
                        //std::cout << std::endl;
                    }

                    //Save(tiny_scene, "EssentialGeometry_start.ply", ESfM_Data(ALL));

                    // Perform Bundle Adjustment of the scene
                    Bundle_Adjustment_Ceres bundle_adjustment_obj;
                    if (bundle_adjustment_obj.Adjust(tiny_scene,
                        Optimize_Options(
                            Intrinsic_Parameter_Type::NONE,
                            Extrinsic_Parameter_Type::ADJUST_ALL,
                            Structure_Parameter_Type::ADJUST_ALL)))
                    {
                        std::vector<double> residuals;
                        // Compute reprojection error
                        const Pose3 pose0 = tiny_scene.poses[tiny_scene.views[0]->id_pose];
                        const Pose3 pose1 = tiny_scene.poses[tiny_scene.views[1]->id_pose];

                        for (const auto& landmark_it : tiny_scene.GetLandmarks())
                        {
                            const Landmark& landmark = landmark_it.second;
                            const Observations& obs = landmark.obs;
                            Observations::const_iterator iterObs_xI = obs.find(tiny_scene.views[0]->id_view);
                            Observations::const_iterator iterObs_xJ = obs.find(tiny_scene.views[1]->id_view);

                            const Observation& ob_x0 = iterObs_xI->second;
                            const Observation& ob_x1 = iterObs_xJ->second;

                            const Vec2 residual_I = cameraL.residual(pose0(landmark.X), ob_x0.x);
                            const Vec2 residual_J = cameraR.residual(pose1(landmark.X), ob_x1.x);
                            residuals.emplace_back(residual_I.norm());
                            residuals.emplace_back(residual_J.norm());

                        }

                        std::cout << "Residual statistics (pixels):" << std::endl;
                        minMaxMeanMedian<double>(residuals.cbegin(), residuals.cend(), std::cout);

                        //Save(tiny_scene, "EssentialGeometry_refined.ply", ESfM_Data(ALL));
                    }
                }
            }
        }
    }
}

bool RelativePoseSolver::SolveEssentialMatrixGurobi(
    const Mat3X& x1,
    const Mat3X& x2,
    std::vector<Mat3>* pvec_E)
{
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    try
    {
        //variables: 9 entries in E, row major
        std::vector<GRBVar> vars_E;
        for (int i = 0; i < 9; i++)
        {
            vars_E.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS));
        }

        //slack variables for each (row) LHS term of the essential matrix system Ae = 0
        std::vector<GRBVar> vars_LHS;
        for (int i = 0; i < x1.cols(); i++)
        {
            vars_LHS.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS));
        }

        //boolean flags for relaxing certain ("outlier") matchings?
        /*std::vector<GRBVar> vars_relax;
        for (int i = 0; i < x1.cols(); i++)
        {
            vars_relax.push_back(model.addVar(0, 1, 0, GRB_BINARY));
        }*/

        model.update();

        //obj: minimize sum of squared of LHS terms
        GRBQuadExpr obj;
        for (int i = 0; i < x1.cols(); i++)
        {
            obj += vars_LHS[i] * vars_LHS[i];
        }
        model.setObjective(obj);  //to minimize

        //obj: least squares of Ae = 0  (A is rows of size-9 cols, e is the vector form of E)
        //GRBQuadExpr obj;
        //for (int i = 0; i < x1.cols(); i++)
        //{
        //    std::vector<float> row;
        //    row.push_back(x1(0, i) * x2(0, i));
        //    row.push_back(x1(1, i) * x2(0, i));
        //    row.push_back(x1(2, i) * x2(0, i));
        //    row.push_back(x1(0, i) * x2(1, i));
        //    row.push_back(x1(1, i) * x2(1, i));
        //    row.push_back(x1(2, i) * x2(1, i));
        //    row.push_back(x1(0, i) * x2(2, i));
        //    row.push_back(x1(1, i) * x2(2, i));
        //    row.push_back(x1(2, i) * x2(2, i));

        //    GRBLinExpr term;
        //    for (int j = 0; j < 9; j++)  //w.r.t. a row of A
        //    {
        //        term += row[j] * vars_E[j];
        //    }
        //    
        //    obj += term * term;            
        //}
        //model.setObjective(obj);  //to minimize

        ////constraints: 

        //LHS term slack variables:
        for (int i = 0; i < x1.cols(); i++)
        {
            std::vector<float> row;  //LHS weights
            row.push_back(x1(0, i) * x2(0, i));
            row.push_back(x1(1, i) * x2(0, i));
            row.push_back(x1(2, i) * x2(0, i));
            row.push_back(x1(0, i) * x2(1, i));
            row.push_back(x1(1, i) * x2(1, i));
            row.push_back(x1(2, i) * x2(1, i));
            row.push_back(x1(0, i) * x2(2, i));
            row.push_back(x1(1, i) * x2(2, i));
            row.push_back(x1(2, i) * x2(2, i));

            GRBLinExpr term;
            for (int j = 0; j < 9; j++)  //w.r.t. a row of A
            {
                term += row[j] * vars_E[j];
            }

            model.addConstr(term == vars_LHS[i]);
        }
        
        //e vector is unit vector
        {
            GRBQuadExpr term;
            for (int j = 0; j < 9; j++)
            {
                term += vars_E[j] * vars_E[j];
            }

            model.addQConstr(term == 1);
        }        

        //solve!
        model.set(GRB_IntParam_NonConvex, 2);

        model.optimize();
        int status = model.get(GRB_IntAttr_Status);
        if (status == 3)
        {
            //infeasible
            std::cout << "[SolveEssentialMatrixGurobi] the problem is infeasible." << std::endl;
            return false;
        }
        else if (status != 9/*time-out*/ && status != 2 && status != 11 && status != 13)
        {
            //some other failure
            std::cout << "[SolveEssentialMatrixGurobi] optimize failed! status:" << status << std::endl;
            return false;
        }

        //get results
        Mat3 E;
        for (int i = 0; i < 9; i++)
        {
            E(i / 3, i % 3) = vars_E[i].get(GRB_DoubleAttr_X);
        }

        pvec_E->emplace_back(E);
    }
    catch (GRBException e)
    {
        std::cout << "Gurobi excception:" << e.getMessage() << std::endl;
    }
}