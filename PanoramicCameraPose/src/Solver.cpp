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

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;


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

        // original ACRANSAC Eigth Point Algorithm
        
        //-- Essential matrix robust estimation from spherical bearing vectors
        /*
        std::vector<uint32_t> vec_inliers;

        
        // Define the AContrario angular error adaptor
        using KernelType =
            openMVG::robust::ACKernelAdaptor_AngularRadianError<
            //openMVG::essential::kernel::ThreePointUprightRelativePoseSolver,
            openMVG::EightPointRelativePoseSolver, // Use the 8 point solver in order to estimate E
            //openMVG::essential::kernel::FivePointSolver, // Use the 5 point solver in order to estimate E
            openMVG::AngularError,
            Mat3>;

        KernelType kernel(xL_spherical, xR_spherical);

        // Robust estimation of the Essential matrix and its precision
        Mat3 E;
        const double precision = std::numeric_limits<double>::infinity(); // infinity() for weighted sample // D2R(4.0); 0.0698132  // 
        std::cout << "precision:" << precision << std::endl;
        //const std::pair<double, double> ACRansacOut =
        //  ACRANSAC(kernel, vec_inliers, 1024, &E, precision, true);
        const std::pair<double, double> ACRansacOut = Weighted_ACRANSAC(kernel, vec_inliers, match_points.weights, 1024, &E, precision, true);
        const double& threshold = ACRansacOut.first;

        std::cout << "\n Angular threshold found: " << R2D(threshold) << "(Degree)" << std::endl;
        std::cout << "\n #Putatives/#inliers : " << xL_spherical.cols() << "/" << vec_inliers.size() << "\n" << std::endl;

        const bool bVertical = true;
        InlierMatches2SVG
        (
            jpg_filenameL,
            { imageL.Width(), imageL.Height() },
            featureL,
            jpg_filenameR,
            { imageR.Width(), imageR.Height() },
            featureR,
            vec_PutativeMatches,
            vec_inliers,
            "04_inliers.svg",
            bVertical
        );
        */
        
        std::vector<uint32_t> vec_inliers(L.size());
        std::iota(vec_inliers.begin(), vec_inliers.end(), 0);

        //solve essential matrix E:
        std::vector<Mat3> Es;
        if (method == 0)
        {
            //Pure Eigth Point Algorithm
            openMVG::EightPointRelativePoseSolver::Solve(xL_spherical, xR_spherical, &Es);
        }
        else if (method == 1)
        {
            //solve essential matrix by Gurobi?
            SolveEssentialMatrixGurobi(xL_spherical, xR_spherical, &Es);
        }

        std::cout << "solved essential matrix:" << std::endl << Es[0] << std::endl;

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

            float ratio = 0.0f;
            int cnt = 0;
            const float camera_height = 1.58f;
            float center_to_floor = 0.0f;

            std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;
            for (auto i : inliers_indexes) std::cout << i << " ";
            std::cout << std::endl;

            // Add a new landmark (3D point with its image observations)
            for (int i = 0; i < inliers_indexes.size(); ++i)
            {
                Landmark landmark;
                landmark.X = inliers_X[i];
                landmark.obs[tiny_scene.views[0]->id_view] = Observation(xL.col(inliers_indexes[i]), 0);
                landmark.obs[tiny_scene.views[1]->id_view] = Observation(xR.col(inliers_indexes[i]), 0);
                tiny_scene.structure.insert({ tiny_scene.structure.size(), landmark });
            }
            // TODO: scale and compare with ground truth.
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
