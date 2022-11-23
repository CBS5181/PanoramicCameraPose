#include "pch.h"
#include "Solver.h"
#include "gurobi_c++.h"
#include <imgui/imgui.h>
#include "Utils/Eval.h"


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
            SolveEssentialMatrixGurobi(xL_spherical, xR_spherical, match_points.user_flags, &Es);
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
        
        if (RelativePoseFromEssential(xL_spherical,
            xR_spherical,
            Es[0], vec_inliers,
            &relative_pose,
            &inliers_indexes,
            &inliers_X))
        {
            // check inlier indexs
            std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;
            for (auto i : inliers_indexes) std::cout << i << " ";
            std::cout << std::endl;
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

            // ==Ground truth Pose==
            // get folder name : pano_Rxx_T(x,x,x)
            std::filesystem::path p(jpg_filenameR);
            Pose3 pose_gt = Utils::ParseStrToPose(p.parent_path().filename().string());
            Utils::EvaluationMetrics(pose_gt, relative_pose);

            // Add a new landmark (3D point with its image observations)
            for (int i = 0; i < inliers_indexes.size(); ++i)
            {
                Landmark landmark;
                landmark.X = inliers_X[i];
                landmark.obs[tiny_scene.views[0]->id_view] = Observation(xL.col(inliers_indexes[i]), 0);
                landmark.obs[tiny_scene.views[1]->id_view] = Observation(xR.col(inliers_indexes[i]), 0);
                tiny_scene.structure.insert({ tiny_scene.structure.size(), landmark });
            }

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

                Save(tiny_scene, "EssentialGeometry_refined.ply", ESfM_Data(ALL));
            }

        }
    }
}



bool RelativePoseSolver::SolveEssentialMatrixGurobi(
    const Mat3X& x1,
    const Mat3X& x2,
    const std::vector<bool>& user_flags,
    std::vector<Mat3>* pvec_E)
{
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    //note: we assume non-user matchings are ceiling and then floor points (half and half)
    //so there are (num_non-user / 2) possibilities of ceiling-to-ceiling / floor-to-floor matching  
    int num_non_user_matchings = 0;
    for (int i = 0; i < user_flags.size(); i++)
    {
        if (!user_flags[i])
            num_non_user_matchings++;
    }
    std::cout << "num_non_user_matchings:" << num_non_user_matchings << std::endl;

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

        //for every x1 ceiling point, it can be: 1) matched to one of the x2 ceiling point, or 2) discarded
        //however, addition touser-specified matching, in total we need (at least) 8 matchings
        
        //one-hot boolean flags for the ceiling-to-ceiling matching possibilities
        //std::vector<GRBVar> vars_possibilities;
        //int num_possibilities = num_non_user_matchings / 2;
        //for (int i = 0; i < num_possibilities; i++)
        //{
         //   vars_possibilities.push_back(model.addVar(0, 1, 0, GRB_BINARY));
        //}

        //for every x1 ceiling point, it is either one of the matching possibility flags , or it is discarded

        //boolean flags for relaxing certain rows of the essential matrix system
        //note: user-specified matchings are NOT relaxable
        std::vector<GRBVar> vars_relax;
        for (int i = 0; i < x1.cols(); i++)
        {
            vars_relax.push_back(model.addVar(0, 1, 0, GRB_BINARY));
        }

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

        //LHS term slack variable constraint:
        const float LARGE_NUM = 1000;
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

            //note: if relax flag is active, the constraint is relaxed
            model.addConstr(term - vars_LHS[i] >= -LARGE_NUM * vars_relax[i]);
            model.addConstr(term - vars_LHS[i] <= LARGE_NUM * vars_relax[i]);
        }

        //because we need 8 matchings in total, 
        //(# of non-user matchings - # of active relax flags) + # of user-specified matchings >= 8   -->
        //# of active relax flags <= # of non-user matchings + # of user-specified matchings - 8
        //# of active relax flags <= # total matchings - 8
        {
            GRBLinExpr sum;
            for (int i = 0; i < x1.cols(); i++)
            {
                sum += vars_relax[i];
            }

            model.addConstr(sum <= x1.cols() - 8);
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

        //report results
        std::cout << "relax flags: ";
        for (int i = 0; i < x1.cols(); i++)
        {
            std::cout << vars_relax[i].get(GRB_DoubleAttr_X);
        }
        std::cout << std::endl;

        std::cout << "LHS terms: ";
        for (int i = 0; i < x1.cols(); i++)
        {
            std::cout << vars_LHS[i].get(GRB_DoubleAttr_X);
        }

        pvec_E->emplace_back(E);
    }
    catch (GRBException e)
    {
        std::cout << "Gurobi excception:" << e.getMessage() << std::endl;
    }
}


