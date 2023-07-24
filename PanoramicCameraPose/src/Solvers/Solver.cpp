#include "pch.h"
#include "Solver.h"
#include "gurobi_c++.h"
#include <imgui/imgui.h>
#include "Utils/Eval.h"
#include "ToolLayer.h"


using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;



void RelativePoseSolver::Solve(const char* jpg_filenameL, const char* jpg_filenameR, 
    const std::vector<MatchPoints>& match_points_all, std::vector<glm::vec2>& errors, int method)
{
    const int debug_outputs_level = 0;  //0: no outputs at all, 1: minimal output, 2: all output debug messages and pictures

    Image<unsigned char> imageL, imageR;
    ReadImage(jpg_filenameL, &imageL);
    ReadImage(jpg_filenameR, &imageR);

    //for every given match_points, find a solution, then report the "best" one
    
    errors.clear();
    
    for (int ii = 0; ii < match_points_all.size(); ii++)
    {
        const MatchPoints& match_points = match_points_all[ii];

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

        if (debug_outputs_level >= 2)
        {
            std::cout
                << "Left image count: " << featureL.size() << std::endl
                << "Right image count: " << featureR.size() << std::endl;

            //- Draw features on the two image (side by side)        
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
        
        if (debug_outputs_level >= 2)
        {
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
        }

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
        Mat3 E;
        {
            std::vector<Mat3> Es;
            if (method == 0)  //Pure Eigth Point Algorithm            
            {
                openMVG::EightPointRelativePoseSolver::Solve(xL_spherical, xR_spherical, &Es);
            }
            else if (method == 1)  //solve essential matrix by Gurobi?
            {
                SolveEssentialMatrixGurobi(xL_spherical, xR_spherical, &Es);
            }
            // ZInD test cases 75-100/10112
            // Gurobi excception:Double value is Nan.
            // Gurobi bug at model.setObjective(obj); causes E size is zero.!
            if (Es.size())
                E = Es[0];  //just take the first solved E
            else // use eight-point instead.
            {
                openMVG::EightPointRelativePoseSolver::Solve(xL_spherical, xR_spherical, &Es);
                E = Es[0];
            }
        }

        //report essential matrix equation errors
        if (debug_outputs_level >= 2)
        {
            std::cout << "Solved essential matrix:" << std::endl << E << std::endl;
            
            std::cout << "Essential matrix equation residuals:" << std::endl;
            for (int i = 0; i < xL_spherical.cols(); i++)
            {
                Eigen::Vector3d x1 = xL_spherical.col(i);
                Eigen::Vector3d x2 = xR_spherical.col(i);
                std::cout << x2.transpose() * E * x1 << std::endl;
            }
        }
        if (debug_outputs_level >= 1)
        {
            //just report abs sum of essential matrix equation residuals
            float abs_sum = 0;
            for (int i = 0; i < xL_spherical.cols(); i++)
            {
                Eigen::Vector3d x1 = xL_spherical.col(i);
                Eigen::Vector3d x2 = xR_spherical.col(i);
                abs_sum += abs( x2.transpose() * E * x1 );
            }

            char str[1000] = { NULL };
            sprintf(str, "Essential matrix equation error: %f", abs_sum);
            //AddTextToShow(str);
            std::cout << str << std::endl;
        }

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
            if (debug_outputs_level >= 2)
            {
                // check inlier indexs
                std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;
                for (auto i : inliers_indexes) std::cout << i << " ";
                std::cout << std::endl;
            }

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
            std::string str = p.parent_path().filename().string();
            Pose3 pose_gt;
            if (str == "pano_secondary")
            {
                pose_gt = Utils::LoadM3DPose();
                //pose_gt = Utils::LoadZInDPose();
            }
            else
            {
                pose_gt = Utils::ParseStrToPose(str);
            }

            //evaluate error metrics and save calculated errors
            float rotation_error = -1, translation_error = -1;
            Utils::EvaluationMetrics(pose_gt, relative_pose, &rotation_error, &translation_error);
            errors.push_back(glm::vec2(rotation_error, translation_error));

            // Add a new landmark (3D point with its image observations)
            for (int i = 0; i < inliers_indexes.size(); ++i)
            {
                Landmark landmark;
                landmark.X = inliers_X[i];
                landmark.obs[tiny_scene.views[0]->id_view] = Observation(xL.col(inliers_indexes[i]), 0);
                landmark.obs[tiny_scene.views[1]->id_view] = Observation(xR.col(inliers_indexes[i]), 0);
                tiny_scene.structure.insert({ tiny_scene.structure.size(), landmark });
            }

            //// Perform Bundle Adjustment of the scene
            //Bundle_Adjustment_Ceres bundle_adjustment_obj;
            //if (bundle_adjustment_obj.Adjust(tiny_scene,
            //    Optimize_Options(
            //        Intrinsic_Parameter_Type::NONE,
            //        Extrinsic_Parameter_Type::ADJUST_ALL,
            //        Structure_Parameter_Type::ADJUST_ALL)))
            //{
            //    std::vector<double> residuals;
            //    // Compute reprojection error
            //    const Pose3 pose0 = tiny_scene.poses[tiny_scene.views[0]->id_pose];
            //    const Pose3 pose1 = tiny_scene.poses[tiny_scene.views[1]->id_pose];

            //    for (const auto& landmark_it : tiny_scene.GetLandmarks())
            //    {
            //        const Landmark& landmark = landmark_it.second;
            //        const Observations& obs = landmark.obs;
            //        Observations::const_iterator iterObs_xI = obs.find(tiny_scene.views[0]->id_view);
            //        Observations::const_iterator iterObs_xJ = obs.find(tiny_scene.views[1]->id_view);

            //        const Observation& ob_x0 = iterObs_xI->second;
            //        const Observation& ob_x1 = iterObs_xJ->second;

            //        const Vec2 residual_I = cameraL.residual(pose0(landmark.X), ob_x0.x); // pose0() °µ rotation ©M translation
            //        const Vec2 residual_J = cameraR.residual(pose1(landmark.X), ob_x1.x);
            //        residuals.emplace_back(residual_I.norm());
            //        residuals.emplace_back(residual_J.norm());

            //    }

            //    if (debug_outputs_level >= 1)
            //    {
            //        std::cout << "(BA) Residual statistics (pixels):" << std::endl;
            //        minMaxMeanMedian<double>(residuals.cbegin(), residuals.cend(), std::cout);
            //    }

            //    if (debug_outputs_level >= 2)
            //    {
            //        openMVG::sfm::Save(tiny_scene, "EssentialGeometry_refined.ply", ESfM_Data(ALL));
            //    }
            //}

        }
        else
        {
		    float rotation_error = -999, translation_error = -999;
		    errors.push_back(glm::vec2(rotation_error, translation_error));
            AddTextToShow("RelativePoseFromEssential() failed!");
            //ToolLayer::s_TextLog.AddLog("RelativePoseFromEssential() failed!\n");
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

    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel model = GRBModel(env);

    try
    {
        //variables: 9 entries in E, row major
        std::vector<GRBVar> vars_E;
        for (int i = 0; i < 9; i++)
        {
            vars_E.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS));
        }

        model.update();

        //obj: least squares of Ae = 0  (A is rows of size-9 cols, e is the vector form of E)
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
                term += row[j] * vars_E[j];
            }
            
            obj += term * term; // argmin(X) for|AX|^2 
        }
        model.setObjective(obj);  //to minimize

        ////constraints: s.t. |X|^2 = 1 (regularization constraint)

        //E vector is unit vector
        {
            GRBQuadExpr term;
            for (int j = 0; j < 9; j++)
            {
                term += vars_E[j] * vars_E[j];
            }

            model.addQConstr(term == 1);
        }

        //2D rotation (along y-axis) and 2D translation constraint:
        //E[0,0], E[2,0], E[1,1], E[0,2], E[2,2] are zeros!
        if (true)
        {
            model.addConstr(vars_E[0] == 0);
            model.addConstr(vars_E[2] == 0);
            model.addConstr(vars_E[4] == 0);
            model.addConstr(vars_E[6] == 0);
            model.addConstr(vars_E[8] == 0);
        }
        //solve!
        model.getEnv().set(GRB_IntParam_OutputFlag, false);  //silent
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
        std::cout << "Gurobi exception:" << e.getMessage() << std::endl;
    }
}

bool RelativePoseSolver::SolveEssentialMatrixGurobiWithRelaxFlags(
    const Mat3X& x1,
    const Mat3X& x2,
    const std::vector<bool>& user_flags,
    std::vector<Mat3>* pvec_E)
{
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    //note: we assume non-user matchings are ceiling and then floor points (half and half)
    //so there are (num_non-user / 2) possibilities of ceiling-to-ceiling / floor-to-floor matching  
    int num_non_user_matchings = x1.cols();
    if (user_flags.size() > 0)
    {
        for (int i = 0; i < user_flags.size(); i++)
        {
            if (!user_flags[i])
                num_non_user_matchings++;
        }
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

        //for every x1 ceiling point, it is either one of the matching possibility flags , or it is discarde
        //boolean flags for relaxing certain rows of the essential matrix system
        //note: user-specified matchings are NOT relaxable
        std::vector<GRBVar> vars_relax;
        for (int i = 0; i < x1.cols(); i++)
        {
            vars_relax.push_back(model.addVar(0, 1, 0, GRB_BINARY));
        }

        model.update();

        //objective: minimize sum of squared of LHS terms
        GRBQuadExpr obj;
        for (int i = 0; i < x1.cols(); i++)
        {
            //higher weight fro user-specified matchings?
            float weight = 1;
            if (user_flags.size() > 0 && user_flags[i])
            {
                weight = 1000;
            }

            obj += weight * vars_LHS[i] * vars_LHS[i];
        }
        model.setObjective(obj);  //to minimize

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

        //because we need at least 8 matchings in total, 
        //(# of non-user matchings - # of active relax flags) + # of user-specified matchings >= 8   -->
        //# of active relax flags <= # of non-user matchings + # of user-specified matchings - 8
        //# of active relax flags <= # total matchings - 8
        if (x1.cols() >= 8)
        {
            GRBLinExpr sum;
            for (int i = 0; i < x1.cols(); i++)
            {
                sum += vars_relax[i];
            }

            //model.addConstr(sum <= x1.cols() - 8);
            model.addConstr(sum <= 0);
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
            std::cout << vars_relax[i].get(GRB_DoubleAttr_X) << " ";
        }
        std::cout << std::endl;

        std::cout << "LHS terms: ";
        for (int i = 0; i < x1.cols(); i++)
        {
            std::cout << vars_LHS[i].get(GRB_DoubleAttr_X) << " ";
        }

        pvec_E->emplace_back(E);
    }
    catch (GRBException e)
    {
        std::cout << "Gurobi excception:" << e.getMessage() << std::endl;
    }
}

bool RelativePoseSolver::SolveEssentialMatrixGurobiMulti(
    const Mat3X& x1,
    const Mat3X& x2,
    const std::vector<std::pair<int,int>>& indices,
    std::vector<Mat3>* pvec_E)
{
    assert(x1.rows() == x2.rows());
    assert(x1.cols() == x2.cols());

    //note: we assume layout (non-user) matchings are ceiling and then floor points (first half and second half)
    int num_sides = 0;
    for (int i = 0; i < indices.size(); i++)
    {
        std::cout << "index# " << i << ": " << indices[i].first << "," << indices[i].second << std::endl;

        if (indices[i].first >= 0 && num_sides < indices[i].first + 1)
            num_sides = indices[i].first + 1;
    }
    num_sides /= 2;  //half and half
    std::cout << "num_sides:" << num_sides << std::endl;

    //we prepare possible pair-wise combinations of layout x1/x2 records
    std::vector<std::pair<int, int>> combinations;
    std::map<int, std::vector<int>> record_combination_map;  //first: record index, second: its combinations
    {
        for (int i = 0; i < indices.size(); i++)
        {
            std::pair<int, int> index0 = indices[i];
            if (index0.second < 0)
                continue;  //skip user-specified records

            //let's find its possible matching records
            for (int j = 0; j < indices.size(); j++)
            {
                std::pair<int, int> index1 = indices[j];
                if (index1.second < 0)
                    continue;  //skip user-specified records

                //first half is ceiling, second half is floor
                if (index0.first < num_sides && index1.first < num_sides)
                {
                    //possible ceil-ceill combinations
                    if (index0.second == index1.second)
                    {
                        //save j as i's combination 
                        record_combination_map[i].push_back(j);

                        //gotcha!
                        combinations.push_back(std::make_pair(i, j));
                    }
                }
                else if (index0.first >= num_sides && index1.first >= num_sides)
                {
                    //floor-floor combinations
                    if (index0.second == index1.second)
                    {
                        //save j as i's combination 
                        record_combination_map[i].push_back(j);

                        //gotcha!
                        combinations.push_back(std::make_pair(i, j));
                    }
                }
            }
        }
    }

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

        //we have to consider every possible way of layout (ceiling-to-ceiling / floor-to-floor) matching,
        //and every such way is a row of LHS terms of the essential matrix system Ae = 0:
        std::vector<GRBVar> vars_LHS;  // slack variables for each layout LHS term
        for (int i = 0; i < combinations.size(); i++)
        {
            vars_LHS.push_back(model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS));
        }

        //for every x1 ceiling point, it is either one of the matching possibility flags , or it is discarded

        //boolean flags for taking certain rows of the essential matrix system
        std::vector<GRBVar> vars_take;
        for (int i = 0; i < combinations.size(); i++)
        {
            vars_take.push_back(model.addVar(0, 1, 0, GRB_BINARY));
        }

        model.update();

        //objective: minimize sum of squared of layout LHS terms
        GRBQuadExpr obj;
        for (int i = 0; i < combinations.size(); i++)
        {
            obj += vars_LHS[i] * vars_LHS[i];
        }
        model.setObjective(obj);  //to minimize

        ////constraints: 

        //LHS term slack variable constraint:
        const float LARGE_NUM = 1000;
        for (int i = 0; i < combinations.size(); i++)
        {
            int x1_index = combinations[i].first;
            int x2_index = combinations[i].second;

            std::vector<float> row;  //LHS weight terms
            row.push_back(x1(0, x1_index) * x2(0, x2_index));
            row.push_back(x1(1, x1_index) * x2(0, x2_index));
            row.push_back(x1(2, x1_index) * x2(0, x2_index));
            row.push_back(x1(0, x1_index) * x2(1, x2_index));
            row.push_back(x1(1, x1_index) * x2(1, x2_index));
            row.push_back(x1(2, x1_index) * x2(1, x2_index));
            row.push_back(x1(0, x1_index) * x2(2, x2_index));
            row.push_back(x1(1, x1_index) * x2(2, x2_index));
            row.push_back(x1(2, x1_index) * x2(2, x2_index));

            //std::cout << row[0] << " " << row[1] << " " << row[2] << " " << row[3] << " " << row[4] << " "
            //    << row[5] << " " << row[6] << " " << row[7] << " " << row[8] << std::endl;

            GRBLinExpr term;
            for (int j = 0; j < 9; j++)  //w.r.t. a row of A
            {
                term += row[j] * vars_E[j];
            }

            //note: if the corresponding "take" flag is off, this constraint is dropped
            model.addConstr((term - vars_LHS[i]) >= (-LARGE_NUM * (1 - vars_take[i])));
            model.addConstr((term - vars_LHS[i]) <= (LARGE_NUM * (1 - vars_take[i])));
        }

        //for every x1 record's combinations, at most one of them is true
        for (std::map<int, std::vector<int>>::iterator itr = record_combination_map.begin(); itr != record_combination_map.end(); itr++)
        {
            GRBLinExpr sum;

            std::cout << "map " << (*itr).first << ":";

            std::vector<int>& cs = (*itr).second;
            for (int i = 0; i < cs.size(); i++)
            {
                std::cout << cs[i] << ",";
                sum += vars_take[cs[i]];
            }
            std::cout << std::endl;

            //model.addConstr(sum <= 1);
            model.addConstr(sum == 1);
        }

        //layout combinatorics: for entry of the same order of every x1 record's combinations, they are the same
        {
            std::vector< std::vector<int> > records_at_same_orders(record_combination_map.begin()->second.size());
            for (std::map<int, std::vector<int>>::iterator itr = record_combination_map.begin(); itr != record_combination_map.end(); itr++)
            {
                std::vector<int>& cs = (*itr).second;
                if (cs.size() == records_at_same_orders.size())
                {
                    for (int i = 0; i < cs.size(); i++)
                    {
                        records_at_same_orders[i].push_back(cs[i]);
                    }
                }
            }

            for (int i = 0; i < records_at_same_orders.size(); i++)
            {
                //the "take" variables of these records are all the same
                //we equal them circularly
                std::vector<int>& records = records_at_same_orders[i];
                for (int j = 0; j < records.size(); j++)
                {
                    int r0 = records[j];

                    std::cout << r0 << "-";
                    int r1 = records[(j + 1) % records.size()];
                    model.addConstr(vars_take[r0] == vars_take[r1]);
                }
                std::cout << std::endl;
            }
        }

        //together with user-specified matchings, we need at least 8 matchings total
        //so sum of active "take" flags should >= 8 - # of user matchings
        /*{
            GRBLinExpr sum;
            for (int i = 0; i < vars_take.size(); i++)
            {
                sum += vars_take[i];
            }
            model.addConstr(sum >= 8 - (x1.cols() - num_layout_matchings));
        }*/

        //user-specified matchings: as hard constraints
        for (int i = 0; i < x1.cols(); i++)
        {
            if (indices[i].first == -1)
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

                model.addConstr(term == 0);
            }
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
        for (int i = 0; i < combinations.size(); i++)
        {
            std::cout << combinations[i].first << "-" << combinations[i].second << ": take=" <<
                vars_take[i].get(GRB_DoubleAttr_X) << " LHS=" << vars_LHS[i].get(GRB_DoubleAttr_X) << std::endl;
        }

        //debug:
        for (int i = 0; i < combinations.size(); i++)
        {
            int x1_index = combinations[i].first;
            int x2_index = combinations[i].second;

            std::vector<float> row;  //LHS weight terms
            row.push_back(x1(0, x1_index) * x2(0, x2_index));
            row.push_back(x1(1, x1_index) * x2(0, x2_index));
            row.push_back(x1(2, x1_index) * x2(0, x2_index));
            row.push_back(x1(0, x1_index) * x2(1, x2_index));
            row.push_back(x1(1, x1_index) * x2(1, x2_index));
            row.push_back(x1(2, x1_index) * x2(1, x2_index));
            row.push_back(x1(0, x1_index) * x2(2, x2_index));
            row.push_back(x1(1, x1_index) * x2(2, x2_index));
            row.push_back(x1(2, x1_index) * x2(2, x2_index));

            float val = 0;
            for (int j = 0; j < 9; j++)  //w.r.t. a row of A
            {
                val += row[j] * vars_E[j].get(GRB_DoubleAttr_X);
            }

            std::cout << "val:" << val << std::endl;

        }

        pvec_E->emplace_back(E);
    }
    catch (GRBException e)
    {
        std::cout << "Gurobi excception:" << e.getMessage() << std::endl;
    }
}

