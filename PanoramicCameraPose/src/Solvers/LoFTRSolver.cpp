#include "pch.h"
#include "Solvers/Solver.h"
#include "Utils/Eval.h"
#include <imgui/imgui.h>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;


void LoFTRSolver::Solve(const char* jpg_filenameL, const char* jpg_filenameR, const std::string& loftr_filename, MatchPoints& match_points)
{
    Image<unsigned char> imageL, imageR;
    ReadImage(jpg_filenameL, &imageL);
    ReadImage(jpg_filenameR, &imageR);

    // Setup 1 Load LoFTR matching points into match_points
    std::filesystem::path fname1{ jpg_filenameL }, fname2{ jpg_filenameR };

    match_points.ClearPixel();
    std::ifstream file(loftr_filename);
    if (file.fail())
    {
        std::cout << "LoFTR file for this paired panoramic images doesn't exist.\n";
        return ;
    }
    std::string str;
    unsigned int cnt = 0;
    while (std::getline(file, str))
    {
        // read LoFTR matching pixels
        std::istringstream iss(str);
        glm::vec2 loftr_matching_left, loftr_match_right;
        glm::vec3 pos{0.0f};
        iss >> loftr_matching_left.x >> loftr_matching_left.y >> loftr_match_right.x >> loftr_match_right.y;

        // depth ground trugh from 3D scene
        //unsigned int ind = corner_pixel.y * 1024 + corner_pixel.x;
        const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
        //s_MatchPoints.AddPoint(corner_pixel, corner_pixel2, col, 10, m_PanoPos_gt[ind], false/*not user-specified*/); // LED2Net weight = 10
        match_points.AddPoint(loftr_matching_left, loftr_match_right, col, 10, pos, pos);
        ++cnt;
    }

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

    {
        // Draw correspondences after Nearest Neighbor ratio filter
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
    {
        //A. get back interest point and send it to the robust estimation framework
        Mat2X
            xL(2, vec_PutativeMatches.size()),
            xR(2, vec_PutativeMatches.size());

        for (size_t k = 0; k < vec_PutativeMatches.size(); ++k) {
            const PointFeature& imaL = featureL[vec_PutativeMatches[k].i_];
            const PointFeature& imaR = featureR[vec_PutativeMatches[k].j_];
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
                openMVG::essential::kernel::ThreePointUprightRelativePoseSolver,
                //openMVG::EightPointRelativePoseSolver, // Use the 8 point solver in order to estimate E
                //openMVG::essential::kernel::FivePointSolver, // Use the 5 point solver in order to estimate E
                openMVG::AngularError,
                Mat3>;

            KernelType kernel(xL_spherical, xR_spherical);

            // Robust estimation of the Essential matrix and its precision
            Mat3 E;
            const double precision = std::numeric_limits<double>::infinity(); // D2R(4.0f)
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
                featureL,
                jpg_filenameR,
                { imageR.Width(), imageR.Height() },
                featureR,
                vec_PutativeMatches,
                vec_inliers,
                "04_inliers.svg",
                bVertical
            );

            if (vec_inliers.size() > 8) // 60 is used to filter solution with few common geometric matches (unstable solution)
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
                    std::cout << "inliers_indexes : " << inliers_indexes.size() << std::endl;
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

                        int ind = glm::round(xL.col(inliers_indexes[i]).y()) * 1024 + glm::round(xL.col(inliers_indexes[i]).x());
                        const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
                        match_points.AddPoint(glm::vec2(xL.col(inliers_indexes[i]).x(), xL.col(inliers_indexes[i]).y()), glm::vec2(xR.col(inliers_indexes[i]).x(), xR.col(inliers_indexes[i]).y()), col, 10);
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

                        Save(tiny_scene, "EssentialGeometry_refined.ply", ESfM_Data(ALL));
                    }
                }
            }
        }
    }
}