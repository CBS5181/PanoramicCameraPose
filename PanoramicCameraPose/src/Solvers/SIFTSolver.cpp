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
            0.8f, matching::BRUTE_FORCE_L2,
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
                openMVG::essential::kernel::ThreePointUprightRelativePoseSolver,
                //openMVG::EightPointRelativePoseSolver, // Use the 8 point solver in order to estimate E
                //openMVG::essential::kernel::FivePointSolver, // Use the 5 point solver in order to estimate E
                openMVG::AngularError,
                Mat3>;

            KernelType kernel(xL_spherical, xR_spherical);

            // Robust estimation of the Essential matrix and its precision
            Mat3 E;
            const double precision = std::numeric_limits<double>::infinity(); //D2R(4.0);  //
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
            
            if (vec_inliers.size() >= 0) // 60 is used to filter solution with few common geometric matches (unstable solution)
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

                        int ind = glm::round(xL.col(inliers_indexes[i]).y()) * 1024 + glm::round(xL.col(inliers_indexes[i]).x());
                        const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
                        match_points.AddPoint(glm::vec2(xL.col(inliers_indexes[i]).x(), xL.col(inliers_indexes[i]).y()), glm::vec2(xR.col(inliers_indexes[i]).x(), xR.col(inliers_indexes[i]).y()), col, 10);
                        //std::cout << "X: " << xL.col(inliers_indexes[i]).x() << " Y: " << xL.col(inliers_indexes[i]).y() << " ind: " << ind << std::endl;
                        //std::cout << "3D point:     " << inliers_X[i].x() << " " << inliers_X[i].y() << " " << inliers_X[i].z() << std::endl;
                        //std::cout << "Match " << i << std::endl;
                        //std::cout << "3D point:     " << std::setw(10) << point.x << " " << std::setw(10) << point.y << " " << std::setw(10) << point.z << std::endl;
                        //std::cout << "3D point(gt): " << std::setw(10) << pos_gt[ind].x << " " << std::setw(10) << pos_gt[ind].y << " " << std::setw(10) << pos_gt[ind].z << std::endl;
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

                        Save(tiny_scene, "EssentialGeometry_refined.ply", ESfM_Data(ALL));
                    }
                }
            }
        }
    }
}