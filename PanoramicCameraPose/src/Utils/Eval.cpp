#pragma once
#include "pch.h"
#include "Eval.h"
#include <regex>  // regex_match, smatch
#include "ToolLayer.h"

namespace Utils
{
    std::vector<std::string> split(std::string s) {
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

    std::tuple<double, double, double, double> ParseStrToRT(std::string& str)
    {
        // example: pano_R90_T(0_0,0_5,0_0) => rotate(90 degree) and Translation (0, 0.5, 0)
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

        return std::make_tuple(degree, x, y, z);
    }

    openMVG::geometry::Pose3 ParseStrToPose(std::string& str)
    {
        // Step 1: Separate rotation angle and translation  
        auto[ degree, x, y, z ] = ParseStrToRT(str);

        // Step 2: Get rotation matrix and translation vector
        // Transition to OpenMVG coordinate system
        Eigen::Matrix3d transition;
        transition <<   -1.0, 0.0, 0.0,      // zind: change x to -x, y to z axis, and z to -y axis
                        0.0, 0.0, -1.0,
                        0.0, 1.0, 0.0;
        // zind座標系下的z軸為旋轉軸，因此zind的相對旋轉角度 順時鐘為正 逆時鐘為負
        openMVG::Vec3 rotaAxis = transition * openMVG::Vec3(0.0, 0.0, 1.0); 
        Eigen::AngleAxisd aa(glm::radians(degree), rotaAxis);
        openMVG::Mat3 R = aa.toRotationMatrix();
        
        Eigen::IOFormat vfmt(6, 0, "", "", "", "", "[", "]");
        openMVG::Vec3 t{ x, y, z };
        t = transition * t;
        std::cout << t.format(vfmt) << std::endl;
        t.normalize();
        return openMVG::geometry::Pose3{ R, t };
    }

    openMVG::geometry::Pose3 LoadM3DPose()
    {
        
        double tx, ty, tz, qx, qy, qz, qw;
        std::string name = ToolLayer::s_FileManager.GetPano01Filepath().parent_path().string() + "/relative_pose.txt";
        // Step 1: Load relative_pose.txt
        std::ifstream file;
        file.open(name);
        if (file.is_open())
        {
            std::string str;
            
            while (std::getline(file, str))
            {
                std::istringstream iss(str);
                iss >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            }
        }
        else
        {
            std::cout << "There is no relative file " << std::quoted(name) << std::endl;
        }
        

        // Step 2: Get rotation matrix and translation vector
        // Transition to OpenMVG coordinate system
        Eigen::Quaternion<double> quat{ qw, qx, -qz, -qy }; // m3d: change x to x, y to -z, z to -y
        
        openMVG::Mat3 R = quat.toRotationMatrix();
        std::cout << R;
        openMVG::Vec3 t{ tx, -ty, -tz };
        Eigen::IOFormat vfmt(6, 0, "", "", "", "", "[", "]");
        std::cout << t.format(vfmt) << std::endl;
        t.normalize();
        return openMVG::geometry::Pose3{ R, t };
    }

    openMVG::geometry::Pose3 LoadZInDPose()
    {

        double tx, ty, tz, qx, qy, qz, qw;
        std::string name = ToolLayer::s_FileManager.GetPano01Filepath().parent_path().string() + "/relative_pose.txt";
        // Step 1: Load relative_pose.txt
        std::ifstream file;
        file.open(name);
        if (file.is_open())
        {
            std::string str;

            while (std::getline(file, str))
            {
                std::istringstream iss(str);
                iss >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            }
        }
        else
        {
            std::cout << "There is no relative file " << std::quoted(name) << std::endl;
        }


        // Step 2: Get rotation matrix and translation vector
        // Transition to OpenMVG coordinate system
        Eigen::Quaternion<double> quat{ qw, qx, -qz, -qy }; // zind: change x to -x, y to z axis, and z to -y axis

        openMVG::Mat3 R = quat.toRotationMatrix();
        std::cout << R;
        openMVG::Vec3 t{ -tx, -tz, ty };
        Eigen::IOFormat vfmt(6, 0, "", "", "", "", "[", "]");
        std::cout << t.format(vfmt) << std::endl;
        t.normalize();
        return openMVG::geometry::Pose3{ R, t };
    }

    void EvaluationMetrics(const openMVG::geometry::Pose3& pose_gt, 
        const openMVG::geometry::Pose3& pose_est, float* rotation_error, float* translation_error)
    {
        Eigen::IOFormat fmt(6, 0, " ", "\n", "[", "]"), vfmt(6, 0, "", "", "", "", "[", "]"); // Matrix format and vector format.
        std::cout << "Estimated Pose\n";
        std::cout << "[             R               |    T    ]" << std::endl;
        std::cout << std::fixed << pose_est.asMatrix().format(fmt) << std::endl << std::endl;

        std::cout << "pose center(normalized): " << pose_est.center().normalized() << std::endl;
        // calculate rotation angle and axis from rotation martix
        Eigen::AngleAxisd angleAxis(pose_est.rotation());
        Eigen::Vector3d& axis = angleAxis.axis();
        std::cout << "Rotation Axis: " << axis.format(vfmt) << "\tAngle: " << angleAxis.angle() * (180.0 / M_PI) << std::endl << std::endl;

        // ====Ground Truth====
        std::cout << "Ground Truth Pose\n";
        std::cout << "[             R               |    T    ]" << std::endl;
        std::cout << std::fixed << pose_gt.asMatrix().format(fmt) << std::endl << std::endl;
        std::cout << "pose center(normalized): " << pose_gt.center().normalized() << std::endl;
        Eigen::AngleAxisd angleAxis_gt(pose_gt.rotation());
        Eigen::Vector3d& axis_gt = angleAxis_gt.axis();
        std::cout << "Rotation Axis: " << axis_gt.format(vfmt) << "\tAngle: " << angleAxis_gt.angle() * (180.0 / M_PI) << std::endl << std::endl;
        
        // ======================================================================================================================
        // Evaluation metrics method 
        // From paper Pose Estimation for Two-View Panoramas based on Keypoint Matching: a Comparative Studyand Critical Analysis
        // CVPRW 2022
        // ======================================================================================================================
        // Rotation error (RE)
        double RE = std::acos(std::clamp(((pose_gt.rotation().transpose() * pose_est.rotation()).trace() - 1.0) / 2.0, -1.0, 1.0));

        // Translation angular error (TAE)
        double TAE = std::acos(std::clamp(pose_gt.translation().dot(pose_est.translation()), -1.0, 1.0));

        std::cout << std::string(50, '=') << std::endl;
        
        std::stringstream buffer;
        if(rotation_error)
            *rotation_error = RE * 180.0 / M_PI;
        buffer << "Angular Rotation Error (in degree): " << std::setprecision(4) << RE * 180.0 / M_PI;
        std::cout << buffer.str() << std::endl;
        //ToolLayer::s_TextLog.AddLog("%s\t", buffer.str().c_str());
        //AddTextToShow(buffer.str().c_str());
        
        // buffer.clear(); // The clear() member function is used to clear the error state of the stream, not clear the contents of a stringstream.
        buffer.str(std::string()); // use this instead!.
        if (translation_error)
            *translation_error = TAE * 180.0 / M_PI;
        buffer << "Angular Translation error (in degree): " << std::setprecision(4) << TAE * 180.0 / M_PI;
        std::cout << buffer.str() << std::endl;
        //ToolLayer::s_TextLog.AddLog("%s\n", buffer.str().c_str());
        //AddTextToShow(buffer.str().c_str());   
        std::cout << std::string(50, '=') << std::endl;
    }
}

