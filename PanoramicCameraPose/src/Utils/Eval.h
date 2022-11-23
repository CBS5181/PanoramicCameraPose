#pragma once
#include "glm/glm.hpp"
#include <string>
#include <vector>

namespace Utils
{
	std::vector<std::string> split(std::string s);
	openMVG::geometry::Pose3 ParseStrToPose(std::string& str);
	void EvaluationMetrics(const openMVG::geometry::Pose3& pose_gt, const openMVG::geometry::Pose3& pose_est);
}