#pragma once
#include <vector>
#include "glm/glm.hpp"

class RelativePoseSolver
{
public:
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR, const std::vector<glm::vec2>& L, const std::vector<glm::vec2>& R, const std::vector<float>& depth_gt, std::vector<uint32_t> weights);
};