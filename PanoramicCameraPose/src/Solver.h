#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "MatchPoints.h"


class SIFTSolver
{
public:
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR, const std::vector<glm::vec3>& pos_gt, MatchPoints& match_points);
};

class RelativePoseSolver
{
public:
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR, const MatchPoints& match_points);
};