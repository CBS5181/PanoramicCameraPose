#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "MatchPoints.h"

class MatchPoints;

class RelativePoseSolver
{
public:
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR, const MatchPoints& match_points);
};