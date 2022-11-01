#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "MatchPoints.h"
#include "openMVG/numeric/eigen_alias_definition.hpp"

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

//solve essential matrix by Gurobi
	//x1,x2: columns of bearing vectors of feature points
	//pvec_E: solved essential matrix
class GurobiSolver
{
public:
	static void Solve(const openMVG::Mat3X& x1,
		const openMVG::Mat3X& x2,
		std::vector<openMVG::Mat3>* pvec_E);
};