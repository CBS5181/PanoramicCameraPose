#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "MatchPoints.h"

#include "openMVG/numeric/eigen_alias_definition.hpp"

class MatchPoints;

class RelativePoseSolver
{
public:
	//method: 0=8-point algorithm, 1=Gurobi
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR,
		const MatchPoints& match_points, int method = 0);

	//solve essential matrix by Gurobi
	//x1,x2: columns of bearing vectors of matched feature points
	//pvec_E: solved essential matrix
	static bool SolveEssentialMatrixGurobi(
		const openMVG::Mat3X& x1,
		const openMVG::Mat3X& x2,
		std::vector<openMVG::Mat3>* pvec_E);

};