#pragma once

#include "glm/glm.hpp"
#include "MatchPoints.h"
#include <vector>

class SIFTSolver
{
public:
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR, const std::vector<glm::vec3>& pos_gt, MatchPoints& match_points);
};

class LoFTRSolver
{
public:
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR, const std::string& loftr_filename, MatchPoints& match_points);
};

class RelativePoseSolver
{
public:
	//method: 0=8-point algorithm, 1=Gurobi
	//match_points_all: try multiple match-points
	//errors: solved errors (rotation angle error and translation angle error)
	static void Solve(const char* jpg_filenameL, const char* jpg_filenameR,
		const std::vector<MatchPoints> &match_points_all, std::vector<glm::vec2> &errors, int method);

	//solve essential matrix by Gurobi
	//x1,x2: columns of bearing vectors of matched feature points
	//user_flags: flags of "user specified" matchings (size = x1.cols)
	//pvec_E: solved essential matrix
	static bool SolveEssentialMatrixGurobiWithRelaxFlags(
		const openMVG::Mat3X& x1,
		const openMVG::Mat3X& x2,
		const std::vector<bool>& user_flags,
		std::vector<openMVG::Mat3>* pvec_E);

	static bool SolveEssentialMatrixGurobi(
		const openMVG::Mat3X& x1,
		const openMVG::Mat3X& x2,
		std::vector<openMVG::Mat3>* pvec_E);

	//test:
	static bool SolveEssentialMatrixGurobiMulti(
		const openMVG::Mat3X& x1,
		const openMVG::Mat3X& x2,
		const std::vector<std::pair<int, int>>& indices,
		std::vector<openMVG::Mat3>* pvec_E);
};