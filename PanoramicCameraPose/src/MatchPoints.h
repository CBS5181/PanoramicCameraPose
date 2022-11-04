#pragma once
#include <vector>
#include "glm/glm.hpp"
struct MatchPoints
{
	MatchPoints() {}
	
	size_t cnt = 0;
	std::vector<glm::vec2> left_pixels;
	std::vector<glm::vec2> right_pixels;
	std::vector<unsigned int> v_color;
	std::vector<uint32_t> weights;
	std::vector<glm::vec3> positions;
	std::vector<bool> user_flags;  //is this matching user-specified?

	size_t size() { return cnt; }

	void AddPoint(const glm::vec2& left, const glm::vec2& right, const unsigned int color, const uint32_t weight, const glm::vec3& pos, const bool IsUser=false)
	{
		left_pixels.push_back(left);
		right_pixels.push_back(right);
		v_color.push_back(color);
		weights.push_back(weight);
		positions.push_back(pos);
		++cnt;
		user_flags.push_back(IsUser);
	}

	void ClearPixel()
	{
		left_pixels.clear();
		right_pixels.clear();
		v_color.clear();
		weights.clear();
		positions.clear();
		cnt = 0;
	}

	void RotateRightPixels()
	{
		// rotation to the right
		auto mid_riter = right_pixels.rbegin() + (cnt / 2);
		std::rotate(mid_riter, mid_riter + 1, right_pixels.rend()); // ceiling points
		std::rotate(right_pixels.rbegin(), right_pixels.rbegin() + 1, mid_riter); // floor points
	}
};