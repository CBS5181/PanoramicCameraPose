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
	std::vector<glm::vec3> left_positions;  // 3D position from LED2-Net
	std::vector<glm::vec3> right_positions; // 3D position from LED2-Net
	std::vector<bool> user_flags;  //is this matching user-specified?

	size_t size() { return cnt; }

	void AddPoint(const glm::vec2& left, const glm::vec2& right, const unsigned int color, const uint32_t weight, const glm::vec3& left_pos = glm::vec3(0.0f, 0.0f, 0.0f), const glm::vec3& right_pos = glm::vec3(0.0f, 0.0f, 0.0f), const bool IsUser = false)
	{
		left_pixels.push_back(left);
		right_pixels.push_back(right);
		v_color.push_back(color);
		weights.push_back(weight);
		left_positions.push_back(left_pos);
		right_positions.push_back(right_pos);
		++cnt;
		user_flags.push_back(IsUser);
	}

	void ClearPixel()
	{
		left_pixels.clear();
		right_pixels.clear();
		v_color.clear();
		weights.clear();
		left_positions.clear();
		right_positions.clear();
		cnt = 0;
	}

	void RotateRightPixels()
	{
		// rotation to the right
		auto mid_riter = right_pixels.rbegin() + (cnt / 2);
		std::rotate(mid_riter, mid_riter + 1, right_pixels.rend()); // ceiling points
		std::rotate(right_pixels.rbegin(), right_pixels.rbegin() + 1, mid_riter); // floor points

		auto mid_pos_riter = right_positions.rbegin() + (cnt / 2);
		std::rotate(mid_pos_riter, mid_pos_riter + 1, right_positions.rend()); // ceiling points
		std::rotate(right_positions.rbegin(), right_positions.rbegin() + 1, mid_pos_riter); // floor points
	}
};