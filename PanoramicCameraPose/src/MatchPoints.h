#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "CornerPoints.h"

struct MatchPoints
{
	MatchPoints() {}
	
	std::vector<glm::vec2> left_pixels;
	std::vector<glm::vec2> right_pixels;
	std::vector<unsigned int> v_color;
	std::vector<uint32_t> weights;
	std::vector<glm::vec3> left_positions;  // 3D position from layout predictions
	std::vector<glm::vec3> right_positions; // 3D position from layout predictions
	
	size_t size() { return left_pixels.size(); }

	void AddPoint(const glm::vec2& left, const glm::vec2& right, const unsigned int color, 
		const uint32_t weight, const glm::vec3& left_pos = glm::vec3(0.0f, 0.0f, 0.0f), const glm::vec3& right_pos = glm::vec3(0.0f, 0.0f, 0.0f))
	{
		left_pixels.push_back(left);
		right_pixels.push_back(right);
		v_color.push_back(color);
		weights.push_back(weight);
		left_positions.push_back(left_pos);
		right_positions.push_back(right_pos);
	}

	void ClearPixel()
	{
		left_pixels.clear();
		right_pixels.clear();
		v_color.clear();
		weights.clear();
		left_positions.clear();
		right_positions.clear();
	}

	void RotateRightPixels()
	{
		// rotation to the right
		auto mid_riter = right_pixels.rbegin() + (size() / 2);
		std::rotate(mid_riter, mid_riter + 1, right_pixels.rend()); // ceiling points
		std::rotate(right_pixels.rbegin(), right_pixels.rbegin() + 1, mid_riter); // floor points

		auto mid_pos_riter = right_positions.rbegin() + (size() / 2);
		std::rotate(mid_pos_riter, mid_pos_riter + 1, right_positions.rend()); // ceiling points
		std::rotate(right_positions.rbegin(), right_positions.rbegin() + 1, mid_pos_riter); // floor points
	}

	void DeletePoint(int index)
	{
		if (index < 0 || index >= size())
			return;

		left_pixels.erase(left_pixels.begin() + index);
		right_pixels.erase(right_pixels.begin() + index);
		v_color.erase(v_color.begin() + index);
		weights.erase(weights.begin() + index);
		left_positions.erase(left_positions.begin() + index);
		right_positions.erase(right_positions.begin() + index);
	}

	void CovertCornerToMatch(CornerPoints& left_corner, CornerPoints& right_corner)
	{
		// Easily match when num of corners is equal.
		if (!left_corner.isLoad || !right_corner.isLoad)
		{
			std::cout << "Please Load Corner First!\n";
			return ; 
		}
		if (left_corner.floor_pixels.size() == right_corner.floor_pixels.size())
		{
			size_t num = left_corner.floor_pixels.size() + left_corner.ceil_pixels.size();
			left_pixels.reserve(num);
			right_pixels.reserve(num);
			
			std::move(left_corner.ceil_pixels.begin(), left_corner.ceil_pixels.end(), std::back_inserter(left_pixels));
			std::move(left_corner.floor_pixels.begin(), left_corner.floor_pixels.end(), std::back_inserter(left_pixels));
			std::move(left_corner.ceil_positions.begin(), left_corner.ceil_positions.end(), std::back_inserter(left_positions));
			std::move(left_corner.floor_positions.begin(), left_corner.floor_positions.end(), std::back_inserter(left_positions));

			std::move(right_corner.ceil_pixels.begin(), right_corner.ceil_pixels.end(), std::back_inserter(right_pixels));
			std::move(right_corner.floor_pixels.begin(), right_corner.floor_pixels.end(), std::back_inserter(right_pixels));
			std::move(right_corner.ceil_positions.begin(), right_corner.ceil_positions.end(), std::back_inserter(right_positions));
			std::move(right_corner.floor_positions.begin(), right_corner.floor_positions.end(), std::back_inserter(right_positions));

			for (size_t i = 0; i < num; ++i)
			{
				const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
				v_color.push_back(col);
				weights.push_back(10);
			}
			
			// Please note that the original container elements are left in an unspecified but valid state after std::move is called.
			// So clear for safety.
			left_corner.ClearPoints();
			right_corner.ClearPoints();
		}
		else
		{
			// TODO: Corner to Match algorithm
			std::cout << "TODO: Corner to Match algorithm\n";
		}
	}
};