#pragma once
#include <vector>
#include "glm/glm.hpp"
#include "imgui/imgui.h"

struct ImDrawList;

struct CornerPoints
{
	void RenderCorners(ImDrawList* draw_list, const ImVec2& viewport, float ratio, bool isPano02, bool isTransfrom = false) const;
	void AddPoints(const std::vector<glm::vec2>& pixels, const std::vector<glm::vec3>& positions);
	void ClearPoints();
	bool isLoad = false;
	std::vector<glm::vec2> ceil_pixels;
	std::vector<glm::vec2> floor_pixels;
	std::vector<glm::vec3> ceil_positions;  // 3D position from LED2-Net
	std::vector<glm::vec3> floor_positions; // 3D position from LED2-Net
};

