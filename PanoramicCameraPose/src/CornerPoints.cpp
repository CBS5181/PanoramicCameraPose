#include "pch.h"
#include "CornerPoints.h"


void CornerPoints::RenderCorners(ImDrawList* draw_list, const ImVec2& viewport, float ratio, bool isPano02,  bool isTransfrom) const
{
	if (!isLoad) return ;
	const ImU32& col_ceil = IM_COL32(0, 0, 255, 255); // show ceiling corners with blue color 
	const ImU32& col_floor = IM_COL32(255, 0, 0, 255); // show ceiling corners with red color
	const ImU32& col_trans = IM_COL32(255, 255, 0, 255);
	if (isTransfrom)
	{
		for (auto& pixel : ceil_pixels)
		{
			draw_list->AddNgonFilled(ImVec2{ viewport.x + pixel.x * ratio, viewport.y + (pixel.y + isPano02 * 512) * ratio }, 5.0f * ratio, col_trans, 4);
		}

		for (auto& pixel : floor_pixels)
		{
			draw_list->AddNgonFilled(ImVec2{ viewport.x + pixel.x * ratio, viewport.y + (pixel.y + isPano02 * 512) * ratio }, 5.0f * ratio, col_trans, 4);
		}
	}
	else
	{
		for (auto& pixel : ceil_pixels)
		{
			draw_list->AddCircleFilled(ImVec2{ viewport.x + pixel.x * ratio, viewport.y + (pixel.y + isPano02 * 512) * ratio }, 5.0f * ratio, col_ceil);
		}

		for (auto& pixel : floor_pixels)
		{
			draw_list->AddCircleFilled(ImVec2{ viewport.x + pixel.x * ratio, viewport.y + (pixel.y + isPano02 * 512) * ratio }, 5.0f * ratio, col_floor);
		}
	}
	
	
}

void CornerPoints::AddPoints(const std::vector<glm::vec2>& pixels, const std::vector<glm::vec3>& positions)
{
	// split to ceil and floor corners
	const std::size_t half_size = pixels.size() / 2;
	ceil_pixels.assign(pixels.begin(), pixels.begin() + half_size);
	floor_pixels.assign(pixels.begin() + half_size, pixels.end());
	ceil_positions.assign(positions.begin(), positions.begin() + half_size);
	floor_positions.assign(positions.begin() + half_size, positions.end());
}

void CornerPoints::ClearPoints()
{
	
	if (ceil_pixels.size() == 0) return;
	isLoad = false; // turn off corner pixels
	ceil_pixels.clear();
	floor_pixels.clear();
	ceil_positions.clear();
	floor_positions.clear();
}
