#include "ToolLayer.h"
#include "imgui/imgui.h"
#include "PanoLayer.h"
#include <utility>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "Solver.h"

const unsigned int IMG_WIDTH = 1024;
const unsigned int IMG_HEIGHT = 512;

// Make the UI compact because there are so many fields
static void PushStyleCompact()
{
	ImGuiStyle& style = ImGui::GetStyle();
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(style.FramePadding.x, (float)(int)(style.FramePadding.y * 0.60f)));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(style.ItemSpacing.x, (float)(int)(style.ItemSpacing.y * 0.60f)));
}

static void PopStyleCompact()
{
	ImGui::PopStyleVar(2);
}

ToolLayer::ToolLayer() : m_pano01_depth(IMG_WIDTH * IMG_HEIGHT)
{
	srand(time(0));

	// read depth txt
	std::ifstream file("assets/txt/pano01_pos_depth.txt");
	std::string str;
	unsigned int ind = 0;
	while (std::getline(file, str))
	{
		std::istringstream iss(str);
		float depth_value;
		iss >> depth_value;
		m_pano01_depth[ind] = depth_value;
		ind++;
	}
}

void ToolLayer::OnUIRender()
{
	const float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();
	ImGui::Begin("Tool");

	unsigned int ind = PanoLayer::s_left_pixel.y * IMG_WIDTH + PanoLayer::s_left_pixel.x;
	ImGui::Text("Depth : %f", m_pano01_depth[ind]);
	ImGui::Text("Left Pixel X: %f  Y: %f", PanoLayer::s_left_pixel.x, PanoLayer::s_left_pixel.y);
	ImGui::Text("Right Pixel X: %f  Y: %f", PanoLayer::s_right_pixel.x, PanoLayer::s_right_pixel.y);

	
	if (ImGui::Button("Match"))
	{
		unsigned int ind = PanoLayer::s_left_pixel.y * IMG_WIDTH + PanoLayer::s_left_pixel.x;
		m_match_depth.push_back(m_pano01_depth[ind]);
		const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
		s_MatchPoints.AddPoint(PanoLayer::s_left_pixel, PanoLayer::s_right_pixel, col, 100); // user pick weight = 100
	}

	// match table
	{
		static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;

		PushStyleCompact();
		ImGui::CheckboxFlags("ImGuiTableFlags_ScrollY", &flags, ImGuiTableFlags_ScrollY);
		PopStyleCompact();

		// When using ScrollX or ScrollY we need to specify a size for our table container!
		// Otherwise by default the table will fit all available space, like a BeginChild() call.
		ImVec2 outer_size = ImVec2(0.0f, TEXT_BASE_HEIGHT * 9);
		if (ImGui::BeginTable("Matching points", 5, flags, outer_size))
		{
			ImGui::TableSetupScrollFreeze(0, 1); // Make top row always visible
			ImGui::TableSetupColumn("Match #", ImGuiTableColumnFlags_None);
			ImGui::TableSetupColumn("pano_01", ImGuiTableColumnFlags_None);
			ImGui::TableSetupColumn("pano_02", ImGuiTableColumnFlags_None);
			ImGui::TableSetupColumn("depth", ImGuiTableColumnFlags_None);
			ImGui::TableSetupColumn("weight", ImGuiTableColumnFlags_None);
			ImGui::TableHeadersRow();

			for (int row = 0; row < s_MatchPoints.size(); ++row)
			{
				ImGui::TableNextRow();
				for (int column = 0; column < 5; column++)
				{
					ImGui::TableSetColumnIndex(column);

					switch (column)
					{
					case 0:
						ImGui::Text("Match %d", row);
						break;
					case 1:
						ImGui::Text("(%d, %d)", (int)s_MatchPoints.left_pixels[row].x, (int)s_MatchPoints.left_pixels[row].y);
						break;
					case 2:
						ImGui::Text("(%d, %d)", (int)s_MatchPoints.right_pixels[row].x, (int)s_MatchPoints.right_pixels[row].y);
						break;
					case 3:
						ImGui::Text("%f", m_match_depth[row]);
						break;
					case 4:
						ImGui::Text("%u", s_MatchPoints.weights[row]);
						break;
					default:
						ImGui::Text("Hello %d,%d", column, row);
						break;
					}
				}
			}

			ImGui::EndTable();
		}
	}

	// Load LED2-Net corners
	if (ImGui::Button("Load Corner"))
	{
		std::ifstream file("assets/txt/pano01_pred_corner_XY.txt");
		std::ifstream file2("assets/txt/pano03_pred_corner_XY.txt");
		//std::ifstream file2("pano_neg_PI_pred_corner_XY.txt");
		std::string str, str2;
		unsigned int cnt = 0;
		while (std::getline(file, str) && std::getline(file2, str2))
		{
			// read pano01 corner pixels
			std::istringstream iss(str);
			glm::vec2 corner_pixel;
			iss >> corner_pixel.x >> corner_pixel.y;

			// read pano02 corner pixels
			std::istringstream iss2(str2);
			glm::vec2 corner_pixel2;
			iss2 >> corner_pixel2.x >> corner_pixel2.y;

			// depth ground trugh from 3D scene
			unsigned int ind = corner_pixel.y * IMG_WIDTH + corner_pixel.x;
			std::cout << ind << std::endl;
			m_match_depth.push_back(m_pano01_depth[ind]);
			const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
			s_MatchPoints.AddPoint(corner_pixel, corner_pixel2, col, 10); // LED2Net weight = 10
			++cnt;
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Rotate Matching")) // Rotate right image corner for correct matching
	{
		s_MatchPoints.RotateRightPixels();
	}

	if (ImGui::Button("Save Match points")) // Save current matching points
	{
		std::ofstream file;
		file.open("assets/txt/matching.txt");
		auto& left = s_MatchPoints.left_pixels;
		auto& right = s_MatchPoints.right_pixels;
		auto& weights = s_MatchPoints.weights;
		for (int i = 0; i < s_MatchPoints.size(); ++i)
		{
			file << left[i].x << " " << left[i].y << " " << right[i].x << " " << right[i].y << " " << m_match_depth[i] << " " << weights[i] << "\n";
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Load Match points")) // Load matching points from txt file
	{
		std::ifstream file;
		file.open("assets/txt/matching.txt");
		std::string str;
		glm::vec2 left, right;
		float depth;
		uint32_t weight;
		while (std::getline(file, str))
		{
			std::istringstream iss(str);
			iss >> left.x >> left.y >> right.x >> right.y >> depth >> weight;
			m_match_depth.push_back(depth);
			const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
			s_MatchPoints.AddPoint(left, right, col, weight);
		}
		std::cout << "Read match points : " << s_MatchPoints.size() << std::endl;
	}

	if (ImGui::Button("Calculate Relative Pose") && s_MatchPoints.size() >= 8) // Solver
	{
		RelativePoseSolver::Solve("assets/img/pano01.jpg", "assets/img/pano03.jpg", s_MatchPoints.left_pixels, s_MatchPoints.right_pixels, m_match_depth, s_MatchPoints.weights);
	}

}

MatchPoints ToolLayer::s_MatchPoints;