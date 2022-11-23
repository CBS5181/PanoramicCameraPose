#include "pch.h"
#include "ToolLayer.h"
#include "imgui/imgui.h"
#include "PanoLayer.h"
#include <ctime>
#include "Solvers/Solver.h"
#include "Utils/WindowsPlatformUtils.h"
#include <glm/gtx/string_cast.hpp>

const unsigned int IMG_WIDTH = 1024;
const unsigned int IMG_HEIGHT = 512;

extern float deltaTime;

namespace fs = std::filesystem;

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

//convert spherical coordinate (azimuth,zenith) and 3D position, using LED2-net' assumption (NOT the standard spherical coordinate formula!)
static glm::vec2 PosToSpherical(glm::vec3 p)
{
	p = glm::normalize(p);

	glm::vec2 c;
	c.y = acos(-p.y);  //zenith
	//azimuth:
	if (p.x > 0)
	{
		c.x = atan2(p.z, p.x);
	}
	else if (p.x < 0 && p.z >=0)
	{
		c.x = atan2(p.z, p.x) + glm::pi<float>();
	}
	else if (p.x < 0 && p.z < 0)
	{
		c.x = atan2(p.z, p.x) - glm::pi<float>();
	}
	else if (p.x == 0 && p.z > 0)
	{
		c.x = glm::pi<float>() / 2;
	}
	else if (p.x == 0 && p.z < 0)
	{
		c.x = -glm::pi<float>() / 2;
	}
	else  //undefined
	{
		std::cout << "[PosToSpherical] azimuth undefined!" << std::endl;
		c.x = 0;
	}

	return c;
}

ToolLayer::ToolLayer() : m_PanoPos_gt(IMG_WIDTH * IMG_HEIGHT)
{
	srand(time(0));

	// read pos txt
	std::ifstream file("assets/ground_truth/pano_ori_pos_gt.txt");
	std::string str;
	unsigned int ind = 0;
	while (std::getline(file, str))
	{
		std::istringstream iss(str);
		glm::vec3 pos;
		iss >> pos.x >> pos.y >> pos.z;
		m_PanoPos_gt[ind] = pos;
		ind++;
	}

	// set default filepath for test quickly
	/*s_FileManager.SetPano01Filepath("assets/test_data/pano_orig");
	s_FileManager.SetPano02Filepath("assets/test_data/pano_R90_T(0,0_5,0)");*/
	s_FileManager.SetPano01Filepath("assets/test_data/ZInD/01/pano_orig");
	s_FileManager.SetPano02Filepath("assets/test_data/ZInD/01/pano_R38_506_T(-0_692,0_28,0)");
}

void ToolLayer::OnUIRender()
{
	const float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();
	ImGui::Begin("Tool");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	unsigned int ind = PanoLayer::s_left_pixel.y * IMG_WIDTH + PanoLayer::s_left_pixel.x;
	ImGui::Text("Position : %f %f %f", m_PanoPos_gt[ind].x, m_PanoPos_gt[ind].y, m_PanoPos_gt[ind].z);
	ImGui::Text("Left Pixel X: %f  Y: %f", PanoLayer::s_left_pixel.x, PanoLayer::s_left_pixel.y);
	ImGui::Text("Right Pixel X: %f  Y: %f", PanoLayer::s_right_pixel.x, PanoLayer::s_right_pixel.y);

	// File Open
	{
		char buffer[2][256];
		memset(buffer, 0, sizeof(buffer));
		std::strncpy(buffer[0], s_FileManager.GetPano01Filepath().filename().string().c_str(), sizeof(buffer[0]));
		std::strncpy(buffer[1], s_FileManager.GetPano02Filepath().filename().string().c_str(), sizeof(buffer[1]));
		
		// Pano 01
		ImGui::InputTextWithHint("Pano01", "open / drag and drop folder here...", buffer[0], sizeof(buffer[0]), ImGuiInputTextFlags_ReadOnly);
		if (ImGui::BeginDragDropTarget())
		{
			if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("FILES"))
			{
				const wchar_t* filepath = (const wchar_t*)payload->Data;
				if (fs::is_directory(filepath)) // only accept folder
				{
					s_FileManager.SetPano01Filepath(filepath);
					s_MatchPoints.ClearPixel();
				}
			}
			ImGui::EndDragDropTarget();
		}
		ImGui::SameLine();
		if (ImGui::Button("Open..."))
		{
			std::wstring filepath = FileDialogs::OpenFolder();
			if (!filepath.empty())
			{
				s_FileManager.SetPano01Filepath(filepath); // set and load texture
				s_MatchPoints.ClearPixel();
			}
		}

		// Pano 02
		ImGui::InputTextWithHint("Pano02", "open / drag and drop folder here...", buffer[1], sizeof(buffer[1]), ImGuiInputTextFlags_ReadOnly);
		if (ImGui::BeginDragDropTarget())
		{
			if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("FILES"))
			{
				const wchar_t* filepath = (const wchar_t*)payload->Data;
				if (fs::is_directory(filepath))
				{
					s_FileManager.SetPano02Filepath(filepath);
					s_MatchPoints.ClearPixel();
				}
			}
			ImGui::EndDragDropTarget();
		}
		ImGui::SameLine();
		ImGui::PushID(1); // avoid imgui same name issue
		if (ImGui::Button("Open..."))
		{
			//std::string filepath = FileDialogs::OpenFile("JPG (*.jpg;*.jpeg;*.jpe;*.jfif)\0*.jpg;*.jpeg;*.jpe;*.jfif\0");
			std::wstring filepath = FileDialogs::OpenFolder();
			if (!filepath.empty())
			{
				s_FileManager.SetPano02Filepath(filepath);
				s_MatchPoints.ClearPixel();
			}
		}
		ImGui::PopID();
	}
	ImGui::Separator();
	if (ImGui::Button("Match"))  //add a user-specified matching
	{
		unsigned int ind = PanoLayer::s_left_pixel.y * IMG_WIDTH + PanoLayer::s_left_pixel.x;
		const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
		/* Now Matching point by human doesn't have position information */
		s_MatchPoints.AddPoint(PanoLayer::s_left_pixel, PanoLayer::s_right_pixel, col, 100, glm::vec3{0.0f}, glm::vec3{0.0f}, true/*is user-specified*/); // user pick weight = 100
	}

	// match table
	{
		static ImGuiTableFlags flags = ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersOuter | ImGuiTableFlags_BordersV | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;

		PushStyleCompact();
		ImGui::CheckboxFlags("ImGuiTableFlags_ScrollY", &flags, ImGuiTableFlags_ScrollY);
		PopStyleCompact();

		// When using ScrollX or ScrollY we need to specify a size for our table container!
		// Otherwise by default the table will fit all available space, like a BeginChild() call.
		ImVec2 outer_size = ImVec2(0.0f, TEXT_BASE_HEIGHT * 12);
		static int selectIndex = -1;
		if (ImGui::BeginTable("Matching points", 4, flags, outer_size))
		{
			ImGui::TableSetupScrollFreeze(0, 1); // Make top row always visible
			ImGui::TableSetupColumn("Match #", ImGuiTableColumnFlags_None);
			ImGui::TableSetupColumn("Pano_01", ImGuiTableColumnFlags_None);
			ImGui::TableSetupColumn("Pano_02", ImGuiTableColumnFlags_None);
			//ImGui::TableSetupColumn("Position_groundtruth", ImGuiTableColumnFlags_None);
			ImGui::TableHeadersRow();

			for (int row = 0; row < s_MatchPoints.size(); ++row)
			{
				ImGui::TableNextRow();
				auto& L = s_MatchPoints.left_pixels[row];
				auto& R = s_MatchPoints.right_pixels[row];
				auto& p = s_MatchPoints.left_positions[row];
				const bool matchIsSelectd = selectIndex == row;
				for (int column = 0; column < 4; column++)
				{
					ImGui::TableSetColumnIndex(column);
					switch (column)
					{
					case 0:
					{
						// Selectable for inspecting matching point
						char label[32];
						sprintf(label, "%d", row);
						ImGuiSelectableFlags selectable_flags = ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowItemOverlap;
						if (ImGui::Selectable(label, matchIsSelectd, selectable_flags, ImVec2(0, TEXT_BASE_HEIGHT)))
						{
							PanoLayer::s_left_pixel = L;
							PanoLayer::s_right_pixel = R;
							selectIndex = (selectIndex == row ? -1 : row);
						}
						//ImGui::Text("Match %d", row);
						break;
					}
					case 1:
						ImGui::Text("(%.2f, %.2f)", L.x, L.y);
						break;
					case 2:
						ImGui::Text("(%.2f, %.2f)", R.x, R.y);
						break;
					case 3:
						ImGui::Text("(%.4f, %.4f, %.4f)", p.x, p.y, p.z);
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
	if (ImGui::Button("Load Corners"))
	{
		std::filesystem::path corner_path01 = s_FileManager.GetPano01Filepath() / "pred_corner.txt";
		std::filesystem::path corner_path02 = s_FileManager.GetPano02Filepath() / "pred_corner.txt";
		if (corner_path01.empty() || corner_path02.empty())
		{
			std::cout << "Please Upload two panoramic images!\n";
		}
		else
		{
			s_MatchPoints.ClearPixel();
			std::ifstream file(corner_path01);
			std::ifstream file2(corner_path02);

			std::string str, str2;
			unsigned int cnt = 0;
			while (std::getline(file, str) && std::getline(file2, str2))
			{
				// read pano01 corner pixels
				std::istringstream iss(str);
				glm::vec2 corner_pixel;
				glm::vec3 pos;
				iss >> corner_pixel.x >> corner_pixel.y >> pos.x >> pos.y >> pos.z;

				// read pano02 corner pixels
				std::istringstream iss2(str2);
				glm::vec2 corner_pixel2;
				glm::vec3 pos2;
				iss2 >> corner_pixel2.x >> corner_pixel2.y >> pos2.x >> pos2.y >> pos2.z;

				// depth ground trugh from 3D scene
				unsigned int ind = corner_pixel.y * IMG_WIDTH + corner_pixel.x;
				const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
				//s_MatchPoints.AddPoint(corner_pixel, corner_pixel2, col, 10, m_PanoPos_gt[ind], false/*not user-specified*/); // LED2Net weight = 10
				s_MatchPoints.AddPoint(corner_pixel, corner_pixel2, col, 10, pos, pos2, false/*not user-specified*/);
				++cnt;
			}
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Rotate Matching")) // Rotate right image corner for correct matching
	{
		s_MatchPoints.RotateRightPixels();
	}

	// Save/Load temporary matching points
	{
		if (ImGui::Button("Save Match points")) // Save current matching points
		{
			std::string name = "assets/matching_data/" + s_FileManager.GetPano01Filepath().filename().string() + "_and_" + s_FileManager.GetPano02Filepath().filename().string() + ".txt";
			std::ofstream file;
			file.open(name);
			if (file.is_open())
			{
				auto& left = s_MatchPoints.left_pixels;
				auto& right = s_MatchPoints.right_pixels;
				auto& weights = s_MatchPoints.weights;
				auto& left_positions = s_MatchPoints.left_positions;
				auto& right_positions = s_MatchPoints.right_positions;
				for (int i = 0; i < s_MatchPoints.size(); ++i)
				{
					file << left[i].x << " " << left[i].y << " " << right[i].x << " " << right[i].y << " " << weights[i] << " " << left_positions[i].x << " " << left_positions[i].y << " " << left_positions[i].z << " " << right_positions[i].x << " " << right_positions[i].y << " " << right_positions[i].z << "\n";
				}
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Load Match points")) // Load matching points from txt file
		{
			s_MatchPoints.ClearPixel(); //  clear before load
			std::string name = "assets/matching_data/" + s_FileManager.GetPano01Filepath().filename().string() + "_and_" + s_FileManager.GetPano02Filepath().filename().string() + ".txt";
			std::ifstream file;
			file.open(name);
			if (file.is_open())
			{
				std::string str;
				glm::vec2 left, right;
				uint32_t weight;
				glm::vec3 left_pos, right_pos;
				while (std::getline(file, str))
				{
					std::istringstream iss(str);
					iss >> left.x >> left.y >> right.x >> right.y >> weight >> left_pos.x >> left_pos.y >> left_pos.z >> right_pos.x >> right_pos.y >> right_pos.z;
					const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
					s_MatchPoints.AddPoint(left, right, col, weight, left_pos, right_pos);
				}
			}
			else
			{
				std::cout << "There is no matching file " << std::quoted(name) << std::endl;
			}
		}
	}
	ImGui::Separator();

	std::string left_img = s_FileManager.GetPano01Filepath().string() + "/color.jpg";
	std::string right_img = s_FileManager.GetPano02Filepath().string() + "/color.jpg";
	static int method_type = 0; //0: 8-point algorithm, 1:Gurobi

	const char* solve_methods[] = { "8-point", "Gurobi" };
	if (ImGui::Button("Calculate Relative Pose"))
	{
		RelativePoseSolver::Solve(left_img.c_str(), right_img.c_str(), s_MatchPoints, method_type);
	}
	ImGui::SameLine();
	if (ImGui::Button("SIFT")) // SIFT Solver
	{
		s_MatchPoints.ClearPixel();
		SIFTSolver::Solve(left_img.c_str(), right_img.c_str(), m_PanoPos_gt, s_MatchPoints);
	}
	ImGui::SameLine();
	if (ImGui::Button("LoFTR")) // LoFTR Solver
	{
		s_MatchPoints.ClearPixel();
		LoFTRSolver::Solve(left_img.c_str(), right_img.c_str(), s_MatchPoints);
	}
	ImGui::Separator();
	ImGui::Combo("Solve Method", &method_type, solve_methods, IM_ARRAYSIZE(solve_methods));
}

// static member initialization
MatchPoints ToolLayer::s_MatchPoints;
FileManager ToolLayer::s_FileManager;