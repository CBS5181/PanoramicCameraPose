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

//convert spherical coordinate (azimuth,zenith) and 3D position by the standard spherical coordinate formulas
//flip_x: flip x's sign for LED2-Net's coordinate space
static glm::vec2 PosToSpherical(glm::vec3 p, bool flip_x = true)
{
	p = glm::normalize(p);

	//flip x?
	if (flip_x)
	{
		p.x = -p.x;
		
	}
	else
	{
		// HorizonNet's coordinate space
		std::swap(p.x, p.y);
		p.y = -p.y;
	}

	glm::vec2 c;
	c.y = acos(p.z);  //zenith

	//azimuth:
	if (p.x > 0)
	{
		c.x = atan(p.y / p.x);
	}
	else if (p.x < 0 && p.y >= 0)
	{
		c.x = atan(p.y / p.x) + glm::pi<float>();
	}
	else if (p.x < 0 && p.y < 0)
	{
		c.x = atan(p.y / p.x) - glm::pi<float>();
	}
	else if (p.x == 0 && p.y > 0) 
	{
		c.x = glm::pi<float>() / 2;
	}
	else if (p.x == 0 && p.y < 0)
	{
		c.x = -glm::pi<float>() / 2;
	}
	else  //undefined
	{
		std::cout << "[PosToSpherical] azimuth undefined!" << std::endl;
		c.x = 0;
	}

	//azimuth don't be negative
	if (c.x < 0)
		c.x = 2 * glm::pi<float>() + c.x;

	return c;
}

//convert a standard spherical coord to LED2-net's spherical coord
static glm::vec2 SphericalToXY(glm::vec2 s, float width, float height)
{
	glm::vec2 xy;
	xy.x = s.x / (2 * glm::pi<float>()) * width;
	xy.y = s.y / (glm::pi<float>()) * height;

	return xy;
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
	s_FileManager.SetPano01Info("assets/test_data/ZInD/02/pano_orig");
	s_FileManager.SetPano02Info("assets/test_data/ZInD/02/pano_R0_294_T(-0_282,-0_692,0_001)");
	
	//s_FileManager.SetPano01Info("assets/test_data/ZInD/75-100/019/pano_orig");
	//s_FileManager.SetPano02Info("assets/test_data/ZInD/75-100/019/pano_R1_049_T(0_467,0_052,-0_003)");

	//s_FileManager.SetPano01Info("assets/test_data/ZInD/50-75/021/pano_orig");
	//s_FileManager.SetPano02Info("assets/test_data/ZInD/50-75/021/pano_R89_981_T(0_582,0_351,0_003)");

	//s_FileManager.SetPano01Info("assets/test_data/ZInD/50-75/032/pano_orig");
	//s_FileManager.SetPano02Info("assets/test_data/ZInD/50-75/032/pano_R-148_452_T(-0_785,0_42,0_002)");

	//s_FileManager.SetPano01Info("assets/test_data/ZInD/50-75/054/pano_orig");
	//s_FileManager.SetPano02Info("assets/test_data/ZInD/50-75/054/pano_R89_427_T(-0_38,0_596,0_003)");

	//s_FileManager.SetPano01Info("assets/test_data/ZInD/50-75/123/pano_orig");
	//s_FileManager.SetPano02Info("assets/test_data/ZInD/50-75/123/pano_R0_364_T(0_318,-0_701,-0_001)");
}

void ToolLayer::SubdivideMatching(bool circular)
{
	//add intermediate points on edges of the layout?
	//assume the original points are upper ones and then lower ones

	const int subd = 4;  //how many subdivided vertices to add alone an edge?
	int num_corners = s_MatchPoints.left_positions.size() / 2;

	//new set of MatchPoints (to replace old one later)
	MatchPoints new_points;

	//test: additional "extrapolation" points?
	//assume there are 4 points in CCW order toward the viewer
	if (true && s_MatchPoints.left_positions.size() == 4 && s_MatchPoints.right_positions.size() == 4)
	{
		std::vector<glm::vec3> new_left_points;
		std::vector<glm::vec2> new_left_pixels;
		std::vector<glm::vec3> new_right_points;
		std::vector<glm::vec2> new_right_pixels;

		for (int Case = 0; Case < 2; Case++)  //left and right panorama:
		{
			glm::vec3 p0, p1, p2, p3;  //the 4 corners of the plane in CCW order

			if (Case == 0)
			{
				p0 = s_MatchPoints.left_positions[0];
				p1 = s_MatchPoints.left_positions[1];
				p2 = s_MatchPoints.left_positions[2];
				p3 = s_MatchPoints.left_positions[3];
			}
			else
			{
				p0 = s_MatchPoints.right_positions[0];
				p1 = s_MatchPoints.right_positions[1];
				p2 = s_MatchPoints.right_positions[2];
				p3 = s_MatchPoints.right_positions[3];
			}

			//plane normal:
			glm::vec3 n = glm::cross(glm::normalize((p2 - p1)), glm::normalize((p1 - p0)));
			n = glm::normalize(n);
			
			//populate more points on the adjacent ceiling and floor planes:
			const int subd = 6;  //inclusive of two ends
			const float extrude_len = glm::length(p1 - p0) / 5;
			for (int s = 0; s < subd; s++)
			{
				//ceiling: from edge p0->p1, extrude along normal
				{
					glm::vec3 pos = (p0 + (p1 - p0) * (float)s / (float)(subd - 1)) + n * extrude_len;
					glm::vec2 pixel = SphericalToXY(PosToSpherical(pos, false), 1024, 512);
					if (Case == 0)
					{
						new_left_points.push_back(pos);
						new_left_pixels.push_back(pixel);
					}
					else
					{
						new_right_points.push_back(pos);
						new_right_pixels.push_back(pixel);
					}
				}

				//floor: from edge p2->p3, extrude along normal
				{
					glm::vec3 pos = (p2 + (p3 - p2) * (float)s / (float)(subd - 1)) + n * extrude_len;
					glm::vec2 pixel = SphericalToXY(PosToSpherical(pos, false), 1024, 512);
					if (Case == 0)
					{
						new_left_points.push_back(pos);
						new_left_pixels.push_back(pixel);
					}
					else
					{
						new_right_points.push_back(pos);
						new_right_pixels.push_back(pixel);
					}
				}
			}
		}

		for (int ii = 0; ii < new_left_points.size(); ii++)
		{
			const ImU32 col = ImColor(ImVec4(0, 1, 1, 1.0f));

			new_points.AddPoint(new_left_pixels[ii], new_right_pixels[ii], col, 10,
				new_left_points[ii], new_right_points[ii]);
		}

		std::cout << "new_points:" << new_points.size() << std::endl;
	}

	if (!circular)
	{
		//ceiling then floor:
		for (int Case = 0; Case < 2; Case++)
		{
			for (int i = 0; i < num_corners - 1; i++)
			{
				int offset = 0;
				if (Case == 1)
					offset += num_corners;  //the latter half

				//add the original point (on the start):
				new_points.AddPoint(s_MatchPoints.left_pixels[offset + i],
					s_MatchPoints.right_pixels[offset + i],
					s_MatchPoints.v_color[offset + i],
					10,
					s_MatchPoints.left_positions[offset + i],
					s_MatchPoints.right_positions[offset + i]);

				//create and add the new intermediate points:
				glm::vec3 p0 = s_MatchPoints.left_positions[offset + i];
				glm::vec3 p1 = s_MatchPoints.left_positions[offset + i + 1];
				glm::vec3 P0 = s_MatchPoints.right_positions[offset + i];
				glm::vec3 P1 = s_MatchPoints.right_positions[offset + i + 1];
				for (int j = 0; j < subd; j++)
				{
					//left:
					glm::vec3 p = p0 + (p1 - p0) * (float)(j + 1) / (float)(subd + 1);
					//the point's 2d pixel pos?
					glm::vec2 xy = SphericalToXY(PosToSpherical(p, false), 1024, 512); // PosToSpherical sets false for HorizonNet's coordinate system

					//right:
					glm::vec3 P = P0 + (P1 - P0) * (float)(j + 1) / (float)(subd + 1);
					//the point's 2d pixel pos?
					glm::vec2 XY = SphericalToXY(PosToSpherical(P, false), 1024, 512); // PosToSpherical sets false for HorizonNet's coordinate system

					const ImU32 col = ImColor(ImVec4(1, 1, 1, 1.0f));

					//index of this point pair:
					std::pair index(offset + i, j + 1);

					new_points.AddPoint(xy, XY, col, 10, p, P);
				}

				//add the original point (on the end):
				new_points.AddPoint(s_MatchPoints.left_pixels[offset + i + 1],
					s_MatchPoints.right_pixels[offset + i + 1],
					s_MatchPoints.v_color[offset + i + 1],
					10,
					s_MatchPoints.left_positions[offset + i + 1],
					s_MatchPoints.right_positions[offset + i + 1]);


			}
		}
	}
	else  //circular add
	{
		//ceiling then floor:
		for (int Case = 0; Case < 2; Case++)
		{
			for (int i = 0; i < num_corners; i++)
			{
				int offset = 0;
				if (Case == 1)
					offset += num_corners;  //the latter half

				//add the original point (on the begin):
				new_points.AddPoint(s_MatchPoints.left_pixels[offset + i],
					s_MatchPoints.right_pixels[offset + i],
					s_MatchPoints.v_color[offset + i],
					10,
					s_MatchPoints.left_positions[offset + i],
					s_MatchPoints.right_positions[offset + i]);

				//create and add the new intermediate points:
				glm::vec3 p0 = s_MatchPoints.left_positions[offset + i];
				glm::vec3 p1 = s_MatchPoints.left_positions[offset + (i + 1) % num_corners];
				glm::vec3 P0 = s_MatchPoints.right_positions[offset + i];
				glm::vec3 P1 = s_MatchPoints.right_positions[offset + (i + 1) % num_corners];
				for (int j = 0; j < subd; j++)
				{
					//left:
					glm::vec3 p = p0 + (p1 - p0) * (float)(j + 1) / (float)(subd + 1);
					//the point's 2d pixel pos?
					glm::vec2 xy = SphericalToXY(PosToSpherical(p, false), 1024, 512); // PosToSpherical sets false for HorizonNet's coordinate system

					//right:
					glm::vec3 P = P0 + (P1 - P0) * (float)(j + 1) / (float)(subd + 1);
					//the point's 2d pixel pos?
					glm::vec2 XY = SphericalToXY(PosToSpherical(P, false), 1024, 512); // PosToSpherical sets false for HorizonNet's coordinate system

					const ImU32 col = ImColor(ImVec4(1, 1, 1, 1.0f));

					//index of this point pair:
					std::pair index(offset + i, j + 1);

					new_points.AddPoint(xy, XY, col, 10, p, P);
				}
			}
		}
	}

	s_MatchPoints = new_points;
}

void ToolLayer::OnUIRender()
{
	const float TEXT_BASE_HEIGHT = ImGui::GetTextLineHeightWithSpacing();
	ImGui::Begin("Tool");
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	int ind = PanoLayer::s_left_pixel.y * IMG_WIDTH + PanoLayer::s_left_pixel.x;
	if (ind >= 0)
	{
		ImGui::Text("Position : %f %f %f", m_PanoPos_gt[ind].x, m_PanoPos_gt[ind].y, m_PanoPos_gt[ind].z);
		ImGui::Text("Left Pixel X: %f  Y: %f", PanoLayer::s_left_pixel.x, PanoLayer::s_left_pixel.y);
		ImGui::Text("Right Pixel X: %f  Y: %f", PanoLayer::s_right_pixel.x, PanoLayer::s_right_pixel.y);
	}

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
					s_FileManager.SetPano01Info(filepath);
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
				s_FileManager.SetPano01Info(filepath); // set and load texture
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
					s_FileManager.SetPano02Info(filepath);
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
				s_FileManager.SetPano02Info(filepath);
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
		s_MatchPoints.AddPoint(PanoLayer::s_left_pixel, PanoLayer::s_right_pixel, col, 100, PanoLayer::s_left_pos, PanoLayer::s_right_pos);
	}

	if (ImGui::Button("Corner->Match"))
	{
		s_MatchPoints.CovertCornerToMatch(s_FileManager.GetPano01Corners(), s_FileManager.GetPano02Corners());
	}

	static int selectIndex = -1;

	ImGui::SameLine();
	if (ImGui::Button("Delete Match"))
	{
		if (selectIndex >= 0)
		{
			//delete #selectIndex s_MatchPoints record
			s_MatchPoints.DeletePoint(selectIndex);
			selectIndex = -1;
			PanoLayer::s_left_pixel = glm::vec2(-1, -1);
			PanoLayer::s_right_pixel = glm::vec2(-1, -1);
		}
	}

	if (ImGui::Button("Show Transformed corners"))
	{
		s_FileManager.GetTransformCorners().isLoad = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset"))
	{
		s_MatchPoints.ClearPixel();
		s_FileManager.DisableShowCorners();
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

	static int current_corner_type = 1;
	// Load LED2-Net corners
	if (ImGui::Button("Load Corners"))
	{
		const char* corner_filename[] = { "pred_corner_raw.txt", "pred_corner.txt", "pred_corner_peak.txt", "pred_corner_LED2Net.txt", "pred_corner_LGT.txt"};
		std::filesystem::path corner_path01 = s_FileManager.GetPano01Filepath() / corner_filename[current_corner_type];
		std::filesystem::path corner_path02 = s_FileManager.GetPano02Filepath() / corner_filename[current_corner_type];
		std::filesystem::path transCornersPath = s_FileManager.GetPano02Filepath().parent_path() / "trans_corner.txt";
		
		if (corner_path01.empty() || corner_path02.empty())
		{
			std::cout << "Please Upload " << corner_filename[current_corner_type] << std::endl;
		}
		else if (transCornersPath.empty())
		{
			std::cout << "Please Upload trans_corner.txt" << std::endl;
		}
		else
		{
			s_MatchPoints.ClearPixel();
			s_FileManager.SetPano01Corners(corner_path01);
			s_FileManager.SetPano02Corners(corner_path02);
			s_FileManager.SetTransformCorners(transCornersPath);
			s_FileManager.GetPano01Corners().isLoad = true;
			s_FileManager.GetPano02Corners().isLoad = true;
			
			/*====== Following code segments for loading corners as matching points======
	
			std::ifstream file(corner_path01);
			std::ifstream file2(corner_path02);
			std::string str, str2;
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

				const ImU32 col = ImColor(ImVec4((rand() % 256) / 255.0f, (rand() % 256) / 255.0f, (rand() % 256) / 255.0f, 1.0f));
				
				//which layout side, 0-th point on the side  
				std::pair<int, int> index = std::make_pair(s_MatchPoints.size(), 0);

				s_MatchPoints.AddPoint(corner_pixel, corner_pixel2, col, 10, pos, pos2);
			}
			*/
		}
	}
	ImGui::SameLine();
	const char* corner_type[] = { "w/o post", "post", "peak", "LED2Net", "LGT-Net"};
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
	ImGui::Combo("Corner Type", &current_corner_type, corner_type, IM_ARRAYSIZE(corner_type));

	if (ImGui::Button("Rotate")) // Rotate right image corner for correct matching
	{
		s_MatchPoints.RotateRightPixels();
	}
	ImGui::SameLine();
	if (ImGui::Button("Subd"))
	{
		SubdivideMatching(false);
	}
	ImGui::SameLine();
	if (ImGui::Button("SubdCircular"))  //circular subdivide
	{
		SubdivideMatching(true);
	}

	// Save/Load temporary matching points
	{
		if (ImGui::Button("Save Match points")) // Save current matching points
		{
			std::string name = s_FileManager.GetPano01Filepath().parent_path().string() + "/matching.txt";
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
			std::string name = s_FileManager.GetPano01Filepath().parent_path().string() + "/matching.txt";
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
	if (ImGui::Button("Calculate"))
	{
		std::vector<MatchPoints> match_points_all;
		match_points_all.push_back(s_MatchPoints);

		RelativePoseSolver::Solve(left_img.c_str(), right_img.c_str(), match_points_all, method_type);
	}
	ImGui::SameLine();
	if (ImGui::Button("CalculateAll"))
	{
		std::vector<MatchPoints> match_points_all;
		match_points_all.push_back(s_MatchPoints);

		RelativePoseSolver::Solve(left_img.c_str(), right_img.c_str(), match_points_all, method_type);
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
		std::string loftr_file = s_FileManager.GetPano01Filepath().parent_path().string() + "/loftr.txt";
		std::cout << loftr_file << std::endl;
		LoFTRSolver::Solve(left_img.c_str(), right_img.c_str(), loftr_file, s_MatchPoints);
	}
	ImGui::Separator();
	ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
	ImGui::Combo("Solve Method", &method_type, solve_methods, IM_ARRAYSIZE(solve_methods));
}

// static member initialization
MatchPoints ToolLayer::s_MatchPoints;
FileManager ToolLayer::s_FileManager;