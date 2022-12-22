#pragma once
#include <string>
#include <filesystem>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "CornerPoints.h"

struct Texture
{
	GLuint texID;
	int width = 1024, height = 512;
};

class FileManager
{
public:
	void SetPano01Info(const std::filesystem::path& filepath) 
	{ 
		m_pano01_path = filepath; 
		LoadTextureFromFile(m_pano01_path, &m_pano01_tex.texID, &m_pano01_tex.width, &m_pano01_tex.height);
		LoadCornersFromFile(m_pano01_path / "pred_corner.txt", true);
	}
	void SetPano02Info(const std::filesystem::path& filepath)
	{ 
		m_pano02_path = filepath; 
		LoadTextureFromFile(m_pano02_path, &m_pano02_tex.texID, &m_pano02_tex.width, &m_pano02_tex.height);
		LoadCornersFromFile(m_pano02_path / "pred_corner.txt", false);
	}
	const std::filesystem::path& GetPano01Filepath() const { return m_pano01_path; }
	const std::filesystem::path& GetPano02Filepath() const { return m_pano02_path; }

	
	CornerPoints& GetPano01Corners() { return m_pano01_corners; }
	CornerPoints& GetPano02Corners() { return m_pano02_corners; }

	const Texture& GetPano01Texture() const { return m_pano01_tex; }
	const Texture& GetPano02Texture() const { return m_pano02_tex; }
	static bool LoadTextureFromFile(const std::filesystem::path& filepath, GLuint* out_texture, int* out_width, int* out_height);
	
private:
	bool LoadCornersFromFile(const std::filesystem::path& filepath, bool isPano01);

private:
	std::filesystem::path m_pano01_path;
	std::filesystem::path m_pano02_path;
	Texture m_pano01_tex;
	Texture m_pano02_tex;
	CornerPoints m_pano01_corners;
	CornerPoints m_pano02_corners;
};

