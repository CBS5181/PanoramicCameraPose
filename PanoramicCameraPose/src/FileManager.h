#pragma once
#include <string>
#include <filesystem>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

struct Texture
{
	GLuint texID;
	int width = 1024, height = 512;
};

class FileManager
{
public:
	void SetPano01Filepath(const std::filesystem::path& filepath) { m_pano01_path = filepath; LoadTextureFromFile(m_pano01_path, &m_pano01_tex.texID, &m_pano01_tex.width, &m_pano01_tex.height); }
	void SetPano02Filepath(const std::filesystem::path& filepath) { m_pano02_path = filepath; LoadTextureFromFile(m_pano02_path, &m_pano02_tex.texID, &m_pano02_tex.width, &m_pano02_tex.height); }
	const std::filesystem::path& GetPano01Filepath() const { return m_pano01_path; }
	const std::filesystem::path& GetPano02Filepath() const { return m_pano02_path; }

	const Texture& GetPano01Texture() const { return m_pano01_tex; }
	const Texture& GetPano02Texture() const { return m_pano02_tex; }
	static bool LoadTextureFromFile(const std::filesystem::path& filepath, GLuint* out_texture, int* out_width, int* out_height);

private:
	std::filesystem::path m_pano01_path;
	std::filesystem::path m_pano02_path;
	Texture m_pano01_tex;
	Texture m_pano02_tex;
};

