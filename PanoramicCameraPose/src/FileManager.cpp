#include "pch.h"
#include "FileManager.h"
#include "stb_image.h"


bool FileManager::LoadTextureFromFile(const std::filesystem::path& filepath, GLuint* out_texture, int* out_width, int* out_height)
{
	std::filesystem::path p = filepath;
	if (std::filesystem::is_directory(p)) // bug fixed! aim.png cannot rendered without this if condition!
	{
		p = p / "color.jpg";
	}
	auto s = p.string();
	const char* filename = s.c_str();
	// Load from file
	int image_width = 0;
	int image_height = 0;
	int channels = 0;
	//stbi_set_flip_vertically_on_load(true);
	unsigned char* image_data = stbi_load(filename, &image_width, &image_height, &channels, 0);
	if (image_data == NULL)
		return false;

	// Create a OpenGL texture identifier
	GLuint image_texture;
	glGenTextures(1, &image_texture);
	glBindTexture(GL_TEXTURE_2D, image_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

	// Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
	if (channels == 3)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);

	stbi_image_free(image_data);

	*out_texture = image_texture;
	if (out_width != nullptr && out_height != nullptr)
	{
		*out_width = image_width;
		*out_height = image_height;
	}
	
	return true;
}

bool FileManager::LoadCornersFromFile(const std::filesystem::path& filepath, bool isPano01)
{
	auto& corners = (isPano01) ? m_pano01_corners : m_pano02_corners;
	std::ifstream file(filepath);
	std::string str;
	std::vector<glm::vec2> corner_pixels;
	std::vector<glm::vec3> positions;
	if (!file.is_open())
	{
		std::cout << filepath << "\nPredicted corner points from LED2-Net not found!\n";
		return false;
	}

	while (std::getline(file, str))
	{
		// read pano01 corner pixels
		std::istringstream iss(str);
		glm::vec2 corner_pixel;
		glm::vec3 pos;
		iss >> corner_pixel.x >> corner_pixel.y >> pos.x >> pos.y >> pos.z;

		corner_pixels.push_back(corner_pixel);
		positions.push_back(pos);
		
	}
	corners.ClearPoints();
	corners.AddPoints(corner_pixels, positions);
	
	return true;
}
