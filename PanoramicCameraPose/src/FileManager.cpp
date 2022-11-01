#include "FileManager.h"
#include "stb_image.h"
#include <iostream>

bool FileManager::LoadTextureFromFile(const std::filesystem::path& filepath, GLuint* out_texture, int* out_width, int* out_height)
{
	std::filesystem::path p = filepath;
	if (std::filesystem::is_directory(p))
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