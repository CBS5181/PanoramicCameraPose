#pragma once
#include "Layer.h"
#include "glm/glm.hpp"
#include "glad/glad.h"

class PanoLayer : public Layer
{
public:
	PanoLayer();
	virtual void OnAttach() override;
	virtual void OnUIRender() override;
	virtual void OnUpdate() override;

	static glm::vec2 s_left_pixel, s_right_pixel; // spherical coordinate
	static bool isMouseButtonLeftClick;

private:

	int m_image_width = 0;
	int m_image_height = 0;
	GLuint m_left_image = 0, m_right_image = 0;
	glm::vec2 m_ViewportSize = { 0.0f, 0.0f };
	glm::vec2 m_ViewportBounds[2];
};