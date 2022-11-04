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
	glm::vec2 m_ViewportSize = { 1024.0f, 512.0f };
	glm::vec2 m_ViewportBounds[2] = {};
	glm::vec2 m_left_mouse_pixel = { 0.0f, 0.0f }, m_right_mouse_pixel = { 0.0f, 0.0f };
	float m_ratio = 1.0f;
};