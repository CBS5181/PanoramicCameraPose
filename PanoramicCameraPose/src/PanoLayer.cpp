#include "Application.h"
#include "PanoLayer.h"
#include "ToolLayer.h"
#include "stb_image.h"
#include "imgui/imgui.h"
#include <iostream>

bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
    // Load from file
    int image_width = 0;
    int image_height = 0;
    int channels = 0;
    stbi_set_flip_vertically_on_load(true);
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
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;
    if (out_width != nullptr && out_height != nullptr)
    {
        *out_width = image_width;
        *out_height = image_height;
    }
    std::cout << "[Utils::Load Texture From File] Successful!" << std::endl;
    return true;
}

PanoLayer::PanoLayer() {}

void PanoLayer::OnAttach()
{
	LoadTextureFromFile("assets/test_data/pano_orig/color.jpg", &m_left_image, &m_image_width, &m_image_height);
	LoadTextureFromFile("assets/test_data/pano_T(0,0_5,0)/color.jpg", &m_right_image, &m_image_width, &m_image_height);
}

void PanoLayer::OnUIRender()
{
    // Panorama view
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{ 0, 0 });
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
    ImGui::Begin("Viewport 01");

    auto viewportMinRegion = ImGui::GetWindowContentRegionMin();
    auto viewportMaxRegion = ImGui::GetWindowContentRegionMax();
    auto viewportOffset = ImGui::GetWindowPos();
    m_ViewportBounds[0] = { viewportMinRegion.x + viewportOffset.x, viewportMinRegion.y + viewportOffset.y };
    m_ViewportBounds[1] = { viewportMaxRegion.x + viewportOffset.x, viewportMaxRegion.y + viewportOffset.y };

    ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
    m_ViewportSize = { viewportPanelSize.x, viewportPanelSize.y };

    ImGui::Image((ImTextureID)m_left_image, ImVec2{ (float)m_image_width, (float)m_image_height }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });
    ImGui::Image((ImTextureID)m_right_image, ImVec2{ (float)m_image_width, (float)m_image_height }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    static float sz = 30.0f;
    static ImVec4 colf = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
    const ImU32 col = ImColor(colf);
    // draw mouse picking circle
    draw_list->AddCircle(ImVec2{ m_ViewportBounds[0].x + s_left_pixel.x, m_ViewportBounds[0].y + s_left_pixel.y }, sz * 0.5f, col);
    draw_list->AddCircle(ImVec2{ m_ViewportBounds[0].x + s_right_pixel.x, m_ViewportBounds[0].y + s_right_pixel.y + 512 }, sz * 0.5f, col);

    // draw already matching points
    for (int i = 0; i < ToolLayer::s_MatchPoints.size(); ++i)
    {
        //auto& [left_match, right_match] = matching_points[i];
        glm::vec2& left_match = ToolLayer::s_MatchPoints.left_pixels[i];
        glm::vec2& right_match = ToolLayer::s_MatchPoints.right_pixels[i];
        const ImU32 col_rand = ToolLayer::s_MatchPoints.v_color[i];
        draw_list->AddCircleFilled(ImVec2{ m_ViewportBounds[0].x + left_match.x, m_ViewportBounds[0].y + left_match.y }, 5.0f, col_rand);
        draw_list->AddCircleFilled(ImVec2{ m_ViewportBounds[0].x + right_match.x, m_ViewportBounds[0].y + right_match.y + 512 }, 5.0f, col_rand);
    }

    ImGui::End();
    ImGui::PopStyleVar(2);
}

void PanoLayer::OnUpdate()
{
    auto [mx, my] = ImGui::GetMousePos();
    mx -= m_ViewportBounds[0].x;
    my -= m_ViewportBounds[0].y;
    glm::vec2 viewportSize = m_ViewportBounds[1] - m_ViewportBounds[0];
    //my = viewportSize.y - my; // 改為左下(0, 0) y往上為正
    //std::cout << "x: " << mx << " y: " << my << std::endl;
    int mouseX = (int)mx;
    int mouseY = (int)my;
    // mouse picking
    if (mouseX >= 0 && mouseY >= 0 && mouseX < (int)viewportSize.x && mouseY < (int)viewportSize.y) 
    {
        if (isMouseButtonLeftClick)
        {
            if (mouseY < 512)
            {
                s_left_pixel.x = mx;
                s_left_pixel.y = my;
            }
            else
            {
                s_right_pixel.x = mx;
                s_right_pixel.y = my - 512;
            }
        }
    }
}

Application* CreateApplication(int argc, char** argv)
{
    ApplicationSpecification spec;

    Application* app = new Application(spec);
    app->PushLayer<PanoLayer>();
    app->PushLayer<ToolLayer>();
    return app;
}

glm::vec2 PanoLayer::s_left_pixel(0.0f, 0.0f);
glm::vec2 PanoLayer::s_right_pixel(0.0f, 0.0f);
bool PanoLayer::isMouseButtonLeftClick = false;