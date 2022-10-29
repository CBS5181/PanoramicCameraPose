#include "Application.h"
#include "PanoLayer.h"
#include "ToolLayer.h"
#include "imgui/imgui.h"
#include <iostream>



PanoLayer::PanoLayer() {}

void PanoLayer::OnAttach()
{
	
}

void PanoLayer::OnUIRender()
{
    // Panorama view
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2{ 0, 0 });
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
    ImGui::Begin("Viewport 01", NULL); // ImGuiWindowFlags_NoScrollbar?

    auto viewportMinRegion = ImGui::GetWindowContentRegionMin();
    auto viewportMaxRegion = ImGui::GetWindowContentRegionMax();
    
    auto viewportOffset = ImGui::GetWindowPos();
    m_ViewportBounds[0] = { viewportMinRegion.x + viewportOffset.x, viewportMinRegion.y + viewportOffset.y };
    m_ViewportBounds[1] = { viewportMaxRegion.x + viewportOffset.x, viewportMaxRegion.y + viewportOffset.y };

    
    auto& left_image = ToolLayer::s_FileManager.GetPano01Texture();
    auto& right_image = ToolLayer::s_FileManager.GetPano02Texture();

    ImVec2 viewportPanelSize = ImGui::GetContentRegionAvail();
    if (ImGui::IsAnyItemActive() == false)
    {
        m_ratio = m_ViewportSize[0] / left_image.width;
        m_ViewportSize = { viewportPanelSize.x, viewportPanelSize.y };
    }

    ImGui::Image((ImTextureID)left_image.texID, ImVec2{ (float)left_image.width * m_ratio, (float)left_image.height * m_ratio }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });
    ImGui::Image((ImTextureID)right_image.texID, ImVec2{ (float)right_image.width * m_ratio, (float)right_image.height * m_ratio }, ImVec2{ 0, 1 }, ImVec2{ 1, 0 });

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    static float sz = 30.0f;
    static ImVec4 colf = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
    const ImU32 col = ImColor(colf);
    // draw mouse picking circle
    draw_list->AddCircle(ImVec2{ m_ViewportBounds[0].x + m_left_mouse_pixel.x , m_ViewportBounds[0].y + m_left_mouse_pixel.y }, sz * 0.5f * m_ratio, col);
    draw_list->AddCircle(ImVec2{ m_ViewportBounds[0].x + m_right_mouse_pixel.x , m_ViewportBounds[0].y + m_right_mouse_pixel.y + 512 * m_ratio }, sz * 0.5f * m_ratio, col);

    // draw already matching points
    for (int i = 0; i < ToolLayer::s_MatchPoints.size(); ++i) 
    {
        //auto& [left_match, right_match] = matching_points[i];
        glm::vec2& left_match = ToolLayer::s_MatchPoints.left_pixels[i];
        glm::vec2& right_match = ToolLayer::s_MatchPoints.right_pixels[i];
        const ImU32 col_rand = ToolLayer::s_MatchPoints.v_color[i];
        draw_list->AddCircleFilled(ImVec2{ m_ViewportBounds[0].x + left_match.x * m_ratio, m_ViewportBounds[0].y + left_match.y * m_ratio }, 5.0f * m_ratio, col_rand);
        draw_list->AddCircleFilled(ImVec2{ m_ViewportBounds[0].x + right_match.x * m_ratio, m_ViewportBounds[0].y + (right_match.y + 512) * m_ratio }, 5.0f * m_ratio, col_rand);
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
    if (mouseX >= 0 && mouseY >= 0 && mouseX < (int)viewportSize.x && mouseY < (int)1024 * m_ratio) 
    {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
        {
            if (mouseY < 512 * m_ratio)
            {
                m_left_mouse_pixel.x = mouseX;
                m_left_mouse_pixel.y = mouseY;
                s_left_pixel.x = glm::min(glm::round(m_left_mouse_pixel.x / m_ratio), 1023.0f);
                s_left_pixel.y = glm::min(glm::round(m_left_mouse_pixel.y / m_ratio), 511.0f);
                
            }
            else
            {
                m_right_mouse_pixel.x = mouseX;
                m_right_mouse_pixel.y = mouseY - 512 * m_ratio;
                s_right_pixel.x = glm::min(glm::round(m_right_mouse_pixel.x / m_ratio), 1023.0f);
                s_right_pixel.y = glm::min(glm::round(m_right_mouse_pixel.y / m_ratio), 511.0f);
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