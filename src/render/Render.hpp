#ifndef RENDER_RENDER_HPP_
#define RENDER_RENDER_HPP_

#include <imgui.h>           // ImDrawList
#include <aidrive/Types.hpp> // Vector

namespace aidrive
{
namespace render
{

/**
 * some useful consts
 */

/**
 * COLORS
 */
const ImU32 COLOR_BLACK = ImColor(0.0f, 0.0f, 0.0f, 1.0f);
const ImU32 COLOR_WHITE = ImColor(1.0f, 1.0f, 1.0f, 1.0f);
const ImU32 COLOR_RED   = ImColor(1.0f, 0.0f, 0.0f, 1.0f);
const ImU32 COLOR_GREEN = ImColor(0.0f, 1.0f, 0.0f, 1.0f);
const ImU32 COLOR_BLUE  = ImColor(0.0f, 0.0f, 1.0f, 1.0f);

/**
 * \@brief draw_list is owned by ImGui
 * ImDrawList* draw_list = ImGui::GetWindowDrawList();
 * \@note argument ImGui built in custom rendering features
 */
void drawRect(ImDrawList* drawList,
              const std::vector<ImVec2>& vertices,
              ImU32 color,
              float32_t thickness);

/**
 * Renderer handles view and projection
 */
class Renderer
{
public:
    Renderer() = default;

    void setImDrawList(ImDrawList* list);

    void setEye(const Vector2f& eye);
    void setPixelPerMeter(float32_t pm);

    void drawRect(const Vector3f& pose, // 2d pose {x,y,hdg}
                  const Rect2f& dim);   // rect dimension)

private:
    Matrix3f m_view{Matrix3f::Identity()};
    Matrix3f m_projection{Matrix3f::Identity()};

    ImDrawList* m_list{nullptr};

}; // class Renderer

} // namespace render
} // namespace aidrive

#endif // RENDER_RENDER_HPP