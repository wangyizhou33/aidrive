#ifndef RENDER_RENDER_HPP_
#define RENDER_RENDER_HPP_

#include <imgui.h>           // ImDrawList
#include <aidrive/Types.hpp> // Vector

namespace aidrive
{
namespace render
{

/**
 * draw_list is owned by ImGui
 * ImDrawList* draw_list = ImGui::GetWindowDrawList();
 */

void drawRect(ImDrawList* drawList,
              const Vector3f& pose, // 2d pose {x,y,hdg}
              const Rect2f& dim);   // rect dimension

} // namespace render
} // namespace aidrive

#endif // RENDER_RENDER_HPP