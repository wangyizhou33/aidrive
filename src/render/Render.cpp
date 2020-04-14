#include "Render.hpp"
#include <vector>

namespace aidrive
{
namespace render
{

ImVec2 convert(const Vector2f& in)
{
    return {in[0], in[1]};
}

void drawRect(ImDrawList* drawList,
              const Vector3f& pose,
              const Rect2f& dim)
{
    if (!drawList)
    {
        return;
    }

    Vector2f pos{pose[0], pose[1]};
    float32_t hdg = pose[2];

    Matrix2f rot{};

    rot << std::cos(hdg), std::sin(hdg),
        -std::sin(hdg), std::cos(hdg);

    std::vector<Vector2f> vertices{};
    vertices.push_back(pos + rot * Vector2f{dim.length / 2.f, dim.width / 2.f});
    vertices.push_back(pos + rot * Vector2f{dim.length / 2.f, -dim.width / 2.f});
    vertices.push_back(pos + rot * Vector2f{-dim.length / 2.f, -dim.width / 2.f});
    vertices.push_back(pos + rot * Vector2f{-dim.length / 2.f, dim.width / 2.f});

    std::vector<LineSeg2f> edges{};
    edges.push_back({vertices[0], vertices[1]});
    edges.push_back({vertices[1], vertices[2]});
    edges.push_back({vertices[2], vertices[3]});
    edges.push_back({vertices[3], vertices[0]});

    // now draw
    // TODO: pass via param[in]
    constexpr float32_t thickness = 3.0f;
    const ImU32 color             = ImColor(0.0f, 0.0f, 0.0f, 1.0f);

    for (const LineSeg2f& edge : edges)
    {
        drawList->AddLine(convert(edge.v0), convert(edge.v1), color, thickness);
    }
}

} // namespace render
} // namespace aidrive
