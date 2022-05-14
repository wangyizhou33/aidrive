#ifndef COORD_2D_HPP_
#define COORD_2D_HPP_

#include <aidrive/Types.hpp>

using Vector2f = aidrive::Vector2f;

struct Coord2d
{

    Coord2d()
        : Coord2d(0, 0)
    {
    }

    Coord2d(int32_t x_, int32_t y_)
        : x(x_)
        , y(y_)
    {
    }

    Coord2d(const Vector2f& pos, float32_t cellSize)
    {
        x = static_cast<int32_t>(floor((pos.x() + 0.5f * cellSize) / cellSize));
        y = static_cast<int32_t>(floor((pos.y() + 0.5f * cellSize) / cellSize));
    }

    static Coord2d compute(float32_t cellSize, const Vector2f& pos)
    {
        Coord2d index;

        // position indices
        float32_t halfCell = 0.5f * cellSize;
        index.x            = static_cast<int32_t>(floor((pos.x() + halfCell) / cellSize));
        index.y            = static_cast<int32_t>(floor((pos.y() + halfCell) / cellSize));

        return index;
    }

    inline Vector2f getCenter(float32_t cellSize) const
    {
        return Vector2f(static_cast<float32_t>(x) * cellSize, static_cast<float32_t>(y) * cellSize);
    }

    inline Vector2f getMin(float32_t cellSize) const
    {
        return Vector2f(static_cast<float32_t>(x) * cellSize - 0.5f * cellSize, static_cast<float32_t>(y) * cellSize - 0.5f * cellSize);
    }

    inline Vector2f getMax(float32_t cellSize) const
    {
        return Vector2f(static_cast<float32_t>(x) * cellSize + 0.5f * cellSize, static_cast<float32_t>(y) * cellSize + 0.5f * cellSize);
    }

    int32_t x;
    int32_t y;

    bool operator==(const Coord2d& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Coord2d& other) const
    {
        return !(*this == other);
    }

    Coord2d operator+(const Coord2d& other) const
    {
        return {x + other.x, y + other.y};
    }

    Coord2d operator-(const Coord2d& other) const
    {
        return {x - other.x, y - other.y};
    }

    int32_t operator<(const Coord2d& index) const
    {
        return x < index.x || (x == index.x && y < index.y);
    }

    int32_t operator()(const Coord2d& index) const
    {
        return 42061 * index.x + 11969 * index.y;
    }
};

class Coord2dHash
{
public:
    uint32_t operator()(const Coord2d& index) const
    {
        return static_cast<uint32_t>(42061 * index.x + 11969 * index.y);
    }
};

#endif // COORD_2D_HPP_
