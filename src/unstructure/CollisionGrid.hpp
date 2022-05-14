#ifndef COLLISION_GRID_HPP_
#define COLLISION_GRID_HPP_

#include "./Coord2d.hpp"
#include <unordered_set> // std::unordered_set

class CollisionGrid
{
public:
    CollisionGrid(float32_t cellSize, uint32_t maxCellCount);
    ~CollisionGrid() = default;

    float32_t getCellSize() const
    {
        return m_cellSize;
    }

    void addCells(const float32_t* positions, int32_t numPositions);
    void addCells(const int32_t* coords, int32_t numCoords);

    void removeCells(const float32_t* positions, int32_t numPositions);
    void removeCells(const int32_t* coords, int32_t numCoords);

    void clear();

    // no need anymore
    bool findClosestCollisionCell(Coord2d& collisionCell, const Vector2f& pos);

    const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>* getCells() const { return &m_collision; }

public:
    // hack, this should be private, otherwise D*Lite search could miss collision updates.
    // the problem is that the path smoother also uses the collision grid.
    void update();

private:
    // returns the changes from the last update
    // (not the currently buffered additions and removals)
    bool hasChanged() const;
    const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>& getAddedCells() const;
    const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>& getRemovedCells() const;

    bool isDrivable(const Vector2f& index) const;
    bool isDrivable(const Coord2d& index) const;
    bool wasDrivable(const Coord2d& index) const;

    // assumes src was drivable
    bool isDrivable(const Coord2d& srcIndex, const Coord2d& dstIndex) const;
    bool wasDrivable(const Coord2d& srcIndex, const Coord2d& dstIndex) const;

    float32_t m_cellSize;

    std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>> m_collision;
    std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>> m_added;
    std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>> m_removed;

    std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>> m_addedPrevious;
    std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>> m_removedPrevious;

    friend class HybridAStar;
    friend class DStarLite;
}; // class CollisionGrid

#endif // COLLISION_GRID_HPP_
