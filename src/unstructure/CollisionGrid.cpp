#include "CollisionGrid.hpp"


// The reserved size of a hashset has to be bigger than the
// maximum number of stored elements. This multiplier is a trade-off between
// memory and performance, as performance degrades when getting
// close to capacity.
#define HASHSET_SIZE_MULTIPLIER 1.3

CollisionGrid::CollisionGrid(float32_t cellSize, uint32_t maxCellCount)
    : m_cellSize(cellSize)
    , m_collision(static_cast<size_t>(HASHSET_SIZE_MULTIPLIER * maxCellCount))
    , m_added(static_cast<size_t>(HASHSET_SIZE_MULTIPLIER * maxCellCount))
    , m_removed(static_cast<size_t>(HASHSET_SIZE_MULTIPLIER * maxCellCount))
    , m_addedPrevious(static_cast<size_t>(HASHSET_SIZE_MULTIPLIER * maxCellCount))
    , m_removedPrevious(static_cast<size_t>(HASHSET_SIZE_MULTIPLIER * maxCellCount))
{
}

void CollisionGrid::addCells(const float32_t* positions, int32_t numPositions)
{
    for (int32_t i = 0; i < numPositions; ++i)
    {
        Coord2d coord(Vector2f(positions[2 * i], positions[2 * i + 1]), m_cellSize);

        if (isDrivable(coord))
        {
            m_added.insert(coord);
        }
        m_removed.erase(coord);
    }
}

void CollisionGrid::addCells(const int32_t* coords, int32_t numCoords)
{
    for (int32_t i = 0; i < numCoords; ++i)
    {
        Coord2d coord(coords[2 * i], coords[2 * i + 1]);
        if (isDrivable(coord))
        {
            m_added.insert(coord);
        }
        m_removed.erase(coord);
    }
}

void CollisionGrid::removeCells(const float32_t* positions, int32_t numPositions)
{
    for (int32_t i = 0; i < numPositions; ++i)
    {
        Coord2d coord(Vector2f(positions[2 * i], positions[2 * i + 1]), m_cellSize);
        if (!isDrivable(coord))
        {
            m_removed.insert(coord);
        }
        m_added.erase(coord);
    }
}

void CollisionGrid::removeCells(const int32_t* coords, int32_t numCoords)
{
    for (int32_t i = 0; i < numCoords; ++i)
    {
        Coord2d coord(coords[2 * i], coords[2 * i + 1]);
        if (!isDrivable(coord))
        {
            m_removed.insert(Coord2d(coords[2 * i], coords[2 * i + 1]));
        }
        m_added.erase(coord);
    }
}

void CollisionGrid::clear()
{
    for (auto it = m_collision.begin(); it != m_collision.end(); ++it)
    {
        removeCells(&(it->x), 1);
    }
}

bool CollisionGrid::hasChanged() const
{
    return m_addedPrevious.size() > 0 || m_removedPrevious.size() > 0;
}

const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>& CollisionGrid::getAddedCells() const
{
    return m_addedPrevious;
}

const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>& CollisionGrid::getRemovedCells() const
{
    return m_removedPrevious;
}

void CollisionGrid::update()
{
    for (auto it = m_added.begin(); it != m_added.end(); ++it)
    {
        m_collision.insert(*it);
    }

    m_addedPrevious.swap(m_added);
    m_added.clear();

    for (auto it = m_removed.begin(); it != m_removed.end(); ++it)
    {
        m_collision.erase(*it);
    }
    m_removedPrevious.swap(m_removed);
    m_removed.clear();
}

bool CollisionGrid::isDrivable(const Vector2f& pos) const
{
    Coord2d index(pos, m_cellSize);
    auto it = m_collision.find(index);
    return it == m_collision.end();
}

bool CollisionGrid::isDrivable(const Coord2d& index) const
{
    auto it = m_collision.find(index);
    return it == m_collision.end();
}

bool CollisionGrid::wasDrivable(const Coord2d& index) const
{
    bool nowDrivable      = isDrivable(index);
    bool collisionAdded   = m_addedPrevious.find(index) != m_addedPrevious.end();
    bool collisionRemoved = m_removedPrevious.find(index) != m_removedPrevious.end();
    return (nowDrivable && !collisionRemoved) || (!nowDrivable && collisionAdded);
}

bool CollisionGrid::isDrivable(const Coord2d& srcIndex, const Coord2d& dstIndex) const
{
    // assumes src is drivable

    if (!isDrivable(dstIndex))
        return false;

    Coord2d diff = dstIndex - srcIndex;
    if (abs(diff.x) + abs(diff.y) < 2)
    {
        // direct connection
        return true;
    }
    else
    {
        Coord2d index1(srcIndex.x + diff.x, srcIndex.y);
        Coord2d index2(srcIndex.x, srcIndex.y + diff.y);
        return isDrivable(index1) || isDrivable(index2);
    }
}

bool CollisionGrid::wasDrivable(const Coord2d& srcIndex, const Coord2d& dstIndex) const
{
    // assumes src was drivable

    if (!wasDrivable(dstIndex))
        return false;

    Coord2d diff = dstIndex - srcIndex;
    if (abs(diff.x) + abs(diff.y) < 2)
    {
        // non-diagonal connection
        return true;
    }
    else
    {
        // diagonal connection
        Coord2d index1(srcIndex.x + diff.x, srcIndex.y);
        Coord2d index2(srcIndex.x, srcIndex.y + diff.y);
        return wasDrivable(index1) || wasDrivable(index2);
    }
}
