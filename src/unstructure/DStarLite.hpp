#ifndef DSTARLITE_HPP_
#define DSTARLITE_HPP_

// D* Lite implementation based on paper by Sven Koenig and Maxim Likhachev
#include <memory> // shared_ptr
#include "CollisionGrid.hpp"

class DStarLite
{
public:
    DStarLite()
    {
        float32_t cellSize = 1.0f;
        uint32_t maxNumCollisionCells = 10000;
        m_collision.reset(new CollisionGrid(cellSize, maxNumCollisionCells));
    }

    std::shared_ptr<CollisionGrid> getCollisionGrid()
    {
        return m_collision;
    }


private:
    std::shared_ptr<CollisionGrid> m_collision;


}; // class DStarLite




#endif // DSTARLITE_HPP_