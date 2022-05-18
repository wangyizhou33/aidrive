#include "DStarLite.hpp"
#include <iostream>

DStarLite::DStarLite()
    : m_maxNumNodes(0)
    , m_currentNumNodes(0)
    , m_cellSize(0.0f)
    , m_diagonalDist(0.0f)
    , m_collision(nullptr)
    , m_startNode(nullptr)
    , m_targetNode(nullptr)
    , m_startSearchFromScratch(true)
    , m_keyCorrection(0.0f)
{
    // neighbor array for iterating

    // first 4 are direct neighbors, ordered clockwise
    m_neighbors[0] = Coord2d(0, 1);  // up
    m_neighbors[1] = Coord2d(1, 0);  // right
    m_neighbors[2] = Coord2d(0, -1); // down
    m_neighbors[3] = Coord2d(-1, 0); // left

    // diagonals, ordered clockwise, such that (i + 4) % 4 is between i and i+1
    m_neighbors[4] = Coord2d(1, 1);   // right up
    m_neighbors[5] = Coord2d(1, -1);  // right down
    m_neighbors[6] = Coord2d(-1, -1); // left down
    m_neighbors[7] = Coord2d(-1, 1);  // left up

    // last entry is the cell itself
    m_neighbors[8] = Coord2d(0, 0);
}


void DStarLite::init(int32_t maxNumNodes, float32_t cellSize, int32_t maxNumCollisionCells)
{
    m_maxNumNodes = maxNumNodes;
    m_nodes.resize(maxNumNodes);
    // make hash container doesn't fill up to avoid frequent hash collisions
    constexpr float32_t HASH_CONTAINER_SIZE_MULTIPLIER = 1.5f;
    m_nodeMap.reset(new std::unordered_map<Coord2d, DNode*, Coord2d, std::equal_to<Coord2d>>(HASH_CONTAINER_SIZE_MULTIPLIER * maxNumNodes));
    m_openSet.reset(new std::priority_queue<std::pair<DNode*, DNode::Key>, std::vector<std::pair<DNode*, DNode::Key>>, DNodePairCompare>());
    m_currentNumNodes = 0;

    m_cellSize     = cellSize;
    m_diagonalDist = sqrtf(2.0f) * cellSize;

    m_startNode              = nullptr;
    m_targetNode             = nullptr;
    m_startSearchFromScratch = true;

    m_collision.reset(new CollisionGrid(cellSize, maxNumCollisionCells));
}

void DStarLite::clearSearchState()
{
    m_currentNumNodes = 0;
    m_startNode       = nullptr;
    m_targetNode      = nullptr;
    m_openSet.reset(new std::priority_queue<std::pair<DNode*, DNode::Key>, std::vector<std::pair<DNode*, DNode::Key>>, DNodePairCompare>());  // clear it
    m_nodeMap->clear();
    m_keyCorrection = 0.0f;
}

void DStarLite::setTarget(const Vector2f& targetPos)
{
    // create start node
    m_targetPos = targetPos;
}

DNode* DStarLite::createNode(const Coord2d& index)
{
    DNode* node = nullptr;
    if (m_currentNumNodes < m_maxNumNodes)
    {
        node                = &m_nodes[m_currentNumNodes++];
        Vector2f cellCenter = index.getCenter(m_cellSize);
        node->init(index, cellCenter);
    }
    else
    {
        std::cout << "DStarLite::createNode: preallocated space for nodes is too small.\n";
    }
    return node;
}

DNode* DStarLite::getOrCreateNode(const Coord2d& index)
{
    DNode* n = (*m_nodeMap)[index];
    if (n == nullptr)
    {
        n = createNode(index);
        if (n != nullptr)
        {
            (*m_nodeMap)[index] = n;
        }
    }

    return n;
}

float32_t DStarLite::startDistEstimate(const DNode& n)
{
    return (m_startNode->getPos() - n.getPos()).norm();
}

DNode::Key DStarLite::calculateKey(const DNode& n)
{
    float32_t targetDistance = std::min(n.getTargetDistance(), n.getTargetDistanceLookahead());
    DNode::Key key;
    key.totalDistanceEstimate = targetDistance + startDistEstimate(n) + m_keyCorrection;
    key.targetDistance        = targetDistance;
    key.updateId              = 0;

    return key;
}

void DStarLite::updateNodeKey(DNode& n)
{
    DNode::Key keyNew = calculateKey(n);
    n.setKey(keyNew);
    if (!isFloatEqual(n.getTargetDistance(), n.getTargetDistanceLookahead()))
    {
        // update priority queue entry
        m_openSet->push(std::pair<DNode*, DNode::Key>(&n, n.getKey()));
    }
    // else node is locally consistent, so should not be part of the open set (Invariant 2)
    // (all existing entries are invalid now, as setKey has changed the updateId of the node's key)
}

// pop invalid entries from queue until a valid one is found
DNode* DStarLite::openSetTop()
{
    DNode* result = nullptr;
    while (m_openSet->size() > 0)
    {
        const std::pair<DNode*, DNode::Key>& top = m_openSet->top();
        if (top.second.updateId == top.first->getKey().updateId)
        {
            result = top.first;
            break;
        }

        m_openSet->pop();
    }

    return result;
}


bool DStarLite::computeShortestPath()
{
    // terminate when these conditions are true:
    // -- start vertex key is less than or equal to top of priority queue
    // -- start vertex is not locally underconsistent
    DNode* node = openSetTop();
    while (
        (node != nullptr && (node->getKey() < m_startNode->getKey())) ||             // top node key < start node key
        m_startNode->getTargetDistance() < m_startNode->getTargetDistanceLookahead() // start node is locally underconsistent
        )
    {

        // get top of priority queue (smallest key)
        m_openSet->pop();

        DNode::Key keyOld = node->getKey();
        DNode::Key keyNew = calculateKey(*node);
        if (keyOld < keyNew)
        {
            node->setKey(keyNew);
            m_openSet->push(std::pair<DNode*, DNode::Key>(node, node->getKey()));
        }
        else if (node->getTargetDistance() > node->getTargetDistanceLookahead())
        {
            // locally overconsistent (targetDistance is larger than the lookahead value)

            // make locally consistent
            node->setTargetDistance(node->getTargetDistanceLookahead());

            //for all predecessors of node
            Coord2d nodeIndex = node->getIndex();
            bool nodeDrivable = m_collision->isDrivable(nodeIndex);
            for (int32_t i = 0; i < 8; ++i)
            { // iterating neighbors, excluding current node
                Coord2d neighborIndex = nodeIndex + m_neighbors[i];
                DNode* neighbor       = getOrCreateNode(neighborIndex);
                if (neighbor == nullptr)
                    return false;

                if (neighborIndex != m_targetNode->getIndex())
                {
                    float32_t edgeCost = std::numeric_limits<float32_t>::max();
                    if (nodeDrivable && m_collision->isDrivable(nodeIndex, neighborIndex))
                    {
                        edgeCost = (i < 4) ? m_cellSize : m_diagonalDist;
                    }

                    float32_t minTargetDistance = std::min(neighbor->getTargetDistanceLookahead(), edgeCost + node->getTargetDistance());
                    neighbor->setTargetDistanceLookahead(minTargetDistance);
                }

                // update
                updateNodeKey(*neighbor);
            }
        }
        else
        {
            // locally underconsistent (targetDistance is smaller than the lookahead value)

            float32_t oldTargetDistance = node->getTargetDistance();

            // make locally overconsistent or consistent
            node->setTargetDistance(std::numeric_limits<float32_t>::max());

            //for all predecessors of node, and node itself
            // update their lookahead value
            Coord2d nodeIndex = node->getIndex();
            bool nodeDrivable = m_collision->isDrivable(nodeIndex);
            for (int32_t i = 0; i < 9; ++i)
            { // iterating neighbors, and current node
                Coord2d neighborIndex = nodeIndex + m_neighbors[i];
                DNode* neighbor       = getOrCreateNode(neighborIndex);
                if (neighbor == nullptr)
                    return false;

                float32_t edgeCost = std::numeric_limits<float32_t>::max();
                if (nodeDrivable && m_collision->isDrivable(nodeIndex, neighborIndex))
                {
                    edgeCost = (i < 4) ? m_cellSize : ((i < 8) ? m_diagonalDist : 0.0f);
                }

                if (edgeCost < std::numeric_limits<float32_t>::max() &&
                    oldTargetDistance < std::numeric_limits<float32_t>::max() &&
                    isFloatEqual(neighbor->getTargetDistanceLookahead(), edgeCost + oldTargetDistance))
                {
                    // neighbor had it's value propagated from current node, needs update

                    if (neighbor->getIndex() != m_targetNode->getIndex())
                    {
                        if (!updateLookahead(*neighbor))
                            return false;
                    }
                }
                updateNodeKey(*neighbor);
            }
        }

        node = openSetTop();
    }

    return true;
}


bool DStarLite::updateLookahead(DNode& node)
{
    Coord2d nodeIndex = node.getIndex();
    bool nodeDrivable = m_collision->isDrivable(nodeIndex);

    float32_t targetDistanceLookahead = std::numeric_limits<float32_t>::max();
    for (int32_t i = 0; i < 8; ++i)
    { // iterating neighbors, excluding current node

        Coord2d successorIndex = nodeIndex + m_neighbors[i];
        DNode* s               = getOrCreateNode(successorIndex);
        if (s == nullptr)
            return false;

        float32_t edgeCost = std::numeric_limits<float32_t>::max();
        if (nodeDrivable && m_collision->isDrivable(nodeIndex, successorIndex) &&
            s->getTargetDistance() < std::numeric_limits<float32_t>::max())
        {
            edgeCost                             = (i < 4) ? m_cellSize : m_diagonalDist;
            float32_t newTargetDistanceLookahead = edgeCost + s->getTargetDistance();

            targetDistanceLookahead = std::min(targetDistanceLookahead, newTargetDistanceLookahead);
        }
    }

    node.setTargetDistanceLookahead(targetDistanceLookahead);
    return true;
}

bool DStarLite::findPath(float32_t& pathLength, const Vector2f& startPos)
{
    // init result values
    bool ok    = true;
    pathLength = std::numeric_limits<float32_t>::max();

    if (!m_targetPos)
    {
        // no target set, early out
        return false;
    }

    // check if we have a new target node
    Coord2d index      = Coord2d::compute(m_cellSize, *m_targetPos);
    bool newTargetNode = (m_targetNode == nullptr || m_targetNode->getIndex() != index);

    // start from scratch if we have a new target
    if (newTargetNode)
    {
        std::cout << "D*Lite: reset due to new target node.\n";
        m_startSearchFromScratch = true;
    }

    // start from scratch to avoid floating point issues in calculateKey
    // due to too large key correction
    constexpr float32_t MAX_KEY_CORRECTION = 1e9;
    if (m_keyCorrection > MAX_KEY_CORRECTION)
    {
        std::cout << "D*Lite: reset due to large key correction term.\n";
        m_startSearchFromScratch = true;
    }

    if (m_startSearchFromScratch)
    {
        clearSearchState();

        // create new target node
        m_targetNode = getOrCreateNode(index);
    }

    // update start node
    DNode* previousStartNode = m_startNode;
    Coord2d startIndex       = Coord2d::compute(m_cellSize, startPos);
    m_startNode              = getOrCreateNode(startIndex);

    ok &= m_startNode != nullptr;  // start node required
    ok &= m_targetNode != nullptr; // target node required
    if (ok)
    {
        // update key correction
        if (previousStartNode != nullptr &&
            previousStartNode->getIndex() != m_startNode->getIndex()) // new start node
        {
            if (m_openSet->size() > 0)
            {
                // Update key correction to keep the priority queue order intact (Invariant 3)
                //
                // Difference to D*Lite paper:
                // - paper updates key correction only on edge cost changes, because it expects moving along the optimal path (?)
                // - here we want to support incremental searches with arbitrary start nodes, so we also increase key when collision hasn't changed
                m_keyCorrection += startDistEstimate(*previousStartNode);
            }
            else
            {
                m_keyCorrection = 0.0f;
            }

            // need to update the start node, so that the termination condition in computeShortestPath is correct
            updateNodeKey(*m_startNode);
        }

        // update collision
        m_collision->update();
        bool collisionChanged = m_collision->hasChanged();

        if (m_collision->isDrivable(startIndex) && m_collision->isDrivable(m_targetNode->getIndex()))
        {
            // both start and target are drivable, so a path is possible

            if (m_startSearchFromScratch)
            {
                // new search
                m_targetNode->setTargetDistanceLookahead(0.0f);
                updateNodeKey(*m_targetNode);
            }
            else if (collisionChanged)
            {
                // adapt search state to collision change
                ok &= handleAdditions();
                ok &= handleRemovals();
            }

            // update search state
            ok &= computeShortestPath();

            // get pathLength for search state
            pathLength = m_startNode->getTargetDistanceLookahead();

            // return false if the start node is not reachable from the target node
            ok &= pathLength < std::numeric_limits<float32_t>::max();
        }
        else
        {
            // no result when start or target is not drivable
            ok = false;
        }
    }

    // if there was an error, restart from scratch next time,
    // as the current state might not be consistent
    if (!ok)
    {
        std::cout << "D*Lite: reset due to failed search.\n";
    }
    m_startSearchFromScratch = !ok;

    return ok;
}


bool DStarLite::updatePath(float32_t* path, uint32_t* pathSize)
{
    uint32_t pathPointIndex = 0;
    DNode* previous         = nullptr;
    DNode* node             = getOrCreateNode(m_startNode->getIndex());
    while (node != nullptr && node->getIndex() != m_targetNode->getIndex())
    {
        if (path && pathPointIndex < *pathSize)
        {
            path[2 * pathPointIndex + 0] = node->getPos().x();
            path[2 * pathPointIndex + 1] = node->getPos().y();
        }
        ++pathPointIndex;

        Coord2d nodeIndex = node->getIndex();

        DNode* minNode    = nullptr;
        float32_t minCost = std::numeric_limits<float32_t>::max();
        for (int32_t i = 0; i < 8; ++i)
        { // iterating neighbors, excluding current node
            Coord2d neighborIndex = nodeIndex + m_neighbors[i];
            if (!m_collision->isDrivable(nodeIndex, neighborIndex))
                continue;

            DNode* neighbor = getOrCreateNode(neighborIndex);
            if (neighbor == nullptr)
            {
                std::cout << "DStarLite::updatePath: not at index "
                          << neighborIndex.x << " " << neighborIndex.y << " "
                          << "not found.\n";
                *pathSize = 0;
                return false;
            }

            float32_t edgeCost = (i < 4) ? m_cellSize : m_diagonalDist;

            if (edgeCost + neighbor->getTargetDistance() < minCost)
            {
                minCost = edgeCost + neighbor->getTargetDistance();
                minNode = neighbor;
            }
        }

        if (previous == minNode)
            break;

        previous = node;
        node     = minNode;
    }

    if (path && pathPointIndex < *pathSize)
    {
        path[2 * pathPointIndex + 0] = m_targetNode->getPos().x();
        path[2 * pathPointIndex + 1] = m_targetNode->getPos().y();
    }
    ++pathPointIndex;

    *pathSize = pathPointIndex;
    return true;
}


bool DStarLite::getPath(float32_t* path, uint32_t* pathSize)
{
    bool ok = true;
    if (m_startNode == nullptr || m_targetNode == nullptr)
    {
        *pathSize = 0;
    }
    else
    {
        ok = updatePath(path, pathSize);
    }
    return ok;
}


bool DStarLite::handleAdditions()
{
    // handle additions
    const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>& addedCells = m_collision->getAddedCells();
    for (auto it = addedCells.begin(); it != addedCells.end(); ++it)
    {
        Coord2d nodeIndex = *it;
        DNode* node       = getOrCreateNode(nodeIndex);
        if (node == nullptr)
            return false;

        node->setTargetDistanceLookahead(std::numeric_limits<float32_t>::max());
        updateNodeKey(*node);

        bool nodeWasDrivable = m_collision->wasDrivable(nodeIndex);

        // update affected diagonal edges between direct neighbors
        //
        // example (O is a drivable cell, # is a blocked cell):
        //
        // previous   current
        //
        // OO#        OO#
        // OOO   =>   O#O
        // OOO        OOO
        //
        // previously it was possible to go from the direct top to direct left neighbor,
        // but now that edge is blocked.
        //
        for (int32_t i = 0; i < 4; ++i)
        { // iterating direct neighbors
            Coord2d node1Index = nodeIndex + m_neighbors[i];
            Coord2d node2Index = nodeIndex + m_neighbors[(i + 1) % 4];

            if (!m_collision->wasDrivable(node1Index) || !m_collision->wasDrivable(node2Index))
                continue; // no change

            // check diagonal node between the 2 direct neighbors.
            // if it's not drivable, the edge is now blocked, so the 2 involved
            // nodes need an update
            Coord2d checkNodeIndex = nodeIndex + m_neighbors[i + 4];
            if (!m_collision->isDrivable(checkNodeIndex))
            {
                DNode* node1 = getOrCreateNode(node1Index);
                DNode* node2 = getOrCreateNode(node2Index);
                if (node1 != nullptr && node2 != nullptr)
                {
                    if (!updateLookahead(*node1))
                        return false;

                    updateNodeKey(*node1);

                    if (!updateLookahead(*node2))
                        return false;

                    updateNodeKey(*node2);
                }
            }
        }

        // update neighbors edges
        for (int32_t i = 0; i < 8; ++i)
        { // iterating neighbors, excluding current node

            Coord2d neighborIndex = nodeIndex + m_neighbors[i];
            DNode* neighbor       = getOrCreateNode(neighborIndex);
            if (neighbor == nullptr)
                return false;

            if (neighbor->getIndex() == m_targetNode->getIndex())
                continue;

            if (m_collision->wasDrivable(neighborIndex) && !m_collision->isDrivable(neighborIndex))
            {
                neighbor->setTargetDistanceLookahead(std::numeric_limits<float32_t>::max());
                updateNodeKey(*neighbor);
            }
            else if (nodeWasDrivable && m_collision->wasDrivable(nodeIndex, neighborIndex))
            {
                // collision added, hence current edge cost is FLT_MAX.
                // node was drivable (as collision can only be added on drivable nodes),
                // and neighbor was drivable, thus oldEdgeCost < FLT_MAX
                float32_t oldEdgeCost = (i < 4) ? m_cellSize : m_diagonalDist;
                if (isFloatEqual(node->getTargetDistance(), std::numeric_limits<float32_t>::max()) ||
                    isFloatEqual(node->getTargetDistance(), std::numeric_limits<float32_t>::max()) ||
                    isFloatEqual(neighbor->getTargetDistanceLookahead(), oldEdgeCost + node->getTargetDistance()))
                {
                    // this neighbor needs to be updated, since it's target distance value was
                    // propagated from current node
                    if (!updateLookahead(*neighbor))
                        return false;

                    updateNodeKey(*neighbor);
                }
            }
        }
    }

    return true;
}

bool DStarLite::handleRemovals()
{
    // handle removals
    const std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>& removedCells = m_collision->getRemovedCells();
    for (auto it = removedCells.begin(); it != removedCells.end(); ++it)
    {
        Coord2d nodeIndex = *it;
        DNode* node       = getOrCreateNode(nodeIndex);
        if (node == nullptr)
            return false;

        if (nodeIndex == m_targetNode->getIndex())
            continue;

        // update affected diagonal edges
        // (edges between non-diagonal neighbors)
        //
        // example (O is a drivable cell, # is a blocked cell):
        //
        // previous   current
        //
        // OO#        OO#
        // O#O   =>   OOO
        // OOO        OOO
        //
        // previously it was not possible to go from the direct top to direct left neighbor,
        // but now that edge is not blocked anymore.
        //
        for (int32_t i = 0; i < 4; ++i)
        {
            Coord2d diagNodeIndex = nodeIndex + m_neighbors[i + 4];
            if (m_collision->wasDrivable(diagNodeIndex))
                continue; // no change

            // consider the 2 direct neighbors, where diagNodeIndex is in between.
            // if both are drivable, then they need to be updated, as there is
            // a new edge between them now, so potentially there is a shorter path.
            Coord2d node1Index = nodeIndex + m_neighbors[i];
            Coord2d node2Index = nodeIndex + m_neighbors[(i + 1) % 4];

            if (m_collision->isDrivable(node1Index) && m_collision->isDrivable(node2Index))
            {
                DNode* node1 = getOrCreateNode(node1Index);
                DNode* node2 = getOrCreateNode(node2Index);
                if (node1 != nullptr && node2 != nullptr)
                {
                    if (!updateLookahead(*node1))
                        return false;
                    updateNodeKey(*node1);

                    if (!updateLookahead(*node2))
                        return false;
                    updateNodeKey(*node2);
                }
            }
        }

        // update current node's lookahead
        if (!updateLookahead(*node))
            return false;
        updateNodeKey(*node);
    }

    return true;
}