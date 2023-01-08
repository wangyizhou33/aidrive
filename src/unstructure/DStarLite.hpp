#ifndef DSTARLITE_HPP_
#define DSTARLITE_HPP_

// D* Lite implementation based on paper by Sven Koenig and Maxim Likhachev
#include <memory>        // shared_ptr
#include <limits>        // std::numeric_limits
#include <optional>      // std::optional
#include <queue>         // std::priority_queue
#include <unordered_map> // std::unordered_map

#include "CollisionGrid.hpp"
#include <aidrive/Utils.hpp>

class DNode
{
public:
    struct Key
    {

        Key() = default;

        float32_t totalDistanceEstimate{std::numeric_limits<float32_t>::max()}; //!< estimate of total path distance, + priority queue correction term
        float32_t targetDistance{std::numeric_limits<float32_t>::max()};        //!< distance to target

        uint64_t updateId{0};

        bool operator<(const Key& other) const
        {
            if (totalDistanceEstimate < other.totalDistanceEstimate)
            {
                return true;
            }
            else if (isFloatEqual(totalDistanceEstimate, other.totalDistanceEstimate) && targetDistance < other.targetDistance)
            {
                return true;
            }

            return false;
        }

        bool operator==(const Key& other) const
        {
            return (isFloatEqual(totalDistanceEstimate, other.totalDistanceEstimate) && isFloatEqual(targetDistance, other.targetDistance));
        }
    };

    DNode()
        : m_index(INT_MAX, INT_MAX)
        , m_pos(0.0f, 0.0f)
        , m_targetDistanceLookAhead(std::numeric_limits<float32_t>::max())
        , m_targetDistance(std::numeric_limits<float32_t>::max())
        , m_key()
    {
    }

    inline void init(const Coord2d& cellIndex, const Vector2f& pos)
    {
        m_index                   = cellIndex;
        m_pos                     = pos;
        m_targetDistanceLookAhead = std::numeric_limits<float32_t>::max();
        m_targetDistance          = std::numeric_limits<float32_t>::max();
        m_key                     = Key();
    }

    inline const Coord2d& getIndex() const
    {
        return m_index;
    }

    inline const Vector2f& getPos() const
    {
        return m_pos;
    }

    inline void setTargetDistanceLookahead(float32_t targetDistanceLookahead)
    {
        m_targetDistanceLookAhead = targetDistanceLookahead;
    }

    inline float32_t getTargetDistanceLookahead() const
    {
        return m_targetDistanceLookAhead;
    }

    inline void setTargetDistance(float32_t g)
    {
        m_targetDistance = g;
    }

    inline float32_t getTargetDistance() const
    {
        return m_targetDistance;
    }

    inline void setKey(const DNode::Key& key)
    {
        uint64_t newUpdateId = m_key.updateId + 1;
        m_key                = key;
        m_key.updateId       = newUpdateId;
    }

    inline const DNode::Key& getKey() const
    {
        return m_key;
    }

private:
    Coord2d m_index;
    Vector2f m_pos;

    float32_t m_targetDistanceLookAhead; //!< corresponds to Rhs in paper
    float32_t m_targetDistance;          //!< corresponds to G in paper

    Key m_key;
};

// for m_openSet
class DNodePairCompare
{
public:
    bool operator()(const std::pair<DNode*, DNode::Key>& n1, const std::pair<DNode*, DNode::Key>& n2) const
    {
        return !(n1.second < n2.second);
    }
};


class DStarLite
{
public:
    DStarLite();

   ~DStarLite() = default;

    void init(int32_t maxNumNodes, float32_t cellSize, int32_t maxNumCollisionCells);

    float32_t getCellSize() const
    {
        return m_cellSize;
    }

    void setTarget(const Vector2f& targetPos);

    bool findPath(float32_t& pathLength, const Vector2f& startPos);

    uint32_t getNumPathNodes();
    bool getPath(float32_t* path, uint32_t* pathSize);

    std::shared_ptr<CollisionGrid> getCollisionGrid()
    {
        return m_collision;
    }

    const std::vector<DNode>& getNodes() const
    {
        return m_nodes;
    }

    uint32_t getMaxNumNodes() const
    {
        return static_cast<uint32_t>(m_maxNumNodes);
    }

private:
    void clearSearchState();

    bool setTargetNode(const Vector2f& targetPos);

    bool handleRemovals();
    bool handleAdditions();

    DNode* createNode(const Coord2d& index);
    DNode* getOrCreateNode(const Coord2d& index);

    float32_t startDistEstimate(const DNode& n); // corresponds to h in paper
    DNode::Key calculateKey(const DNode& n);
    void updateNodeKey(DNode& n);
    bool computeShortestPath();

    bool updateLookahead(DNode& node);
    void handleRemoveCollision(DNode* u, DNode* v);

    bool updatePath(float32_t* path, uint32_t* pathSize);

    DNode* openSetTop();

    std::shared_ptr<CollisionGrid> m_collision;

    int32_t m_maxNumNodes;
    int32_t m_currentNumNodes;
    float32_t m_cellSize;
    float32_t m_diagonalDist;

    // first direct neighbors, then diagonals, then cell itself
    Coord2d m_neighbors[9];

    std::vector<DNode> m_nodes;
    std::unique_ptr<std::unordered_map<Coord2d, DNode*, Coord2d, std::equal_to<Coord2d>>> m_nodeMap;

    // m_openSet contains locally inconsistent nodes
    std::unique_ptr<std::priority_queue<std::pair<DNode*, DNode::Key>, std::vector<std::pair<DNode*, DNode::Key>>, DNodePairCompare>> m_openSet;

    DNode* m_startNode;
    DNode* m_targetNode;

    std::optional<Vector2f> m_targetPos;
    bool m_startSearchFromScratch;

    float32_t m_keyCorrection; // keeps the order of the mOpenSet consistent when the start node moves


}; // class DStarLite




#endif // DSTARLITE_HPP_