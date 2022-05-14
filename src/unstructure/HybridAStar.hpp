#ifndef HYBRIDASTAR_HPP_
#define HYBRIDASTAR_HPP_

#include <iostream>
#include <aidrive/Types.hpp>
#include <limits>    // numeric_limits
#include <stdexcept> // runtime_error
#include <memory>    // unique_ptr
#include <unordered_map>
#include <queue> // priority_queue
#include <ReedsShepp.hpp>
#include "DStarLite.hpp"
#include <memory> // shared_ptr

using Vector2f = aidrive::Vector2f;
using Matrix2f = aidrive::Matrix2f;

aidrive::Matrix2f Rotation2D(float32_t const angle)
{
    aidrive::Matrix2f ret;

    ret << std::cos(angle), -std::sin(angle),
        std::sin(angle), std::cos(angle);

    return ret;
}

enum DrivingState
{
    BACKWARD = -1,
    STANDING = 0,
    FORWARD  = 1
};

/// User callback to define the drivable space
typedef bool (*IsDrivableCallback)(const float32_t srcPosition[2],
                                   const float32_t srcHeading[2],
                                   const float32_t dstPosition[2],
                                   const float32_t dstHeading[2],
                                   void* userData);

/// User callback to add user defined costs used by the path planner.
typedef float32_t (*ExternalCostCallback)(const float32_t position[2],
                                          const float32_t heading[2],
                                          void* userData);

template <class T>
bool isEqual(T a, T b)
{
    return ((a - b) < std::numeric_limits<T>::epsilon()) &&
           ((b - a) < std::numeric_limits<T>::epsilon());
}

struct CarNodeIndex
{
    CarNodeIndex() = default;
    CarNodeIndex(int32_t x_, int32_t y_, int32_t orientationIndex_, int32_t drivingDir_)
        : x(x_)
        , y(y_)
        , orientationIndex(orientationIndex_)
        , drivingDir(drivingDir_)
    {
    }

    static CarNodeIndex compute(float32_t cellSize, float32_t orientationBucketSize, int32_t numOrientationBuckets,
                                const Vector2f& pos,
                                const Vector2f& heading,
                                int32_t drivingDir);

    bool operator==(const CarNodeIndex& other) const
    {
        return x == other.x &&
               y == other.y &&
               orientationIndex == other.orientationIndex &&
               drivingDir == other.drivingDir;
    }

    bool operator!=(const CarNodeIndex& other) const
    {
        return !(*this == other);
    }

    int32_t x{0};
    int32_t y{0};

    // 0 is north, then the buckets go around clockwise
    int32_t orientationIndex{0};

    int32_t drivingDir{0}; // -1 is backward, 1 is forward, 0 is not moving
};

class CarNodeHash
{
public:
    size_t operator()(const CarNodeIndex& index) const
    {
        return static_cast<size_t>(42061 * index.x + 11969 * index.y + 4703 * index.orientationIndex + 13 * index.drivingDir);
    }
};

class CarNode
{
public:
    CarNode()
        : m_pos(0.0f, 0.0f)
        , m_heading(0.0f, 0.0f)
        , m_costFromStart(std::numeric_limits<float32_t>::max())
        , m_costEstimateTotal(std::numeric_limits<float32_t>::max())
    {
    }

    inline void init(CarNode* previous,
                     const CarNodeIndex& cellIndex,
                     const Vector2f& pos,
                     const Vector2f& heading,
                     int32_t steeringAngle,
                     float32_t costFromStart,
                     float32_t costEstimateTotal)
    {
        m_previous = previous;
        m_index    = cellIndex;
        m_pos      = pos;
        m_heading  = heading;
        if (isEqual(m_heading.norm(), 0.0f))
        {
            throw std::runtime_error(
                "HybridAStar: Cannot create CarNode from heading with norm of zero");
        }
        m_heading.normalize();
        m_steeringAngle     = steeringAngle;
        m_costFromStart     = costFromStart;
        m_costEstimateTotal = costEstimateTotal;
    }

    inline CarNode* getPrevious() const
    {
        return m_previous;
    }

    inline const CarNodeIndex& getIndex() const
    {
        return m_index;
    }

    inline const Vector2f& getPos() const
    {
        return m_pos;
    }

    inline const Vector2f& getHeading() const
    {
        return m_heading;
    }

    inline int32_t getSteeringAngle() const
    {
        return m_steeringAngle;
    }

    inline float32_t getCost() const
    {
        return m_costFromStart;
    }

    inline float32_t getEstimate() const
    {
        return m_costEstimateTotal;
    }

    inline void setVisitIndex(int32_t index)
    {
        m_visitIndex = index;
    }

    inline int32_t getVisitIndex() const
    {
        return m_visitIndex;
    }

private:
    CarNode* m_previous{nullptr};

    int32_t m_visitIndex{-1};
    CarNodeIndex m_index;
    Vector2f m_pos;
    Vector2f m_heading;

    float32_t m_costFromStart;
    float32_t m_costEstimateTotal;

    int32_t m_steeringAngle{0};

    friend class CarNodeHash;
};

class CarNodePairCompare
{
public:
    bool operator()(const std::pair<CarNode*, float32_t>& n1, const std::pair<CarNode*, float32_t>& n2) const
    {
        return n1.second > n2.second;
    }
};

class HybridAStar
{
public:
    struct Params
    {
        uint32_t maxNumNodes          = 1000000;
        uint32_t maxNumCollisionCells = 10000;
        float32_t maxPathLength       = 1000.0f;
        float32_t carTurningRadius    = 10.0f;
        float32_t cellSize            = 1.0f;
        float32_t distWeight          = 1.0f;
        float32_t dirSwitchCost       = 10.0f;
        float32_t backwardsMultiplier = 2.0f;
    };

    enum class State
    {
        UNINITIALIZED,
        READY,
        IN_PROGRESS
    };

    HybridAStar();
    ~HybridAStar();

    /**
        \brief Initialization

        The performance of the path search depends on the parameters cellSize, carTurningRadius and angleTolerance. Depending on those parameters, the bucket
        size for angle buckets per cell position varies:



        \param  maxNumNodes             Maximum size of preallocated node buffer. Search stops if number if this number has been reached.
        \param  carTurningRadius        Turning radius of the car. Higher value creates more search nodes.
        \param  cellSize                New search nodes are created whenever reaching a new cell.
                                        cellSize must be smaller than 2*carTurningRadius. The cellSize also defines the maximum angle change between 2 adjacent nodes.
                                        A cellSize of 2*carTurningRadius means there can be a 180� turn in 1 step, thus it must be smaller than that.
                                        With a big cellSize (relative to carTurningRadius), numSteeringAngles should be increased to get finer steps.
        \param  numSubsteps             Must be > 0. Number of calls to isDrivable along the path from one node to the next.
        \param  numSteeringAngles       Number of steering angles to one side. Must be greater than 0.
                                        Might be changed to a higher value if the provided angleTolerance requires a finer steering angle changes.
        \param  angleTolerance          Tolerance on orientation for accepting a match with the target. A small tolerance will increase the numSteeringAngles to make the target reachable.
        \param  distWeight              Cost function parameter. Multiplied with the euclidean distance between 2 nodes.
        \param  dirSwitchCost           Cost function parameter. Constant cost when switching forward/backward driving.
        \param  backwardsMultiplier     Cost function parameter. Multiplied for with total cost (external cost not included) when driving backwards.
        \param  maxNumCollisionCells    Maximum number of cells added to the CollisionGrid used in the holonomicWithObstacles heurstic.
                                        This specifies the size of the used hashsets, so this is performance and memory relevant
        \param  maxPathLength           Maximum expected length of a path without obstacles. This is needed
                                        for preallocating space for Reeds-Shepp results
    */
    void init(int32_t maxNumNodes,
              float32_t carTurningRadius,
              int32_t& numSteeringAngles,
              float32_t cellSize,
              int32_t numSubsteps,
              float32_t angleTolerance,
              float32_t distWeight,
              float32_t dirSwitchCost,
              float32_t backwardsMultiplier,
              bool useHolonomicWithObstaclesHeuristic,
              int32_t maxNumCollisionCells,
              float32_t maxPathLength);

    void reset() { clear(); }

    /**
        \brief Function pointer to describe collision with environment.

        The function should return true if the provided const references to the vec2d's position and heading
        are considered to be drivable, false otherwise.
        If no function is set everything is considered to be drivable.
    */
    void setIsDrivableCallback(IsDrivableCallback callback, void* userData);

    void getIsDrivableCallback(IsDrivableCallback* callback, void** userData);

    void setExternalCostCallback(ExternalCostCallback callback, void* userData);

    void getExternalCostCallback(ExternalCostCallback* callback, void** userData);

    /**
        \brief Find a path from start to target.

        \param[in]  startPos            Initial position.
        \param[in]  startHeading        Initial orientation.
        \param[in]  startDrivingDir     -1: currently driving backward, 0: currently not moving, 1: currently driving forward
        \param[in]  targetPos           Target position.
        \param[in]  targetHeading       Target orientation.
        \param[in]  targetDrivingDir    -1: target driving backward, 0: target not moving, 1: target driving forward
        \param[in]  reedsSheppN         Try to find an analytical solution every Nth visited node, more often the closer the nodes are to the goal.
                                        0 to disable reedsShepp expansions.
        \param[in]  maxNodeExpansionCount Stop search after given number of node expansions, continue the search
                                          at the next findPath call where it stopped.

        \returns State after this call.
    */
    State findPath(const Vector2f& startPos, const Vector2f& startHeading, DrivingState startDrivingDir,
                   const Vector2f& targetPos, const Vector2f& targetHeading, DrivingState targetDrivingDir,
                   uint32_t reedsSheppN,
                   uint32_t maxNodeExpansionCount);

    State continueFindPath(uint32_t maxNodeExpansionCount);

    State getState() const;

    const CarNode* getResult() const;
    uint32_t getPathNodeCount() const;

    uint32_t getPathCount() const { return m_pathNodeCount; };
    void getPath(float32_t* pathPoints,
                 float32_t* pathHeadings,
                 DrivingState* pathDrivingDirs,
                 int32_t numPoints);

    void getDStarLitePath(float32_t* pathPoints, uint32_t* pathPointCount);

    /**
       \brief Get the search tree in A* planner.
    */
    void getSearchLines(float32_t* searchLines, uint32_t* searchLineCount);
    /**
        \brief Number of all created nodes.
    */
    int32_t getCreatedNodeCount() const;

    /**
        \brief Get pointer to array of all created nodes.
    */
    const CarNode* getCreatedNodes() const;

    /**
        \brief Get number of created orientation buckets.
    */
    int32_t getOrientationBucketCount() const;

    /**
        \brief DStarLite path finder so its nodes can be read by user
    */
    std::shared_ptr<DStarLite> getDStarLite()
    {
        return m_DStarLite;
    }

    /**
        \brief Return whether found path is reedsShepp
    */
    bool isInitialPathReedsShepp() { return m_initialReedsSheppPathFound; }

private:
    void clear();

    void searchLoop(uint32_t maxNodeExpansionCount);

    CarNodeIndex computeIndex(const Vector2f& pos,
                              const Vector2f& heading,
                              int32_t drivingDir) const;
    CarNode* getNode(const CarNodeIndex& n);

    CarNode* createNode();
    bool createNeighbors(CarNode& n, bool backward);
    CarNode* getNextNode();
    float32_t getDistance(const CarNode& n1, const CarNode& n2) const;
    float32_t getEdgeCost(float32_t dist, int32_t drivingDir1, int32_t drivingDir2) const;
    float32_t getEstimate(const Vector2f& pos, const Vector2f& posTarget);
    bool isTarget(const CarNode& n) const;

    bool reedsShepp(CarNode* startNode,
                    const Vector2f& startPos,
                    const Vector2f& startHeading,
                    int32_t startDrivingDir,
                    const Vector2f& targetPos,
                    const Vector2f& targetHeading,
                    int32_t targetDrivingDir);

    void setResult(CarNode* resultNode);

    const char* toString(State s);

    int32_t m_maxNodeCount;
    int32_t m_currentNodeCount;
    float32_t m_cellSize;
    float32_t m_carTurningRadius;
    int32_t m_numSteeringAngles;
    int32_t m_substepCount;

    float32_t m_stepRotation;

    std::vector<float32_t> m_substepLengths;
    std::vector<Matrix2f> m_substepRotations;

    std::vector<float32_t> m_stepArcLengths;

    int32_t m_orientationBucketCount;
    float32_t m_orientationBucketSize;

    // cost weights
    float32_t m_distWeight;
    float32_t m_dirSwitchCost;
    float32_t m_backwardsMultiplier;

    float32_t m_angleTolerance;

    bool m_DStarNeedsUpdate;

    // search state variables
    Vector2f m_startPos;
    Vector2f m_targetPos;
    Vector2f m_targetHeading;
    int32_t m_targetDrivingDir;
    uint32_t m_reedsSheppN;
    CarNode* m_targetNode;
    CarNode* m_currentNode;

    int32_t m_numVisitedNodes;

    int32_t m_stepsSinceReedsShepp;

    // Hybrid A* data
    std::vector<CarNode> m_nodes;
    std::unique_ptr<
        std::unordered_map<CarNodeIndex, CarNode*, CarNodeHash, std::equal_to<CarNodeIndex>>>
        m_nodeMap;
    std::unique_ptr<std::priority_queue<std::pair<CarNode*, float32_t>,
                                        std::vector<std::pair<CarNode*, float32_t>>,
                                        CarNodePairCompare>>
        m_openSet;

    // function pointer for user side collision check
    IsDrivableCallback m_isDrivable;
    void* m_isDrivableUserData;

    bool isDrivable(const float32_t srcPosition[2],
                    const float32_t srcHeading[2],
                    const float32_t dstPosition_[2],
                    const float32_t dstHeading_[2]);

    ExternalCostCallback m_externalCost;
    void* m_externalCostUserData;

    std::shared_ptr<DStarLite> m_DStarLite;

    // Reeds-Shepp data
    ReedsShepp m_reedsShepp;
    // intermediate arrays
    float32_t m_RSAngleStepSize;
    float32_t m_RSMaxPathLength;
    std::vector<RSPoint> m_RSPath;

    // result nodes
    std::vector<CarNode> m_RSNodes;
    int32_t m_RSNodeCount;

    const CarNode* m_result;
    uint32_t m_pathNodeCount;

    bool m_initialReedsSheppPathFound;

    State m_state;

}; // class HybridAStar

#endif // HYBRIDASTAR_HPP_
