#include "HybridAStar.hpp"

/**
\brief Returns angle in [-PI,PI], positive values for clockwise rotation
*/
inline float32_t angleNormalizedVectors(const Vector2f& v1, const Vector2f& v2)
{
    float32_t dot = v1.dot(v2);

    if (dot > 1.0f)
        dot = 1.0f;
    if (dot < -1.0f)
        dot = -1.0f;

    float32_t angle = acosf(dot);

    // positive for clockwise rotation
    float32_t cross = v1.x() * v2.y() - v1.y() * v2.x();
    if (cross < 0.0f)
        angle *= -1;

    return angle;
}

// index computation of a node based on position, orientation and driving direction
CarNodeIndex CarNodeIndex::compute(float32_t cellSize, float32_t orientationBucketSize, int32_t numOrientationBuckets,
                                   const Vector2f& pos, const Vector2f& heading, int32_t drivingDir)
{
    CarNodeIndex index;

    // position indices
    float32_t halfCell = 0.5f * cellSize;
    index.x            = static_cast<int32_t>(floor((pos.x() + halfCell) / cellSize));
    index.y            = static_cast<int32_t>(floor((pos.y() + halfCell) / cellSize));

    // orientation index
    Vector2f up(0.0f, 1.0f);

    float32_t orientation = angleNormalizedVectors(heading, up);

    // first bucket is for angles pointing north
    orientation += orientationBucketSize / 2.0f;

    if (orientation < 0)
        orientation += static_cast<float>(2.0 * M_PI);

    float32_t orientationIndexFloat = orientation / orientationBucketSize;
    index.orientationIndex          = static_cast<int32_t>(orientationIndexFloat) % numOrientationBuckets;

    index.drivingDir = drivingDir;

    return index;
}

HybridAStar::HybridAStar()
    : m_maxNodeCount(0)
    , m_currentNodeCount(0)
    , m_cellSize(0.0f)
    , m_carTurningRadius(0.0f)
    , m_numSteeringAngles(1)
    , m_substepCount(1)
    , m_stepRotation(0.0f)
    , m_orientationBucketCount(0)
    , m_orientationBucketSize(0.0f)
    , m_distWeight(1.0f)
    , m_dirSwitchCost(0.0f)
    , m_backwardsMultiplier(1.0f)
    , m_angleTolerance(0.0f)
    , m_DStarNeedsUpdate(false)
    , m_startPos(0.0f, 0.0f)
    , m_targetPos(0.0f, 0.0f)
    , m_targetHeading(0.0f, 0.0f)
    , m_targetDrivingDir(0)
    , m_reedsSheppN(0)
    , m_targetNode(nullptr)
    , m_currentNode(nullptr)
    , m_numVisitedNodes(0)
    , m_stepsSinceReedsShepp(0)
    , m_isDrivable(nullptr)
    , m_isDrivableUserData(nullptr)
    , m_externalCost(nullptr)
    , m_externalCostUserData(nullptr)
    // , m_DStarLite(nullptr)
    , m_RSAngleStepSize(0.0f)
    , m_RSMaxPathLength(0.0f)
    , m_RSNodeCount(0)
    , m_result(nullptr)
    , m_pathNodeCount(0)
    , m_initialReedsSheppPathFound(false)
    , m_state(State::UNINITIALIZED)
{
}

HybridAStar::~HybridAStar()
{
    m_nodes.clear();
}

void HybridAStar::init(int32_t maxNumNodes,
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
                       float32_t maxPathLength)
{
    m_carTurningRadius = carTurningRadius;
    m_cellSize         = std::min(cellSize, 2.0f * carTurningRadius);

    if (numSteeringAngles < 1)
    {
        numSteeringAngles = 1;
    }

    m_substepCount = std::max(1, numSubsteps);

    // cost weights
    m_distWeight          = distWeight;
    m_dirSwitchCost       = dirSwitchCost;
    m_backwardsMultiplier = backwardsMultiplier;

    // compute angle between tangent and line to the target point on the radius circle:
    // * The angle between a chord and the tangent line at one of its intersection points
    //   equals half of the central angle subtended by the chord.
    // * central angle = 2 * asin(chord / 2*radius)
    float32_t maxStepRotation = asinf(m_cellSize / (2.0f * carTurningRadius));

    // split up the maximum possible rotation
    m_stepRotation = maxStepRotation / numSteeringAngles;

    // make sure the rotation steps are fine enough to reach the target
    m_angleTolerance = angleTolerance * static_cast<float>(M_PI / 180.0);
    if (m_angleTolerance < m_stepRotation)
    {
        numSteeringAngles = static_cast<int32_t>(ceil(maxStepRotation / m_angleTolerance));
        m_stepRotation    = maxStepRotation / numSteeringAngles;
    }
    m_numSteeringAngles = numSteeringAngles;

    // we need to make sure that we create a new node for each steering angle.
    // if the angle tolerance is very small, we need even more buckets
    float32_t bucketSize = m_stepRotation;
    float32_t numBuckets = static_cast<float>(2.0 * M_PI) / bucketSize;

    // round up to the next multiple of 4, so we have symmetric buckets for the 4 main directions
    m_orientationBucketCount = static_cast<int32_t>(4.0f * ceilf(numBuckets / 4.0f));
    m_orientationBucketSize  = static_cast<float>(2.0 * M_PI) / m_orientationBucketCount;

    // prepare angles for neighbor node search
    float32_t substepRotation = m_stepRotation / numSubsteps;
    m_substepRotations.clear();
    m_substepLengths.clear();
    m_stepArcLengths.clear();
    m_substepRotations.push_back(Matrix2f::Identity()); // straight
    m_substepLengths.push_back(m_cellSize / m_substepCount);
    m_stepArcLengths.push_back(m_cellSize);
    for (int32_t i = 1; i <= numSteeringAngles; ++i)
    {
        // compute the sizes of the substeps such that they reach
        // the same point on the circle as one cellsize step

        // compute radius from chord and angle (see comment above).
        // substepRotation is half the central angle.
        float32_t substepRadius = m_cellSize / (2 * sinf(i * substepRotation));

        // radius r, arclength a, chord c
        // a = 2 * r * asin(c / 2*r)
        // a_substep = b / numSubsteps
        float32_t substepLength = 2 * substepRadius * sinf(asinf(m_cellSize / (2 * substepRadius)) / m_substepCount);

        m_substepLengths.push_back(substepLength);
        m_substepLengths.push_back(substepLength);

        m_substepRotations.emplace_back(Rotation2D(-i * substepRotation)); // right
        m_substepRotations.emplace_back(Rotation2D(i * substepRotation));  // left

        // store the actual driven arc size of the whole step
        float32_t stepRadius = m_cellSize / (2 * sinf(i * m_stepRotation));
        float32_t arcLength  = stepRadius * 2 * i * m_stepRotation;
        m_stepArcLengths.push_back(arcLength);
        m_stepArcLengths.push_back(arcLength);
    }

    // preallocate space for nodes
    m_currentNodeCount = 0;
    m_maxNodeCount     = maxNumNodes;
    m_nodes.resize(maxNumNodes);
    constexpr float32_t HASH_CONTAINER_SIZE_MULTIPLIER = 1.5f;
    m_nodeMap.reset(new std::unordered_map<CarNodeIndex, CarNode*,
                                           CarNodeHash, std::equal_to<CarNodeIndex>>(HASH_CONTAINER_SIZE_MULTIPLIER * maxNumNodes));
    m_openSet.reset(new std::priority_queue<std::pair<CarNode*, float32_t>,
                                            std::vector<std::pair<CarNode*, float32_t>>,
                                            CarNodePairCompare>()); // HASH_CONTAINER_SIZE_MULTIPLIER * maxNumNodes

    // compute max num reeds shepp path nodes
    m_RSAngleStepSize   = 2.0f * asinf(m_cellSize / (2 * m_carTurningRadius));
    m_RSMaxPathLength   = maxPathLength;
    int32_t maxNumSteps = static_cast<int32_t>(ceil(2.0f * M_PI / m_RSAngleStepSize)) +
                          static_cast<int32_t>(ceil(m_RSMaxPathLength / m_cellSize));

    m_RSPath.resize(maxNumSteps);
    m_RSNodes.resize(maxNumSteps);

    // if (useHolonimicWithObstaclesHeuristic)
    // {
        m_DStarLite.reset(new DStarLite());
    //     m_DStarLite->init(maxNumNodes, cellSize, maxNumCollisionCells);
    // }
    // else
    // {
    //     m_DStarLite.reset();
    // }

    m_numVisitedNodes = 0;

    setResult(nullptr);
    m_state = State::READY;
}

void HybridAStar::setIsDrivableCallback(IsDrivableCallback callback, void* userData)
{
    m_isDrivable         = callback;
    m_isDrivableUserData = userData;
}

void HybridAStar::getIsDrivableCallback(IsDrivableCallback* callback, void** userData)
{
    *callback = m_isDrivable;
    *userData = m_isDrivableUserData;
}

void HybridAStar::setExternalCostCallback(ExternalCostCallback callback, void* userData)
{
    m_externalCost         = callback;
    m_externalCostUserData = userData;
}

void HybridAStar::getExternalCostCallback(ExternalCostCallback* callback, void** userData)
{
    *callback = m_externalCost;
    *userData = m_externalCostUserData;
}

// main function
HybridAStar::State HybridAStar::findPath(const Vector2f& startPos, const Vector2f& startHeading, DrivingState startDrivingDir,
                                         const Vector2f& targetPos, const Vector2f& targetHeading, DrivingState targetDrivingDir,
                                         uint32_t reedsSheppN,
                                         uint32_t maxNodeExpansionCount)
{
    if (m_state == State::IN_PROGRESS)
    {
        std::cout << "HybridAStar::findPath: An unfinished path search is being reset.";
    }
    m_state = State::IN_PROGRESS;

    // make sure we start with a clean state
    clear();

    // keep start pos for dStarLite path
    m_DStarNeedsUpdate           = true;
    m_startPos                   = startPos;
    m_initialReedsSheppPathFound = false;

    // store values for later continuation
    m_targetPos        = targetPos;
    m_targetHeading    = targetHeading;
    m_targetDrivingDir = targetDrivingDir;
    m_reedsSheppN      = reedsSheppN;

    // first try a reedsShepp path
    if (
        m_reedsSheppN > 0 &&
        reedsShepp(nullptr,
                   startPos, startHeading, startDrivingDir,
                   targetPos, targetHeading, targetDrivingDir))
    {

        CarNode* resultNode = (m_RSNodeCount > 0) ? &m_RSNodes[m_RSNodeCount - 1] : nullptr;
        setResult(resultNode);
        m_initialReedsSheppPathFound = true;
        m_state                      = State::READY;
        return m_state;
    }

    // check if reachable by D* path
    // if (m_DStarLite != nullptr)
    // {
    //     m_DStarLite->setTarget(targetPos);
    //     float32_t dStarLitePathLength = 0.0f;
    //     if (!m_DStarLite->findPath(dStarLitePathLength, startPos))
    //     {
    //         m_state = State::READY;
    //         return m_state;
    //     }
    // }

    // create target node

    // early out if target pos is not reachable
    // if (m_isDrivable != nullptr && !m_isDrivable(&targetPos.x(), &targetHeading.x(), &targetPos.x(), &targetHeading.x(), m_isDrivableUserData))
    if(!isDrivable(&targetPos.x(), &targetHeading.x(), &targetPos.x(), &targetHeading.x()))
    {
        setResult(nullptr);
        m_state = State::READY;
        std::cout << "target is not reachable" << std::endl;
        return m_state;
    }

    m_targetNode = createNode();
    m_targetNode->init(nullptr, computeIndex(targetPos, targetHeading, targetDrivingDir), targetPos, targetHeading, 0, 0.0f, 0.0f);

    // create start node
    CarNode* startNode        = createNode();
    float32_t initialEstimate = getEstimate(startPos, targetPos);
    startNode->init(nullptr, computeIndex(startPos, startHeading, startDrivingDir), startPos, startHeading, 0, 0.0f, initialEstimate);
    (*m_nodeMap)[startNode->getIndex()] = startNode;

    // search loop
    m_currentNode = startNode;
    searchLoop(maxNodeExpansionCount);
    return m_state;
}

HybridAStar::State HybridAStar::continueFindPath(uint32_t maxNodeExpansionCount)
{
    if (m_state != State::IN_PROGRESS)
    {
        std::cout << "HybridAStar::continueFindPath can only run when state is \"in progress\". Current state is "
                  << toString(m_state) << "\n";

        return m_state;
    }

    searchLoop(maxNodeExpansionCount);
    return m_state;
}

HybridAStar::State HybridAStar::getState() const
{
    return m_state;
}

const CarNode* HybridAStar::getResult() const
{
    return m_result;
}

uint32_t HybridAStar::getPathNodeCount() const
{
    return m_pathNodeCount;
}

// walk through result and return the path nodes
void HybridAStar::getPath(float32_t* pathPoints,
                          float32_t* pathHeadings,
                          DrivingState* pathDrivingDirs,
                          int32_t numPoints)
{
    const CarNode* node = getResult();

    if (node != nullptr)
    {
        int32_t startIndex = (numPoints < static_cast<int32_t>(m_pathNodeCount)) ? numPoints - 1 : m_pathNodeCount - 1;
        for (int32_t i = startIndex; i >= 0; --i)
        {
            if (pathPoints != nullptr)
            {
                Vector2f pos          = node->getPos();
                pathPoints[2 * i + 0] = pos.x();
                pathPoints[2 * i + 1] = pos.y();
            }

            if (pathHeadings != nullptr)
            {
                Vector2f heading        = node->getHeading();
                pathHeadings[2 * i + 0] = heading.x();
                pathHeadings[2 * i + 1] = heading.y();
            }

            if (pathDrivingDirs != nullptr)
            {
                pathDrivingDirs[i] = static_cast<DrivingState>(node->getIndex().drivingDir);
            }

            node = node->getPrevious();

            if (node == nullptr)
                break;
        }
    }
}

void HybridAStar::getSearchLines(float32_t* searchLines, uint32_t* searchLineCount)
{
    int32_t numCreatedNodes = getCreatedNodeCount();
    uint32_t lineCount      = numCreatedNodes > 0 ? numCreatedNodes - 2 : 0; // all nodes except start and target

    *searchLineCount = std::min(lineCount, *searchLineCount);

    if (searchLines != nullptr)
    {
        const CarNode* nodes = getCreatedNodes();
        uint32_t numNodes    = getCreatedNodeCount();
        unsigned nodeIndex   = 0;
        unsigned lineIndex   = 0;
        while (lineIndex < *searchLineCount && nodeIndex < numNodes)
        {
            const CarNode* node = nodes + nodeIndex;

            if (node && node->getPrevious())
            {
                searchLines[4 * lineIndex + 0] = node->getPos().x();
                searchLines[4 * lineIndex + 1] = node->getPos().y();
                searchLines[4 * lineIndex + 2] = node->getPrevious()->getPos().x();
                searchLines[4 * lineIndex + 3] = node->getPrevious()->getPos().y();

                ++lineIndex;
            }

            ++nodeIndex;
        }
    }

    *searchLineCount = lineCount;
}

int32_t HybridAStar::getCreatedNodeCount() const
{
    return m_currentNodeCount;
}

const CarNode* HybridAStar::getCreatedNodes() const
{
    return (m_nodes.size() > 0) ? &m_nodes[0] : nullptr;
}

int32_t HybridAStar::getOrientationBucketCount() const
{
    return m_orientationBucketCount;
}

void HybridAStar::clear()
{
    // clear state
    m_currentNodeCount                                 = 0;
    m_numVisitedNodes                                  = 0;
    constexpr float32_t HASH_CONTAINER_SIZE_MULTIPLIER = 1.5f;
    m_nodeMap.reset(new std::unordered_map<CarNodeIndex, CarNode*,
                                           CarNodeHash, std::equal_to<CarNodeIndex>>(HASH_CONTAINER_SIZE_MULTIPLIER * m_maxNodeCount));
    m_openSet.reset(new std::priority_queue<std::pair<CarNode*, float32_t>,
                                            std::vector<std::pair<CarNode*, float32_t>>,
                                            CarNodePairCompare>()); // HASH_CONTAINER_SIZE_MULTIPLIER * maxNumNodes

    m_stepsSinceReedsShepp = 0;

    m_targetNode  = nullptr;
    m_currentNode = nullptr;
    setResult(nullptr);
}

void HybridAStar::searchLoop(uint32_t maxNodeExpansionCount)
{
    bool done                   = true; // flags whether search is finished in this function call
    bool ok                     = true; // flags whether an error occurred during the search (usually when running out of nodes)
    uint32_t nodeExpansionCount = 0;
    while (ok && m_currentNode != nullptr)
    {
        ++nodeExpansionCount;

        // track searched nodes for debugging
        m_currentNode->setVisitIndex(m_numVisitedNodes);
        ++m_numVisitedNodes;

        if (isTarget(*m_currentNode))
        {
            break;
        }

        // reeds-shepp test
        if (m_reedsSheppN > 0)
        {
            float32_t estimatedFractionOfPath = (m_currentNode->getEstimate() - m_currentNode->getCost()) / m_currentNode->getEstimate();

            int32_t nextRSStep = static_cast<int32_t>(m_reedsSheppN * estimatedFractionOfPath);
            if (nextRSStep <= m_stepsSinceReedsShepp)
            {
                m_stepsSinceReedsShepp = 0;
                if (reedsShepp(m_currentNode,
                               m_currentNode->getPos(), m_currentNode->getHeading(), m_currentNode->getIndex().drivingDir,
                               m_targetPos, m_targetHeading, m_targetDrivingDir))
                {
                    break;
                }
            }
            else
            {
                ++m_stepsSinceReedsShepp;
            }
        }

        ok &= createNeighbors(*m_currentNode, false);
        ok &= createNeighbors(*m_currentNode, true);
        m_currentNode = getNextNode();

        if (maxNodeExpansionCount > 0 && nodeExpansionCount >= maxNodeExpansionCount)
        {
            done = false;
            break;
        }
    }

    if (done)
    {
        // search is finished

        // Reeds-Shepp node if available, Hybrid-A* target node otherwise, nullptr if nothing found.
        CarNode* resultNode = (m_RSNodeCount > 0) ? &m_RSNodes[m_RSNodeCount - 1] : ok ? m_currentNode : nullptr;
        setResult(resultNode);
        m_currentNode = nullptr;
        m_state       = State::READY;
    }
    else
    {
        m_state = State::IN_PROGRESS;
    }
}

CarNodeIndex HybridAStar::computeIndex(const Vector2f& pos, const Vector2f& heading, int32_t drivingDir) const
{
    return CarNodeIndex::compute(m_cellSize, m_orientationBucketSize, m_orientationBucketCount,
                                 pos, heading, drivingDir);
}

CarNode* HybridAStar::getNode(const CarNodeIndex& nodeIndex)
{
    auto iter = m_nodeMap->find(nodeIndex);

    CarNode* node = nullptr;
    if (iter != m_nodeMap->end())
    {
        node = iter->second;
    }

    return node;
}

CarNode* HybridAStar::createNode()
{
    CarNode* node = nullptr;
    if (m_currentNodeCount < m_maxNodeCount)
    {
        node = &m_nodes[m_currentNodeCount++];
    }
    return node;
}

// node expansion
bool HybridAStar::createNeighbors(CarNode& n, bool backward)
{
    const Vector2f heading = n.getHeading();
    const Vector2f pos     = n.getPos();
    const int32_t x        = n.getIndex().x;
    const int32_t y        = n.getIndex().y;

    // for each steering angle
    assert(m_substepLengths.size() == m_substepRotations.size());
    assert(m_stepArcLengths.size() == m_substepRotations.size());
    for (size_t i = 0; i < m_substepRotations.size(); ++i)
    {
        float32_t distance         = 0.0f;
        int32_t steeringAngleIndex = ((i % 2 == 1) ? -(static_cast<int32_t>(i) + 1) / 2 : (static_cast<int32_t>(i) + 1) / 2);
        float32_t angleStep        = steeringAngleIndex * m_stepRotation;
        float32_t angle            = 0.0f;
        Vector2f newHeading        = heading;
        Vector2f newPos            = pos;
        CarNodeIndex newIndex;
        bool found    = false;
        bool drivable = true;
        while (!found && drivable)
        {
            distance += m_stepArcLengths[i];
            angle += angleStep;

            for (int32_t substep = 0; substep < m_substepCount; ++substep)
            {
                // continue until we're in a new cell
                Vector2f prevPos     = newPos;
                Vector2f prevHeading = newHeading;
                const Matrix2f rot   = backward ? m_substepRotations[i].transpose() : // rotation matrix: inverse == transposed
                                         m_substepRotations[i];
                newHeading = rot * newHeading; // step vector

                Vector2f substepVector = newHeading * m_substepLengths[i];
                if (backward)
                {
                    newPos -= substepVector;
                }
                else
                {
                    newPos += substepVector;
                }
                newHeading = rot * newHeading; // new tangent

                {
                    // we also have to check the dStarLite isDrivable function because
                    // we have to ensure that only nodes are created that are also valid dStar nodes,
                    // otherwise it could happen that we search for an unreachable node in getEstimate.
                    if ((m_DStarLite != nullptr && !m_DStarLite->getCollisionGrid()->isDrivable(newPos)) ||
                        // (m_isDrivable != nullptr &&
                        //  !m_isDrivable(&prevPos.x(), &prevHeading.x(),
                        //                &newPos.x(), &newHeading.x(), m_isDrivableUserData)))
                       (!isDrivable(&prevPos.x(), &prevHeading.x(), &newPos.x(), &newHeading.x())))
                    {
                        drivable = false;
                        break;
                    }
                }

                newIndex = computeIndex(newPos, newHeading, backward ? -1 : 1);
            }

            if (drivable && (newIndex.x != x || newIndex.y != y))
            {
                // we're in a new cell now
                found = true;
                break;
            }
        }

        if (!found)
            continue;

        // get existing node if it already exists
        CarNode* neighbor = getNode(newIndex);

        if (neighbor != nullptr && neighbor->getVisitIndex() > 0)
            continue;

        // update cost of this neighbor
        float32_t edgeCost = getEdgeCost(distance, n.getIndex().drivingDir, newIndex.drivingDir);

        if (m_externalCost != nullptr)
        {
            // add external cost of the node if callback is provided
            edgeCost += m_externalCost(&newPos.x(), &newHeading.x(), m_externalCostUserData);
        }

        float32_t newCost = n.getCost() + edgeCost;
        if (neighbor == nullptr || newCost < neighbor->getCost())
        {
            if (neighbor == nullptr)
            {
                // create and add new node
                neighbor = createNode();
                if (neighbor == nullptr)
                    return false;

                (*m_nodeMap)[newIndex] = neighbor;
            }

            // add new node to open set
            float32_t estimate = newCost + getEstimate(newPos, m_targetNode->getPos());
            neighbor->init(&n, newIndex, newPos, newHeading, steeringAngleIndex, newCost, estimate);

            m_openSet->push(std::pair<CarNode*, float32_t>(neighbor, neighbor->getEstimate()));
        }
    }

    return true;
}

// get the next node from the priority queue with the minimum total estimate
CarNode* HybridAStar::getNextNode()
{
    if (m_openSet->size() == 0)
        return nullptr;

    std::pair<CarNode*, float32_t> next = m_openSet->top();
    m_openSet->pop();

    while (!isEqual(next.second, next.first->getEstimate())) // out of date
    {
        next = m_openSet->top();
        m_openSet->pop();
    }

    return next.first;
}

float32_t HybridAStar::getDistance(const CarNode& n1, const CarNode& n2) const
{
    return (n2.getPos() - n1.getPos()).norm();
}

float32_t HybridAStar::getEdgeCost(float32_t dist, int32_t drivingDir1, int32_t drivingDir2) const
{
    float32_t cost = m_distWeight * dist;

    // punish direction switch
    if (abs(drivingDir2 - drivingDir1) == 2)
    {
        cost += m_dirSwitchCost;
    }

    // punish backward driving
    if (drivingDir2 == -1)
    {
        cost *= m_backwardsMultiplier;
    }

    return cost;
}

// heuristic to direct the node expasion towards the target
float32_t HybridAStar::getEstimate(const Vector2f& pos, const Vector2f& posTarget)
{
    float32_t estimate = (pos - posTarget).norm();

    // if (m_DStarLite != nullptr)
    // {
    //     float32_t holonomicCost = 0.0f;
    //     if (!m_DStarLite->findPath(holonomicCost, pos))
    //     {
    //         // estimate must be optimistic in A* search
    //         holonomicCost = 0.0f;
    //     }
    //     estimate = std::max(estimate, holonomicCost);
    // }

    return estimate;
}

bool HybridAStar::isTarget(const CarNode& n) const
{
    CarNodeIndex nodeIndex   = n.getIndex();
    CarNodeIndex targetIndex = m_targetNode->getIndex();

    if (nodeIndex.x != targetIndex.x || nodeIndex.y != targetIndex.y ||
        (targetIndex.drivingDir != 0 && nodeIndex.drivingDir != targetIndex.drivingDir))
    {
        return false;
    }

    float32_t angle = acosf(n.getHeading().dot(m_targetNode->getHeading()));

    return angle < m_angleTolerance;
}

// evaluate a ReedsShepp path and check the result for collisions
bool HybridAStar::reedsShepp(CarNode* startNode,
                             const Vector2f& startPos, const Vector2f& startHeading, int32_t startDrivingDir,
                             const Vector2f& targetPos, const Vector2f& targetHeading, int32_t targetDrivingDir)
{
    // Prepare input data

    // heading==(1.0f, 0.0f) corresponds to theta=0.0f, positive for counter-clock rotation
    Vector2f referenceVec(1.0f, 0.0f);
    float32_t sourceTheta = -angleNormalizedVectors(startHeading, referenceVec);
    float32_t destTheta   = -angleNormalizedVectors(targetHeading, referenceVec);

    uint32_t pathPointCount = 0;
    float64_t pathLength    = std::numeric_limits<float64_t>::max();

    RSPoint startState;
    startState.x           = startPos.x();
    startState.y           = startPos.y();
    startState.orientation = sourceTheta;
    startState.direction   = static_cast<RSDirection>(startDrivingDir);

    RSPoint targetState;

    targetState.x           = targetPos.x();
    targetState.y           = targetPos.y();
    targetState.orientation = destTheta;
    targetState.direction   = static_cast<RSDirection>(targetDrivingDir);

    // run reeds-shepp path search
    m_reedsShepp.setParameters(m_carTurningRadius, m_backwardsMultiplier, m_dirSwitchCost);
    m_reedsShepp.evaluateRS(&m_RSPath[0], &pathPointCount, &pathLength, startState, targetState, m_cellSize,
                            static_cast<uint32_t>(m_RSPath.size()));

    if (static_cast<size_t>(pathPointCount) > m_RSNodes.size())
        return false;

    // transcribe output into a CarNode array
    m_RSNodeCount     = 0;
    CarNode* previous = startNode;

    if (startNode == nullptr && pathPointCount > 0)
    {
        CarNode* n   = &m_RSNodes[m_RSNodeCount++];
        Vector2f pos = Vector2f(static_cast<float32_t>(m_RSPath[0].x), static_cast<float32_t>(m_RSPath[0].y));

        auto rotation    = Rotation2D(static_cast<float32_t>(m_RSPath[0].orientation));
        Vector2f heading = rotation * Vector2f(1.0f, 0.0f);
        n->init(previous, CarNodeIndex(computeIndex(pos, heading, static_cast<int32_t>(m_RSPath[0].direction))), pos, heading, 0,
                static_cast<float32_t>(pathLength), static_cast<float32_t>(pathLength));
        previous = n;
    }

    for (uint32_t i = 1; i < pathPointCount; i++)
    {
        CarNode* n = &m_RSNodes[m_RSNodeCount++];

        Vector2f pos = Vector2f(static_cast<float32_t>(m_RSPath[i].x), static_cast<float32_t>(m_RSPath[i].y));

        auto rotation    = Rotation2D(static_cast<float32_t>(m_RSPath[i].orientation));
        Vector2f heading = rotation * Vector2f(1.0f, 0.0f);

        n->init(previous, CarNodeIndex(computeIndex(pos, heading, static_cast<int32_t>(m_RSPath[i].direction))), pos, heading, 0,
                static_cast<float32_t>(pathLength), static_cast<float32_t>(pathLength));

        Vector2f previousPos = previous != nullptr ? previous->getPos() : Vector2f(static_cast<float32_t>(m_RSPath[i - 1].x), static_cast<float32_t>(m_RSPath[i - 1].y));

        Vector2f previousHeading;
        if (previous != nullptr)
        {
            previousHeading = previous->getHeading();
        }
        else
        {
            auto rotation_  = Rotation2D(static_cast<float32_t>(m_RSPath[i - 1].orientation));
            previousHeading = rotation_ * Vector2f(1.0f, 0.0f);
        }
        // if (m_isDrivable != nullptr &&
        //     m_isDrivable(&previousPos.x(), &previousHeading.x(), &pos.x(), &heading.x(),
        //                  m_isDrivableUserData) == false)
        if(isDrivable(&previousPos.x(), &previousHeading.x(), &pos.x(), &heading.x()) == false)
        {
            m_RSNodeCount = 0;
            return false;
        }

        previous = n;
    }

    return true;
}

// store the result for later access
void HybridAStar::setResult(CarNode* node)
{
    m_result = node;

    // count the path nodes
    m_pathNodeCount = 0;
    if (node != nullptr)
    {
        m_pathNodeCount = 1;
        while (node->getPrevious())
        {
            ++m_pathNodeCount;
            node = node->getPrevious();
        }
    }
}

const char* HybridAStar::toString(State s)
{
    const char* result = nullptr;
    switch (s)
    {
    case State::UNINITIALIZED:
        result = "Uninitialized";
        break;
    case State::READY:
        result = "Ready";
        break;
    case State::IN_PROGRESS:
        result = "In Progress";
        break;
    default:
        throw std::runtime_error("Unexpected enum value.");
    }
    return result;
}

template <class Container, class ContainerElement>
class IsNotInContainer
{
private:
    // not assignable due to reference member
    void operator=(const IsNotInContainer& other);

public:
    IsNotInContainer(const Container& container)
        : mContainer(container)
    {
    }

    bool operator()(const ContainerElement& element)
    {
        return mContainer.find(element) == mContainer.end();
    }

private:
    const Container& mContainer;
};

// see paper "A Fast Voxel Traversal Algorithm"
// this ensures we sample all cells that the line touches
template <class CellAction>
bool lineVoxelized(const Vector2f& srcPos, const Vector2f& dstPos, float cellSize, CellAction op)
{
    Coord2d coord(srcPos, cellSize);
    Coord2d endCoord(dstPos, cellSize);

    Vector2f srcCellMinPos = coord.getMin(cellSize);
    Vector2f srcCellMaxPos = coord.getMax(cellSize);

    // step directions
    int stepX = srcPos.x() < dstPos.x() ? 1 : -1;
    int stepY = srcPos.y() < dstPos.y() ? 1 : -1;

    // x
    float deltaX      = abs(dstPos.x() - srcPos.x());
    float tDeltaX     = deltaX != 0.0f ? cellSize / deltaX : 0.0f;
    float cellBorderX = stepX > 0 ? srcCellMaxPos.x() : srcCellMinPos.x();
    float tMaxX       = deltaX != 0.0f ? abs(cellBorderX - srcPos.x()) / deltaX : std::numeric_limits<float32_t>::max();

    // y
    float deltaY      = abs(dstPos.y() - srcPos.y());
    float tDeltaY     = deltaY != 0.0f ? cellSize / deltaY : 0.0f;
    float cellBorderY = stepY > 0 ? srcCellMaxPos.y() : srcCellMinPos.y();
    float tMaxY       = deltaY != 0.0f ? abs(cellBorderY - srcPos.y()) / deltaY : std::numeric_limits<float32_t>::max();

    bool ok = false;
    for (;;)
    {
        ok = op(coord);
        if (!ok || (tMaxX >= 1.0f && tMaxY >= 1.0f))
            break;

        if (tMaxX < tMaxY)
        {
            tMaxX += tDeltaX;
            coord.x += stepX;
        }
        else
        {
            tMaxY += tDeltaY;
            coord.y += stepY;
        }
    }

    return ok;
}

bool HybridAStar::isDrivable(const float32_t srcPosition[2],
                             const float32_t srcHeading[2],
                             const float32_t dstPosition_[2],
                             const float32_t dstHeading_[2])
{
    float32_t CarLength = 5.0f;
    float32_t CarWidth  = 2.0f;

    Vector2f dstPosition(dstPosition_[0], dstPosition_[1]);
    Vector2f dstHeading(dstHeading_[0], dstHeading_[1]);

    auto mRot90 = Eigen::Rotation2Df(static_cast<float>(M_PI_2)).matrix();

    Vector2f sideVector = mRot90 * dstHeading;

    Vector2f corners[5] = {
        dstPosition - dstHeading * 0.5f * CarLength - sideVector * 0.5f * CarWidth,
        dstPosition - dstHeading * 0.5f * CarLength + sideVector * 0.5f * CarWidth,
        dstPosition + dstHeading * 0.5f * CarLength + sideVector * 0.5f * CarWidth,
        dstPosition + dstHeading * 0.5f * CarLength - sideVector * 0.5f * CarWidth};
    corners[4] = corners[0];

    auto collision = getDStarLite()->getCollisionGrid()->getCells();
    for (int i = 0; i < 4; ++i)
    {
        if (!lineVoxelized(corners[i],
                           corners[i + 1],
                           getDStarLite()->getCollisionGrid()->getCellSize(),
                           IsNotInContainer<std::unordered_set<Coord2d, Coord2d, std::equal_to<Coord2d>>, Coord2d>(*collision)))
        {
            return false;
        }
    }

    return true;

}