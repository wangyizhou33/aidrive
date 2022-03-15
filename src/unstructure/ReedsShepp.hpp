#ifndef REEDSSHEPP_HPP_
#define REEDSSHEPP_HPP_

#include <aidrive/Types.hpp>

enum RSCurveTransform
{
    NONE       = 0,
    TIMEFLIP   = 1 << 0,
    REFLECTION = 1 << 1,

    TIMEFLIP_AND_REFLECTION = TIMEFLIP | REFLECTION
};

enum class RSSteering
{
    LEFT     = -1,
    STRAIGHT = 0,
    RIGHT    = 1
};

enum class RSDirection : int32_t
{
    BACKWARD = -1,
    NONE     = 0,
    FORWARD  = 1
};

struct RSCurve
{
    RSCurve()
    {
        for (uint32_t i = 0; i < 5; ++i)
        {
            steerings[i]      = RSSteering::STRAIGHT;
            directions[i]     = RSDirection::NONE;
            segmentLengths[i] = 0.0;
        }
    }

    uint32_t segmentCount{0};
    RSSteering steerings[5]{};
    RSDirection directions[5]{};
    float64_t segmentLengths[5]{};
    float64_t cost{std::numeric_limits<float64_t>::max()};
};

struct RSPoint
{
    float64_t x;
    float64_t y;
    float64_t orientation;
    RSDirection direction;
};

class ReedsShepp
{
public:
    ReedsShepp() = default;

    ReedsShepp(float64_t radius, float64_t backwardsWeight, float64_t dirSwitchCost)
    {
        setParameters(radius, backwardsWeight, dirSwitchCost);
    }

    // set the turning radius and cost weights
    void setParameters(float64_t radius, float64_t backwardsWeight, float64_t dirSwitchCost);

    // evaluate all ReedsShepp paths and select the optimal one
    void evaluateRS(RSPoint* path, uint32_t* pathPointCount, float64_t* pathCost,
                    const RSPoint& startState, const RSPoint& targetState,
                    float64_t lineStepSize, uint32_t maxPathPointCount) const;

    // evaluate path sampler Reeds Shepp (Dubins) curves and select the optimal one if exists
    // (only LSL, LSR with reflection)
    bool evaluateDubins(RSPoint* path, uint32_t* pathPointCount, float64_t* pathCost,
                        const RSPoint& startState, const RSPoint& targetState,
                        float64_t lineStepSize, uint32_t maxPathPointCount) const;

    // select optimal Reeds-Shepp curve from Reeds-Shepp curve set
    void selectRSCurve(RSCurve* curve, const RSPoint& startState, const RSPoint& targetState) const;

    // select optimal path sample curve from Dubins curve set
    bool selectDubins(RSCurve* curve, const RSPoint& startState, const RSPoint& targetState) const;

    // discretize a given Reeds-Shepp curve
    uint32_t discretizeRSCurve(RSPoint* path,
                               const RSCurve& curve,
                               const RSPoint& startState,
                               float64_t lineStepSize,
                               uint32_t maxPathPointCount) const;

    // set Reeds-Shepp curve radius
    template <class T>
    inline void setRadius(T r)
    {
        m_radius = static_cast<float64_t>(r);
    }

    // get Reeds-Shepp curve radius
    inline float64_t getRadius() const
    {
        return m_radius;
    }

protected:
    // select optimal RSCurve with minimal cost from the set described by curveTransform
    // note: 'curve' is only updated if an RSCurve is found with cost smaller than 'curve->cost'
    void selectRSCurve(RSCurve* curve, const RSPoint& startState, const RSPoint& targetState, RSCurveTransform curveTransform) const;

    // Transform angle into range [0,2*PI)
    // Note: The Reeds-Shepp paper defines it to transform into [-pi,pi)
    // We prefer positive angles, so we can also consider turns that are bigger
    // than half a circle. This can be needed when backwards driving has higher cost.
    // LaValle's implementation uses positive angles.
    float64_t mod2pi(float64_t angle) const;

    // switch LEFT and RIGHT
    void reflect(RSSteering outSteerings[5], const RSSteering inSteerings[5]) const;

    // switch FORWARD and BACKWARD
    void timeflip(RSDirection outDirections[5], const RSDirection inDirections[5]) const;

    // Adapt input parameters x, y and phi according to the given curve transform.
    // See Reeds-Shepp paper, beginning of section 8.
    void transformInputParams(float64_t* x, float64_t* y, float64_t* phi, RSCurveTransform curveTransform) const;

    // modify curve according to transform (reflect / timeflip)
    void modifyCurve(RSCurve* curve, RSCurveTransform curveTransform) const;

    // Compute cost of a curve.
    // Note: Reeds-Shepp assumes cost to be the length of the curve. We have modified this to allow
    // weighting of driving direction and to punish direction switches. We have not verified that the
    // Reeds-Shepp set of curves is still sufficient. That is, it is not clear that when using cost weights,
    // the optimal paths is within the Reeds-Shepp set.
    float64_t computeCost(const RSCurve* curve, RSDirection startDir, RSDirection endDir) const;

    // curve construction

private:
    bool LSL(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
             RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool LSR(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
             RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool L_R_L(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
               RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool L_RL(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
              RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool LR_L(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
              RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool LR_LR(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
               RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool L_RL_R(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool L_RSL(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
               RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool L_RSR(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
               RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool LSR_L(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
               RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool LSL_R(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
               RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;
    bool L_RSL_R(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                 RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const;

private:
    // parameter computations
    void LSL_computeParams(float64_t* t, float64_t* u, float64_t* v,
                           float64_t x, float64_t y, float64_t phi) const;
    bool LSR_computeParams(float64_t* t, float64_t* u, float64_t* v,
                           float64_t x, float64_t y, float64_t phi) const;
    bool L_R_L_computeParams(float64_t* t, float64_t* u, float64_t* v,
                             float64_t x, float64_t y, float64_t phi) const;
    bool L_RL_computeParams(float64_t* t, float64_t* u, float64_t* v,
                            float64_t x, float64_t y, float64_t phi) const;
    bool LR_LR_computeParams(float64_t* t, float64_t* u, float64_t* v,
                             float64_t x, float64_t y, float64_t phi) const;
    bool L_RL_R_computeParams(float64_t* t, float64_t* u, float64_t* v,
                              float64_t x, float64_t y, float64_t phi) const;
    bool L_RSR_computeParams(float64_t* t, float64_t* u, float64_t* v,
                             float64_t x, float64_t y, float64_t phi) const;
    bool L_RSL_computeParams(float64_t* t, float64_t* u, float64_t* v,
                             float64_t x, float64_t y, float64_t phi) const;
    bool L_RSL_R_computeParams(float64_t* t, float64_t* u, float64_t* v,
                               float64_t x, float64_t y, float64_t phi) const;

    // curve discretization

    // adds point along a line, starting at startPoint. startPoint is not added.
    uint32_t discretizeLine(RSPoint* points, uint32_t maxPointCount,
                            const RSPoint& startPoint, float64_t length, float64_t stepSize,
                            RSDirection direction) const;

    // adds point along a circle arc, starting at startPoint. startPoint is not added.
    uint32_t discretizeArc(RSPoint* points, uint32_t maxPointCount,
                           const RSPoint& startPoint, float64_t radius, float64_t length, float64_t stepSize,
                           RSSteering steering, RSDirection direction) const;

    void pointOnCircle(float64_t* x, float64_t* y, float64_t xCenter, float64_t yCenter, float64_t radius,
                       float64_t angle) const;

    // function to check whether yaw rate is positive or not
    bool isYawRatePositive(RSSteering steering, RSDirection direction) const;

    // function to transform original states to the coordinate system with start state as the origin and scaled down dimensions
    bool transformCoordinate(float64_t& x,
                             float64_t& y,
                             float64_t& phi,
                             const RSPoint& startState,
                             const RSPoint& targetState) const;

private:
    float64_t m_radius{1.0f};
    float64_t m_backwardsMultiplier{1.0f};
    float64_t m_dirSwitchCost{0.0f};
};

#endif // REEDSSHEPP_HPP_