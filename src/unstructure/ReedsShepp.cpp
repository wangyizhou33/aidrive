#include "ReedsShepp.hpp"
#include <stdexcept> // runtime_error

static constexpr float64_t EPS = std::numeric_limits<float64_t>::epsilon();

float64_t ReedsShepp::mod2pi(float64_t angle) const
{
    while (angle < 0.0)
    {
        angle = angle + 2.0 * M_PI;
    }

    while (angle >= 2.0 * M_PI)
    {
        angle = angle - 2.0 * M_PI;
    }

    return angle;
}

void ReedsShepp::reflect(RSSteering outSteerings[5], const RSSteering inSteerings[5]) const
{
    for (uint32_t i = 0; i < 5; ++i)
    {
        if (inSteerings[i] == RSSteering::LEFT)
        {
            outSteerings[i] = RSSteering::RIGHT;
        }
        else if (inSteerings[i] == RSSteering::RIGHT)
        {
            outSteerings[i] = RSSteering::LEFT;
        }
        else
        {
            outSteerings[i] = RSSteering::STRAIGHT;
        }
    }
}

void ReedsShepp::timeflip(RSDirection outDirections[5], const RSDirection inDirections[5]) const
{
    for (uint32_t i = 0; i < 5; ++i)
    {
        if (inDirections[i] == RSDirection::FORWARD)
        {
            outDirections[i] = RSDirection::BACKWARD;
        }
        else if (inDirections[i] == RSDirection::BACKWARD)
        {
            outDirections[i] = RSDirection::FORWARD;
        }
    }
}

void ReedsShepp::transformInputParams(float64_t* x, float64_t* y, float64_t* phi,
                                      RSCurveTransform curveTransform) const
{
    if (curveTransform & RSCurveTransform::TIMEFLIP)
    {
        *x   = -*x;
        *phi = -*phi;
    }
    if (curveTransform & RSCurveTransform::REFLECTION)
    {
        *y   = -*y;
        *phi = -*phi;
    }
}

void ReedsShepp::modifyCurve(RSCurve* curve, RSCurveTransform curveTransform) const
{
    if (curveTransform & RSCurveTransform::TIMEFLIP)
    {
        timeflip(curve->directions, curve->directions);
    }
    if (curveTransform & RSCurveTransform::REFLECTION)
    {
        reflect(curve->steerings, curve->steerings);
    }
}

float64_t ReedsShepp::computeCost(const RSCurve* curve, RSDirection startDir, RSDirection endDir) const
{
    float64_t cost = 0.0;

    // add segment lengths, weighted by driving direction
    for (uint32_t i = 0; i < curve->segmentCount; ++i)
    {
        float64_t w = curve->directions[i] == RSDirection::BACKWARD ? m_backwardsMultiplier : 1.0;

        // we need to multiply m_radius here, so that relation to m_dirSwitchCost is correct
        cost += w * m_radius * curve->segmentLengths[i];
    }

    // count direction switches
    // check if direction switch is necessary at start
    uint32_t dirSwitchCount = 0;
    if (startDir != RSDirection::NONE && startDir != curve->directions[0])
    {
        ++dirSwitchCount;
    }

    // check if direction switch is necessary at end
    if (endDir != RSDirection::NONE && endDir != curve->directions[curve->segmentCount - 1])
    {
        ++dirSwitchCount;
    }

    for (uint32_t i = 1; i < curve->segmentCount; ++i)
    {
        if (curve->directions[i] != curve->directions[i - 1])
        {
            ++dirSwitchCount;
        }
    }

    cost += dirSwitchCount * m_dirSwitchCost;

    return cost;
}

//-----------------------------------------------------------------------------------------------------------
// ( C S C ) paths, formula 8.1 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
void ReedsShepp::LSL_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                   float64_t x, float64_t y, float64_t phi) const
{
    // find outer tangent between 2 circles to compute the curve segments t, u, v
    float64_t xi  = x - std::sin(phi);
    float64_t eta = y - 1 + std::cos(phi);

    // cartesian to polar transform
    *t = mod2pi(std::atan2(eta, xi));    // angle
    *u = std::sqrt(xi * xi + eta * eta); // distance
    *v = mod2pi(phi - *t);               // t + v = phi
}

bool ReedsShepp::LSL(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                     RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    LSL_computeParams(&t, &u, &v, x, y, phi);

    curve->segmentCount = 3;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::STRAIGHT;
    curve->steerings[2] = RSSteering::LEFT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::FORWARD;
    curve->directions[2] = RSDirection::FORWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C S C ) paths, formula 8.2 in Reeds-Shepp paper
// Typo in paper: should be (u1, t1) = ... instead of (u1, t)
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::LSR_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                   float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x + std::sin(phi);
    float64_t eta = y - 1.0 - std::cos(phi);

    float64_t u1 = std::sqrt(xi * xi + eta * eta); // cartesian to polar: distance
    if (u1 < 2.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi); // cartesian to polar: angle

    *u              = std::sqrt(u1 * u1 - 4.0);
    float64_t alpha = std::atan2(2.0, *u); // cartesian to polar: angle
    *t              = mod2pi(theta + alpha);
    *v              = mod2pi(*t - phi);

    return true;
}

bool ReedsShepp::LSR(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                     RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!LSR_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 3;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::STRAIGHT;
    curve->steerings[2] = RSSteering::RIGHT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::FORWARD;
    curve->directions[2] = RSDirection::FORWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C | C | C ) paths. Formula 8.3 in paper is not correct.
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::L_R_L_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                     float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x - std::sin(phi);
    float64_t eta = y - 1.0 + std::cos(phi);

    if ((std::abs(xi) < EPS) && (std::abs(eta) < EPS))
    {
        return false;
    }

    float64_t u1 = std::sqrt(xi * xi + eta * eta);
    if (u1 > 4.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);
    float64_t alpha = std::acos(u1 / 4.0);
    *t              = mod2pi(M_PI_2 + alpha + theta);
    *u              = mod2pi(M_PI - 2 * alpha);
    *v              = mod2pi(phi - *t - *u); // all arc lengths add up to orientation difference: t + u + v = phi

    return true;
}

bool ReedsShepp::L_R_L(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                       RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_R_L_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 3;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::LEFT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::BACKWARD;
    curve->directions[2] = RSDirection::FORWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);
    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C | C C ) paths, formula 8.4 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::L_RL_computeParams(float64_t* t, float64_t* u, float64_t* v, float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x - std::sin(phi);
    float64_t eta = y - 1.0 + std::cos(phi);
    if ((std::abs(xi) < EPS) && (std::abs(eta) < EPS))
    {
        return false;
    }

    float64_t u1 = std::sqrt(xi * xi + eta * eta);
    if (u1 > 4.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);
    float64_t alpha = std::acos(u1 / 4.0);
    *t              = mod2pi(M_PI_2 + alpha + theta);
    *u              = mod2pi(M_PI - 2 * alpha);
    *v              = mod2pi(*t + *u - phi);

    return true;
}

bool ReedsShepp::L_RL(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                      RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RL_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 3;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::LEFT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::BACKWARD;
    curve->directions[2] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C C | C ) paths, formula 8.4 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::LR_L(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                      RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    // use L_RL formula with backwards transform

    // switch timeflip for backwards transform
    int32_t curveModInt = curveTransform;
    curveModInt ^= static_cast<int32_t>(RSCurveTransform::TIMEFLIP);

    transformInputParams(&x, &y, &phi, static_cast<RSCurveTransform>(curveModInt));

    // backwards transform
    float64_t xBackwards = x * std::cos(phi) + y * std::sin(phi);
    float64_t yNew       = x * std::sin(phi) - y * std::cos(phi);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RL_computeParams(&t, &u, &v, xBackwards, yNew, phi))
    {
        return false;
    }

    curve->segmentCount = 3;

    curve->segmentLengths[0] = v;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = t;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::LEFT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::FORWARD;
    curve->directions[2] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);
    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C Cu | Cu C ) paths, formula 8.7 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::LR_LR_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                     float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x + std::sin(phi);
    float64_t eta = y - 1.0 - std::cos(phi);

    if ((std::abs(xi) < EPS) && (std::abs(eta) < EPS))
    {
        return false;
    }

    float64_t u1 = std::sqrt(xi * xi + eta * eta);
    if (u1 > 4.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);
    if (u1 > 2.0)
    {
        float64_t alpha = std::acos(0.25 * u1 - 0.5);
        *t              = mod2pi(M_PI_2 + theta - alpha);
        *u              = mod2pi(M_PI - alpha);
        *v              = mod2pi(phi - *t + 2.0 * (*u));
    }
    else
    {
        float64_t alpha = std::acos(0.25 * u1 + 0.5);
        *t              = mod2pi(M_PI_2 + theta + alpha);
        *u              = mod2pi(alpha);
        *v              = mod2pi(phi - *t + 2.0 * (*u));
    }

    return true;
}

bool ReedsShepp::LR_LR(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                       RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!LR_LR_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 4;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = u;
    curve->segmentLengths[3] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::LEFT;
    curve->steerings[3] = RSSteering::RIGHT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::FORWARD;
    curve->directions[2] = RSDirection::BACKWARD;
    curve->directions[3] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C | Cu Cu | C ) paths, formula 8.8 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::L_RL_R_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                      float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x + std::sin(phi);
    float64_t eta = y - 1.0 - std::cos(phi);

    if ((std::abs(xi) < EPS) && (std::abs(eta) < EPS))
    {
        return false;
    }

    float64_t u1 = std::sqrt(xi * xi + eta * eta);
    if (u1 > 6.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);

    float64_t va1 = (5.0 - u1 * u1 / 4.0) / (4.0);
    if ((va1 < 0.0) || (va1 > 1.0))
    {
        return false;
    }

    *u = std::acos(va1);

    float64_t va2   = std::sin(*u);
    float64_t alpha = std::asin(2.0 * va2 / u1);
    *t              = mod2pi(M_PI_2 + theta + alpha);
    *v              = mod2pi(*t - phi);

    return true;
}

bool ReedsShepp::L_RL_R(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                        RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RL_R_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 4;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = u;
    curve->segmentLengths[3] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::LEFT;
    curve->steerings[3] = RSSteering::RIGHT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::BACKWARD;
    curve->directions[2] = RSDirection::BACKWARD;
    curve->directions[3] = RSDirection::FORWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C | C2 S C ) paths, formula 8.9 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::L_RSL_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                     float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x - std::sin(phi);
    float64_t eta = y - 1.0 + std::cos(phi);
    float64_t u1  = std::sqrt(xi * xi + eta * eta);
    if (u1 < 2.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);
    *u              = std::sqrt(u1 * u1 - 4) - 2.0;

    if (*u < 0.0)
    {
        return false;
    }

    float64_t alpha = std::atan2(2.0, *u + 2);
    *t              = mod2pi(M_PI_2 + theta + alpha);
    *v              = mod2pi(*t + M_PI_2 - phi);

    return true;
}

bool ReedsShepp::L_RSL(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                       RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RSL_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 4;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = M_PI_2;
    curve->segmentLengths[2] = u;
    curve->segmentLengths[3] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::STRAIGHT;
    curve->steerings[3] = RSSteering::LEFT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::BACKWARD;
    curve->directions[2] = RSDirection::BACKWARD;
    curve->directions[3] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C | C2 S C ) paths, formula 8.10 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::L_RSR_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                     float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x + std::sin(phi);
    float64_t eta = y - 1.0 - std::cos(phi);
    float64_t u1  = std::sqrt(xi * xi + eta * eta);

    if (u1 < 2.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);
    *t              = mod2pi(M_PI_2 + theta);
    *u              = u1 - 2.0;
    *v              = mod2pi(phi - *t - M_PI_2);

    return true;
}

bool ReedsShepp::L_RSR(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                       RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RSR_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 4;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = M_PI_2;
    curve->segmentLengths[2] = u;
    curve->segmentLengths[3] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::STRAIGHT;
    curve->steerings[3] = RSSteering::RIGHT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::BACKWARD;
    curve->directions[2] = RSDirection::BACKWARD;
    curve->directions[3] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C S C2 | C ) paths, formula 8.9 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::LSR_L(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                       RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    // use L_RSL formula with backwards transform

    // switch timeflip for backwards transform
    int32_t curveModInt = curveTransform;
    curveModInt ^= static_cast<int32_t>(RSCurveTransform::TIMEFLIP);

    transformInputParams(&x, &y, &phi, static_cast<RSCurveTransform>(curveModInt));

    // backwards transform
    float64_t xBackwards = x * std::cos(phi) + y * std::sin(phi);
    float64_t yBackwards = x * std::sin(phi) - y * std::cos(phi);

    float64_t t = 0.0f;
    float64_t u = 0.0f;
    float64_t v = 0.0f;
    if (!L_RSL_computeParams(&t, &u, &v, xBackwards, yBackwards, phi))
    {
        return false;
    }

    curve->segmentCount = 4;

    curve->segmentLengths[0] = v;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = M_PI_2;
    curve->segmentLengths[3] = t;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::STRAIGHT;
    curve->steerings[2] = RSSteering::RIGHT;
    curve->steerings[3] = RSSteering::LEFT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::FORWARD;
    curve->directions[2] = RSDirection::FORWARD;
    curve->directions[3] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C S C2 | C ) paths, formula 8.10 in Reeds-Shepp paper
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::LSL_R(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                       RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    // use L_RSR formula with backwards transform

    // switch timeflip for backwards transform
    int32_t curveModInt = curveTransform;
    curveModInt ^= static_cast<int32_t>(RSCurveTransform::TIMEFLIP);
    curveModInt ^= static_cast<int32_t>(RSCurveTransform::REFLECTION);

    transformInputParams(&x, &y, &phi, static_cast<RSCurveTransform>(curveModInt));

    // backwards transform
    float64_t xBackwards = x * std::cos(phi) + y * std::sin(phi);
    float64_t yNew       = x * std::sin(phi) - y * std::cos(phi);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RSR_computeParams(&t, &u, &v, xBackwards, yNew, phi))
    {
        return false;
    }

    curve->segmentCount = 4;

    curve->segmentLengths[0] = v;
    curve->segmentLengths[1] = u;
    curve->segmentLengths[2] = M_PI_2;
    curve->segmentLengths[3] = t;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::STRAIGHT;
    curve->steerings[2] = RSSteering::LEFT;
    curve->steerings[3] = RSSteering::RIGHT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::FORWARD;
    curve->directions[2] = RSDirection::FORWARD;
    curve->directions[3] = RSDirection::BACKWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

//-----------------------------------------------------------------------------------------------------------
// ( C | C2 S C2 | C ) paths, formula 8.11 in Reeds-Shepp paper
// (typo in paper, it says l+r-r-l-r+ instead of l+r-s-l-r+)
//-----------------------------------------------------------------------------------------------------------
bool ReedsShepp::L_RSL_R_computeParams(float64_t* t, float64_t* u, float64_t* v,
                                       float64_t x, float64_t y, float64_t phi) const
{
    float64_t xi  = x + std::sin(phi);
    float64_t eta = y - 1.0 - std::cos(phi);
    float64_t u1  = std::sqrt(xi * xi + eta * eta);
    if (u1 < 4.0)
    {
        return false;
    }

    float64_t theta = std::atan2(eta, xi);
    *u              = std::sqrt(u1 * u1 - 4.0) - 4.0;
    if (*u < 0.0)
    {
        return false;
    }
    float64_t alpha = std::atan2(2.0, *u + 4.0);
    *t              = mod2pi(M_PI_2 + theta + alpha);
    *v              = mod2pi(*t - phi);

    return true;
}

bool ReedsShepp::L_RSL_R(RSCurve* curve, float64_t x, float64_t y, float64_t phi,
                         RSDirection startDir, RSDirection endDir, RSCurveTransform curveTransform) const
{
    transformInputParams(&x, &y, &phi, curveTransform);

    float64_t t = 0.0;
    float64_t u = 0.0;
    float64_t v = 0.0;
    if (!L_RSL_R_computeParams(&t, &u, &v, x, y, phi))
    {
        return false;
    }

    curve->segmentCount = 5;

    curve->segmentLengths[0] = t;
    curve->segmentLengths[1] = M_PI_2;
    curve->segmentLengths[2] = u;
    curve->segmentLengths[3] = M_PI_2;
    curve->segmentLengths[4] = v;

    curve->steerings[0] = RSSteering::LEFT;
    curve->steerings[1] = RSSteering::RIGHT;
    curve->steerings[2] = RSSteering::STRAIGHT;
    curve->steerings[3] = RSSteering::LEFT;
    curve->steerings[4] = RSSteering::RIGHT;

    curve->directions[0] = RSDirection::FORWARD;
    curve->directions[1] = RSDirection::BACKWARD;
    curve->directions[2] = RSDirection::BACKWARD;
    curve->directions[3] = RSDirection::BACKWARD;
    curve->directions[4] = RSDirection::FORWARD;

    modifyCurve(curve, curveTransform);

    curve->cost = computeCost(curve, startDir, endDir);

    return true;
}

void ReedsShepp::selectRSCurve(RSCurve* curve, const RSPoint& startState, const RSPoint& targetState, RSCurveTransform curveTransform) const
{
    float64_t x{0.0};
    float64_t y{0.0};
    float64_t phi{0.0};

    if (!transformCoordinate(x, y, phi, startState, targetState))
    {
        return;
    }

    for (auto fn : {
             // ( C S C ) paths, formula 8.1 in Reeds-Shepp paper
             &ReedsShepp::LSL,

             // ( C S C ) paths, formula 8.2 in Reeds-Shepp paper
             &ReedsShepp::LSR,

             // ( C | C | C ) paths, formula 8.3 in Reeds-Shepp paper
             &ReedsShepp::L_R_L,

             // ( C | C C ) paths, formula 8.4 in Reeds-Shepp paper
             &ReedsShepp::L_RL,

             // ( C C | C ) paths, formula 8.4 in Reeds-Shepp paper
             &ReedsShepp::LR_L,

             // ( C Cu | Cu C ) paths, formula 8.7 in Reeds-Shepp paper
             &ReedsShepp::LR_LR,

             // ( C | Cu Cu | C ) paths, formula 8.8 in Reeds-Shepp paper
             &ReedsShepp::L_RL_R,

             // ( C | C2 S C ) paths, formula 8.9 in Reeds-Shepp paper
             &ReedsShepp::L_RSL,

             // ( C | C2 S C ) paths, formula 8.10 in Reeds-Shepp paper
             &ReedsShepp::L_RSR,

             // ( C S C2 | C ) paths, formula 8.9 in Reeds-Shepp paper
             &ReedsShepp::LSR_L,

             // ( C S C2 | C ) paths, formula 8.10 in Reeds-Shepp paper
             &ReedsShepp::LSL_R,

             // ( C | C2 S C2 | C ) paths, formula 8.11 in Reeds-Shepp paper
             // (typo in paper, it says l+r-r-l-r+ instead of l+r-s-l-r+)
             &ReedsShepp::L_RSL_R})
    {
        RSCurve c;
        bool ok = (this->*fn)(&c, x, y, phi, startState.direction, targetState.direction, curveTransform);
        if (ok && c.cost < curve->cost)
        {
            *curve = c;
        }
    }
}

void ReedsShepp::selectRSCurve(RSCurve* curve, const RSPoint& startState, const RSPoint& targetState) const
{
    selectRSCurve(curve, startState, targetState, RSCurveTransform::NONE);
    selectRSCurve(curve, startState, targetState, RSCurveTransform::TIMEFLIP);
    selectRSCurve(curve, startState, targetState, RSCurveTransform::REFLECTION);
    selectRSCurve(curve, startState, targetState, RSCurveTransform::TIMEFLIP_AND_REFLECTION);
}

void ReedsShepp::setParameters(float64_t radius, float64_t backwardsWeight, float64_t dirSwitchCost)
{
    m_radius              = radius;
    m_backwardsMultiplier = backwardsWeight;
    m_dirSwitchCost       = dirSwitchCost;
}

uint32_t ReedsShepp::discretizeLine(RSPoint* points, uint32_t maxPointCount,
                                    const RSPoint& startPoint, float64_t length, float64_t stepSize,
                                    RSDirection direction) const
{
    if (length < 0.0)
    {
        throw std::runtime_error("ReedsShepp: length has to be positive");
    }

    if (stepSize < 0.0)
    {
        throw std::runtime_error("ReedsShepp: stepSize has to be positive");
    }

    float64_t cosOrientation = std::cos(startPoint.orientation);
    float64_t sinOrientation = std::sin(startPoint.orientation);

    // steps along axes
    float64_t xStep = cosOrientation * stepSize;
    float64_t yStep = sinOrientation * stepSize;

    // current offset from startPoint
    float64_t xOffset = 0.0;
    float64_t yOffset = 0.0;

    if (direction == RSDirection::BACKWARD)
    {
        // switch step direction
        xStep = -xStep;
        yStep = -yStep;
    }

    // need to subtract before the loop, in case length is smaller than stepSize
    float64_t remainingLength = length - stepSize;

    // add points along the line
    uint32_t pointCount = 0;
    for (; remainingLength > 0.0 && pointCount < maxPointCount; remainingLength -= stepSize)
    {
        xOffset += xStep;
        yOffset += yStep;
        points[pointCount].x           = startPoint.x + xOffset;
        points[pointCount].y           = startPoint.y + yOffset;
        points[pointCount].orientation = startPoint.orientation;
        points[pointCount].direction   = direction;
        ++pointCount;
    }
    remainingLength += stepSize;

    // add last point
    if (pointCount < maxPointCount && std::abs(remainingLength) > 0.0)
    {
        if (direction == RSDirection::BACKWARD)
        {
            xOffset -= cosOrientation * remainingLength;
            yOffset -= sinOrientation * remainingLength;
        }
        else
        {
            xOffset += cosOrientation * remainingLength;
            yOffset += sinOrientation * remainingLength;
        }
        points[pointCount].x           = startPoint.x + xOffset;
        points[pointCount].y           = startPoint.y + yOffset;
        points[pointCount].orientation = startPoint.orientation;
        points[pointCount].direction   = startPoint.direction;
        ++pointCount;
    }

    return pointCount;
}

void ReedsShepp::pointOnCircle(float64_t* x, float64_t* y,
                               float64_t xCenter, float64_t yCenter, float64_t radius,
                               float64_t angle) const
{
    *x = xCenter + radius * std::cos(angle);
    *y = yCenter + radius * std::sin(angle);
}

bool ReedsShepp::isYawRatePositive(RSSteering steering, RSDirection direction) const
{
    return (steering == RSSteering::LEFT && direction == RSDirection::FORWARD) ||
           (steering == RSSteering::RIGHT && direction == RSDirection::BACKWARD);
}

uint32_t ReedsShepp::discretizeArc(RSPoint* points, uint32_t maxPointCount,
                                   const RSPoint& startPoint, float64_t radius, float64_t length, float64_t stepSize,
                                   RSSteering steering, RSDirection direction) const
{
    if (length < 0.0)
    {
        throw std::runtime_error("ReedsShepp: length has to be positive");
    }

    if (stepSize < 0.0)
    {
        throw std::runtime_error("ReedsShepp: stepSize has to be positive");
    }

    // compute circle center
    float64_t xCenter            = startPoint.x;
    float64_t yCenter            = startPoint.y;
    float64_t currentTheta       = startPoint.orientation; // angle parameter of point's polar coordinates
    float64_t currentOrientation = startPoint.orientation; // angle parameter of point's polar coordinates
    if (steering == RSSteering::RIGHT)
    {
        xCenter += radius * std::sin(startPoint.orientation);
        yCenter -= radius * std::cos(startPoint.orientation);
        currentTheta += M_PI_2;
    }
    else
    {
        xCenter -= radius * std::sin(startPoint.orientation);
        yCenter += radius * std::cos(startPoint.orientation);
        currentTheta -= M_PI_2;
    }

    float64_t angleStep      = stepSize / radius;
    float64_t remainingAngle = length / radius - angleStep;

    // choose rotation direction
    float64_t angleSign = 1.0; // for correct comparison in loop
    if (!isYawRatePositive(steering, direction))
    {
        angleStep      = -angleStep;
        remainingAngle = -remainingAngle;
        angleSign      = -1.0;
    }

    // add points along circle
    uint32_t pointCount = 0;
    for (; angleSign * remainingAngle > 0.0 && pointCount < maxPointCount; remainingAngle -= angleStep)
    {
        currentTheta += angleStep;
        currentOrientation += angleStep;

        pointOnCircle(&points[pointCount].x, &points[pointCount].y,
                      xCenter, yCenter, radius, currentTheta);

        points[pointCount].orientation = mod2pi(currentOrientation);
        points[pointCount].direction   = direction;
        ++pointCount;
    }

    remainingAngle += angleStep;

    // add last point
    if (pointCount < maxPointCount && std::abs(remainingAngle) > 0.0)
    {
        currentTheta += remainingAngle;
        currentOrientation += remainingAngle;

        pointOnCircle(&points[pointCount].x, &points[pointCount].y,
                      xCenter, yCenter, radius, currentTheta);

        points[pointCount].orientation = mod2pi(currentOrientation);
        points[pointCount].direction   = direction;
        ++pointCount;
    }

    return pointCount;
}

uint32_t ReedsShepp::discretizeRSCurve(RSPoint* path,
                                       const RSCurve& curve,
                                       const RSPoint& startState,
                                       float64_t stepSize,
                                       uint32_t maxPathPointCount) const
{
    if (maxPathPointCount == 0)
    {
        return 0;
    }

    uint32_t pointCount = 0;
    path[pointCount]    = startState;
    // take the start direction from first segment > 0
    for (uint32_t i = 0; i < curve.segmentCount; ++i)
    {
        path[pointCount].direction = curve.directions[i];
        if (curve.segmentLengths[i] > 0)
        {
            break;
        }
    }
    ++pointCount;
    for (uint32_t i = 0; i < curve.segmentCount; ++i)
    {
        uint32_t addedPointCount     = 0;
        uint32_t remainingPointCount = maxPathPointCount - pointCount;
        const RSPoint& startPoint    = path[pointCount - 1];
        float64_t length             = m_radius * curve.segmentLengths[i];

        if (curve.steerings[i] == RSSteering::STRAIGHT)
        {
            addedPointCount = discretizeLine(&path[pointCount], remainingPointCount,
                                             startPoint, length, stepSize, curve.directions[i]);
        }
        else
        {
            addedPointCount = discretizeArc(&path[pointCount], remainingPointCount,
                                            startPoint, m_radius, length, stepSize,
                                            curve.steerings[i], curve.directions[i]);
        }

        pointCount += addedPointCount;

        if (pointCount >= maxPathPointCount)
        {
            break;
        }
    }

    return pointCount;
}

void ReedsShepp::evaluateRS(RSPoint* path, uint32_t* pathPointCount, float64_t* pathCost,
                            const RSPoint& startState, const RSPoint& targetState,
                            float64_t lineStepSize, uint32_t maxPathPointCount) const
{
    RSCurve curve;
    selectRSCurve(&curve, startState, targetState);
    *pathCost = curve.cost;

    *pathPointCount = discretizeRSCurve(path, curve, startState, lineStepSize, maxPathPointCount);
}

bool ReedsShepp::evaluateDubins(RSPoint* path, uint32_t* pathPointCount, float64_t* pathCost,
                                const RSPoint& startState, const RSPoint& targetState,
                                float64_t lineStepSize, uint32_t maxPathPointCount) const
{
    RSCurve curve;
    if (selectDubins(&curve, startState, targetState))
    {
        *pathCost       = curve.cost;
        *pathPointCount = discretizeRSCurve(path, curve, startState, lineStepSize, maxPathPointCount);

        return true;
    }
    else
    {
        return false;
    }
}

bool ReedsShepp::selectDubins(RSCurve* curve, const RSPoint& startState, const RSPoint& targetState) const
{
    bool found{false};

    // transform coordinate
    float64_t x{0.0};
    float64_t y{0.0};
    float64_t phi{0.0};
    if (!transformCoordinate(x, y, phi, startState, targetState))
    {
        return false;
    }

    // for lane change, we only consider 3 segmentations: curve, straight, curve
    // also we loop through reflected and non-reflected curves to find the best solution
    // it's possible to get no solution
    for (auto fn : {
             &ReedsShepp::LSL, // ( C S C ) paths, formula 8.1 in Reeds-Shepp paper
             &ReedsShepp::LSR  // ( C S C ) paths, formula 8.2 in Reeds-Shepp paper
         })
    {
        for (RSCurveTransform curveTransform : {
                 RSCurveTransform::NONE,      // no reflection
                 RSCurveTransform::REFLECTION // reflected curves
             })
        {
            RSCurve c;
            bool ok = (this->*fn)(&c, x, y, phi, startState.direction, targetState.direction, curveTransform);
            if (ok && c.cost < curve->cost)
            {
                found  = true;
                *curve = c;
            }
        }
    }

    return found;
}

bool ReedsShepp::transformCoordinate(float64_t& x,
                                     float64_t& y,
                                     float64_t& phi,
                                     const RSPoint& startState,
                                     const RSPoint& targetState) const
{
    // angle between start state orientation and target state orientation
    phi = targetState.orientation - startState.orientation;

    float64_t dx = targetState.x - startState.x;
    float64_t dy = targetState.y - startState.y;

    if ((std::abs(dx) < EPS) &&
        (std::abs(dy) < EPS) &&
        (std::abs(phi) < EPS))
    {
        return false;
    }

    // angle between vector from start to target and x-axis
    float64_t theta = std::atan2(dy, dx);

    // compute angle to compute rotation such that start point is
    // at origin with 0 angle
    float64_t alpha = theta - startState.orientation;

    float64_t distance = std::sqrt(dx * dx + dy * dy);

    // target point when start point is origin
    x = std::cos(alpha) * distance;
    y = std::sin(alpha) * distance;

    // scale down, such that turning radius is 1,
    // so that we can operate in the unit circle and
    // arc length corresponds to angles.
    x /= m_radius;
    y /= m_radius;

    return true;
}