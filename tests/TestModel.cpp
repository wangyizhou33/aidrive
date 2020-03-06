#include <gtest/gtest.h>
#include <model/Model.hpp>

namespace aidrive
{

constexpr float32_t EPS = 1e-3f;

TEST(TestModel, ConstantStraight)
{
    MotionModel model{};

    float32_t v          = 10.0f;
    float32_t elapseTime = 10.0f;

    Eigen::Vector4f state{0.f, 0.f, 0.f, v};

    Eigen::Vector4f finalState = model.advance(state,
                                               0.f,
                                               0.f,
                                               elapseTime);

    ASSERT_NEAR(elapseTime * v, finalState[0] - state[0], EPS);
    ASSERT_NEAR(0.f, finalState[1], EPS);
    ASSERT_NEAR(0.f, finalState[2], EPS);
    ASSERT_NEAR(v, finalState[3], EPS);
}

TEST(TestModel, ConstantCircular)
{
    MotionModel model{};

    float32_t v          = 5.0f;
    float32_t elapseTime = 20.0f;     // long enough
    float32_t delta      = 0.174533f; // 10 deg
    float32_t wheelBase  = 2.8f;      // need to enforce consistency
    float32_t k          = std::tan(delta) / wheelBase;
    float32_t radius     = 1.f / k;

    Eigen::Vector4f state{0.f, -radius, 0.f, v};

    Eigen::Vector4f finalState = model.advance(state,
                                               0.f,   // const speed
                                               delta, // const steering
                                               elapseTime);

    // final state is in the same circle
    ASSERT_NEAR(radius * radius,
                finalState[0] * finalState[0] + finalState[1] * finalState[1],
                EPS);

    ASSERT_NEAR(elapseTime * v * k,
                finalState[2],
                EPS);

    ASSERT_NEAR(v, finalState[3], EPS);
}

} // namespace aidrive
