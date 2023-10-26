#include <gtest/gtest.h>
#include <control/Fit.hpp>

TEST(TestFit, fit)
{
    aidrive::Fit fit;

    float64_t realx{};
    float64_t realy{};
    float64_t realtheta{};
    float64_t realk{};

    auto d = fit.optimize(0.0, 1.0, 0.0, 0.3, 0.1, 100.0, 100.0,
                          realx, realy, realtheta, realk);
}