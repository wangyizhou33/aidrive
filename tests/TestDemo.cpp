#include <gtest/gtest.h>
#include "demo.hpp"

TEST(TestDemo, arithmetics)
{
    EXPECT_EQ(2, demo::add(1, 1));
    EXPECT_EQ(1, demo::add(0, 1));

    EXPECT_EQ(1, demo::subtract(3, 2));

    EXPECT_EQ(1, demo::divide(3, 2));

    EXPECT_THROW(demo::divide(1, 0), std::invalid_argument);
}