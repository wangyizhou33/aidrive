#include <gtest/gtest.h>
#include "gflags/gflags.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>

TEST(TestCereal, serialize)
{
    bool arr[]           = {true, false};
    std::vector<int> vec = {1, 2, 3, 4, 5};

    {
        std::ofstream os("out.cereal");

        // cereal::BinaryOutputArchive archiveOut(os);
        // cereal::XMLOutputArchive archiveOut(os);
        cereal::JSONOutputArchive archiveOut(os);

        archiveOut(CEREAL_NVP(vec),
                   arr);
    }

    {
        // open the same file
        std::ifstream is("out.cereal");

        // cereal::BinaryInputArchive archiveIn(is);
        // cereal::XMLInputArchive archiveIn(is);
        cereal::JSONInputArchive archiveIn(is);

        std::vector<int> vecIn = {0, 0, 0, 0, 0};
        bool arrIn[2];
        archiveIn(vecIn,
                  arrIn);
    }

    for (size_t i = 0; i < vec.size(); ++i)
    {
        ASSERT_EQ(vec[i], vecIn[i]);
    }
    ASSERT_TRUE(arrIn[0]);
    ASSERT_FALSE(arrIn[1]);
}