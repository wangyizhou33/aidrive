#include <gtest/gtest.h>
#include "gflags/gflags.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>

TEST(TestCereal, example)
{
    bool arr[]           = {true, false};
    std::vector<int> vec = {1, 2, 3, 4, 5};

    // cereal::BinaryOutputArchive archive(std::cout);
    // cereal::XMLOutputArchive archive(std::cout);
    cereal::JSONOutputArchive archive( std::cout );

    archive(CEREAL_NVP(vec),
            arr);
}