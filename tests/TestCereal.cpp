#include <gtest/gtest.h>
#include "gflags/gflags.h"

#include <cereal/types/unordered_map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <fstream>
#include <sstream>
#include <aidrive/Types.hpp>
#include <aidrive/json.hpp>

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

        for (size_t i = 0; i < vec.size(); ++i)
        {
            ASSERT_EQ(vec[i], vecIn[i]);
        }
        ASSERT_TRUE(arrIn[0]);
        ASSERT_FALSE(arrIn[1]);
    }
}

struct Boo
{
    bool c_{false};

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(c_);
    }
};

struct Foo
{
    // float32_t a_{0.0f}; // deleted in version 3
    int32_t b_{1};
    Boo d_;

    template <class Archive>
    void serialize(Archive& ar, std::uint32_t const version)
    {
        if (version == 1u)
        {
            float32_t a_{}; // placeholder
            ar(CEREAL_NVP(a_), CEREAL_NVP(b_));
        }
        else if (version == 2u)
        {
            float32_t a_{}; // placeholder
            ar(CEREAL_NVP(a_), CEREAL_NVP(b_));
            ar(CEREAL_NVP(d_));
        }
        else if (version == 3u)
        {
            ar(CEREAL_NVP(b_));
            ar(CEREAL_NVP(d_));
        }
        else
        {
            // do nothing
        }
    }
};
CEREAL_CLASS_VERSION(Foo, 3u);

static const nlohmann::json EXPECTED_RES_VER_1 = nlohmann::json::parse(R"(
    {

        "value0" :
            {
                "a_" : 0.0,
                "b_" : 1,
                "cereal_class_version" : 1
            }
    }
)");

static const nlohmann::json EXPECTED_RES_VER_2 = nlohmann::json::parse(R"(
    {"value0":
        {
            "a_":0.0,
            "b_":1,
            "cereal_class_version":2,
            "d_":
                {
                    "value0": false
                }
        }
    }
)");

static const nlohmann::json EXPECTED_RES_VER_3 = nlohmann::json::parse(R"(
    {"value0":
        {
            "b_":1,
            "cereal_class_version":3,
            "d_":
                {
                    "value0": false
                }
        }
    }
)");

TEST(TestCereal, serialization)
{
    Foo foo;
    std::stringstream ss{};

    {
        cereal::JSONOutputArchive archiveOut(ss);
        archiveOut(foo);
    }

    nlohmann::json res = nlohmann::json::parse(ss.str());

    ASSERT_EQ(res, EXPECTED_RES_VER_3);
}

TEST(TestCereal, deserialization)
{
    Foo foo;

    {
        std::stringstream ss{};
        ss << EXPECTED_RES_VER_1.dump();
        cereal::JSONInputArchive archiveIn(ss);
        ASSERT_NO_THROW(archiveIn(foo));
        ASSERT_EQ(foo.b_, Foo().b_);
        ASSERT_EQ(foo.d_.c_, Foo().d_.c_);
    }

    {
        std::stringstream ss{};
        ss << EXPECTED_RES_VER_2.dump();
        cereal::JSONInputArchive archiveIn(ss);
        ASSERT_NO_THROW(archiveIn(foo));
        ASSERT_EQ(foo.b_, Foo().b_);
        ASSERT_EQ(foo.d_.c_, Foo().d_.c_);
    }

    {
        std::stringstream ss{};
        ss << EXPECTED_RES_VER_3.dump();
        cereal::JSONInputArchive archiveIn(ss);
        ASSERT_NO_THROW(archiveIn(foo));
        ASSERT_EQ(foo.b_, Foo().b_);
        ASSERT_EQ(foo.d_.c_, Foo().d_.c_);
    }
}
