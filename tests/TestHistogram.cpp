#include <gtest/gtest.h>
#include <estimator/histogram.hpp>
#include <deque>
#include <random>

class Histogram_Test : public ::testing::Test
{
public:
    Histogram_Test()
    {
    }

    void resetRolling()
    {
        m_sizeHistogram = rand() % 10000;
        m_numberOfBins  = rand() % 10000;
        m_limitMin      = -10.0f;
        m_limitMax      = 10.0f;
        m_binSize       = (m_limitMax - m_limitMin) / float32_t(m_numberOfBins);

        m_rollingHistogram.reset(new RollingHistogram<float32_t>({m_limitMin, m_limitMax}, m_numberOfBins, m_sizeHistogram));
    }

    void resetDecay()
    {
        m_numberOfBins = rand() % 10000;
        m_limitMin     = -10.0f;
        m_limitMax     = 10.0f;
        m_binSize      = (m_limitMax - m_limitMin) / float32_t(m_numberOfBins);
        std::default_random_engine generator;
        std::uniform_real_distribution<float32_t> distributionDecays(0.0f, 1.0f);
        m_decayFactor = distributionDecays(generator);
        m_decayHistogram.reset(new DecayHistogram<float32_t>({m_limitMin, m_limitMax}, m_numberOfBins, m_decayFactor));
    }

    bool insertRolling(float32_t value, float32_t weight)
    {
        if (m_rollingHistogram->insert(value, weight))
            return true;
        else
            return false;
    }

    bool insertDecay(float32_t value, float32_t weight)
    {
        if (m_decayHistogram->insert(value, weight))
            return true;
        else
            return false;
    }

    float32_t getLimitMin() { return m_limitMin; }
    float32_t getLimitMax() { return m_limitMax; }
    uint32_t getNumberOfBins() { return m_numberOfBins; }
    uint32_t getHistogramSize() { return m_sizeHistogram; }
    float32_t getBinSize() { return m_binSize; }
    float32_t getDecayFactor() { return m_decayFactor; }

    size_t getModeRolling() { return m_rollingHistogram->modeIdx(); }
    float32_t getMeanRolling() { return m_rollingHistogram->mean(); }
    size_t getModeDecay() { return m_decayHistogram->modeIdx(); }
    float32_t getMeanDecay() { return m_decayHistogram->mean(); }

    void fromDequeToVector(const std::deque<float32_t>& dq, const std::deque<float32_t>& dqW, std::vector<float32_t>& hist)
    {
        // Bin data from deque to histogram represented by a vector
        for (uint32_t i = 0; i < dq.size(); ++i)
        {
            if (dq[i] < m_limitMin)
                continue;

            const uint32_t bin = static_cast<uint32_t>(std::floor((dq[i] - m_limitMin) / m_binSize));

            if (bin >= m_numberOfBins)
                continue;

            hist[bin] += dqW[i];
        }
    }

    template <typename T>
    std::vector<size_t> computeModeLinear(const std::vector<T>& hist)
    {
        // Compute the mode of the histogram
        size_t m = static_cast<size_t>(std::distance(std::begin(hist), std::max_element(std::begin(hist), std::end(hist))));

        // Check if there are other bins with the same mode in the histogram
        // This is necessary as several bins might have the same current counting in the ring buffer
        std::vector<size_t> modes;
        uint32_t i = 0;
        for (auto const h : hist)
        {
            if (std::abs(h - hist[m]) <= std::numeric_limits<T>::epsilon())
            {
                modes.push_back(i);
            }
            ++i;
        }

        return modes;
    }

    template <typename T>
    float32_t computeMeanLinear(std::vector<T>& hist)
    {
        float32_t sum = 0.0;
        uint32_t i    = 0;
        float32_t H   = 0.0;
        for (auto const h : hist)
        {
            sum += h * (float32_t(i) * m_binSize + m_limitMin + m_binSize / 2);
            ++i;
            H += h;
        }

        return sum / H;
    }

private:
    std::unique_ptr<RollingHistogram<float32_t>> m_rollingHistogram;
    std::unique_ptr<DecayHistogram<float32_t>> m_decayHistogram;
    float32_t m_limitMin;
    float32_t m_limitMax;
    float32_t m_binSize;
    uint32_t m_sizeHistogram;
    uint32_t m_numberOfBins;
    float32_t m_decayFactor;
};

TEST_F(Histogram_Test, modeRollingHistogram_L0)
{
    // Rolling histogram initialization
    resetRolling();

    // Deque to mimic ring buffer
    std::deque<float32_t> dq;
    std::deque<float32_t> dqW;

    // Number generator
    std::default_random_engine generator;
    std::uniform_real_distribution<float32_t> distribution(getLimitMin(), getLimitMax());
    std::uniform_real_distribution<float32_t> distributionW(0.0f, 1.0f);

    for (uint32_t i = 0; i < 10000; ++i)
    {
        // Simulation histogram
        std::vector<float32_t> histogramSimulator(getNumberOfBins(), 0);

        float32_t number = distribution(generator);
        float32_t weight = distributionW(generator);

        if (dq.size() == getHistogramSize())
        {
            dq.pop_front();
            dqW.pop_front();
        }
        dq.push_back(number);
        dqW.push_back(weight);

        if (insertRolling(number, weight))
        {
            // Compute mode for rolling histogram
            size_t modeRolling = getModeRolling();

            // Compute modes for simulation
            fromDequeToVector(dq, dqW, histogramSimulator);
            std::vector<size_t> modesLinearSearch = computeModeLinear(histogramSimulator);

            ASSERT_EQ((std::find(modesLinearSearch.begin(), modesLinearSearch.end(), modeRolling) == modesLinearSearch.end()), false);
        }
    }
}

TEST_F(Histogram_Test, meanRollingHistogram_L0)
{
    // Rolling histogram initialization
    resetRolling();

    // Deque to mimic rolling histogram
    std::deque<float32_t> dq;
    std::deque<float32_t> dqW;

    // Number generator
    std::default_random_engine generator;
    std::uniform_real_distribution<float32_t> distribution(getLimitMin(), getLimitMax());
    std::uniform_real_distribution<float32_t> distributionW(0.0f, 0.1f);

    for (uint32_t i = 0; i < 10000; ++i)
    {
        // Simulation histogram
        std::vector<float32_t> histogramSimulator(getNumberOfBins(), 0.0f);

        float32_t number = distribution(generator);
        float32_t weight = distributionW(generator);

        if (dq.size() == getHistogramSize())
        {
            dq.pop_front();
            dqW.pop_front();
        }
        dq.push_back(number);
        dqW.push_back(weight);

        if (insertRolling(number, weight))
        {
            // Compute mean for rolling histogram
            auto meanRolling = getMeanRolling();

            // Compute mean for simulaton histogram
            fromDequeToVector(dq, dqW, histogramSimulator);
            auto meanLinear = computeMeanLinear(histogramSimulator);

            ASSERT_NEAR(meanRolling, meanLinear, 0.0001f);
        }
    }
}

TEST_F(Histogram_Test, modeDecayHistogram_L0)
{
    // Initialize decay histogram
    resetDecay();

    // Number generator
    std::default_random_engine generator;
    std::uniform_real_distribution<float32_t> distribution(getLimitMin(), getLimitMax());
    std::uniform_real_distribution<float32_t> distributionW(0.0f, 1.0f);

    // Simulation histogram
    std::vector<float32_t> histogramSimulator(getNumberOfBins(), 0.0f);

    float32_t decayFactor = getDecayFactor();
    float32_t limitMin    = getLimitMin();
    float32_t binSize     = getBinSize();

    for (uint32_t i = 0; i < 10000; ++i)
    {
        float32_t number = distribution(generator);
        float32_t weight = distributionW(generator);

        if (insertDecay(number, weight))
        {
            // Decay histogram
            for (auto& c : histogramSimulator)
                c *= decayFactor;

            // Insert value in simulation histogram
            auto idx = static_cast<uint32_t>(std::floor((number - limitMin) / float32_t(binSize)));
            histogramSimulator.at(idx) += weight;

            // Compute mode for decay histogram
            size_t modeDecay = getModeDecay();

            // Compute modes for simulation histogram
            std::vector<size_t> modesLinearSearch = computeModeLinear(histogramSimulator);

            ASSERT_EQ((std::find(modesLinearSearch.begin(), modesLinearSearch.end(), modeDecay) == modesLinearSearch.end()), false);
        }
    }
}

TEST_F(Histogram_Test, meanDecayHistogram_L0)
{
    // Initialize decay histogram
    resetDecay();

    // Number generator
    std::default_random_engine generator;
    std::uniform_real_distribution<float32_t> distribution(getLimitMin(), getLimitMax());
    std::uniform_real_distribution<float32_t> distributionW(0.0f, 1.0f);

    // Simulation histogram
    std::vector<float32_t> histogramSimulator(getNumberOfBins(), 0.0f);

    float32_t decayFactor = getDecayFactor();
    float32_t limitMin    = getLimitMin();
    float32_t binSize     = getBinSize();

    for (uint32_t i = 0; i < 10000; ++i)
    {
        float32_t number = distribution(generator);
        float32_t weight = distributionW(generator);

        if (insertDecay(number, weight))
        {
            // Decay histogram
            for (auto& c : histogramSimulator)
                c *= decayFactor;

            // Insert value in simulation histogram
            auto idx = static_cast<uint32_t>(std::floor((number - limitMin) / float32_t(binSize)));
            histogramSimulator.at(idx) += weight;

            // Compute mean for decay histogram
            auto meanDecay = getMeanDecay();

            // Compute mean for simulation histogram
            auto meanLinear = computeMeanLinear(histogramSimulator);

            ASSERT_NEAR(meanDecay, meanLinear, 0.0001f);
        }
    }
}

TEST(Histogram_Tests, histogramStats_L0)
{
    // Initialize histogram
    auto const limits   = std::array<float32_t, 2>({0.f, 1.f});
    auto constexpr bins = 100u;
    auto histogram      = std::make_unique<Histogram<float32_t>>(
        limits, bins);

    // Generate some values
    size_t count           = 10;
    float32_t value        = 0.45f;
    float32_t valueOverTwo = value / 2.0f;
    for (size_t i = 0; i < count; i++)
    {
        histogram->insert(value, 1.0f);
    }

    for (size_t i = 0; i < count / 2; i++)
    {
        histogram->insert(valueOverTwo, 1.0f);
    }

    ASSERT_NEAR(histogram->modeCount(), static_cast<float32_t>(count), 1.0e-5);
    ASSERT_NEAR(histogram->modeValue(), value, 0.05f);
    ASSERT_NEAR(histogram->mean(), value * 5.0f / 6.0f, histogram->binSize()); // mean of value x count and value/2 x count/2
}

TEST(Histogram_Tests, outlierCount_L0)
{
    // Initialize histogram
    auto const limits   = std::array<float32_t, 2>{0.f, 1.f};
    auto constexpr bins = 100u;
    auto histogram      = std::make_unique<Histogram<float32_t>>(
        limits, bins);

    // Number generator
    std::default_random_engine generator;
    std::uniform_real_distribution<float32_t> distribution(limits[0], limits[1]);

    // Add inliers only
    auto constexpr ITERATIONS = 100u;
    for (auto i = 0u; i < ITERATIONS; ++i)
        histogram->insert(distribution(generator), 1.f);

    ASSERT_EQ(0u, histogram->outlierCount());

    // Check limit bounds
    histogram->insert(limits[0], 1.f);
    ASSERT_EQ(0u, histogram->outlierCount());
    histogram->insert(limits[1], 1.f);
    ASSERT_EQ(1u, histogram->outlierCount());

    // Add outliers only
    histogram->reset();
    ASSERT_EQ(0u, histogram->outlierCount());

    for (auto i = 0u; i < ITERATIONS; ++i)
        histogram->insert(distribution(generator) + (i % 2 ? -1.1f : 1.1f), 1.f);

    ASSERT_EQ(ITERATIONS, histogram->outlierCount());
}

TEST(Histogram_Tests, decayClone_L0)
{
    // Initialize histogram
    DecayHistogramf objA({-10.0f, 10.0f}, 10, 0.8f);
    objA.insert(1, 1.f);
    objA.insert(1, 1.f);
    objA.insert(1, 1.f);
    objA.insert(5, 1.f);
    objA.insert(5, 1.f);
    objA.insert(9, 1.f);
    auto objB_ = objA.clone();
    auto* objB = dynamic_cast<DecayHistogramf*>(objB_.get());
    ASSERT_NE(objB, nullptr);

    ASSERT_FLOAT_EQ(objA.binSize(), objB->binSize());
    ASSERT_EQ(objA.size(), objB->size());
    for (uint32_t i = 0; i < objA.size(); i++)
    {
        ASSERT_FLOAT_EQ(objA.binValue(i), objB->binValue(i));
        ASSERT_FLOAT_EQ(objA.binValueRange(i)[0], objB->binValueRange(i)[0]);
    }

    ASSERT_FLOAT_EQ(objA.mean(), objB->mean());
    ASSERT_FLOAT_EQ(objA.modeCount(), objB->modeCount());
    ASSERT_FLOAT_EQ(objA.getDecayFactor(), objB->getDecayFactor());
}

TEST(Histogram_Tests, rollingClone_L0)
{
    // Initialize histogram
    RollingHistogramf objA({-10.0f, 10.0f}, 10, 100);
    objA.insert(1, 1.f);
    objA.insert(1, 1.f);
    objA.insert(1, 1.f);
    objA.insert(5, 1.f);
    objA.insert(5, 1.f);
    objA.insert(9, 1.f);
    auto objB_ = objA.clone();
    auto* objB = dynamic_cast<RollingHistogramf*>(objB_.get());
    ASSERT_NE(objB, nullptr);

    ASSERT_FLOAT_EQ(objA.binSize(), objB->binSize());
    ASSERT_EQ(objA.size(), objB->size());
    for (uint32_t i = 0; i < objA.size(); i++)
    {
        ASSERT_FLOAT_EQ(objA.binValue(i), objB->binValue(i));
        ASSERT_FLOAT_EQ(objA.binValueRange(i)[0], objB->binValueRange(i)[0]);
    }

    ASSERT_FLOAT_EQ(objA.mean(), objB->mean());
    ASSERT_FLOAT_EQ(objA.modeCount(), objB->modeCount());
}

TEST_F(Histogram_Test, decayShiftData_L0)
{
    constexpr float32_t RANGE_MIN = -10.f;
    constexpr float32_t RANGE_MAX = +10.f;
    constexpr size_t BIN_COUNT    = 10;

    auto const checkOffset = [](BaseHistogramf const& histogram, BaseHistogramf const& offsetHistogram, float32_t OFFSET) {
        ASSERT_EQ(histogram.size(), offsetHistogram.size());

        for (auto i = 0u; i < histogram.size(); ++i)
        {
            auto const binValue = histogram.binValue(i);

            auto const offsetBinValue = binValue + OFFSET;

            auto const offsetBinIdx = offsetHistogram.binIdx(offsetBinValue);

            if (offsetBinIdx)
            {
                ASSERT_FLOAT_EQ(histogram.at(i), offsetHistogram.at(*offsetBinIdx))
                    << "i: " << i
                    << " j: " << *offsetBinIdx
                    << " OFFSET: " << OFFSET;
            }
        }
    };

    for (auto const OFFSET_FACTOR : {-4.f, -3.f, -2.f, -1.f, 1.f, 0.f, 1.f, 2.f, 3.f, 4.f})
    {
        // Initialize histogram
        DecayHistogramf objA({RANGE_MIN, RANGE_MAX}, BIN_COUNT, 1.0f);
        DecayHistogramf objC({RANGE_MIN, RANGE_MAX}, BIN_COUNT, 1.0f);

        const float32_t TOLERANCE = objA.binSize();
        const float32_t OFFSET    = OFFSET_FACTOR * objA.binSize(); // Make the offset an exact number of bins so we don't have aliasing effects
        const auto OFFSET_INT     = static_cast<int32_t>(OFFSET);
        ASSERT_FLOAT_EQ(OFFSET, static_cast<float32_t>(OFFSET_INT)) << "not an integer offset";

        std::default_random_engine generator;
        std::uniform_real_distribution<float32_t> distribution(RANGE_MIN + std::abs(OFFSET), RANGE_MAX - std::abs(OFFSET));

        for (size_t i = 0; i < 100; ++i)
        {
            auto value = distribution(generator);
            objA.insert(value, 1.f);
            objC.insert(value + OFFSET, 1.f);
        }

        auto objB_ = objA.clone();
        auto* objB = dynamic_cast<DecayHistogramf*>(objB_.get());
        objB->shiftData(OFFSET);

        checkOffset(objA, objC, OFFSET);
        checkOffset(objA, *objB, OFFSET);
        checkOffset(*objB, objC, 0.f);

        // Test right after shift
        ASSERT_NEAR(objA.mean() + OFFSET, objB->mean(), TOLERANCE);
        ASSERT_NEAR(objA.modeValue() + OFFSET, objB->modeValue(), TOLERANCE);
        ASSERT_FLOAT_EQ(objA.modeCount(), objB->modeCount());

        ASSERT_NEAR(objC.mean(), objB->mean(), TOLERANCE);
        ASSERT_NEAR(objC.modeValue(), objB->modeValue(), TOLERANCE);
        ASSERT_FLOAT_EQ(objC.modeCount(), objB->modeCount());

        // Add more data and test again
        for (size_t i = 0; i < 100; ++i)
        {
            auto value = distribution(generator);
            objA.insert(value, 1.f);
            objC.insert(value + OFFSET, 1.f);
            objB->insert(value + OFFSET, 1.f);
        }

        checkOffset(objA, objC, OFFSET);
        checkOffset(objA, *objB, OFFSET);
        checkOffset(*objB, objC, 0.f);

        ASSERT_NEAR(objA.mean() + OFFSET, objB->mean(), TOLERANCE);
        ASSERT_NEAR(objA.modeValue() + OFFSET, objB->modeValue(), TOLERANCE);
        ASSERT_FLOAT_EQ(objA.modeCount(), objB->modeCount());

        ASSERT_NEAR(objC.mean(), objB->mean(), TOLERANCE);
        ASSERT_NEAR(objC.modeValue(), objB->modeValue(), TOLERANCE);
        ASSERT_FLOAT_EQ(objC.modeCount(), objB->modeCount());
    }
}

// Verifies that createZeroCentered factory method ensures a zero-centered middle bin
TEST_F(Histogram_Test, decayZeroCenter_L0)
{
    constexpr float64_t RANGE = 10.f;
    constexpr float64_t TOL   = 1e-14;

    for (uint32_t binCount = 1; binCount < 20; binCount++)
    {
        const float64_t binAccuracy = 2.0f * RANGE / static_cast<float64_t>(binCount);

        auto histogram = DecayHistogramd::createZeroCentered(RANGE, binAccuracy, 0.0);
        histogram->insert(0.0, 1.0);

        EXPECT_NEAR(histogram->mean(), 0.0, TOL);
        EXPECT_NEAR(histogram->modeValue(), 0.0, TOL);
    }
}