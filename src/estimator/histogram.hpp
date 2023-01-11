#ifndef HISTOGRAM_HPP_
#define HISTOGRAM_HPP_

#include <aidrive/Types.hpp>
#include <aidrive/Utils.hpp>
#include "RingBuffer.hpp"
#include <exception>
#include <iostream>
#include <numeric>

template <class TValue_ = float32_t>
class BaseHistogram
{
public:
    using TValue        = TValue_;
    using TCount        = float32_t;
    using TIndex        = size_t;
    using TOutlierCount = uint32_t;

    BaseHistogram()
        : BaseHistogram(std::array<TValue, 2>(TValue(0), TValue(1)), 1)
    {
    }

    BaseHistogram(std::array<TValue, 2> const& limits_, uint32_t const binCount)
        : BaseHistogram(limits_,
                        (limits_[1] - limits_[0]) / static_cast<TValue>(binCount),
                        std::vector<TCount>{},
                        0.0F,
                        0,
                        0)
    {
        m_bins.resize(binCount);
        std::fill(m_bins.begin(), m_bins.end(), 0);

        if (m_limits[0] >= m_limits[1])
        {
            throw std::runtime_error("invalid argument");
        }
        if (binCount == 0)
        {
            throw std::runtime_error("invalid argument");
        }
        if (binCount > static_cast<uint32_t>(std::numeric_limits<int32_t>::max()))
        {
            throw std::runtime_error("invalid argument");
        }
    }

    BaseHistogram(BaseHistogram const& other)
        : BaseHistogram(other.m_limits,
                        other.m_binSize,
                        other.m_bins,
                        other.m_mean,
                        other.m_modeIdx,
                        other.m_outlierCount)
    {
    }

private:
    BaseHistogram(std::array<TValue, 2> const& limits_,
                  TValue const binSize,
                  std::vector<TCount>&& bins,
                  TValue const mean,
                  TIndex const modeIdx,
                  TOutlierCount const outlierCount)
        : m_limits(limits_)
        , m_binSize(binSize)
        , m_bins(std::move(bins))
        , m_mean(mean)
        , m_modeIdx(modeIdx)
        , m_outlierCount(outlierCount)
    {
    }

public:
    virtual ~BaseHistogram() = default;

    auto operator=(BaseHistogram const& other) -> BaseHistogram&
    {
        copyFrom(other);
        return *this;
    }

    BaseHistogram(BaseHistogram&& other) = delete;
    auto operator=(BaseHistogram&& other) -> BaseHistogram& = delete;

    virtual auto clone() const -> std::unique_ptr<BaseHistogram> = 0;

    /// Resets the histogram to a blank state (like after construction)
    virtual void reset()
    {
        for (auto& bin : m_bins)
        {
            bin = 0.0F;
        }
        m_mean         = 0.0F;
        m_modeIdx      = 0;
        m_outlierCount = 0;
    }

    /// Limts of the histogram [lower, upper)
    std::array<TValue, 2> limits() const
    {
        return m_limits;
    }

    /// Distance between two bins' center values
    auto binSize() const -> TValue { return m_binSize; }

    /// Number of bins
    size_t size() const
    {
        return m_bins.size();
    }

    /// Accumulation value of a bin
    auto at(TIndex const idx) const -> TCount
    {
        return m_bins[idx];
    }

    /// Accumulation values of all bins
    std::vector<TCount> bins() const
    {
        return m_bins;
    }

    std::vector<TCount>& bins()
    {
        return m_bins;
    }

    /// Representative value of a bin (center of a bin)
    auto binValue(TIndex const idx) const -> TValue
    {
        return m_limits[0] + static_cast<TValue>(idx) * m_binSize + m_binSize / 2.0F;
    }

    /// Values range covered by a bin [lower, upper)
    std::vector<TValue> binValueRange(TIndex idx) const
    {
        return {m_limits[0] + TValue(idx) * m_binSize, m_limits[0] + TValue(idx + 1) * m_binSize};
    }

    /// Inserts a value into the histogram.
    /// Returns true if value was an inlier.
    virtual bool insert(TValue const value, TCount const weight)
    {
        auto const idx = binIdx(value);
        if (idx)
        {
            m_bins[*idx] += weight;
            if (m_outlierCount > 0)
            {
                --m_outlierCount; // reduce number of recent outliers
            }
            return true;
        }
        else
        {
            ++m_outlierCount; // increase number of recent outliers
            return false;
        }
    }

    /// Bin index of the bin with most accumulation values
    auto modeIdx() const -> TIndex
    {
        return m_modeIdx;
    }

    /// Mean of the histogram distribution
    auto mean() const -> TValue
    {
        return m_mean;
    }

    /// StdDev of the histogram distribution (stddev is computed around mean of the histogram)
    auto stdDev() const -> TValue
    {
        return stdDev(m_mean);
    }

    /// StdDev of the histogram distribution (computed around provided mean)
    auto stdDev(TValue mean_) const -> TValue
    {
        TValue sum = 0;
        TValue num = 0;
        for (TIndex i = 0, e = static_cast<TIndex>(size()); i < e; i++)
        {
            sum += static_cast<TValue>(at(i)) * (binValue(i) - mean_) * (binValue(i) - mean_);
            num += static_cast<TValue>(at(i));
        }

        if (std::abs(num) < std::numeric_limits<TCount>::epsilon())
        {
            return 0;
        }

        return std::sqrt(sum / num);
    }

    /// Representative value of the bin with most accumulated values
    auto modeValue() const -> TValue
    {
        return binValue(m_modeIdx);
    }

    /// Accumulation value of the bin with most accumulated values
    auto modeCount() const -> TCount
    {
        return at(m_modeIdx);
    }

    /// Number of recently recorded outliers (each successful insertion decrements outlier count)
    auto outlierCount() const -> TOutlierCount
    {
        return m_outlierCount;
    }

    /// Bin index returned if input value is in the range of valid values defined by limits, nothing otherwise
    std::optional<TIndex> binIdx(TValue const value) const
    {
        if (value < m_limits[0])
        {
            return {};
        } // value out of range

        if (value > m_limits[1])
        {
            return {};
        } // value out of range

        if (!std::isfinite(value))
        {
            return {};
        } // value is nan or inf

        TIndex const idx = static_cast<TIndex>(std::floor((value - m_limits[0]) / m_binSize));

        if (idx < m_bins.size())
        {
            return idx;
        }
        else
        {
            return {};
        } // consider numerical noise around limit as outlier
    }

    /// Shifts the data of the histogram
    /// Offset is in the same units as the samples
    virtual void shiftData(TValue offset)
    {
        size_t const refBinIdx = offset > 0.0F ? 0 : m_bins.size() - 1;

        auto const refBinValue = binValue(refBinIdx);

        auto const offsetBinIdx = binIdx(refBinValue + offset);
        if (!offsetBinIdx)
        {
            throw std::runtime_error("BaseHistogram::shiftData: offset is too large");
        }
        size_t const idxOffset = *offsetBinIdx - refBinIdx; // this *intentionally* performs a size_t underflow if refBinIdx > offsetBinIdx
        if (idxOffset == 0)
        {
            return;
        }

        offset = binValue(refBinIdx + idxOffset) - refBinValue; // Recalculate offset after truncating

        if (offset > 0.0F)
        {
            // Copy from right to left
            for (size_t i = m_bins.size() - 1; i >= idxOffset; --i)
            {
                m_bins[i] = m_bins[i - idxOffset];
            }
            for (size_t i = 0; i < idxOffset; ++i)
            {
                m_bins[i] = 0.0F;
            }
        }
        else
        {
            // Copy from left to right
            size_t const idxOffsetWrapped = 0U - idxOffset; // undo intentional underflow
            for (size_t i = idxOffsetWrapped; i < m_bins.size(); ++i)
            {
                m_bins[i - idxOffsetWrapped] = m_bins[i];
            }
            for (size_t i = m_bins.size() - idxOffsetWrapped; i < m_bins.size(); ++i)
            {
                m_bins[i] = 0.0F;
            }
        }

        m_mean += offset;
        m_modeIdx += idxOffset;
    }

    TValue computeKurtosis() const
    {
        TCount total = std::accumulate(m_bins.begin(), m_bins.end(), static_cast<TCount>(0));
        std::vector<TValue> probabilities(size(), static_cast<TValue>(0));
        TValue mu{0.};
        TValue moment2{0.};

        for (size_t i = 0u; i < size(); ++i)
        {
            // std::cout << "bin " << i << " value " << binValue(i) << " " << at(i) << std::endl;
            probabilities.at(i) = at(i) / total;
            mu += binValue(i) * probabilities.at(i);
            moment2 += binValue(i) * binValue(i) * probabilities.at(i);
        }
        TValue sigma2 = moment2 - mu * mu;
        TValue sigma4 = sigma2 * sigma2;
        TValue kurtosis{0.f};

        for (size_t i = 0u; i < probabilities.size(); i++)
        {
            kurtosis +=
                probabilities.at(i) * std::pow(binValue(i) - mu, 4) / sigma4;
        }

        return kurtosis - 3.0;
    }

    TValue computeModeCountRatio() const
    {
        TCount total = std::accumulate(m_bins.begin(), m_bins.end(), static_cast<TCount>(0));

        return modeCount() / total;
    }


private:
    std::array<TValue, 2> m_limits;

    TValue m_binSize;

    std::vector<TCount> m_bins;

    TValue m_mean;    // value of mean
    TIndex m_modeIdx; // index of mode bin

    TOutlierCount m_outlierCount;

protected:
    void setMean(TValue const mean) { m_mean = mean; }
    void setModeIdx(TIndex const modeIdx) { m_modeIdx = modeIdx; }

    /// This is a virtual by-value copy.
    /// Assignment operators (operator=) cannot be virtual because the return type between classes
    /// is different. So the assignemnt operators should call this instead to do the copy virtually.
    virtual void copyFrom(BaseHistogram const& other)
    {
        if (m_bins.size() != other.m_bins.size())
        {
            throw std::runtime_error("BaseHistogram: size of other is not equal");
        }

        m_limits       = other.m_limits;
        m_binSize      = other.m_binSize;
        m_mean         = other.m_mean;
        m_modeIdx      = other.m_modeIdx;
        m_outlierCount = other.m_outlierCount;

        std::copy(other.m_bins.begin(), other.m_bins.end(), m_bins.begin());
        ;
    }
};

// template <class TValue_>
// bool operator==(BaseHistogram<TValue_> const& lhs, BaseHistogram<TValue_> const& rhs) noexcept
// {
//     using TCount = float32_t;
//     return ((((
//                   (lhs.limits() == rhs.limits()) &&
//                   (isFloatEqual(lhs.binSize(), rhs.binSize()))) &&
//               (isFloatContainerEqual<span<TCount const>, TCount>(lhs.bins(), rhs.bins()))) &&
//              (isFloatEqual(lhs.mean(), rhs.mean()))) &&
//             (lhs.modeIdx() == rhs.modeIdx())) &&
//            (lhs.outlierCount() == rhs.outlierCount());
// }

/// A simple histogram that keeps a running set of mean/mode statistics
template <class TValue_>
class Histogram : public BaseHistogram<TValue_>
{
public:
    using Base   = BaseHistogram<TValue_>;
    using TValue = typename Base::TValue;
    using TCount = typename Base::TCount;
    using TIndex = typename Base::TIndex;

    Histogram(std::array<TValue, 2> const& limits_, std::uint32_t binCount)
        : Base(limits_, binCount)
    {
    }

    void reset() override
    {
        Base::reset();
        m_totalWeight = 0.0F;
    }

    auto clone() const -> std::unique_ptr<Base> override
    {
        auto value = std::make_unique<Histogram>(Base::limits(), static_cast<uint32_t>(Base::bins().size()));
        *value     = *this;
        return value;
    }

    bool insert(TValue value, TCount weight) override
    {
        if (Base::insert(value, weight))
        {
            auto const idx = *Base::binIdx(value);

            if (idx != Base::modeIdx() && Base::at(idx) > Base::at(Base::modeIdx()))
            {
                Base::setModeIdx(idx);
            }
            TCount totalWeightPrev = m_totalWeight;
            m_totalWeight += weight;
            Base::setMean((Base::mean() * totalWeightPrev + Base::binValue(idx) * weight) / m_totalWeight);
            return true;
        }

        return false;
    }

private:
    float32_t m_totalWeight{0.0F};
};

/// A decaying histogram
/// It does not keep a list of samples.
/// All bins are decayed by the given factor before adding a new sample.
template <class TValue_>
class DecayHistogram : public BaseHistogram<TValue_>
{
public:
    using Base   = BaseHistogram<TValue_>;
    using TValue = typename Base::TValue;
    using TCount = typename Base::TCount;
    using TIndex = typename Base::TIndex;

    using TParam = ::float32_t; // The type of the 3rd constructor parameter so users can template on this

    DecayHistogram()
        : Base()
        , m_decayFactor{}
        , m_decayedNumberOfSamples{}
    {
    }

    DecayHistogram(std::array<TValue, 2> const& limits_, std::uint32_t const binCount, TValue const decayFactor)
        : Base(limits_, binCount)
        , m_decayFactor(decayFactor)
        , m_decayedNumberOfSamples(0.F)
    {
    }

    // Convenience constructor, using zero-centered limits defined by single range (limits: [-range, range]),
    // and infered bin count by provided per-bin accuracy
    DecayHistogram(TValue range, TValue binAccuracy, TValue decayFactor)
        : DecayHistogram(
              {-range, range}, static_cast<uint32_t>(std::ceil((2.F * range) / binAccuracy)), decayFactor)
    {
    }

    // Convenience factory method ensuring that histogram middle bin is zero centered
    static auto createZeroCentered(TValue range, TValue binAccuracy, TValue decayFactor) -> std::unique_ptr<DecayHistogram>
    {
        // Histogram bin count, odd to have middle bin zero centered
        uint32_t const binCountOdd = 2 * static_cast<uint32_t>(std::ceil(range / binAccuracy)) + 1;

        return std::make_unique<DecayHistogram>(std::array<TValue, 2>({-range, range}), binCountOdd, decayFactor);
    }

    void reset() override
    {
        Base::reset();
        m_decayedNumberOfSamples = 0.0F;
    }

    auto operator=(DecayHistogram const& other) -> DecayHistogram&
    {
        copyFrom(other);
        return *this;
    }
    DecayHistogram(DecayHistogram const& other) = default;

    DecayHistogram(DecayHistogram&& other) = delete;
    auto operator=(DecayHistogram&& other) -> DecayHistogram& = delete;

    ~DecayHistogram() override = default;

    auto clone() const -> std::unique_ptr<Base> override
    {
        auto value = std::make_unique<DecayHistogram>(Base::limits(), static_cast<std::uint32_t>(Base::bins().size()), m_decayFactor);
        *value     = *this;
        return value;
    }

    /// Inserts a value into the histogram.
    /// Performs decaying by decayFactor before insertion.
    /// Returns true if value was an inlier.
    bool insert(TValue const value, TCount const weight) final
    {
        decay();

        if (Base::insert(value, weight))
        {
            auto const idx = Base::binIdx(value);
            if (idx)
            {
                // Update histogram mode
                if (Base::modeCount() < Base::at(*idx))
                {
                    Base::setModeIdx(*idx);
                }
                // Update histogram mean
                // For a discrete distribution, the mean is defined as
                // mu = sum_i(P(x_i)*x_i) / sum_i(P(x_i)), so sum_i(P(x_i)*x_i)  = mu * sum_i(P(x_i))
                // Let m_decayedNumberOfSamples be sum_i(P(x_i))

                // Decay m_decayedNumberOfSamples to reflect that the histogram has been decayed.
                m_decayedNumberOfSamples *= m_decayFactor;
                // The new mean is mu' = (sum_i(P(x_i))*mu + new_value * weight) / (sum_i(P(x_i)) + weight)
                TValue const mu = (m_decayedNumberOfSamples * Base::mean() + Base::binValue(*idx) * weight) / (m_decayedNumberOfSamples + weight);
                Base::setMean(mu);
                // m_decayedNumberOfSamples is updated with the new weight
                m_decayedNumberOfSamples += weight;

                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    /// Applies decay to histogram.
    /// Automatically called during insert()
    /// Only use this function to apply decay without data insertion
    void decay()
    {
        for (auto& bin : Base::bins())
        {
            bin *= static_cast<TCount>(m_decayFactor);
        }
    }

    void setDecayFactor(TValue const decayFactor)
    {
        m_decayFactor = decayFactor;
    }

    auto getDecayFactor() const -> TValue { return m_decayFactor; }
    auto getDecayedNumberOfSamples() const -> TValue { return m_decayedNumberOfSamples; }

private:
    TValue m_decayFactor;
    TValue m_decayedNumberOfSamples;

protected:
    void copyFrom(Base const& other) override
    {
        auto& other_ = dynamic_cast<DecayHistogram const&>(other);
        Base::copyFrom(other_);

        m_decayFactor            = other_.m_decayFactor;
        m_decayedNumberOfSamples = other_.m_decayedNumberOfSamples;
    }
};

template <class TValue_>
bool operator==(DecayHistogram<TValue_> const& lhs, DecayHistogram<TValue_> const& rhs) noexcept
{
    return operator==(static_cast<BaseHistogram<TValue_> const&>(lhs), static_cast<BaseHistogram<TValue_> const&>(rhs)) &&
           ((isFloatEqual(lhs.getDecayFactor(), rhs.getDecayFactor())) &&
            (isFloatEqual(lhs.getDecayedNumberOfSamples(), rhs.getDecayedNumberOfSamples())));
}

/// A rolling windows histogram with a fixed number of maximal entries supporting continuous insertion.
/// New values will push out the oldest ones if at capacity.
/// Inlier and outlier values are tracked, but only inliers are represented in the histogram.
template <typename TValue_>
class RollingHistogram : public BaseHistogram<TValue_>
{
public:
    using Base   = BaseHistogram<TValue_>;
    using TValue = typename Base::TValue;
    using TCount = typename Base::TCount;
    using TIndex = typename Base::TIndex;

    using TParam = ::size_t; // The type of the 3rd constructor parameter so users can template on this

    /// Initializes histogram for range [minValue, maxValue). The range is divided in binCount uniform bins.
    RollingHistogram(std::array<TValue, 2> const& limits_, std::uint32_t binCount, ::size_t historyCount)
        : Base(limits_, binCount), m_valueRingBuffer(historyCount), m_countRingBuffer(historyCount)
    {
    }

    void reset() override
    {
        Base::reset();
        m_valueRingBuffer.clear();
        m_countRingBuffer.clear();
        m_numberOfSamples = 0;
    }

    auto operator=(RollingHistogram const& other) -> RollingHistogram&
    {
        copyFrom(other);
        return *this;
    }

    auto clone() const -> std::unique_ptr<Base> override
    {
        auto value = std::make_unique<RollingHistogram>(Base::limits(),
                                                        static_cast<uint32_t>(Base::bins().size()),
                                                        m_countRingBuffer.capacity());
        *value     = *this;
        return value;
    }

    /// Inserts a value into the histogram. Removes oldest value if at capacity.
    /// Outliers are tracked but not represented in the histogram.
    /// Returns true if value was an inlier.
    bool insert(TValue value, TCount weight) final
    {
        // Remove oldest value from histogram if at capacity
        // Actual sample will be removed from the RingBuffer automatically
        TValue sumOfValues = Base::mean() * m_numberOfSamples;
        if (m_valueRingBuffer.full())
        {
            auto frontElement = m_valueRingBuffer.front();

            auto const frontElementBin = Base::binIdx(frontElement);
            if (frontElementBin)
            {
                Base::bins()[*frontElementBin] -= m_countRingBuffer.front(); // remove value from histogram if valid value

                // Update histogram mode if front element belongs to mode bin
                if (*frontElementBin == Base::modeIdx())
                {
                    Base::setModeIdx(static_cast<uint32_t>(std::distance(std::begin(Base::bins()),
                                                                         std::max_element(std::begin(Base::bins()),
                                                                                          std::end(Base::bins())))));
                }

                // Update histogram mean
                sumOfValues -= m_countRingBuffer.front() * Base::binValue(*frontElementBin);
                m_numberOfSamples -= m_countRingBuffer.front();
            }
        }

        static_cast<void>(m_valueRingBuffer.push_back(value));
        static_cast<void>(m_countRingBuffer.push_back(weight));

        if (Base::insert(value, weight))
        {
            auto const idx = Base::binIdx(value);
            if (idx)
            {
                // Update histogram mode
                if (Base::modeCount() < Base::at(*idx))
                {
                    Base::setModeIdx(*idx);
                }

                // Update histogram mean
                m_numberOfSamples += weight;
                Base::setMean((sumOfValues + Base::binValue(*idx) * weight) / m_numberOfSamples);

                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void shiftData(TValue /*offset*/) override
    {
        throw std::runtime_error("RollingHistogram::shiftData");
    }

private:
    CircularBuffer<TValue> m_valueRingBuffer;
    CircularBuffer<TCount> m_countRingBuffer;
    TCount m_numberOfSamples = 0;

protected:
    void copyFrom(Base const& other) override
    {
        auto& other_ = dynamic_cast<RollingHistogram const&>(other);
        Base::copyFrom(other_);

        m_valueRingBuffer = other_.m_valueRingBuffer;
        m_countRingBuffer = other_.m_countRingBuffer;
        m_numberOfSamples = other_.m_numberOfSamples;
    }
};

using BaseHistogramf = BaseHistogram<float32_t>;
using BaseHistogramd = BaseHistogram<float64_t>;

using DecayHistogramf = DecayHistogram<float32_t>;
using DecayHistogramd = DecayHistogram<float64_t>;

using RollingHistogramf = RollingHistogram<float32_t>;
using RollingHistogramd = RollingHistogram<float64_t>;

#endif // HISTOGRAM_HPP_