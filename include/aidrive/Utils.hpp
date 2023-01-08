#ifndef UTILS_HPP_
#define UTILS_HPP_

template <typename FloatT>
inline constexpr bool isFloatEqual(FloatT const a, FloatT b, FloatT const tolerance = std::numeric_limits<FloatT>::epsilon()) noexcept
{
    return std::abs(a - b) <= tolerance;
}

#endif // UTILS_HPP_