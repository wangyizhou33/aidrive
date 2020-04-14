#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <cstdint>
#include <Eigen/Dense>

typedef float float32_t;
typedef double float64_t;

namespace aidrive
{

using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;
using Matrix2f = Eigen::Matrix<float32_t, 2, 2>;

struct Rect2f // no orientation
{
    float32_t length;
    float32_t width;
};

struct LineSeg2f
{
    Vector2f v0;
    Vector2f v1;
};

} // namespace aidrive

#endif // TYPES_HPP