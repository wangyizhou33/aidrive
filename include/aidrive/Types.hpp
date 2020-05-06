#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <cstdint>
#include <Eigen/Dense>

typedef unsigned int uint32_t;
typedef int int32_t;
typedef int long int64_t;
typedef float float32_t;
typedef double float64_t;

namespace aidrive
{

using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector3d = Eigen::Vector3d;
using Vector4f = Eigen::Vector4f;
using Matrix2f = Eigen::Matrix<float32_t, 2, 2>;
using Matrix3f = Eigen::Matrix<float32_t, 3, 3>;

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

struct TrajectoryPoint
{
    Vector2f pos;
    float32_t hdg;
    float32_t v;
    float32_t a;
    int64_t t;
};

struct Trajectory
{
    int64_t timestamp; // std::chrono::microseconds
    std::vector<TrajectoryPoint> points;
};

} // namespace aidrive

#endif // TYPES_HPP