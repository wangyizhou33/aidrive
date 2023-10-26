#ifndef CONTROL_FIT_HPP_
#define CONTROL_FIT_HPP_

#include <aidrive/Types.hpp>

namespace aidrive
{

class Fit
{
public:
    Fit() = default;

    std::vector<Vector3f> optimize(float64_t k1,
                                   float64_t x2,
                                   float64_t y2,
                                   float64_t theta2,
                                   float64_t k2,
                                   float64_t klim,
                                   float64_t dklim,
                                   float64_t& xx2,
                                   float64_t& yy2,
                                   float64_t& ttheta2,
                                   float64_t& kk2);

private:
}; // class Fit

} // namespace aidrive

#endif //