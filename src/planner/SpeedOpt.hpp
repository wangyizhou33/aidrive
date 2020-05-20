#ifndef PLANNER_SPEEDOPT_HPP_
#define PLANNER_SPEEDOPT_HPP_

#include <aidrive/Types.hpp>

namespace aidrive
{
namespace planner
{

class SpeedOpt
{
public:
    SpeedOpt();

    void optimize(float32_t vInit,
                  float32_t aInit);

    std::vector<float32_t> getD() const;
    std::vector<float32_t> getV() const;
    std::vector<float32_t> getA() const;
    std::vector<float32_t> getJ() const;

private:
    static constexpr size_t OPT_STEPS = 50;
    static constexpr float64_t DT = 0.1;
    float64_t m_d[OPT_STEPS]{};
    float64_t m_v[OPT_STEPS]{};
    float64_t m_a[OPT_STEPS]{};
    float64_t m_j[OPT_STEPS]{};

}; // class SpeedOpt


} // namespace aidrive
} // namespace planner


#endif // PLANNER_SPEEDOPT_HPP_