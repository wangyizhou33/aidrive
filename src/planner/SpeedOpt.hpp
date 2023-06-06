#ifndef PLANNER_SPEEDOPT_HPP_
#define PLANNER_SPEEDOPT_HPP_

#include <aidrive/Types.hpp>

namespace aidrive
{
namespace planner
{

///
class SpeedOpt
{
public:
    SpeedOpt();

    // this interface works directly with imgui::CheckBox
    bool& getCurveSpeedToggle() { return m_curveSpeedOn; };

    void optimize(float32_t vInit,
                  float32_t aInit,
                  float32_t xc,
                  float32_t vc);

    std::vector<float32_t> getT() const;
    std::vector<float32_t> getD() const;
    std::vector<float32_t> getV() const;
    std::vector<float32_t> getA() const;
    std::vector<float32_t> getJ() const;
    std::vector<float32_t> getObsD() const;
    std::vector<float32_t> getObsV() const;

    // {s1, v1}, {s2, v2} ...
    std::vector<float32_t> getSpeedLimit() const;
    std::vector<float32_t> getVAsFunctionOfD() const;

    float32_t& getRho() { return m_rho; };
    float32_t& getEqualityWeight() { return EQUALITY_WEIGHT; };
    float32_t& getInequalityWeight() { return INEQUALITY_WEIGHT; };
    float32_t& getJerkWeight() { return JERK_WEIGHT; };
    float32_t& getAccelWeight() { return ACCEL_WEIGHT; };
    float32_t& getVelWeight() { return VEL_WEIGHT; };
    float32_t& getObsWeight() { return OBS_WEIGHT; };
    int32_t& getCutInTime() { return CUT_IN_TIME; };

private:
    static constexpr size_t OPT_STEPS = 60;
    static constexpr float64_t DT     = 0.1;
    float64_t m_d[OPT_STEPS]{};
    float64_t m_v[OPT_STEPS]{};
    float64_t m_a[OPT_STEPS]{};
    float64_t m_j[OPT_STEPS]{};

    bool m_curveSpeedOn{false};

    float32_t m_xc{};
    float32_t m_vc{};
    float32_t m_rho{0.1};

    float32_t EQUALITY_WEIGHT   = 5000000.0;
    float32_t INEQUALITY_WEIGHT = 500000.0;
    float32_t JERK_WEIGHT       = 1000.0;
    float32_t ACCEL_WEIGHT      = 1.0;
    float32_t VEL_WEIGHT        = 1.5;
    float32_t OBS_WEIGHT        = 500.;
    int32_t CUT_IN_TIME          = 0;

}; // class SpeedOpt

} // namespace planner
} // namespace aidrive

#endif // PLANNER_SPEEDOPT_HPP_