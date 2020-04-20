#include "Model.hpp"

namespace aidrive
{

void Integrator::operator()(float32_t& dxdt, float32_t u) const
{
    dxdt = u;
}

void KinematicModel::operator()(Vector3f& dxdt,
                                const Vector3f& x,
                                float32_t v,
                                float32_t delta) const
{
    dxdt[0] = v * std::cos(x[2]);
    dxdt[1] = v * std::sin(x[2]);
    dxdt[2] = v / m_wheelBase * std::tan(delta);
}

Vector4f MotionModel::advance(const Vector4f& state,
                              float32_t a,
                              float32_t delta,
                              float32_t elapseTime) const
{
    std::vector<float32_t> finalState{state[0], state[1], state[2], state[3]};

    boost::numeric::odeint::integrate(CombinedModel(a, delta),
                                      finalState,
                                      0.f,
                                      elapseTime,
                                      1e-3f);

    return {finalState[0], finalState[1], finalState[2], finalState[3]};
}

void CurvatureModel::operator()(const std::vector<float32_t>& x,
                                std::vector<float32_t>& dxds,
                                float32_t /* s */) const
{
    dxds[0] = std::cos(x[2]);
    dxds[1] = std::sin(x[2]);
    dxds[2] = m_k;
}

std::vector<Vector3f> generatePolyline(const Vector3f& pose,
                                       float32_t k,
                                       float32_t deltaS,
                                       float32_t finalS)
{
    std::vector<Vector3f> poly{};

    float32_t steps = std::floor(finalS / deltaS);

    poly.push_back(pose);

    std::vector<float32_t> state{pose[0], pose[1], pose[2]};

    for (float32_t s = 0.0f; s < finalS; s += deltaS)
    {
        boost::numeric::odeint::integrate(CurvatureModel(k),
                                          state,
                                          s,
                                          s + deltaS,
                                          std::min(0.1f, deltaS));

        poly.push_back({state[0], state[1], state[2]});
    }

    return poly;
}

} // namespace aidrive