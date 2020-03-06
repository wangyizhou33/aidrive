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

} // namespace aidrive