#ifndef MODEL_MODEL_HPP_
#define MODEL_MODEL_HPP_

#include <aidrive/Types.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>

#include <vector>
#include <cmath>
namespace aidrive
{

using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;

class Integrator
{
public:
    Integrator() = default;

    /**
     * @brief simple x_dot = u
     */
    void operator()(float32_t& dxdt, float32_t u) const;

}; // class Integrator

class KinematicModel
{
public:
    KinematicModel() = default;

    /**
     * @brief governing ode
     * state x \in R^3
     * x[0] = x, x[1] = y, x[2] = theta
     * input speed v, steering delta
     */
    void operator()(Vector3f& dxdt,
                    const Vector3f& x,
                    float32_t v,
                    float32_t delta) const;

private:
    float32_t m_wheelBase{2.8f};

}; // class KinematicModel

class MotionModel
{

public:
    MotionModel() = default;

    Vector4f advance(const Vector4f& state,
                     float32_t a,
                     float32_t delta,
                     float32_t elapseTime) const; // inputs remain the same during the time

private:
    class CombinedModel
    {
    public:
        CombinedModel(float32_t a, float32_t delta)
            : m_a(a)
            , m_delta(delta){};
        /**
         * @brief combined ode
         * state {x, y, theta, v}
         * input a, delta
         */
        void operator()(const std::vector<float32_t>& x,
                        std::vector<float32_t>& dxdt,
                        float32_t /* t */) const
        {
            float32_t vDot{};
            float32_t v = x[3];
            m_integrator(vDot, m_a);

            const Vector3f xYTheta{x[0], x[1], x[2]};
            Vector3f xYThetaDot{};
            m_kinModel(xYThetaDot, xYTheta, v, m_delta);

            dxdt[0] = xYThetaDot[0];
            dxdt[1] = xYThetaDot[1];
            dxdt[2] = xYThetaDot[2];
            dxdt[3] = vDot;
        }

    private:
        float32_t m_a{};
        float32_t m_delta{};
        Integrator m_integrator{};
        KinematicModel m_kinModel{};
    }; // class CombinedModel

}; // class Model

} // namespace aidrive

#endif // MODEL_MODEL_HPP_