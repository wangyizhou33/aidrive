#ifndef DISTANCE_FOLLOWING_HPP_
#define DISTANCE_FOLLOWING_HPP_

#include <aidrive/Types.hpp>
#include <boost/numeric/odeint.hpp>
#include <cmath>

namespace aidrive
{

template <class T>
class ODEModel
{
public:
    ODEModel() = default;

    /** @brief
     *  x[0] = ego position,
     *  x[1] = ego speed
     *  x[2] = obstacle position
     *  x[3] = obstacle speed
     */
    void operator()(const std::vector<T>& x,
                    std::vector<T>& dxdt,
                    T /*t*/)
    {
        T a = m_idm.computeEgoAccel(x[2], x[3], x[0], x[1]);
        m_plant(x, dxdt, a);
    }

    T& getParamA() {return m_idm.a;};
    T& getParamB() {return m_idm.b;};
    T& getParamHeadway() {return m_idm.headway;};
    T& getParamDelta() {return m_idm.delta;};

private:
    class Plant
    {
    public:
        /**
         *  x[0] = ego position,
         *  x[1] = ego speed
         *  x[2] = obstacle position
         *  x[3] = obstacle speed
         *  a    = ego acceleration
         */
        void operator()(const std::vector<T>& x,
                        std::vector<T>& dxdt,
                        T a)
        {
            dxdt[0] = x[1];
            dxdt[1] = a;
            dxdt[2] = x[3];
            dxdt[3] = 0.0f;
        }

    } m_plant;

    class IDMLaw
    {
    public:
        T computeEgoAccel(T obsS, T obsV, T egoS, T egoV)
        {
            T deltaV = -obsV + egoV;
            T deltaS = -obsS + egoS;
            T sStar  = s0 + std::max(0.0, egoV * headway + egoV * deltaV / 2.0 / std::sqrt(a * b));
            T accel = a * (1.0 - std::pow((egoV / v0), delta) - std::pow(sStar / deltaS, 2.0));
            return (egoV >= 0.0) ? accel : 0.0;
        }

        T a       = 0.3;
        T b       = 2.0;
        T s0      = 2.0;
        T headway = 1.5;
        T v0      = 22.222;
        T delta   = 4.0;

    } m_idm;

}; // class ODEModel

} // namespace aidrive

#endif // DISTANCE_FOLLOWING_HPP_