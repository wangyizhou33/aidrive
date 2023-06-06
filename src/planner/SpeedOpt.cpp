#include <planner/SpeedOpt.hpp>
#include <ceres/ceres.h>
#include <cmath>
namespace aidrive
{
namespace planner
{

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::ScaledLoss;
using ceres::Solve;
using ceres::Solver;
using ceres::TrivialLoss;

#define SMOOTH_BARRIER_FUNCTION_MAX 1e10
#define ACTOR_AVOIDANCE_MAX 1e10

template <class T>
inline int smoothBarrierFunction(T& h, T x, T k_h = ((T)1.0), T x_onset = ((T)0.0), T x_max = ((T)1.0),
                                 T h_max = ((T)SMOOTH_BARRIER_FUNCTION_MAX))
{
    T xo_mx, xm_mx;

    xo_mx = x_onset - x;
    if (xo_mx >= T(0))
    {
        h = ((T)0.0);
        return (0);
    }

    xm_mx = x_max - x;
    if (xm_mx <= T(0))
    {
        h = ((T)h_max);
        return (2);
    }

    T s = x_max - x_onset;
    h   = k_h * exp(s * (((T)1.0) / xo_mx + ((T)1.0) / xm_mx));
    return (1);
}

struct ActorAvoidanceCostParametersCore
{
    float32_t beta_cm = 2.0f;
    float32_t beta_m  = 2.0f;
    float32_t beta_cs = 3.0f;
    float32_t beta_s  = 3.0f;
};

template <class T>
inline int actorAvoidanceCostCore(T& b, T x, T v, T xc, T vc, const ActorAvoidanceCostParametersCore* P)
{
    T d;

    d = xc - x;

    if (d <= ((T)0.0))
    {
        // Failed to separate case
        b = ((T)(ACTOR_AVOIDANCE_MAX));
        return (0);
    }
    if (vc >= ((T)0.0))
    {
        if (v <= ((T)0.0))
        {
            // Separating case
            b = ((T)0.0);
            return (1);
        }
        // Chasing case
        b = v * v / (T(P->beta_s) * (((T)2.0) * d + vc * vc / T(P->beta_cm)));
        return (2);
    }
    if (v <= ((T)0.0))
    {
        // Being chased case
        b = vc * vc / (T(P->beta_cs) * (((T)2.0) * d + v * v / T(P->beta_m)));
        return (3);
    }
    // Antagonists case
    b = ((T)0.5) / d * (v * v / T(P->beta_s) + vc * vc / T(P->beta_cs));
    return (4);
}

/// use DavidCost with equality constraint only
/// disable everything else
class DavidCost
{
public:
    DavidCost(int _i, float32_t _xc, float32_t _vc, float32_t _rho)
        : i(_i)
        , xc(_xc)
        , vc(_vc)
        , rho(_rho){};

    template <typename T>
    bool operator()(const T* const x,
                    const T* const v,
                    T* residual) const
    {
        // min |D - d| is effectively max |d|
        T progress = T(-kp) * T(rho) * x[0];

        T beta{};
        actorAvoidanceCostCore<T>(beta, x[0], v[0], T(xc), T(vc), &P);

        T barrier{};
        smoothBarrierFunction<T>(barrier, beta, T(0.1f));

        T actorAvoidanceCost = T(1.0) * T(i) * T(rho) * barrier;

        residual[0] = T(100) * exp(progress + actorAvoidanceCost);

        return true;
    }

private:
    int32_t i;

    float32_t kp = 0.5f;
    float32_t rho{};

    float32_t xc{};
    float32_t vc{};

    ActorAvoidanceCostParametersCore P{};

}; // class DavidCost

class IdmCost
{
public:
    IdmCost(int _i, float32_t _xc, float32_t _vc)
        : i(_i)
        , xc(_xc)
        , vc(_vc)
    {
    }

    template <typename T>
    bool operator()(const T* const x,
                    const T* const v,
                    T* residual) const
    {
        T sStar = calDesiredS<T>(v[0], T(vc));

        residual[0] = std::min(T(xc) - x[0] - sStar, T(0.));
        return true;
    }

private:
    template <typename T>
    T calDesiredS(T ve, T vo) const
    {
        T d = sqrt(T(a) * T(b));
        return ve * T(timegap) + (ve - vo) / T(2.) / d;
    }

    int32_t i;

    float32_t xc{};
    float32_t vc{};

    float32_t a{1.0f};
    float32_t b{1.4f};
    float32_t timegap{1.4f};

}; // IdmCost

///
class ProgressCost
{
public:
    ProgressCost(int32_t t_, float64_t discount_)
        : t(t_)
        , discount(discount_){};

    template <typename T>
    bool operator()(const T* const d,
                    T* residual) const
    {
        // min |D - d| is effectively max |d|
        residual[0] = T(5) * std::pow(discount, t) * (D - d[0]);
        return true;
    }

private:
    int32_t t{};
    float64_t discount{};
    static constexpr float64_t D{200.0};
};

///
class IntegralCost
{
public:
    IntegralCost(float64_t dt_)
        : dt(dt_){};

    template <typename T>
    bool operator()(const T* const x1, // x_{t+t}
                    const T* const x2, // x_{t}
                    const T* const dx, // dx/dt at t
                    T* residual) const
    {
        // min |D - d| is effectively max |d|
        residual[0] = x1[0] - (x2[0] + dx[0] * dt);
        return true;
    }

private:
    float64_t dt{};
};

///
class TrivialResidual
{
public:
    TrivialResidual(float64_t target_)
        : target(target_){};

    template <typename T>
    bool operator()(const T* const j,
                    T* residual) const
    {
        residual[0] = j[0] - target;
        return true;
    }

private:
    float64_t target{};
};

/// penalize violation of in <= target, or in >= target
class InequalityResidual
{
public:
    enum Sign
    {
        LESS,
        GREATER
    };

    InequalityResidual(float64_t target_, Sign sign_ = LESS)
        : target(target_)
        , sign(sign_){};

    template <typename T>
    bool operator()(const T* const in,
                    T* residual) const
    {
        if (sign == LESS)
        {
            residual[0] = std::max(T(0.0), in[0] - target);
        }
        else // GREATER
        {
            residual[0] = std::min(T(0.0), in[0] - target);
        }
        return true;
    }

private:
    float64_t target{};
    Sign sign{};
};

class SpeedLimitCost
{
public:
    SpeedLimitCost(float _a0, float _a1, float _a2)
        : a0(_a0), a1(_a1), a2(_a2){};

    template <typename T>
    bool operator()(const T* const v,
                    const T* const s,
                    T* residual) const
    {
        residual[0] = std::max(T(0.0), v[0] - (T(a0) + T(a1) * s[0] + T(a2) * s[0] * s[0]));
        return true;
    }

private:
    float32_t a0, a1, a2; // v_max = a0 + a1 * x + a2 * x^2
};

SpeedOpt::SpeedOpt()
{
    std::fill_n(&m_d[0], OPT_STEPS, 0.0);
    std::fill_n(&m_v[0], OPT_STEPS, 0.0);
    std::fill_n(&m_a[0], OPT_STEPS, 0.0);
    std::fill_n(&m_j[0], OPT_STEPS, 0.0);
}

void SpeedOpt::optimize(float32_t vInit,
                        float32_t aInit,
                        float32_t xc,
                        float32_t vc)
{

    // std::cout << "test barrier ============" << std::endl;
    // for (float32_t x = -2.0f; x <= 2.0f; x+= .01f)
    // {
    //     float32_t y;

    //     smoothBarrierFunction(y, x, 1.f/8.f);
    //     std::cout << x << ", " << y - x << std::endl;
    // }

    // std::cout << "test barrier  ============" << std::endl;
    m_xc = xc;
    m_vc = vc;

    float32_t beta{};
    ActorAvoidanceCostParametersCore P{};
    actorAvoidanceCostCore<float32_t>(beta, 0, vInit, xc, vc, &P);
    // std::cout << xc << " " << vc << " " << beta << std::endl;

    float64_t d[OPT_STEPS]{};
    float64_t v[OPT_STEPS]{};
    float64_t a[OPT_STEPS]{};
    float64_t j[OPT_STEPS]{};

    std::copy(&m_d[0], &m_d[OPT_STEPS], &d[0]);
    std::copy(&m_v[0], &m_v[OPT_STEPS], &v[0]);
    std::copy(&m_a[0], &m_a[OPT_STEPS], &a[0]);
    std::copy(&m_j[0], &m_j[OPT_STEPS], &j[0]);

    // set initial condition for v[0], a[0], j[0]
    // now zero-initialize them
    constexpr float64_t ALIM = 2.5f;

    Problem problem;
    constexpr float64_t DISCOUNT = 0.9;

    CostFunction* kinematicCost =
        new AutoDiffCostFunction<IntegralCost, 1, 1, 1, 1>(
            new IntegralCost(DT));

    LossFunction* equalityLoss = new ScaledLoss(
        new TrivialLoss(),
        EQUALITY_WEIGHT,
        ceres::TAKE_OWNERSHIP);

    // initial condition constraint
    {
        CostFunction* dInitEquality =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(0.0));

        problem.AddResidualBlock(dInitEquality, equalityLoss, &d[0]); // always start at d = 0

        CostFunction* vInitEquality =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(vInit));

        problem.AddResidualBlock(vInitEquality, equalityLoss, &v[0]); // match vInit

        CostFunction* aInitEquality =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(aInit));

        problem.AddResidualBlock(aInitEquality, equalityLoss, &a[0]); // match aInit

        CostFunction* jInitEquality =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(0.0));

        problem.AddResidualBlock(jInitEquality, equalityLoss, &j[0]); // match aInit
    }

    LossFunction* loss8 = new ScaledLoss(
        new TrivialLoss(),
        INEQUALITY_WEIGHT,
        ceres::TAKE_OWNERSHIP);

    for (int32_t tIdx = CUT_IN_TIME; tIdx < OPT_STEPS; ++tIdx)
    {
        // obstacle cost
        LossFunction* loss1 = new ScaledLoss(
            new TrivialLoss(),
            OBS_WEIGHT,
            ceres::TAKE_OWNERSHIP);

        CostFunction* cost1 =
            new AutoDiffCostFunction<IdmCost, 1, 1, 1>(
                new IdmCost(tIdx, std::max(0., xc + vc * tIdx * DT), vc));

        problem.AddResidualBlock(cost1, loss1, &d[tIdx], &v[tIdx]);
    }

    for (int32_t tIdx = 0; tIdx < OPT_STEPS; ++tIdx)
    {
        // add box constraint on a
        {
            CostFunction* cost8 =
                new AutoDiffCostFunction<InequalityResidual, 1, 1>(
                    new InequalityResidual(ALIM, InequalityResidual::LESS));

            problem.AddResidualBlock(cost8, loss8, &a[tIdx]);

            CostFunction* cost9 =
                new AutoDiffCostFunction<InequalityResidual, 1, 1>(
                    new InequalityResidual(-ALIM, InequalityResidual::GREATER));

            problem.AddResidualBlock(cost9, loss8, &a[tIdx]);

            CostFunction* cost10 =
                new AutoDiffCostFunction<InequalityResidual, 1, 1>(
                    new InequalityResidual(0.0, InequalityResidual::GREATER));
            problem.AddResidualBlock(cost10, loss8, &v[tIdx]);

            // CostFunction* cost11 =
            //     new AutoDiffCostFunction<InequalityResidual, 1, 1>(
            //         new InequalityResidual(, InequalityResidual::LESS));
            // problem.AddResidualBlock(cost10, loss8, &d[tIdx]);
        }

        // speed limit
        // if (m_curveSpeedOn)
        // {
        //     CostFunction* cost11 =
        //         new AutoDiffCostFunction<SpeedLimitCost, 1, 1, 1>(
        //             new SpeedLimitCost(405.f, -40.f, 1.f));
        //     problem.AddResidualBlock(cost11, loss8, &v[tIdx], &d[tIdx]);
        // }
        // min w_j * sum_i || j_i ||^2
        {
            CostFunction* cost3 =
                new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                    new TrivialResidual(0.0));

            LossFunction* loss3 = new ScaledLoss(
                new TrivialLoss(),
                JERK_WEIGHT,
                ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(cost3, loss3, &j[tIdx]);
        }

        // min w_j * sum_i || a_i ||^2
        {
            CostFunction* cost4 =
                new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                    new TrivialResidual(0.0));

            LossFunction* loss4 = new ScaledLoss(
                new TrivialLoss(),
                ACCEL_WEIGHT,
                ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(cost4, loss4, &a[tIdx]);
        }

        // min w_v * sum_i || v_i - v_target||^2
        {
            CostFunction* cost5 =
                new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                    new TrivialResidual(25.0)); // v limit sd

            LossFunction* loss5 = new ScaledLoss(
                new TrivialLoss(),
                VEL_WEIGHT,
                ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(cost5, loss5, &v[tIdx - 1]);
        }

        // progress constraint
        // {
        //     constexpr float64_t DLIM = 200.0f;
        //     CostFunction* cost10 =
        //         new AutoDiffCostFunction<InequalityResidual, 1, 1>(
        //             new InequalityResidual(DLIM, InequalityResidual::LESS));

        //     problem.AddResidualBlock(cost10, loss8, &d[tIdx]);
        // }
    }

    for (int32_t tIdx = 1; tIdx < OPT_STEPS; ++tIdx)
    {
        // kinematic constraint
        problem.AddResidualBlock(kinematicCost, equalityLoss, &d[tIdx], &d[tIdx - 1], &v[tIdx - 1]);
        problem.AddResidualBlock(kinematicCost, equalityLoss, &v[tIdx], &v[tIdx - 1], &a[tIdx - 1]);
        problem.AddResidualBlock(kinematicCost, equalityLoss, &a[tIdx], &a[tIdx - 1], &j[tIdx - 1]);
    }

    constexpr float32_t eps = 1e-1;

    // Build and solve the problem.
    Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations           = 100;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // std::cout << summary.BriefReport() << std::endl;

    std::copy(&d[0], &d[OPT_STEPS], &m_d[0]);
    std::copy(&v[0], &v[OPT_STEPS], &m_v[0]);
    std::copy(&a[0], &a[OPT_STEPS], &m_a[0]);
    std::copy(&j[0], &j[OPT_STEPS], &m_j[0]);
}

std::vector<float32_t> SpeedOpt::getT() const
{
    std::vector<float32_t> result{};
    result.resize(OPT_STEPS);
    for (size_t i = 0; i < OPT_STEPS; ++i)
    {
        result[i] = i * DT;
    }
    return result;
}

std::vector<float32_t> SpeedOpt::getD() const
{
    std::vector<float32_t> result{};
    result.resize(OPT_STEPS);
    std::copy(&m_d[0], &m_d[OPT_STEPS], result.begin());
    return result;
}

std::vector<float32_t> SpeedOpt::getV() const
{
    std::vector<float32_t> result{};
    result.resize(OPT_STEPS);
    std::copy(&m_v[0], &m_v[OPT_STEPS], result.begin());
    return result;
}

std::vector<float32_t> SpeedOpt::getA() const
{
    std::vector<float32_t> result{};
    result.resize(OPT_STEPS);
    std::copy(&m_a[0], &m_a[OPT_STEPS], result.begin());
    return result;
}

std::vector<float32_t> SpeedOpt::getJ() const
{
    std::vector<float32_t> result{};
    result.resize(OPT_STEPS);
    std::copy(&m_j[0], &m_j[OPT_STEPS], result.begin());
    return result;
}

std::vector<float32_t> SpeedOpt::getSpeedLimit() const
{
    std::vector<float32_t> ret{};
    for (size_t i = 0; i < 50; ++i)
    {
        float32_t s = 1.0 * i; // every 1 meter
        float32_t v = s * s - 40.0 * s + 405.0;

        ret.push_back(v);
    }

    return ret;
}

std::vector<float32_t> SpeedOpt::getVAsFunctionOfD() const
{
    std::vector<float32_t> ret{};

    ret.push_back(m_v[0]);
    for (size_t i = 1; i < 50; ++i)
    {
        float32_t s = 1.0 * i; // every 1 meter

        for (size_t j = 0; j < OPT_STEPS; ++j)
        {
            // find first d > s,
            if (m_d[j] > s)
            {
                std::cout << m_d[j] << std::endl;
                ret.push_back(m_v[j]);
                break;
            }
        }
    }
    return ret;
}

std::vector<float32_t> SpeedOpt::getObsD() const
{
    std::vector<float32_t> ret{};
    for (size_t i = 0; i < OPT_STEPS; ++i)
    {
        ret.push_back(std::max(0.f, m_xc + m_vc * static_cast<float32_t>(i * DT)));
    }

    return ret;
}

std::vector<float32_t> SpeedOpt::getObsV() const
{
    std::vector<float32_t> ret{};

    for (size_t i = 0; i < OPT_STEPS; ++i)
    {
        ret.push_back(m_vc);
    }

    return ret;
}

} // namespace planner
} // namespace aidrive