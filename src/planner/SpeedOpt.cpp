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
                        float32_t aInit)
{
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

    Problem problem;

    constexpr float64_t DISCOUNT          = 0.9;
    constexpr float64_t EQUALITY_WEIGHT   = 1000.0;
    constexpr float64_t INEQUALITY_WEIGHT = 1000.0;
    constexpr float64_t JERK_WEIGHT       = 1.0;
    constexpr float64_t VEL_WEIGHT        = 1.5;

    CostFunction* cost2 =
        new AutoDiffCostFunction<IntegralCost, 1, 1, 1, 1>(
            new IntegralCost(DT));

    LossFunction* loss2 = new ScaledLoss(
        new TrivialLoss(),
        EQUALITY_WEIGHT,
        ceres::TAKE_OWNERSHIP);

    // initial condition constraint
    {
        CostFunction* cost4 =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(0.0));

        problem.AddResidualBlock(cost4, loss2, &d[0]); // always start at d = 0

        CostFunction* cost6 =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(vInit));

        problem.AddResidualBlock(cost6, loss2, &v[0]); // match vInit

        CostFunction* cost7 =
            new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                new TrivialResidual(aInit));

        problem.AddResidualBlock(cost7, loss2, &a[0]); // match aInit
    }

    constexpr float64_t ALIM = 1.5f;

    LossFunction* loss8 = new ScaledLoss(
        new TrivialLoss(),
        INEQUALITY_WEIGHT,
        ceres::TAKE_OWNERSHIP);

    for (int32_t tIdx = 1; tIdx < OPT_STEPS; ++tIdx)
    {

        // CostFunction* cost1 =
        //     new AutoDiffCostFunction<ProgressCost, 1, 1>(
        //         new ProgressCost(tIdx, discount));

        // problem.AddResidualBlock(cost1, NULL, &d[tIdx]);

        // speed limit
        CostFunction* cost11 =
            new AutoDiffCostFunction<SpeedLimitCost, 1, 1, 1>(
                new SpeedLimitCost(405.f, -40.f, 1.f));
        problem.AddResidualBlock(cost11, loss8, &v[tIdx], &d[tIdx]);

        // kinematic constraint
        problem.AddResidualBlock(cost2, loss2, &d[tIdx], &d[tIdx - 1], &v[tIdx - 1]);
        problem.AddResidualBlock(cost2, loss2, &v[tIdx], &v[tIdx - 1], &a[tIdx - 1]);
        problem.AddResidualBlock(cost2, loss2, &a[tIdx], &a[tIdx - 1], &j[tIdx - 1]);

        // min w_j * sum_i || j_i ||^2
        {
            CostFunction* cost3 =
                new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                    new TrivialResidual(0.0));

            LossFunction* loss3 = new ScaledLoss(
                new TrivialLoss(),
                JERK_WEIGHT,
                ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(cost3, loss3, &j[tIdx - 1]);
        }

        // min w_v * sum_i || v_i - v_target||^2
        {
            CostFunction* cost5 =
                new AutoDiffCostFunction<TrivialResidual, 1, 1>(
                    new TrivialResidual(10.0));

            LossFunction* loss5 = new ScaledLoss(
                new TrivialLoss(),
                VEL_WEIGHT,
                ceres::TAKE_OWNERSHIP);

            problem.AddResidualBlock(cost5, loss5, &v[tIdx - 1]);
        }

        // add box constraint on a
        {
            CostFunction* cost8 =
                new AutoDiffCostFunction<InequalityResidual, 1, 1>(
                    new InequalityResidual(ALIM, InequalityResidual::LESS));

            problem.AddResidualBlock(cost8, loss8, &a[tIdx - 1]);

            CostFunction* cost9 =
                new AutoDiffCostFunction<InequalityResidual, 1, 1>(
                    new InequalityResidual(-ALIM, InequalityResidual::GREATER));

            problem.AddResidualBlock(cost9, loss8, &a[tIdx - 1]);
        }

        // progress constraint
        {
            constexpr float64_t DLIM = 200.0f;
            CostFunction* cost10 =
                new AutoDiffCostFunction<InequalityResidual, 1, 1>(
                    new InequalityResidual(DLIM, InequalityResidual::LESS));

            problem.AddResidualBlock(cost10, loss8, &d[tIdx]);
        }
    }

    // Build and solve the problem.
    Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations           = 100;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::copy(&d[0], &d[OPT_STEPS], &m_d[0]);
    std::copy(&v[0], &v[OPT_STEPS], &m_v[0]);
    std::copy(&a[0], &a[OPT_STEPS], &m_a[0]);
    std::copy(&j[0], &j[OPT_STEPS], &m_j[0]);
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

} // namespace planner
} // namespace aidrive