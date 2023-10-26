#include <control/Fit.hpp>
#include <ceres/ceres.h>
#include <cmath>

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

namespace aidrive
{

template <class T>
bool terminate(T in, T thrsld)
{
    return in >= thrsld;
}

template <class T>
void evolve(T& x, T& y, T& psi, T& k, T dk, T ds)
{
    x += cos(psi) * ds;
    y += sin(psi) * ds;
    psi += k * ds;
    k += dk * ds;
}

class Cost
{
public:
    Cost(float64_t x,
         float64_t y,
         float64_t theta,
         float64_t k,
         float64_t initk,
         int32_t N,
         float64_t klim,
         float64_t dklim)
        : x_(x)
        , y_(y)
        , theta_(theta)
        , k_(k)
        , initk_(initk)
        , N_(N)
        , klim_(klim)
        , dklim_(dklim)
    {
    }

    template <typename T>
    bool
    operator()(const T* const dk,
               T* residual) const
    {
        std::vector<T> state;
        state.resize(4u);
        state[0] = T(0.0);    // x
        state[1] = T(0.0);    // y
        state[2] = T(0.0);    // theta
        state[3] = T(initk_); // k

        // integrate the kinetic eqn
        size_t i = 1u;
        for (; i < N_; ++i)
        {
            if (terminate(state[0],
                          T(x_ - DS)))
            {
                evolve(state[0], state[1], state[2], state[3], dk[i], dk[0]);
                residual[3 + i] = T(0.0) * state[3];

                break;
            }
            else
            {
                evolve(state[0], state[1], state[2], state[3], dk[i], T(DS)); // the only difference is the step size

                // l2 norm of dk
                residual[3 + i] = T(0.0) * state[3]; // the intent is to penalze dk
            }
        }

        // make the rest residual 0
        for (size_t j = i + 1u; j < N_ + 4u; ++j)
        {
            residual[j] = T(0.0);
        }

        residual[0] = (state[0] - T(x_)) * T(100);
        residual[1] = (state[1] - T(y_)) * T(100);
        residual[2] = (state[2] - T(theta_)) * T(100);
        residual[3] = (state[3] - T(k_)) * T(100);

        return true;
    }

private:
    float64_t DS{0.1};
    float64_t x_{};
    float64_t y_{};
    float64_t theta_{};
    float64_t k_{};
    float64_t initk_{};
    int32_t N_{};
    float64_t klim_{};
    float64_t dklim_{};
};

std::vector<Vector3f> Fit::optimize(float64_t k1,
                                    float64_t x2,
                                    float64_t y2,
                                    float64_t theta2,
                                    float64_t k2,
                                    float64_t klim,
                                    float64_t dklim,
                                    float64_t& xx2,
                                    float64_t& yy2,
                                    float64_t& ttheta2,
                                    float64_t& kk2)
{

    float64_t ds = 0.1f;

    float64_t length    = std::sqrt(x2 * x2 + y2 * y2);
    constexpr int32_t N = 50;

    // optimization variable
    // a few dk's at the end are not used
    // 0th element  : ds
    // 1st -> end   : dk
    std::array<float64_t, N> dk;

    std::fill(&dk[0], &dk[N], 0.0);

    CostFunction* poseCost =
        new AutoDiffCostFunction<Cost,
                                 4 + N, // residual dim {pos_x_err, pos_y_err, theta_err, k_err, dk_i ....}
                                 N>(    // opt var dim {dk_i}
            new Cost(x2, y2, theta2, k2, k1, N, 0.0, 0.0));

    Problem problem;

    problem.AddResidualBlock(poseCost,
                             NULL,
                             &dk[0]);

    for (size_t i = 1u; i < N; ++i)
    {
        problem.SetParameterLowerBound(&dk[0], i, -dklim);
        problem.SetParameterUpperBound(&dk[0], i, dklim);
    }
    // problem.SetParameterLowerBound(&dk[0], 0, 0.0);
    // problem.SetParameterUpperBound(&dk[0], 0, ds);

    Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations           = 500;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    std::vector<Vector3f> ret{};

    // recover geometry
    float64_t x_terminal = x2;
    k2                   = k1;
    x2                   = 0.0;
    y2                   = 0.0;
    theta2               = 0.0;
    for (size_t i = 1u; i < N; ++i)
    {
        if (terminate(x2, x_terminal - ds))
        {

            evolve(x2, y2, theta2, k2, dk[i], dk[0]);

            std::cout << " dk :" << dk[i]
                      << " x : " << x2
                      << " y : " << y2
                      << " theta: " << theta2
                      << " k: " << k2
                      << " ds: " << dk[0]
                      << std::endl;

            ret.push_back(Vector3f{x2, y2, 0.0f});

            break;
        }
        else
        {
            evolve(x2, y2, theta2, k2, dk[i], ds);

            std::cout << " dk :" << dk[i]
                      << " x : " << x2
                      << " y : " << y2
                      << " theta: " << theta2
                      << " k: " << k2
                      << " ds: " << dk[0]
                      << std::endl;

            ret.push_back(Vector3f{x2, y2, 0.0f});
        }
    }

    // return via param[out]
    xx2     = x2;
    yy2     = y2;
    ttheta2 = theta2;
    kk2     = k2;

    return ret;
}

} // namespace aidrive