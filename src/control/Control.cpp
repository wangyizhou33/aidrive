#include <control/Control.hpp>
#include <model/Model.hpp>
#include <ceres/ceres.h>
#include <cmath>

namespace aidrive
{
namespace control
{

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

/**
 * bump function 
 * @param b result
 * @param x point at which the bump function is evaluated at
 * @param xc center of the bump
 * @param k scale of the bump
 * @param w width of the bump
 */
template <class T>
inline void bumpFunction1D(T& b, T x, T xc, T k, T w)
{
    T xmxc, den, rec;

    xmxc = x - xc;
    den  = ((T)4.0) / (w * w) * xmxc * xmxc - ((T)1.0);

    if (den < ((T)0.0))
    {
        rec = ((T)1.0) / den;
        b   = k * exp(rec);
    }
    else
    {
        b = ((T)0.0);
    }
}

template <class T>
inline T laneKeepCost(T x, T xc)
{
    // left bump
    T xc1 = (T)-1.5 + xc;
    T w1  = (T)0.6;
    T k1  = (T)8.0;

    // right bump
    T xc2 = (T)1.5 + xc;
    T w2  = (T)0.6;
    T k2  = (T)8.0;

    // center
    T xc3 = (T)0.0 + xc;
    T w3  = (T)3.0;
    T k3  = (T)-2.0;

    T b1, b2, b3;
    bumpFunction1D(b1, x, xc1, k1, w1);
    bumpFunction1D(b2, x, xc2, k2, w2);
    bumpFunction1D(b3, x, xc3, k3, w3);

    return b1 + b2 + b3 + (T)0.73;
}

///
class PathCost
{
public:
    PathCost(const std::vector<Vector3d>& reference,
             size_t N_,
             float64_t ds_,
             float64_t dk_)
        : m_reference(reference)
        , N(N_)
        , ds(ds_)
        , dk(dk_) {}

    template <typename T>
    bool operator()(const T* const ctrls,
                    T* residual) const
    {
        std::vector<T> state;
        state.resize(3);
        state[0] = T(0.0);
        state[1] = T(0.0);
        state[2] = T(0.0);

        const T DS = T(ds);

        for (size_t i = 0; i < N; ++i)
        {
            const Vector3d& tgt = m_reference[i];
            residual[i * 3 + 0] = (state[0] - tgt[0]);
            residual[i * 3 + 1] = (state[1] - tgt[1]);
            // residual[i * 3 + 1] = laneKeepCost(T(state[1]), T(tgt[1]));

            // evolution
            state[0] += cos(state[2]) * DS;
            state[1] += sin(state[2]) * DS;
            state[2] += ctrls[i] * DS;

            if (i > 0)
            {
                residual[i * 3 + 2] = T(dk) / DS * (ctrls[i] - ctrls[i - 1]);
            }
            else
            {
                residual[i * 3 + 2] = T(0.0);
            }
        }

        return true;
    }

private:
    const std::vector<Vector3d>& m_reference;
    size_t N;
    float64_t ds;
    float64_t dk{2.0}; // weight on curvature derivative
};

Controller::Controller()
{
    std::fill_n(&m_cmd[0], OPT_STEPS, 0.0);
}

std::vector<Vector3f>
Controller::optimize(const std::vector<Vector3f>& reference)
{
    // std::cout << "test bump ============" << std::endl;
    // for (float32_t x = -2.0f; x <= 2.0f; x+= .01f)
    // {
    //     float32_t y;

    //     // bumpFunction1D(y, x, 0.f, 5.0f, 3.0f);
    //     laneKeepCost(y, x, 0.f);
    //     std::cout << x << ", " << y << std::endl;
    // }

    // std::cout << "test bump ============" << std::endl;

    std::vector<Vector3f> pred{};
    std::vector<Vector3d> reference1{};

    // TODO: resample the reference to match ds and OPT_STEPS
    for (const auto& pt : reference)
    {
        reference1.push_back(Vector3d{pt[0], pt[1], pt[2]});
    }

    // initial condition
    // ego state is alway {0,0,0}
    pred.push_back(Vector3f{0.0f, 0.0f, 0.0f});

    float64_t ds{0.5f};

    Problem problem;

    CostFunction* pathCost =
        new AutoDiffCostFunction<PathCost,
                                 3 * OPT_STEPS, // residual dim {pos_x_err, pos_y_err, ctrl_dev, pos_x_err, ...}
                                 OPT_STEPS>(    // opt var dim m_cmd
            new PathCost(reference1, OPT_STEPS, ds, m_dk));
    problem.AddResidualBlock(pathCost,
                             NULL,
                             &m_cmd[0]);

    for (size_t i = 0; i < OPT_STEPS; ++i)
    {
        problem.SetParameterLowerBound(&m_cmd[0], i, -0.1);
        problem.SetParameterUpperBound(&m_cmd[0], i, 0.1);
    }

    Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations           = 10;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable())
    {
        std::cerr << "[ceres] solution not usable" << std::endl;
    }

    for (size_t i = 0; i < OPT_STEPS; ++i)
    {
        auto oldPt = pred.back();
        auto newPt = oneStep({oldPt[0], oldPt[1], oldPt[2]},
                             m_cmd[i],
                             ds);
        pred.emplace_back(newPt[0], newPt[1], newPt[2]);
    }

    return pred;
}

} // namespace control
} // namespace aidrive
