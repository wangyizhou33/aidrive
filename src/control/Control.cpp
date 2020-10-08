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

///
class PathCost
{
public:
    PathCost(const std::vector<Vector3d>& reference,
             size_t N_,
             float64_t ds_)
        : m_reference(reference)
        , N(N_)
        , ds(ds_) {}

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
            residual[i * 2]     = (state[0] - tgt[0]) * (state[0] - tgt[0]) +
                              (state[1] - tgt[1]) * (state[1] - tgt[1]);

            // evolution
            state[0] += cos(state[2]) * DS;
            state[1] += sin(state[2]) * DS;
            state[2] += ctrls[i] * DS;

            if (i > 0)
            {
                residual[i * 2 + 1] = T(dk) / DS * (ctrls[i] - ctrls[i - 1]);
            }
            else
            {
                residual[i * 2 + 1] = T(0.0);
            }
        }

        return true;
    }

private:
    const std::vector<Vector3d>& m_reference;
    size_t N;
    float64_t ds;
    const float64_t dk{2.0}; // weight on curvature derivative
};

Controller::Controller()
{
    std::fill_n(&m_cmd[0], OPT_STEPS, 0.0);
}

std::vector<Vector3f>
Controller::optimize(const std::vector<Vector3f>& reference)
{
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
                                 2 * OPT_STEPS, // residual dim {pos_err, ctrl_dev, pos_err, ...}
                                 OPT_STEPS>(    // opt var dim m_cmd
            new PathCost(reference1, OPT_STEPS, ds));
    problem.AddResidualBlock(pathCost,
                             NULL,
                             &m_cmd[0]);

    Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations           = 10;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

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
