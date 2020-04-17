#include <gtest/gtest.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <cmath>
#include <cstdio>
#include <iostream>

// Data generated using the following octave code.
//   randn('seed', 23497);
//   m = 0.3;
//   c = 0.1;
//   x=[0:0.075:5];
//   y = exp(m * x + c);
//   noise = randn(size(x)) * 0.2;
//   outlier_noise = rand(size(x)) < 0.05;
//   y_observed = y + noise + outlier_noise;
//   data = [x', y_observed'];
const int kNumObservations = 67;
const double data[]        = {
    0.000000e+00, 1.133898e+00,
    7.500000e-02, 1.334902e+00,
    1.500000e-01, 1.213546e+00,
    2.250000e-01, 1.252016e+00,
    3.000000e-01, 1.392265e+00,
    3.750000e-01, 1.314458e+00,
    4.500000e-01, 1.472541e+00,
    5.250000e-01, 1.536218e+00,
    6.000000e-01, 1.355679e+00,
    6.750000e-01, 1.463566e+00,
    7.500000e-01, 1.490201e+00,
    8.250000e-01, 1.658699e+00,
    9.000000e-01, 1.067574e+00,
    9.750000e-01, 1.464629e+00,
    1.050000e+00, 1.402653e+00,
    1.125000e+00, 1.713141e+00,
    1.200000e+00, 1.527021e+00,
    1.275000e+00, 1.702632e+00,
    1.350000e+00, 1.423899e+00,
    1.425000e+00, 5.543078e+00, // Outlier point
    1.500000e+00, 5.664015e+00, // Outlier point
    1.575000e+00, 1.732484e+00,
    1.650000e+00, 1.543296e+00,
    1.725000e+00, 1.959523e+00,
    1.800000e+00, 1.685132e+00,
    1.875000e+00, 1.951791e+00,
    1.950000e+00, 2.095346e+00,
    2.025000e+00, 2.361460e+00,
    2.100000e+00, 2.169119e+00,
    2.175000e+00, 2.061745e+00,
    2.250000e+00, 2.178641e+00,
    2.325000e+00, 2.104346e+00,
    2.400000e+00, 2.584470e+00,
    2.475000e+00, 1.914158e+00,
    2.550000e+00, 2.368375e+00,
    2.625000e+00, 2.686125e+00,
    2.700000e+00, 2.712395e+00,
    2.775000e+00, 2.499511e+00,
    2.850000e+00, 2.558897e+00,
    2.925000e+00, 2.309154e+00,
    3.000000e+00, 2.869503e+00,
    3.075000e+00, 3.116645e+00,
    3.150000e+00, 3.094907e+00,
    3.225000e+00, 2.471759e+00,
    3.300000e+00, 3.017131e+00,
    3.375000e+00, 3.232381e+00,
    3.450000e+00, 2.944596e+00,
    3.525000e+00, 3.385343e+00,
    3.600000e+00, 3.199826e+00,
    3.675000e+00, 3.423039e+00,
    3.750000e+00, 3.621552e+00,
    3.825000e+00, 3.559255e+00,
    3.900000e+00, 3.530713e+00,
    3.975000e+00, 3.561766e+00,
    4.050000e+00, 3.544574e+00,
    4.125000e+00, 3.867945e+00,
    4.200000e+00, 4.049776e+00,
    4.275000e+00, 3.885601e+00,
    4.350000e+00, 4.110505e+00,
    4.425000e+00, 4.345320e+00,
    4.500000e+00, 4.161241e+00,
    4.575000e+00, 4.363407e+00,
    4.650000e+00, 4.161576e+00,
    4.725000e+00, 4.619728e+00,
    4.800000e+00, 4.737410e+00,
    4.875000e+00, 4.727863e+00,
    4.950000e+00, 4.669206e+00};
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
struct ExponentialResidual
{
    ExponentialResidual(double x, double y)
        : x_(x), y_(y) {}
    template <typename T>
    bool operator()(const T* const m,
                    const T* const c,
                    T* residual) const
    {
        residual[0] = y_ - exp(m[0] * x_ + c[0]);
        return true;
    }

private:
    const double x_;
    const double y_;
};

TEST(TestCeres, robustCurveFitting)
{
    double m = 0.0;
    double c = 0.0;
    Problem problem;
    for (int i = 0; i < kNumObservations; ++i)
    {
        CostFunction* cost_function =
            new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                new ExponentialResidual(data[2 * i], data[2 * i + 1]));
        problem.AddResidualBlock(cost_function,
                                 new CauchyLoss(0.5),
                                 &m, &c);
    }
    Solver::Options options;
    options.linear_solver_type           = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << m << " c: " << c << "\n";
}

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
DEFINE_double(robust_threshold, 0.0, "Robust loss parameter. Set to 0 for "
                                     "normal squared error (no robustification).");
// The cost for a single sample. The returned residual is related to the
// distance of the point from the circle (passed in as x, y, m parameters).
//
// Note that the radius is parameterized as r = m^2 to constrain the radius to
// positive values.
class DistanceFromCircleCost
{
public:
    DistanceFromCircleCost(double xx, double yy)
        : xx_(xx), yy_(yy) {}
    template <typename T>
    bool operator()(const T* const x,
                    const T* const y,
                    const T* const m, // r = m^2
                    T* residual) const
    {
        // Since the radius is parameterized as m^2, unpack m to get r.
        T r = *m * *m;
        // Get the position of the sample in the circle's coordinate system.
        T xp = xx_ - *x;
        T yp = yy_ - *y;
        // It is tempting to use the following cost:
        //
        //   residual[0] = r - sqrt(xp*xp + yp*yp);
        //
        // which is the distance of the sample from the circle. This works
        // reasonably well, but the sqrt() adds strong nonlinearities to the cost
        // function. Instead, a different cost is used, which while not strictly a
        // distance in the metric sense (it has units distance^2) it produces more
        // robust fits when there are outliers. This is because the cost surface is
        // more convex.
        residual[0] = r * r - xp * xp - yp * yp;
        return true;
    }

private:
    // The measured x,y coordinate that should be on the circle.
    double xx_, yy_;
};

TEST(TestCeres, circleFit)
{
    // The input format is simple text. Feed on standard in:
    //
    //   x_initial y_initial r_initial
    //   x1 y1
    //   x2 y2
    //   y3 y3
    //   ...
    //
    // And the result after solving will be printed to stdout:
    //
    //   x y r
    //

    double x, y, r;
    /**
     * input from standard in
    if (scanf("%lg %lg %lg", &x, &y, &r) != 3)
    {
        fprintf(stderr, "Couldn't read first line.\n");
    }
    fprintf(stderr, "Got x, y, r %lg, %lg, %lg\n", x, y, r);
    */

    x = 0.0;
    y = 0.0;
    r = 1.0;

    // Save initial values for comparison.
    double initial_x = x;
    double initial_y = y;
    double initial_r = r;
    // Parameterize r as m^2 so that it can't be negative.
    double m = sqrt(r);
    Problem problem;
    // Configure the loss function.
    LossFunction* loss = NULL;
    if (FLAGS_robust_threshold)
    {
        loss = new CauchyLoss(FLAGS_robust_threshold);
    }

    // Add the residuals.
    /**
     * input from standard in
    while (scanf("%lf %lf\n", &xx, &yy) == 2)
    {
        CostFunction* cost =
            new AutoDiffCostFunction<DistanceFromCircleCost, 1, 1, 1, 1>(
                new DistanceFromCircleCost(xx, yy));
        problem.AddResidualBlock(cost, loss, &x, &y, &m);
        num_points++;
    }
    std::cout << "Got " << num_points << " points.\n";
    */

    auto addPoint = [&x, &y, &m, &loss, &problem](double xx, double yy) {
        CostFunction* cost =
            new AutoDiffCostFunction<DistanceFromCircleCost, 1, 1, 1, 1>(
                new DistanceFromCircleCost(xx, yy));

        problem.AddResidualBlock(cost, loss, &x, &y, &m);
    };

    addPoint(-1.0, 0.0);
    addPoint(1.0, 0.0);
    addPoint(0.0, 1.0);

    // Build and solve the problem.
    Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::DENSE_QR;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    // Recover r from m.
    r = m * m;
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    std::cout << "y : " << initial_y << " -> " << y << "\n";
    std::cout << "r : " << initial_r << " -> " << r << "\n";
}