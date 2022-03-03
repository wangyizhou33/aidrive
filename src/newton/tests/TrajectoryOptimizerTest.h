#ifndef TRAJECTORY_OPTIMIZER_TEST_H
#define TRAJECTORY_OPTIMIZER_TEST_H

#include "../NumericalDerivative.h"
#include "../TrajectoryOptimizer.h"
#include "TestUtil.h"

template <class T, int N> class TrajectoryOptimizerDifferentiable : public differentiableFunction<T>
{
  public:
    trajectoryOptimizer<T, N> *topt;
    void set(trajectoryOptimizer<T, N> *t_opt_in)
    {
        topt = t_opt_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = topt->computeCost(x);
    }
};

template <class T, int N, int UB> class TrajectoryOptimizerJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    trajectoryOptimizer<T, N> *topt;
    void set(trajectoryOptimizer<T, N> *t_opt_in)
    {
        topt = t_opt_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T c, H[2 * N * UB];
        // zeroVector<T>(f, 2 * N);
        // zeroVector<T>(H, 2 * N * UB);
        c = topt->computeHessian(x);
        copyVector<T>(f, topt->getGradient(), 2 * N);
        copyVector<T>(H, topt->getHessian(), 2 * N * UB);
    }
};

template <class T, int N, int UB, int HL> inline T TrajectoryOptimizerTestInstance(long int &seed)
{
    trajectoryOptimizer<T, N> topt;
    int nrXC[N];
    double XC_mem[4 * N];
    double *XC[N];

    T p[4];
    randomVector<T>(p, 4, seed);
    // p[0] *= ((T)0.5);
    // p[1] *= ((T)0.5);
    // p[2] *= ((T)0.5);
    p[3] *= ((T)0.5);
    // zeroVector<T>(p, 4);
    // p[0] = ((T)0.0);
    // p[1] = ((T)2.0);
    // p[2] = ((T)1.0);
    topt.getFrenetTransformer()->get()->set(p);

    for (int i = 0; i < N; i++)
    {
        nrXC[i] = 1;
        XC[i] = &XC_mem[4 * i];

        XC[i][0] = ((T)60.0) + ((T)2.0) * i;
        XC[i][1] = ((T)0.0);
        XC[i][2] = ((T)20.0);
        XC[i][3] = ((T)0.0);
    }

    topt.setContenderData(XC, nrXC);

    // topt.set_progress_cost(false);
    // topt.set_limit_cost(false);
    // topt.set_actor_avoidance_cost(false);

    topt.setVerboseLevel(0);

    // topt.limitP.lat_v_on = false;
    // topt.limitP.lat_nv_on = false;

    // topt.limitP.lon_a_on = false;
    // topt.limitP.lon_na_on = false;
    // topt.limitP.lat_a_on = false;
    // topt.limitP.lat_na_on = false;

    // topt.limitP.lon_j_on = false;
    // topt.limitP.lon_nj_on = false;
    // topt.limitP.lat_j_on = false;
    // topt.limitP.lat_nj_on = false;

    // cout << "Cost: ";
    // cout << topt.computeCost(topt.m_x) << endl;

    T x_mem[2 * (N + HL)], *x;
    x = x_mem + (2 * HL);

    randomVector<T>(x - (2 * HL), 2 * (N + HL), seed);
    // T y = randomScalar<T>(seed);

    T c1, c2, gbase[2 * N], g[2 * N], g2[2 * N], Hbase[2 * N * UB], H[2 * N * UB], H2[2 * N * UB], A2[2 * N * 2 * N];
    // Start with a base
    randomVector<T>(gbase, 2 * N, seed);
    randomVector<T>(Hbase, 2 * N * UB, seed);
    // Copy the base
    copyVector(g, gbase, 2 * N);
    copyVector(H, Hbase, 2 * N * UB);
    c1 = topt.computeCost(x);

    // Add contribution to the base
    c2 = topt.computeHessian(x);
    addVectors<T>(g, g, topt.getGradient(), 2 * N);
    addVectors<T>(H, H, topt.getHessian(), 2 * N * UB);

    // Subtract to get back the contribution
    subVectors<T>(g2, g, gbase, 2 * N);
    subVectors<T>(H2, H, Hbase, 2 * N * UB);
    // Convert Hessian contribution to regular matrix
    SymmetricBandedToRegular<T>(A2, H2, 2 * N, UB);

    TrajectoryOptimizerDifferentiable<T, N> fun;
    fun.set(&topt);
    T xv_mem[2 * (N + HL)], *xv, f[1], J[2 * N];
    xv = xv_mem + (2 * HL);
    copyVector<T>(xv - (2 * HL), x - (2 * HL), 2 * (N + HL));
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-8));

    TrajectoryOptimizerJacobianDifferentiable<T, N, UB> funJ;
    funJ.set(&topt);
    T fJ[2 * N], A[2 * N * 2 * N];
    numericalDerivative<T>(A, &funJ, fJ, xv, 2 * N, 2 * N, (1e-8));

    T e = normDiffVector<T>(&c1, &c2, 1);
    // if (c1 < 1e20)
    {
        // e = scalarMax<T>(e, normDiffVector<T>(g2, J,2*N));
        e = scalarMax<T>(e, normDiffVector<T>(g2, J, 2 * N) / scalarMax<T>(1e-1, normOfVector(J, 2 * N)));
        // e = scalarMax<T>(e, normDiffVector<T>(A2,A,2*N*2*N));
        e = scalarMax<T>(e,
                         normDiffVector<T>(A2, A, 2 * N * 2 * N) / scalarMax<T>(1e-1, normOfVector(A, 2 * N * 2 * N)));
    }

    // e = scalarMax<T>(e, normDiffVector<T>(dbdx,dbdx2,2));
    // e = scalarMax<T>(e, normDiffVector<T>(dbdx ,J, 2)/scalarMax<T>(1e-9,normOfVector(J,2)));
    // e = scalarMax<T>(e, normDiffVector<T>(H1,H2, 4)/scalarMax<T>(1e-9,normOfVector(H2,4)));

    // printMatrix<T>(A, 2*N, 2*N);
    // printMatrix<T>(A2, 2*N, 2*N);
    /*printVector<T>(x, N);
    printVector<T>(Ax, N);
    printVector<T>(y, N);
    */

    return (e);
}

template <class T> inline bool TrajectoryOptimizerTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-5;        // 1e-6;
    T frac_tol = 0.9999; // 0.99;

    int N;
    int UB;

    // N = 10;
    // UB = 4;

    succ = fail = 0;
    for (N = 1; N <= 1; N++)
        for (UB = 1; UB <= 1; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = TrajectoryOptimizerTestInstance<T, 30, 8, 3>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = TrajectoryOptimizerTestInstance<T, 30, 8, 3>(pre_seed);
                    fail++;
                }
                else
                    succ++;
            }
        }

    double frac = ((double)succ) / ((double)(succ + fail));
    if (frac > frac_tol)
    {
        return (true);
    }

    return (false);
}

inline bool TestTrajectoryOptimizer()
{
    bool back;

    back = TrajectoryOptimizerTest<double>();

    return (back);
}

#endif /*TRAJECTORY_OPTIMIZER_TEST_H*/
