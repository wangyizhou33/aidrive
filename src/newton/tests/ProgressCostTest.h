#ifndef PROGRESS_COST_TEST_H
#define PROGRESS_COST_TEST_H

#include "../NumericalDerivative.h"
#include "../cost/ProgressCost.h"
#include "TestUtil.h"

template <class T, int N> class ProgressCostDifferentiable : public differentiableFunction<T>
{
  public:
    progressCostParameters<T, N> *P;
    void set(progressCostParameters<T, N> *P_in)
    {
        P = P_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = trajectoryProgressCost<T, N>(x, P);
    }
};

template <class T, int N, int UB> class ProgressCostJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    progressCostParameters<T, N> *P;
    void set(progressCostParameters<T, N> *P_in)
    {
        P = P_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T c, H[2 * N * UB];
        zeroVector<T>(f, 2 * N);
        zeroVector<T>(H, 2 * N * UB);
        c = trajectoryProgressHessianContribution<T, N>(H, f, x, P);
    }
};

template <class T, int N, int UB> inline T ProgressCostTestInstance(long int &seed)
{
    T x[2 * N];
    progressCostParameters<T, N> P;

    randomVector<T>(x, 2 * N, seed);
    // T y = randomScalar<T>(seed);

    T c1, c2, gbase[2 * N], g[2 * N], g2[2 * N], Hbase[2 * N * UB], H[2 * N * UB], H2[2 * N * UB], A2[2 * N * 2 * N];
    // Start with a base
    randomVector<T>(gbase, 2 * N, seed);
    randomVector<T>(Hbase, 2 * N * UB, seed);
    // Copy the base
    copyVector(g, gbase, 2 * N);
    copyVector(H, Hbase, 2 * N * UB);
    c1 = trajectoryProgressCost<T, N>(x, &P);
    // Add contribution to the base
    c2 = trajectoryProgressHessianContribution<T, N>(H, g, x, &P);
    // Subtract to get back the contribution
    subVectors<T>(g2, g, gbase, 2 * N);
    subVectors<T>(H2, H, Hbase, 2 * N * UB);
    // Convert Hessian contribution to regular matrix
    SymmetricBandedToRegular<T>(A2, H2, 2 * N, UB);

    ProgressCostDifferentiable<T, N> fun;
    fun.set(&P);
    T xv[2 * N], f[1], J[2 * N];
    copyVector<T>(xv, x, 2 * N);
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-8));

    ProgressCostJacobianDifferentiable<T, N, UB> funJ;
    funJ.set(&P);
    T fJ[2 * N], A[2 * N * 2 * N];
    numericalDerivative<T>(A, &funJ, fJ, xv, 2 * N, 2 * N, (1e-8));

    T e = normDiffVector<T>(&c1, &c2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(g2, J, 2 * N));
    e = scalarMax<T>(e, normDiffVector<T>(A2, A, 2 * N * 2 * N));

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

template <class T> inline bool ProgressCostTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-7;        // 1e-6;
    T frac_tol = 0.9999; // 0.99;

    int N;
    int UB;

    // N = 10;
    // UB = 4;

    succ = fail = 0;
    for (N = 1; N <= 1; N++)
        for (UB = 1; UB <= 10; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = ProgressCostTestInstance<T, 30, 4>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = ProgressCostTestInstance<T, 30, 4>(pre_seed);
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

inline bool TestProgressCost()
{
    bool back;

    back = ProgressCostTest<double>();

    return (back);
}

#endif /*PROGRESS_COST_TEST_H*/
