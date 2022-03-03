#ifndef SMOOTHNESS_COST_TEST_H
#define SMOOTHNESS_COST_TEST_H

#include "../NumericalDerivative.h"
#include "../cost/SmoothnessCost.h"
#include "TestUtil.h"

template <class T, int N> class SmoothnessCostDifferentiable : public differentiableFunction<T>
{
  public:
    smoothnessCostParameters<T> *P;
    progressCostParameters<T, N> *progressP;
    void set(smoothnessCostParameters<T> *P_in, progressCostParameters<T, N> *progressP_in)
    {
        P = P_in;
        progressP = progressP_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = smothnessCost<T>(x, P, progressP->stageWeights, N);
    }
};

template <class T, int N, int UB> class SmoothnessCostJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    smoothnessCostParameters<T> *P;
    progressCostParameters<T, N> *progressP;
    T delta_t_inv;
    void set(smoothnessCostParameters<T> *P_in, progressCostParameters<T, N> *progressP_in)
    {
        P = P_in;
        progressP = progressP_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T c, H[2 * N * UB];
        zeroVector<T>(f, 2 * N);
        zeroVector<T>(H, 2 * N * UB);
        c = smoothnessHessianContribution<T>(H, f, x, P, progressP->stageWeights, N, UB);
    }
};

template <class T, int N, int UB, int HL> inline T SmoothnessCostTestInstance(long int &seed)
{
    T x_mem[2 * (N + HL)], *x;
    x = x_mem + (2 * HL);
    smoothnessCostParameters<T> P;
    progressCostParameters<T, N> progressP;
    T k_p, v_ideal, margin_multiplier;

    T planning_horizon = ((T)3.0);
    T delta_t = ((T)planning_horizon) / ((T)N);
    T delta_t_inv = ((T)1.0) / delta_t;

    // k_p = randomScalar<T>(seed);
    k_p = progressP.k_p;
    // v_ideal= randomScalar<T>(seed);
    // v_ideal = ((T)2.5);
    v_ideal = ((T)2.5) + ((T)0.5) * randomScalar<T>(seed);
    // margin_multiplier = ((T)3.2);
    margin_multiplier = ((T)3.2) + ((T)0.5) * randomScalar<T>(seed);

    P.setDefault(delta_t_inv);

    randomVector<T>(x - (2 * HL), 2 * (N + HL), seed);
    // T y = randomScalar<T>(seed);

    T c1, c2, gbase[2 * N], g[2 * N], g2[2 * N], Hbase[2 * N * UB], H[2 * N * UB], H2[2 * N * UB], A2[2 * N * 2 * N];
    // Start with a base
    randomVector<T>(gbase, 2 * N, seed);
    randomVector<T>(Hbase, 2 * N * UB, seed);
    // Copy the base
    copyVector(g, gbase, 2 * N);
    copyVector(H, Hbase, 2 * N * UB);
    c1 = smothnessCost<T>(x, &P, progressP.stageWeights, N);

    // Add contribution to the base
    c2 = smoothnessHessianContribution<T>(H, g, x, &P, progressP.stageWeights, N, UB);

    // Subtract to get back the contribution
    subVectors<T>(g2, g, gbase, 2 * N);
    subVectors<T>(H2, H, Hbase, 2 * N * UB);
    // Convert Hessian contribution to regular matrix
    SymmetricBandedToRegular<T>(A2, H2, 2 * N, UB);

    SmoothnessCostDifferentiable<T, N> fun;
    fun.set(&P, &progressP);
    T xv_mem[2 * (N + HL)], *xv, f[1], J[2 * N];
    xv = xv_mem + (2 * HL);
    copyVector<T>(xv - (2 * HL), x - (2 * HL), 2 * (N + HL));
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-8));

    SmoothnessCostJacobianDifferentiable<T, N, UB> funJ;
    funJ.set(&P, &progressP);
    T fJ[2 * N], A[2 * N * 2 * N];
    numericalDerivative<T>(A, &funJ, fJ, xv, 2 * N, 2 * N, (1e-8));

    T e = normDiffVector<T>(&c1, &c2, 1);
    // e = scalarMax<T>(e, normDiffVector<T>(g2, J,2*N));
    e = scalarMax<T>(e, normDiffVector<T>(g2, J, 2 * N) / scalarMax<T>(1e-1, normOfVector(J, 2 * N)));
    // e = scalarMax<T>(e, normDiffVector<T>(A2,A,2*N*2*N));
    e = scalarMax<T>(e, normDiffVector<T>(A2, A, 2 * N * 2 * N) / scalarMax<T>(1e-1, normOfVector(A, 2 * N * 2 * N)));

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

template <class T> inline bool SmoothnessCostTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-5;        // 1e-6;
    T frac_tol = 0.9999; // 0.99;

    int N;
    int UB;

    succ = fail = 0;
    for (N = 1; N <= 1; N++)
        for (UB = 1; UB <= 1; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = SmoothnessCostTestInstance<T, 30, 10, 5>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = SmoothnessCostTestInstance<T, 30, 10, 5>(pre_seed);
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

inline bool TestSmoothnessCost()
{
    bool back;

    back = SmoothnessCostTest<double>();

    return (back);
}

#endif /*SMOOTHNESS_COST_TEST_H*/
