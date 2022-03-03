#ifndef LIMIT_COSTS_TEST_H
#define LIMIT_COSTS_TEST_H

#include "../NumericalDerivative.h"
#include "../cost/LimitCosts.h"
#include "TestUtil.h"

template <class T, int N> class LimitCostDifferentiable : public differentiableFunction<T>
{
  public:
    limitCostParameters<T> *P;
    progressCostParameters<T, N> *progressP;
    T delta_t_inv;
    void set(limitCostParameters<T> *P_in, progressCostParameters<T, N> *progressP_in, T delta_t_inv_in)
    {
        P = P_in;
        progressP = progressP_in;
        delta_t_inv = delta_t_inv_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = speedLimitCost<T>(x, P, delta_t_inv, progressP->stageWeights, N);
    }
};

template <class T, int N, int UB> class LimitCostJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    limitCostParameters<T> *P;
    progressCostParameters<T, N> *progressP;
    T delta_t_inv;
    void set(limitCostParameters<T> *P_in, progressCostParameters<T, N> *progressP_in, T delta_t_inv_in)
    {
        P = P_in;
        progressP = progressP_in;
        delta_t_inv = delta_t_inv_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T c, H[2 * N * UB];
        zeroVector<T>(f, 2 * N);
        zeroVector<T>(H, 2 * N * UB);
        c = speedLimitHessianContribution<T>(H, f, x, P, delta_t_inv, progressP->stageWeights, N, UB);
    }
};

template <class T, int N, int UB, int HL> inline T LimitCostTestInstance(long int &seed)
{
    T x_mem[2 * (N + HL)], *x;
    x = x_mem + (2 * HL);
    limitCostParameters<T> P;
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

    /*P.setDefault(delta_t_inv,k_p,
        false,false,false,false,
        true ,false,false,false,
        false,false,false,false,
        false,false,false,false,
        0,0,0,0,
        v_ideal,0,0,0,
        0,0,0,0,
        0,0,0,0,
        margin_multiplier);*/

    randomVector<T>(x - (2 * HL), 2 * (N + HL), seed);
    // T y = randomScalar<T>(seed);

    T c1, c2, gbase[2 * N], g[2 * N], g2[2 * N], Hbase[2 * N * UB], H[2 * N * UB], H2[2 * N * UB], A2[2 * N * 2 * N];
    T c3, c4, gonly[2 * N], Honly[2 * N * UB], Aonly[2 * N * 2 * N];
    // Start with a base
    randomVector<T>(gbase, 2 * N, seed);
    randomVector<T>(Hbase, 2 * N * UB, seed);
    // Copy the base
    copyVector(g, gbase, 2 * N);
    copyVector(H, Hbase, 2 * N * UB);
    c1 = speedLimitCost<T>(x, &P, delta_t_inv, progressP.stageWeights, N);
    c3 = speedLimitOnlyCost<T>(x, &P, progressP.stageWeights, N);

    // Add contribution to the base
    c2 = speedLimitHessianContribution<T>(H, g, x, &P, delta_t_inv, progressP.stageWeights, N, UB);
    zeroVector(gonly, 2 * N);
    zeroVector(Honly, 2 * N * UB);
    c4 = speedLimitOnlyHessianContribution<T>(Honly, gonly, x, &P, progressP.stageWeights, N, UB);

    // Subtract to get back the contribution
    subVectors<T>(g2, g, gbase, 2 * N);
    subVectors<T>(H2, H, Hbase, 2 * N * UB);
    // Convert Hessian contribution to regular matrix
    SymmetricBandedToRegular<T>(A2, H2, 2 * N, UB);
    SymmetricBandedToRegular<T>(Aonly, Honly, 2 * N, UB);

    LimitCostDifferentiable<T, N> fun;
    fun.set(&P, &progressP, delta_t_inv);
    T xv_mem[2 * (N + HL)], *xv, f[1], J[2 * N];
    xv = xv_mem + (2 * HL);
    copyVector<T>(xv - (2 * HL), x - (2 * HL), 2 * (N + HL));
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-8));

    LimitCostJacobianDifferentiable<T, N, UB> funJ;
    funJ.set(&P, &progressP, delta_t_inv);
    T fJ[2 * N], A[2 * N * 2 * N];
    numericalDerivative<T>(A, &funJ, fJ, xv, 2 * N, 2 * N, (1e-8));

    T e = normDiffVector<T>(&c1, &c2, 1);
    // e = scalarMax<T>(e, normDiffVector<T>(g2, J,2*N));
    e = scalarMax<T>(e, normDiffVector<T>(g2, J, 2 * N) / scalarMax<T>(1e-1, normOfVector(J, 2 * N)));
    // e = scalarMax<T>(e, normDiffVector<T>(A2,A,2*N*2*N));
    e = scalarMax<T>(e, normDiffVector<T>(A2, A, 2 * N * 2 * N) / scalarMax<T>(1e-1, normOfVector(A, 2 * N * 2 * N)));
    e = scalarMax<T>(e, normDiffVector<T>(&c1, &c3, 1));
    e = scalarMax<T>(e, normDiffVector<T>(&c1, &c4, 1));
    e = scalarMax<T>(e,
                     normDiffVector<T>(Aonly, A, 2 * N * 2 * N) / scalarMax<T>(1e-1, normOfVector(A, 2 * N * 2 * N)));

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

template <class T> inline bool LimitCostTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-5;        // 1e-6;
    T frac_tol = 0.9999; // 0.99;

    int N;
    int UB;

    succ = fail = 0;
    for (N = 1; N <= 1; N++)
        for (UB = 1; UB <= 10; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = LimitCostTestInstance<T, 30, 4, 5>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = LimitCostTestInstance<T, 30, 4, 5>(pre_seed);
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

inline bool TestLimitCost()
{
    bool back;

    back = LimitCostTest<double>();

    return (back);
}

#endif /*LIMIT_COSTS_TEST_H*/
