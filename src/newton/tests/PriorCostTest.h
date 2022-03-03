#ifndef PRIOR_COST_TEST_H
#define PRIOR_COST_TEST_H

#include "../NumericalDerivative.h"
#include "../cost/PriorCost.h"
#include "../cost/ProgressCost.h"
#include "TestUtil.h"

template <class T, int N, class PWFT_LON = scalarFunctionWeightedSquare<T>,
          class PWFT_LAT = scalarFunctionWeightedSquare<T>>
class PriorCostDifferentiable : public differentiableFunction<T>
{
  public:
    T *prior_x;
    priorCostParameters<T, N, PWFT_LON, PWFT_LAT> *P;
    progressCostParameters<T, N> *progressP;

    void set(T *prior_x_in, priorCostParameters<T, N, PWFT_LON, PWFT_LAT> *P_in,
             progressCostParameters<T, N> *progressP_in)
    {
        prior_x = prior_x_in;
        P = P_in;
        progressP = progressP_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = priorCost<T, N, PWFT_LON, PWFT_LAT>(x, prior_x, P, progressP->stageWeights);
    }
};

template <class T, int N, int UB, class PWFT_LON = scalarFunctionWeightedSquare<T>,
          class PWFT_LAT = scalarFunctionWeightedSquare<T>>
class PriorCostJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    T *prior_x;
    priorCostParameters<T, N, PWFT_LON, PWFT_LAT> *P;
    progressCostParameters<T, N> *progressP;

    void set(T *prior_x_in, priorCostParameters<T, N, PWFT_LON, PWFT_LAT> *P_in,
             progressCostParameters<T, N> *progressP_in)
    {
        prior_x = prior_x_in;
        P = P_in;
        progressP = progressP_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T c, H[2 * N * UB];
        zeroVector<T>(f, 2 * N);
        zeroVector<T>(H, 2 * N * UB);
        c = priorHessianContribution<T, N, PWFT_LON, PWFT_LAT>(H, f, x, prior_x, P, progressP->stageWeights, UB);
    }
};

template <class T, int N, int UB, class PWFT_LON = scalarFunctionWeightedSquare<T>,
          class PWFT_LAT = scalarFunctionWeightedSquare<T>>
inline T PriorCostTestInstance(long int &seed)
{
    T x[2 * N], prior_x[2 * N];
    progressCostParameters<T, N> progressP;
    priorCostParameters<T, N, PWFT_LON, PWFT_LAT> P;

    // P.setDefault(PRIOR_COST_LON_WEIGHT_DEFAULT, PRIOR_COST_LAT_WEIGHT_DEFAULT,
    //	PRIOR_COST_LON_NR_STEPS_DEFAULT,
    //	PRIOR_COST_LAT_NR_STEPS_DEFAULT);

    randomVector<T>(x, 2 * N, seed);
    randomVector<T>(prior_x, 2 * N, seed);

    T c1, c2, gbase[2 * N], g[2 * N], g2[2 * N], Hbase[2 * N * UB], H[2 * N * UB], H2[2 * N * UB], A2[2 * N * 2 * N];
    // Start with a base
    randomVector<T>(gbase, 2 * N, seed);
    randomVector<T>(Hbase, 2 * N * UB, seed);
    // Copy the base
    copyVector(g, gbase, 2 * N);
    copyVector(H, Hbase, 2 * N * UB);
    c1 = priorCost<T, N, PWFT_LON, PWFT_LAT>(x, prior_x, &P, progressP.stageWeights);

    // Add contribution to the base
    c2 = priorHessianContribution<T, N, PWFT_LON, PWFT_LAT>(H, g, x, prior_x, &P, progressP.stageWeights, UB);

    // Subtract to get back the contribution
    subVectors<T>(g2, g, gbase, 2 * N);
    subVectors<T>(H2, H, Hbase, 2 * N * UB);
    // Convert Hessian contribution to regular matrix
    SymmetricBandedToRegular<T>(A2, H2, 2 * N, UB);

    PriorCostDifferentiable<T, N, PWFT_LON, PWFT_LAT> fun;
    fun.set(prior_x, &P, &progressP);
    T xv[2 * N], f[1], J[2 * N];
    copyVector<T>(xv, x, 2 * N);
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-7));

    PriorCostJacobianDifferentiable<T, N, UB, PWFT_LON, PWFT_LAT> funJ;
    funJ.set(prior_x, &P, &progressP);
    T fJ[2 * N], A[2 * N * 2 * N];
    numericalDerivative<T>(A, &funJ, fJ, xv, 2 * N, 2 * N, (1e-8));

    T e = normDiffVector<T>(&c1, &c2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(g2, J, 2 * N));
    e = scalarMax<T>(e, normDiffVector<T>(A2, A, 2 * N * 2 * N));

    // e = scalarMax<T>(e, normDiffVector<T>(dbdx,dbdx2,2));
    // e = scalarMax<T>(e, normDiffVector<T>(dbdx ,J, 2)/scalarMax<T>(1e-9,normOfVector(J,2)));
    // e = scalarMax<T>(e, normDiffVector<T>(H1,H2, 4)/scalarMax<T>(1e-9,normOfVector(H2,4)));

    return (e);
}

template <class T> inline bool PriorCostTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-6;        // 1e-6;
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
                T e = PriorCostTestInstance<T, 30, 4>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = PriorCostTestInstance<T, 30, 4>(pre_seed);
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

inline bool TestPriorCost()
{
    bool back;

    back = PriorCostTest<double>();

    return (back);
}

#endif /*PRIOR_COST_TEST_H*/
