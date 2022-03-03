#ifndef LATERAL_COST_TEST_H
#define LATERAL_COST_TEST_H

#include "../NumericalDerivative.h"
#include "../cost/LateralCost.h"
#include "TestUtil.h"

template <class T, int N, class LAT_FT = scalarFunctionPolynomial<T, 3>,
          class LAT_WT = scalarFunctionWeightedBump<T> // scalarFunctionWeightedSquare<T>
          >
class LateralCostDifferentiable : public differentiableFunction<T>
{
  public:
    progressCostParameters<T, N> *progressP;
    LAT_FT *LatFun;
    LAT_WT *LatWeight;
    int nrLaterals;

    void set(LAT_FT *LatFun_in, LAT_WT *LatWeight_in, int nrLaterals_in, progressCostParameters<T, N> *progressP_in)
    {
        LatFun = LatFun_in;
        LatWeight = LatWeight_in;
        nrLaterals = nrLaterals_in;
        progressP = progressP_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = lateralCost<T, LAT_FT, LAT_WT>(x, LatFun, LatWeight, nrLaterals, progressP->stageWeights, N);
    }
};

template <class T, int N, int UB, class LAT_FT = scalarFunctionPolynomial<T, 3>,
          class LAT_WT = scalarFunctionWeightedBump<T> // scalarFunctionWeightedSquare<T>
          >
class LateralCostJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    progressCostParameters<T, N> *progressP;
    LAT_FT *LatFun;
    LAT_WT *LatWeight;
    int nrLaterals;

    void set(LAT_FT *LatFun_in, LAT_WT *LatWeight_in, int nrLaterals_in, progressCostParameters<T, N> *progressP_in)
    {
        LatFun = LatFun_in;
        LatWeight = LatWeight_in;
        nrLaterals = nrLaterals_in;
        progressP = progressP_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T c, H[2 * N * UB];
        zeroVector<T>(f, 2 * N);
        zeroVector<T>(H, 2 * N * UB);
        c = lateralHessianContribution<T, LAT_FT, LAT_WT>(H, f, x, LatFun, LatWeight, nrLaterals,
                                                          progressP->stageWeights, N, UB);
    }
};

template <class T, int N, int UB, class LAT_FT = scalarFunctionPolynomial<T, 3>,
          class LAT_WT = scalarFunctionWeightedBump<T> // scalarFunctionWeightedSquare<T>
          >
inline T LateralCostTestInstance(long int &seed)
{
    T x[2 * N];
    progressCostParameters<T, N> progressP;

    LAT_FT LatFun[2];
    LAT_WT LatWeight[2];
    int nrLaterals = 2;
    T p1[4], p2[4];
    randomVector<T>(p1, 4, seed);
    randomVector<T>(p2, 4, seed);
    LatFun[0].set(p1);
    LatFun[1].set(p2);
    LatWeight[0].setWeight(randomScalar<T>(seed));
    LatWeight[1].setWeight(randomScalar<T>(seed));

    randomVector<T>(x, 2 * N, seed);

    T c1, c2, gbase[2 * N], g[2 * N], g2[2 * N], Hbase[2 * N * UB], H[2 * N * UB], H2[2 * N * UB], A2[2 * N * 2 * N];
    // Start with a base
    randomVector<T>(gbase, 2 * N, seed);
    randomVector<T>(Hbase, 2 * N * UB, seed);
    // Copy the base
    copyVector(g, gbase, 2 * N);
    copyVector(H, Hbase, 2 * N * UB);
    c1 = lateralCost<T, LAT_FT, LAT_WT>(x, LatFun, LatWeight, nrLaterals, progressP.stageWeights, N);

    // Add contribution to the base
    c2 = lateralHessianContribution<T, LAT_FT, LAT_WT>(H, g, x, LatFun, LatWeight, nrLaterals, progressP.stageWeights,
                                                       N, UB);

    // Subtract to get back the contribution
    subVectors<T>(g2, g, gbase, 2 * N);
    subVectors<T>(H2, H, Hbase, 2 * N * UB);
    // Convert Hessian contribution to regular matrix
    SymmetricBandedToRegular<T>(A2, H2, 2 * N, UB);

    LateralCostDifferentiable<T, N, LAT_FT, LAT_WT> fun;
    fun.set(LatFun, LatWeight, nrLaterals, &progressP);
    T xv[2 * N], f[1], J[2 * N];
    copyVector<T>(xv, x, 2 * N);
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-9));

    LateralCostJacobianDifferentiable<T, N, UB, LAT_FT, LAT_WT> funJ;
    funJ.set(LatFun, LatWeight, nrLaterals, &progressP);
    T fJ[2 * N], A[2 * N * 2 * N];
    numericalDerivative<T>(A, &funJ, fJ, xv, 2 * N, 2 * N, (1e-9));

    T e = normDiffVector<T>(&c1, &c2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(g2, J, 2 * N));
    e = scalarMax<T>(e, normDiffVector<T>(A2, A, 2 * N * 2 * N));

    // e = scalarMax<T>(e, normDiffVector<T>(dbdx,dbdx2,2));
    // e = scalarMax<T>(e, normDiffVector<T>(dbdx ,J, 2)/scalarMax<T>(1e-9,normOfVector(J,2)));
    // e = scalarMax<T>(e, normDiffVector<T>(H1,H2, 4)/scalarMax<T>(1e-9,normOfVector(H2,4)));

    return (e);
}

template <class T> inline bool LateralCostTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-3;        // 1e-6;
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
                T e = LateralCostTestInstance<T, 30, 4>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = LateralCostTestInstance<T, 30, 4>(pre_seed);
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

    cout << " FAIL frac: " << frac << endl;
    return (false);
}

inline bool TestLateralCost()
{
    bool back;

    back = LateralCostTest<double>();

    return (back);
}

#endif /*LATERAL_COST_TEST_H*/
