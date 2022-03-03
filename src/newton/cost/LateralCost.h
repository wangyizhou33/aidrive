#ifndef LATERAL_COST_H
#define LATERAL_COST_H

#include "../ScalarFunctionWrappers.h"
#include "ProgressCost.h"

template <class T, class LAT_FT = scalarFunctionPolynomial<T, 3>,
          class LAT_WT = scalarFunctionWeightedBump<T> // scalarFunctionWeightedSquare<T>
          >
inline T lateralCost(const T *x, const LAT_FT *LatFun, const LAT_WT *LatWeight, int nrLaterals, const T *stageWeights,
                     int nrTimesteps)
{
    int i, j;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        for (j = 0; j < nrLaterals; j++)
        {
            acc += stageWeights[i] * LatWeight[j].f(x[2 * i + 1] - LatFun[j].f(x[2 * i]));
        }
    }

    return (acc);
}

template <class T, class LAT_FT = scalarFunctionPolynomial<T, 3>,
          class LAT_WT = scalarFunctionWeightedBump<T> // scalarFunctionWeightedSquare<T>
          >
inline T lateralHessianContribution(T *H, T *g, const T *x, const LAT_FT *LatFun, const LAT_WT *LatWeight,
                                    int nrLaterals, const T *stageWeights, int nrTimesteps, int UB)
{
    int i, j;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        for (j = 0; j < nrLaterals; j++)
        {
            T h1, hp1, hpp1;
            T h2, hp2, hpp2;

            h1 = LatFun[j].H(hpp1, hp1, x[2 * i]);
            h2 = LatWeight[j].H(hpp2, hp2, x[2 * i + 1] - h1);

            acc += stageWeights[i] * h2;

            // gradient contribution
            // stageWeights[i]*hp2*(-hp1) with respect to x
            // stageWeights[i]*hp2*1      with respect to y
            g[2 * i] += stageWeights[i] * hp2 * (-hp1);
            g[2 * i + 1] += stageWeights[i] * hp2;

            // Hessian contribution
            // stageWeights[i]*(hp2*(-hpp1)+hpp2*hp1*hp1) with respect to xx
            // stageWeights[i]*hpp2*(-hp1)                with respect to xy
            // stageWeights[i]*hpp2*1                     with respect to yy

            H[(2 * i) * UB] += stageWeights[i] * (hp2 * (-hpp1) + hpp2 * hp1 * hp1);
            H[(2 * i) * UB + 1] += stageWeights[i] * hpp2 * (-hp1);
            H[(2 * i + 1) * UB] += stageWeights[i] * hpp2;
        }
    }

    return (acc);
}

#endif /*LATERAL_COST_H*/
