#ifndef BUMP_COST_H
#define BUMP_COST_H

#include "../ScalarFunctionWrappers.h"
#include "ProgressCost.h"

// BUMP_FT is the two-dimensional bump style function
// VEL_WT is the function of velocity to weigh the bumps by
template <class T, class BUMP_FT = TwoDimensionalFunctionBump<T>, class VEL_WT = scalarFunctionWeightedSquare<T>>
inline T bumpCost(const T *x, const BUMP_FT *const *BumpFun, const VEL_WT *const *VelWeight, const int *nrBumps,
                  const T *stageWeights, int nrTimesteps, T delta_t_inv)
{
    int i, j;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        for (j = 0; j < nrBumps[i]; j++)
        {
            acc += stageWeights[i] * VelWeight[i][j].f((x[2 * i] - x[2 * (i - 1)]) * delta_t_inv) *
                   BumpFun[i][j].f(&x[2 * i]);
        }
    }

    return (acc);
}

template <class T, class BUMP_FT = TwoDimensionalFunctionBump<T>, class VEL_WT = scalarFunctionWeightedSquare<T>>
inline T bumpHessianContribution(T *H, T *g, const T *x, const BUMP_FT *const *BumpFun, const VEL_WT *const *VelWeight,
                                 const int *nrBumps, const T *stageWeights, int nrTimesteps, int UB, T delta_t_inv)
{
    int i, j;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        for (j = 0; j < nrBumps[i]; j++)
        {
            T h1, hpp1[3], hp1[2];
            T h2, hp2, hpp2;

            h1 = BumpFun[i][j].H(hpp1, hp1, &x[2 * i]);
            h2 = VelWeight[i][j].H(hpp2, hp2, (x[2 * i] - x[2 * (i - 1)]) * delta_t_inv);
            acc += stageWeights[i] * h1 * h2;

            // gradient contribution (written as function of (x_(i-1) y_(i-1) x_i y_i))
            // stageWeights[i]*([0 0 hp1[0] hp1[1]]*h2+h1*hp2*[-1 0 1 0]*delta_t_inv)
            g[2 * i - 2] += stageWeights[i] * (-h1 * hp2 * delta_t_inv);
            // g[2*i-1] += 0;
            g[2 * i] += stageWeights[i] * (h2 * hp1[0] + h1 * hp2 * delta_t_inv);
            g[2 * i + 1] += stageWeights[i] * (h2 * hp1[1]);

            // Hessian contribution
            // stageWeights[i]*
            //	(
            //      [-1 0 1 0]'[0 0 hp1[0] hp1[1]]*hp2*delta_t_inv+
            //		[0 0    0      0    ]*h2
            //      [0 0    0      0    ]
            //      [- - hpp1[0] hpp1[1]]
            //      [- -   -     hpp1[2]]
            //	    +
            //      [-1 0 1 0]'[-1 0 1 0]*hpp2*h1*delta_t_inv*delta_t_inv
            //		[0 0 hp1[0] hp1[1]]'*[-1 0 1 0]*hp2*delta_t_inv
            //)
            T t1 = stageWeights[i] * hp2 * delta_t_inv;
            T t2 = stageWeights[i] * h2;
            T t3 = stageWeights[i] * hpp2 * h1 * delta_t_inv * delta_t_inv;

            H[(2 * i - 2) * UB] += t3;
            // H[(2*i-2)*UB+1] += 0;
            H[(2 * i - 2) * UB + 2] += (-t1 * hp1[0]) - t3;
            H[(2 * i - 2) * UB + 3] += (-t1 * hp1[1]);
            // H[(2*i-1)*UB  ] += 0;
            // H[(2*i  )*UB  ] += 0;
            // H[(2*i  )*UB+1] += 0;
            H[(2 * i) * UB + 2] += ((T)2.0) * (t1 * hp1[0]) + t2 * hpp1[0] + t3;
            H[(2 * i) * UB + 3] += (t1 * hp1[1]) + t2 * hpp1[1];
            H[(2 * i + 1) * UB + 3] += t2 * hpp1[2];
        }
    }

    return (acc);
}

#endif /*BUMP_COST_H*/
