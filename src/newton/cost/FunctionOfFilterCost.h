#ifndef FUNCTION_OF_FILTER_COST_H
#define FUNCTION_OF_FILTER_COST_H

#include "../ScalarFunctionWrappers.h"
#include "ProgressCost.h"

// Add a scalar function f->f() of a filter dot product against x up to and including time step i
template <class T, class fun>
inline T functionOfFilterCost_i(int i, const T *x, const T *taps, int nrTaps, const fun *f, T weight)
{
    int j;
    T acc;

    const T *xp = x + (2 * (i + 1) - nrTaps);

    acc = ((T)0.0);
    for (j = 0; j < nrTaps; j++)
    {
        acc += taps[j] * xp[j];
    }
    return (weight * f->f(acc));
}

template <class T, class fun>
inline T functionOfFilterHessianContribution_i(int i, T *H, T *g, const T *x, const T *taps, int nrTaps, const fun *f,
                                               T weight, int UB)
{
    int j, shift;
    T acc;

    shift = (2 * (i + 1) - nrTaps);
    const T *xp = x + shift;

    acc = ((T)0.0);
    for (j = 0; j < nrTaps; j++)
    {
        acc += taps[j] * xp[j];
    }

    T fval, fp, fpp;
    fval = f->H(fpp, fp, acc);

    T *gp = g + shift;
    int jstart = max_of<int>(0, -shift);
    T wfp = weight * fp;
    for (j = jstart; j < nrTaps; j++)
    {
        // Jacobian of f(filtersum) is f'(filtersum)*taps
        gp[j] += wfp * taps[j];

        T wfpptj = weight * fpp * taps[j];
        for (int k = j; k < nrTaps; k++)
        {
            // Hessian of f(filtersum) is D(f'(filtersum)*taps)=
            // taps^T*f''(filtersum)*taps
            H[(shift + j) * UB + k - j] += wfpptj * taps[k];
        }
    }

    return (weight * fval);
}

template <class T, class fun>
inline T functionOfFilterCost(const T *x, const T *taps, int nrTaps, const fun *f, const T *stageWeights,
                              int nrTimesteps)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        acc += functionOfFilterCost_i<T, fun>(i, x, taps, nrTaps, f, stageWeights[i]);
    }

    return (acc);
}

template <class T, class fun>
inline T functionOfFilterHessianContribution(T *H, T *g, const T *x, const T *taps, int nrTaps, const fun *f,
                                             const T *stageWeights, int nrTimesteps, int UB)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        acc += functionOfFilterHessianContribution_i(i, H, g, x, taps, nrTaps, f, stageWeights[i], UB);
    }

    return (acc);
}

#endif /*FUNCTION_OF_FILTER_COST_H*/
