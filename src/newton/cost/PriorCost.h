#ifndef PRIOR_COST_H
#define PRIOR_COST_H

#define PRIOR_COST_LON_WEIGHT_DEFAULT 1.0 //
#define PRIOR_COST_LAT_WEIGHT_DEFAULT 1.0 //
#define PRIOR_COST_LON_NR_STEPS_DEFAULT 5 //
#define PRIOR_COST_LAT_NR_STEPS_DEFAULT 5 //

#include "../ScalarFunctionWrappers.h"

// T is the floating point type
// N is the number of time steps
// PWFT_LON,PWFT_LAT (Prior Weight Function Type) are the type of weight function used for
// the longitudinal and lateral differences to prior trajectory. It needs to have an init function
// setWeight(T weight)
// It also has to have the core cost and two derivatives of cost functions
// T f(T x) const
// T H(T& fpp, T& fp, T x) const
// where the return value is cost, and fp and fpp are first and second derivative, respectively
template <class T, int N, class PWFT_LON = scalarFunctionWeightedSquare<T>,
          class PWFT_LAT = scalarFunctionWeightedSquare<T>>
class priorCostParameters
{
  public:
    PWFT_LON lon_wf[N];
    PWFT_LAT lat_wf[N];

    void setDefault(T lon_weight = PRIOR_COST_LON_WEIGHT_DEFAULT, T lat_weight = PRIOR_COST_LAT_WEIGHT_DEFAULT,
                    int lon_nr_steps = PRIOR_COST_LON_NR_STEPS_DEFAULT,
                    int lat_nr_steps = PRIOR_COST_LAT_NR_STEPS_DEFAULT)
    {
        for (int i = 0; i < N; i++)
        {
            if (i < lon_nr_steps)
            {
                lon_wf[i].setWeight(lon_weight);
            }
            else
            {
                lon_wf[i].setWeight((T)0.0);
            }

            if (i < lat_nr_steps)
            {
                lat_wf[i].setWeight(lat_weight);
            }
            else
            {
                lat_wf[i].setWeight((T)0.0);
            }
        }
    }

    priorCostParameters()
    {
        setDefault();
    }
};

template <class T, int N, class PWFT_LON = scalarFunctionWeightedSquare<T>,
          class PWFT_LAT = scalarFunctionWeightedSquare<T>>
inline T priorCost(const T *x, const T *prior_x, const priorCostParameters<T, N, PWFT_LON, PWFT_LAT> *P,
                   const T *stageWeights)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < N; i++)
    {
        acc += stageWeights[i] * P->lon_wf[i].f(x[2 * i] - prior_x[2 * i]);
        acc += stageWeights[i] * P->lat_wf[i].f(x[2 * i + 1] - prior_x[2 * i + 1]);
    }

    return (acc);
}

template <class T, int N, class PWFT_LON = scalarFunctionWeightedSquare<T>,
          class PWFT_LAT = scalarFunctionWeightedSquare<T>>
inline T priorHessianContribution(T *H, T *g, const T *x, const T *prior_x,
                                  const priorCostParameters<T, N, PWFT_LON, PWFT_LAT> *P, const T *stageWeights, int UB)
{
    int i;
    T acc, hp1, hp2, hpp1, hpp2;

    acc = ((T)0.0);
    for (i = 0; i < N; i++)
    {
        acc += stageWeights[i] * P->lon_wf[i].H(hpp1, hp1, x[2 * i] - prior_x[2 * i]);
        acc += stageWeights[i] * P->lat_wf[i].H(hpp2, hp2, x[2 * i + 1] - prior_x[2 * i + 1]);

        // gradient contribution
        g[2 * i] += stageWeights[i] * hp1;
        g[2 * i + 1] += stageWeights[i] * hp2;

        // Hessian contribution
        H[(2 * i) * UB] += stageWeights[i] * hpp1;
        H[(2 * i + 1) * UB] += stageWeights[i] * hpp2;
    }

    return (acc);
}

#endif /*PRIOR_COST_H*/
