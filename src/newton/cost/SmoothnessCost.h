#ifndef SMOOTHNESS_COST_H
#define SMOOTHNESS_COST_H

#define SMOOTHNESS_COST_WEIGHT_LONG_POSITION_DEFAULT 0.0 // zero because Usually no reason to prefer to stay at origin
#define SMOOTHNESS_COST_WEIGHT_LAT_POSITION_DEFAULT 0.0  // zero because usually no reason to prefer to stay at origin
#define SMOOTHNESS_COST_WEIGHT_LONG_VELOCITY_DEFAULT 0.00001    //
#define SMOOTHNESS_COST_WEIGHT_LAT_VELOCITY_DEFAULT 0.00001     //
#define SMOOTHNESS_COST_WEIGHT_LONG_ACCELERATION_DEFAULT 0.0001 // 2.0  //
#define SMOOTHNESS_COST_WEIGHT_LAT_ACCELERATION_DEFAULT 0.0001  // 2.0  //
#define SMOOTHNESS_COST_WEIGHT_LONG_JERK_DEFAULT 0.001          // 0.1 //
#define SMOOTHNESS_COST_WEIGHT_LAT_JERK_DEFAULT 0.001           // 0.1

#include "FunctionOfFilterCost.h"

// WFT (Weight Function Type) needs to have an initialization function
// void setWeight(T weight)
// It also has to have the core cost and two derivatives of cost functions
// T f(T x) const
// T H(T& fpp, T& fp, T x) const
// where the return value is cost, and fp and fpp are first and second derivative, respectively
template <class T, class WFT = scalarFunctionWeightedSquare<T>> class smoothnessCostParameters
{
  public:
    WFT f_p_long;
    WFT f_p_lat;
    WFT f_v_long;
    WFT f_v_lat;
    WFT f_a_long;
    WFT f_a_lat;
    WFT f_j_long;
    WFT f_j_lat;
    T taps_p_long[2];
    T taps_p_lat[2];
    T taps_v_long[4];
    T taps_v_lat[4];
    T taps_a_long[6];
    T taps_a_lat[6];
    T taps_j_long[8];
    T taps_j_lat[8];
    int nrTaps_p_long = 2;
    int nrTaps_p_lat = 2;
    int nrTaps_v_long = 4;
    int nrTaps_v_lat = 4;
    int nrTaps_a_long = 6;
    int nrTaps_a_lat = 6;
    int nrTaps_j_long = 8;
    int nrTaps_j_lat = 8;

    void setDefault(T delta_t_inv, T weight_p_long = ((T)SMOOTHNESS_COST_WEIGHT_LONG_POSITION_DEFAULT),
                    T weight_p_lat = ((T)SMOOTHNESS_COST_WEIGHT_LAT_POSITION_DEFAULT),
                    T weight_v_long = ((T)SMOOTHNESS_COST_WEIGHT_LONG_VELOCITY_DEFAULT),
                    T weight_v_lat = ((T)SMOOTHNESS_COST_WEIGHT_LAT_VELOCITY_DEFAULT),
                    T weight_a_long = ((T)SMOOTHNESS_COST_WEIGHT_LONG_ACCELERATION_DEFAULT),
                    T weight_a_lat = ((T)SMOOTHNESS_COST_WEIGHT_LAT_ACCELERATION_DEFAULT),
                    T weight_j_long = ((T)SMOOTHNESS_COST_WEIGHT_LONG_JERK_DEFAULT),
                    T weight_j_lat = ((T)SMOOTHNESS_COST_WEIGHT_LAT_JERK_DEFAULT))
    {
        f_p_long.setWeight(weight_p_long);
        f_p_lat.setWeight(weight_p_lat);
        f_v_long.setWeight(weight_v_long);
        f_v_lat.setWeight(weight_v_lat);
        f_a_long.setWeight(weight_a_long);
        f_a_lat.setWeight(weight_a_lat);
        f_j_long.setWeight(weight_j_long);
        f_j_lat.setWeight(weight_j_lat);

        T delta_t_inv2 = delta_t_inv * delta_t_inv;
        T delta_t_inv3 = delta_t_inv2 * delta_t_inv;

        // Velocity
        taps_p_long[0] = ((T)(1.0));
        taps_p_long[1] = ((T)(0.0));

        taps_p_lat[0] = ((T)(0.0));
        taps_p_lat[1] = ((T)(1.0));

        taps_v_long[0] = delta_t_inv * ((T)(-1.0));
        taps_v_long[1] = ((T)(0.0));
        taps_v_long[2] = delta_t_inv * ((T)(1.0));
        taps_v_long[3] = ((T)(0.0));

        taps_v_lat[0] = ((T)(0.0));
        taps_v_lat[1] = delta_t_inv * ((T)(-1.0));
        taps_v_lat[2] = ((T)(0.0));
        taps_v_lat[3] = delta_t_inv * ((T)(1.0));

        taps_a_long[0] = delta_t_inv2 * ((T)(1.0));
        taps_a_long[1] = ((T)(0.0));
        taps_a_long[2] = delta_t_inv2 * ((T)(-2.0));
        taps_a_long[3] = ((T)(0.0));
        taps_a_long[4] = delta_t_inv2 * ((T)(1.0));
        taps_a_long[5] = ((T)(0.0));

        taps_a_lat[0] = ((T)(0.0));
        taps_a_lat[1] = delta_t_inv2 * ((T)(1.0));
        taps_a_lat[2] = ((T)(0.0));
        taps_a_lat[3] = delta_t_inv2 * ((T)(-2.0));
        taps_a_lat[4] = ((T)(0.0));
        taps_a_lat[5] = delta_t_inv2 * ((T)(1.0));

        taps_j_long[0] = delta_t_inv3 * ((T)(-1.0));
        taps_j_long[1] = ((T)(0.0));
        taps_j_long[2] = delta_t_inv3 * ((T)(3.0));
        taps_j_long[3] = ((T)(0.0));
        taps_j_long[4] = delta_t_inv3 * ((T)(-3.0));
        taps_j_long[5] = ((T)(0.0));
        taps_j_long[6] = delta_t_inv3 * ((T)(1.0));
        taps_j_long[7] = ((T)(0.0));

        taps_j_lat[0] = ((T)(0.0));
        taps_j_lat[1] = delta_t_inv3 * ((T)(-1.0));
        taps_j_lat[2] = ((T)(0.0));
        taps_j_lat[3] = delta_t_inv3 * ((T)(3.0));
        taps_j_lat[4] = ((T)(0.0));
        taps_j_lat[5] = delta_t_inv3 * ((T)(-3.0));
        taps_j_lat[6] = ((T)(0.0));
        taps_j_lat[7] = delta_t_inv3 * ((T)(1.0));
    }

    smoothnessCostParameters()
    {
        setDefault(((T)1.0));
    }
};

template <class T, class WFT = scalarFunctionWeightedSquare<T>>
inline T smothnessCost(const T *x, const smoothnessCostParameters<T, WFT> *P, const T *stageWeights, int nrTimesteps)
{
    T acc;

    acc = ((T)0.0);

    acc += functionOfFilterCost<T, WFT>(x, P->taps_p_long, P->nrTaps_p_long, &(P->f_p_long), stageWeights, nrTimesteps);
    acc += functionOfFilterCost<T, WFT>(x, P->taps_p_lat, P->nrTaps_p_lat, &(P->f_p_lat), stageWeights, nrTimesteps);

    acc += functionOfFilterCost<T, WFT>(x, P->taps_v_long, P->nrTaps_v_long, &(P->f_v_long), stageWeights, nrTimesteps);
    acc += functionOfFilterCost<T, WFT>(x, P->taps_v_lat, P->nrTaps_v_lat, &(P->f_v_lat), stageWeights, nrTimesteps);

    acc += functionOfFilterCost<T, WFT>(x, P->taps_a_long, P->nrTaps_a_long, &(P->f_a_long), stageWeights, nrTimesteps);
    acc += functionOfFilterCost<T, WFT>(x, P->taps_a_lat, P->nrTaps_a_lat, &(P->f_a_lat), stageWeights, nrTimesteps);

    acc += functionOfFilterCost<T, WFT>(x, P->taps_j_long, P->nrTaps_j_long, &(P->f_j_long), stageWeights, nrTimesteps);
    acc += functionOfFilterCost<T, WFT>(x, P->taps_j_lat, P->nrTaps_j_lat, &(P->f_j_lat), stageWeights, nrTimesteps);

    return (acc);
}

template <class T, class WFT = scalarFunctionWeightedSquare<T>>
inline T smoothnessHessianContribution(T *H, T *g, const T *x, const smoothnessCostParameters<T, WFT> *P,
                                       const T *stageWeights, int nrTimesteps, int UB)
{
    T acc;

    acc = ((T)0.0);

    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_p_long, P->nrTaps_p_long, &(P->f_p_long),
                                                       stageWeights, nrTimesteps, UB);
    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_p_lat, P->nrTaps_p_lat, &(P->f_p_lat),
                                                       stageWeights, nrTimesteps, UB);

    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_v_long, P->nrTaps_v_long, &(P->f_v_long),
                                                       stageWeights, nrTimesteps, UB);
    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_v_lat, P->nrTaps_v_lat, &(P->f_v_lat),
                                                       stageWeights, nrTimesteps, UB);

    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_a_long, P->nrTaps_a_long, &(P->f_a_long),
                                                       stageWeights, nrTimesteps, UB);
    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_a_lat, P->nrTaps_a_lat, &(P->f_a_lat),
                                                       stageWeights, nrTimesteps, UB);

    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_j_long, P->nrTaps_j_long, &(P->f_j_long),
                                                       stageWeights, nrTimesteps, UB);
    acc += functionOfFilterHessianContribution<T, WFT>(H, g, x, P->taps_j_lat, P->nrTaps_j_lat, &(P->f_j_lat),
                                                       stageWeights, nrTimesteps, UB);

    return (acc);
}

#endif /*SMOOTHNESSCOST_H*/
