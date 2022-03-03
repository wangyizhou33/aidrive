#ifndef LIMIT_COSTS_H
#define LIMIT_COSTS_H

#define LIMIT_COST_LON_P_LIMIT_DEFAULT 200.0           // 200.0 //m
#define LIMIT_COST_LON_P_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_P_ON_DEFAULT false

#define LIMIT_COST_LON_NP_LIMIT_DEFAULT 100.0           // 100.0 //m
#define LIMIT_COST_LON_NP_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_NP_ON_DEFAULT false

#define LIMIT_COST_LAT_P_LIMIT_DEFAULT 50.0            // 50.0 //m
#define LIMIT_COST_LAT_P_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LAT_P_ON_DEFAULT false

#define LIMIT_COST_LAT_NP_LIMIT_DEFAULT 50.0            // 50.0 //m
#define LIMIT_COST_LAT_NP_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LAT_NP_ON_DEFAULT false

#define LIMIT_COST_LON_V_LIMIT_DEFAULT 30.0             // 30.0 //m/s
#define LIMIT_COST_LON_V_MARGIN_MULTIPLIER_DEFAULT 1.03 // 1.5
#define LIMIT_COST_LON_V_ON_DEFAULT true

#define LIMIT_COST_LON_NV_LIMIT_DEFAULT 10.0            // m/s
#define LIMIT_COST_LON_NV_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_NV_ON_DEFAULT true

#define LIMIT_COST_LAT_V_LIMIT_DEFAULT 5.0             // m/s
#define LIMIT_COST_LAT_V_MARGIN_MULTIPLIER_DEFAULT 1.1 //
#define LIMIT_COST_LAT_V_ON_DEFAULT true

#define LIMIT_COST_LAT_NV_LIMIT_DEFAULT 5.0             // m/s
#define LIMIT_COST_LAT_NV_MARGIN_MULTIPLIER_DEFAULT 1.1 //
#define LIMIT_COST_LAT_NV_ON_DEFAULT true

#define LIMIT_COST_LON_A_LIMIT_DEFAULT 3.0             // 3.0 //m/s^2 accelerating
#define LIMIT_COST_LON_A_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_A_ON_DEFAULT true

#define LIMIT_COST_LON_NA_LIMIT_DEFAULT 5.0             // 5.0 //m/s^2 braking
#define LIMIT_COST_LON_NA_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_NA_ON_DEFAULT true

#define LIMIT_COST_LAT_A_LIMIT_DEFAULT 3.0             // 3.0 //m/s^2
#define LIMIT_COST_LAT_A_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LAT_A_ON_DEFAULT true

#define LIMIT_COST_LAT_NA_LIMIT_DEFAULT 3.0             // 3.0 //m/s^2
#define LIMIT_COST_LAT_NA_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LAT_NA_ON_DEFAULT true

#define LIMIT_COST_LON_J_LIMIT_DEFAULT 2.0             // 2.0 //m/s^3
#define LIMIT_COST_LON_J_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_J_ON_DEFAULT true

#define LIMIT_COST_LON_NJ_LIMIT_DEFAULT 2.0             // 2.0 //m/s^3
#define LIMIT_COST_LON_NJ_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LON_NJ_ON_DEFAULT true

#define LIMIT_COST_LAT_J_LIMIT_DEFAULT 2.0             // 2.0 //m/s^3
#define LIMIT_COST_LAT_J_MARGIN_MULTIPLIER_DEFAULT 1.5 //
#define LIMIT_COST_LAT_J_ON_DEFAULT true

#define LIMIT_COST_LAT_NJ_LIMIT_DEFAULT 2.0             // 2.0 //m/s^3
#define LIMIT_COST_LAT_NJ_MARGIN_MULTIPLIER_DEFAULT 1.5 //  //
#define LIMIT_COST_LAT_NJ_ON_DEFAULT true

#include "../ScalarFunctionWrappers.h"
#include "FunctionOfFilterCost.h"
#include "ProgressCost.h"

// BFT (Barrier Function Type) is the type of barrier function used for all the limits
// It needs to have an initialization function
// void set(T k_h, T x_onset, T x_max)
// where k_h is a weight, x_onset is where it starts rising (typically being zero before that) and x_max is the barrier
// position It also has to have the core cost and two derivatives of cost functions
// T f(T x) const
// T H(T& fpp, T& fp, T x) const
// where the return value is cost, and fp and fpp are first and second derivative, respectively
template <class T, class BFT = scalarFunctionSmoothBarrier<T>> class limitCostParameters
{
  public:
    bool lon_p_on;
    bool lon_np_on;
    bool lat_p_on;
    bool lat_np_on;

    bool lon_v_on;
    bool lon_nv_on;
    bool lat_v_on;
    bool lat_nv_on;

    bool lon_a_on;
    bool lon_na_on;
    bool lat_a_on;
    bool lat_na_on;

    bool lon_j_on;
    bool lon_nj_on;
    bool lat_j_on;
    bool lat_nj_on;

    BFT lon_p_f;
    BFT lon_np_f;
    BFT lat_p_f;
    BFT lat_np_f;

    BFT lon_v_f;
    BFT lon_nv_f;
    BFT lat_v_f;
    BFT lat_nv_f;

    BFT lon_a_f;
    BFT lon_na_f;
    BFT lat_a_f;
    BFT lat_na_f;

    BFT lon_j_f;
    BFT lon_nj_f;
    BFT lat_j_f;
    BFT lat_nj_f;

    T lon_p_taps[2];
    T lon_np_taps[2];
    T lat_p_taps[2];
    T lat_np_taps[2];

    T lon_v_taps[4];
    T lon_nv_taps[4];
    T lat_v_taps[4];
    T lat_nv_taps[4];

    T lon_a_taps[6];
    T lon_na_taps[6];
    T lat_a_taps[6];
    T lat_na_taps[6];

    T lon_j_taps[8];
    T lon_nj_taps[8];
    T lat_j_taps[8];
    T lat_nj_taps[8];

    int lon_p_nrT;
    int lon_np_nrT;
    int lat_p_nrT;
    int lat_np_nrT;

    int lon_v_nrT;
    int lon_nv_nrT;
    int lat_v_nrT;
    int lat_nv_nrT;

    int lon_a_nrT;
    int lon_na_nrT;
    int lat_a_nrT;
    int lat_na_nrT;

    int lon_j_nrT;
    int lon_nj_nrT;
    int lat_j_nrT;
    int lat_nj_nrT;

    T lon_p_ideal;
    T lon_np_ideal;
    T lat_p_ideal;
    T lat_np_ideal;

    T lon_v_ideal;
    T lon_nv_ideal;
    T lat_v_ideal;
    T lat_nv_ideal;
    T lon_a_ideal;
    T lon_na_ideal;
    T lat_a_ideal;
    T lat_na_ideal;
    T lon_j_ideal;
    T lon_nj_ideal;
    T lat_j_ideal;
    T lat_nj_ideal;
    T lon_p_margin_multiplier;
    T lon_np_margin_multiplier;
    T lat_p_margin_multiplier;
    T lat_np_margin_multiplier;
    T lon_v_margin_multiplier;
    T lon_nv_margin_multiplier;
    T lat_v_margin_multiplier;
    T lat_nv_margin_multiplier;
    T lon_a_margin_multiplier;
    T lon_na_margin_multiplier;
    T lat_a_margin_multiplier;
    T lat_na_margin_multiplier;
    T lon_j_margin_multiplier;
    T lon_nj_margin_multiplier;
    T lat_j_margin_multiplier;
    T lat_nj_margin_multiplier;

    T lon_k_p, lon_p_onset, lon_p_max;
    T lon_k_np, lon_np_onset, lon_np_max;
    T lat_k_p, lat_p_onset, lat_p_max;
    T lat_k_np, lat_np_onset, lat_np_max;

    T lon_k_v, lon_v_onset, lon_v_max;
    T lon_k_nv, lon_nv_onset, lon_nv_max;
    T lat_k_v, lat_v_onset, lat_v_max;
    T lat_k_nv, lat_nv_onset, lat_nv_max;

    T lon_k_a, lon_a_onset, lon_a_max;
    T lon_k_na, lon_na_onset, lon_na_max;
    T lat_k_a, lat_a_onset, lat_a_max;
    T lat_k_na, lat_na_onset, lat_na_max;

    T lon_k_j, lon_j_onset, lon_j_max;
    T lon_k_nj, lon_nj_onset, lon_nj_max;
    T lat_k_j, lat_j_onset, lat_j_max;
    T lat_k_nj, lat_nj_onset, lat_nj_max;

    T delta_t_inv, k_p;

    // delta_t_inv is inverse of time step. k_p is progress weight from progress cost
    void setDefault(T delta_t_inv_in, T k_p_in)
    {
        delta_t_inv = delta_t_inv_in;
        k_p = k_p_in;

        lon_p_on = LIMIT_COST_LON_P_ON_DEFAULT;
        lon_np_on = LIMIT_COST_LON_NP_ON_DEFAULT;
        lat_p_on = LIMIT_COST_LAT_P_ON_DEFAULT;
        lat_np_on = LIMIT_COST_LAT_NP_ON_DEFAULT;
        lon_v_on = LIMIT_COST_LON_V_ON_DEFAULT;
        lon_nv_on = LIMIT_COST_LON_NV_ON_DEFAULT;
        lat_v_on = LIMIT_COST_LAT_V_ON_DEFAULT;
        lat_nv_on = LIMIT_COST_LAT_NV_ON_DEFAULT;
        lon_a_on = LIMIT_COST_LON_A_ON_DEFAULT;
        lon_na_on = LIMIT_COST_LON_NA_ON_DEFAULT;
        lat_a_on = LIMIT_COST_LAT_A_ON_DEFAULT;
        lat_na_on = LIMIT_COST_LAT_NA_ON_DEFAULT;
        lon_j_on = LIMIT_COST_LON_J_ON_DEFAULT;
        lon_nj_on = LIMIT_COST_LON_NJ_ON_DEFAULT;
        lat_j_on = LIMIT_COST_LAT_J_ON_DEFAULT;
        lat_nj_on = LIMIT_COST_LAT_NJ_ON_DEFAULT;
        lon_p_ideal = ((T)LIMIT_COST_LON_P_LIMIT_DEFAULT);
        lon_np_ideal = ((T)LIMIT_COST_LON_NP_LIMIT_DEFAULT);
        lat_p_ideal = ((T)LIMIT_COST_LAT_P_LIMIT_DEFAULT);
        lat_np_ideal = ((T)LIMIT_COST_LAT_NP_LIMIT_DEFAULT);
        lon_v_ideal = ((T)LIMIT_COST_LON_V_LIMIT_DEFAULT);
        lon_nv_ideal = ((T)LIMIT_COST_LON_NV_LIMIT_DEFAULT);
        lat_v_ideal = ((T)LIMIT_COST_LAT_V_LIMIT_DEFAULT);
        lat_nv_ideal = ((T)LIMIT_COST_LAT_NV_LIMIT_DEFAULT);
        lon_a_ideal = ((T)LIMIT_COST_LON_A_LIMIT_DEFAULT);
        lon_na_ideal = ((T)LIMIT_COST_LON_NA_LIMIT_DEFAULT);
        lat_a_ideal = ((T)LIMIT_COST_LAT_A_LIMIT_DEFAULT);
        lat_na_ideal = ((T)LIMIT_COST_LAT_NA_LIMIT_DEFAULT);
        lon_j_ideal = ((T)LIMIT_COST_LON_J_LIMIT_DEFAULT);
        lon_nj_ideal = ((T)LIMIT_COST_LON_NJ_LIMIT_DEFAULT);
        lat_j_ideal = ((T)LIMIT_COST_LAT_J_LIMIT_DEFAULT);
        lat_nj_ideal = ((T)LIMIT_COST_LAT_NJ_LIMIT_DEFAULT);
        lon_p_margin_multiplier = ((T)LIMIT_COST_LON_P_MARGIN_MULTIPLIER_DEFAULT);
        lon_np_margin_multiplier = ((T)LIMIT_COST_LON_NP_MARGIN_MULTIPLIER_DEFAULT);
        lat_p_margin_multiplier = ((T)LIMIT_COST_LAT_P_MARGIN_MULTIPLIER_DEFAULT);
        lat_np_margin_multiplier = ((T)LIMIT_COST_LAT_NP_MARGIN_MULTIPLIER_DEFAULT);
        lon_v_margin_multiplier = ((T)LIMIT_COST_LON_V_MARGIN_MULTIPLIER_DEFAULT);
        lon_nv_margin_multiplier = ((T)LIMIT_COST_LON_NV_MARGIN_MULTIPLIER_DEFAULT);
        lat_v_margin_multiplier = ((T)LIMIT_COST_LAT_V_MARGIN_MULTIPLIER_DEFAULT);
        lat_nv_margin_multiplier = ((T)LIMIT_COST_LAT_NV_MARGIN_MULTIPLIER_DEFAULT);
        lon_a_margin_multiplier = ((T)LIMIT_COST_LON_A_MARGIN_MULTIPLIER_DEFAULT);
        lon_na_margin_multiplier = ((T)LIMIT_COST_LON_NA_MARGIN_MULTIPLIER_DEFAULT);
        lat_a_margin_multiplier = ((T)LIMIT_COST_LAT_A_MARGIN_MULTIPLIER_DEFAULT);
        lat_na_margin_multiplier = ((T)LIMIT_COST_LAT_NA_MARGIN_MULTIPLIER_DEFAULT);
        lon_j_margin_multiplier = ((T)LIMIT_COST_LON_J_MARGIN_MULTIPLIER_DEFAULT);
        lon_nj_margin_multiplier = ((T)LIMIT_COST_LON_NJ_MARGIN_MULTIPLIER_DEFAULT);
        lat_j_margin_multiplier = ((T)LIMIT_COST_LAT_J_MARGIN_MULTIPLIER_DEFAULT);
        lat_nj_margin_multiplier = ((T)LIMIT_COST_LAT_NJ_MARGIN_MULTIPLIER_DEFAULT);

        this->prepare();
    }

    void set_lon_p(bool lon_p_on_in = LIMIT_COST_LON_P_ON_DEFAULT,
                   T lon_p_ideal_in = ((T)LIMIT_COST_LON_P_LIMIT_DEFAULT),
                   T lon_p_margin_multiplier_in = ((T)LIMIT_COST_LON_P_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_p_on = lon_p_on_in;
        lon_p_ideal = lon_p_ideal_in;
        lon_p_margin_multiplier = lon_p_margin_multiplier_in;
    }
    void set_lon_np(bool lon_np_on_in = LIMIT_COST_LON_NP_ON_DEFAULT,
                    T lon_np_ideal_in = ((T)LIMIT_COST_LON_NP_LIMIT_DEFAULT),
                    T lon_np_margin_multiplier_in = ((T)LIMIT_COST_LON_NP_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_np_on = lon_np_on_in;
        lon_np_ideal = lon_np_ideal_in;
        lon_np_margin_multiplier = lon_np_margin_multiplier_in;
    }
    void set_lat_p(bool lat_p_on_in = LIMIT_COST_LAT_P_ON_DEFAULT,
                   T lat_p_ideal_in = ((T)LIMIT_COST_LAT_P_LIMIT_DEFAULT),
                   T lat_p_margin_multiplier_in = ((T)LIMIT_COST_LAT_P_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_p_on = lat_p_on_in;
        lat_p_ideal = lat_p_ideal_in;
        lat_p_margin_multiplier = lat_p_margin_multiplier_in;
    }
    void set_lat_np(bool lat_np_on_in = LIMIT_COST_LAT_NP_ON_DEFAULT,
                    T lat_np_ideal_in = ((T)LIMIT_COST_LAT_NP_LIMIT_DEFAULT),
                    T lat_np_margin_multiplier_in = ((T)LIMIT_COST_LAT_NP_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_np_on = lat_np_on_in;
        lat_np_ideal = lat_np_ideal_in;
        lat_np_margin_multiplier = lat_np_margin_multiplier_in;
    }
    void set_lon_v(bool lon_v_on_in = LIMIT_COST_LON_V_ON_DEFAULT,
                   T lon_v_ideal_in = ((T)LIMIT_COST_LON_V_LIMIT_DEFAULT),
                   T lon_v_margin_multiplier_in = ((T)LIMIT_COST_LON_V_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_v_on = lon_v_on_in;
        lon_v_ideal = lon_v_ideal_in;
        lon_v_margin_multiplier = lon_v_margin_multiplier_in;
    }
    void set_lon_nv(bool lon_nv_on_in = LIMIT_COST_LON_NV_ON_DEFAULT,
                    T lon_nv_ideal_in = ((T)LIMIT_COST_LON_NV_LIMIT_DEFAULT),
                    T lon_nv_margin_multiplier_in = ((T)LIMIT_COST_LON_NV_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_nv_on = lon_nv_on_in;
        lon_nv_ideal = lon_nv_ideal_in;
        lon_nv_margin_multiplier = lon_nv_margin_multiplier_in;
    }
    void set_lat_v(bool lat_v_on_in = LIMIT_COST_LAT_V_ON_DEFAULT,
                   T lat_v_ideal_in = ((T)LIMIT_COST_LAT_V_LIMIT_DEFAULT),
                   T lat_v_margin_multiplier_in = ((T)LIMIT_COST_LAT_V_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_v_on = lat_v_on_in;
        lat_v_ideal = lat_v_ideal_in;
        lat_v_margin_multiplier = lat_v_margin_multiplier_in;
    }
    void set_lat_nv(bool lat_nv_on_in = LIMIT_COST_LAT_NV_ON_DEFAULT,
                    T lat_nv_ideal_in = ((T)LIMIT_COST_LAT_NV_LIMIT_DEFAULT),
                    T lat_nv_margin_multiplier_in = ((T)LIMIT_COST_LAT_NV_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_nv_on = lat_nv_on_in;
        lat_nv_ideal = lat_nv_ideal_in;
        lat_nv_margin_multiplier = lat_nv_margin_multiplier_in;
    }
    void set_lon_a(bool lon_a_on_in = LIMIT_COST_LON_A_ON_DEFAULT,
                   T lon_a_ideal_in = ((T)LIMIT_COST_LON_A_LIMIT_DEFAULT),
                   T lon_a_margin_multiplier_in = ((T)LIMIT_COST_LON_A_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_a_on = lon_a_on_in;
        lon_a_ideal = lon_a_ideal_in;
        lon_a_margin_multiplier = lon_a_margin_multiplier_in;
    }
    void set_lon_na(bool lon_na_on_in = LIMIT_COST_LON_NA_ON_DEFAULT,
                    T lon_na_ideal_in = ((T)LIMIT_COST_LON_NA_LIMIT_DEFAULT),
                    T lon_na_margin_multiplier_in = ((T)LIMIT_COST_LON_NA_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_na_on = lon_na_on_in;
        lon_na_ideal = lon_na_ideal_in;
        lon_na_margin_multiplier = lon_na_margin_multiplier_in;
    }
    void set_lat_a(bool lat_a_on_in = LIMIT_COST_LAT_A_ON_DEFAULT,
                   T lat_a_ideal_in = ((T)LIMIT_COST_LAT_A_LIMIT_DEFAULT),
                   T lat_a_margin_multiplier_in = ((T)LIMIT_COST_LAT_A_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_a_on = lat_a_on_in;
        lat_a_ideal = lat_a_ideal_in;
        lat_a_margin_multiplier = lat_a_margin_multiplier_in;
    }

    void set_lat_na(bool lat_na_on_in = LIMIT_COST_LAT_NA_ON_DEFAULT,
                    T lat_na_ideal_in = ((T)LIMIT_COST_LAT_NA_LIMIT_DEFAULT),
                    T lat_na_margin_multiplier_in = ((T)LIMIT_COST_LAT_NA_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_na_on = lat_na_on_in;
        lat_na_ideal = lat_na_ideal_in;
        lat_na_margin_multiplier = lat_na_margin_multiplier_in;
    }

    void set_lon_j(bool lon_j_on_in = LIMIT_COST_LON_J_ON_DEFAULT,
                   T lon_j_ideal_in = ((T)LIMIT_COST_LON_J_LIMIT_DEFAULT),
                   T lon_j_margin_multiplier_in = ((T)LIMIT_COST_LON_J_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_j_on = lon_j_on_in;
        lon_j_ideal = lon_j_ideal_in;
        lon_j_margin_multiplier = lon_j_margin_multiplier_in;
    }
    void set_lon_nj(bool lon_nj_on_in = LIMIT_COST_LON_NJ_ON_DEFAULT,
                    T lon_nj_ideal_in = ((T)LIMIT_COST_LON_NJ_LIMIT_DEFAULT),
                    T lon_nj_margin_multiplier_in = ((T)LIMIT_COST_LON_NJ_MARGIN_MULTIPLIER_DEFAULT))
    {
        lon_nj_on = lon_nj_on_in;
        lon_nj_ideal = lon_nj_ideal_in;
        lon_nj_margin_multiplier = lon_nj_margin_multiplier_in;
    }
    void set_lat_j(bool lat_j_on_in = LIMIT_COST_LAT_J_ON_DEFAULT,
                   T lat_j_ideal_in = ((T)LIMIT_COST_LAT_J_LIMIT_DEFAULT),
                   T lat_j_margin_multiplier_in = ((T)LIMIT_COST_LAT_J_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_j_on = lat_j_on_in;
        lat_j_ideal = lat_j_ideal_in;
        lat_j_margin_multiplier = lat_j_margin_multiplier_in;
    }
    void set_lat_nj(bool lat_nj_on_in = LIMIT_COST_LAT_NJ_ON_DEFAULT,
                    T lat_nj_ideal_in = ((T)LIMIT_COST_LAT_NJ_LIMIT_DEFAULT),
                    T lat_nj_margin_multiplier_in = ((T)LIMIT_COST_LAT_NJ_MARGIN_MULTIPLIER_DEFAULT))
    {
        lat_nj_on = lat_nj_on_in;
        lat_nj_ideal = lat_nj_ideal_in;
        lat_nj_margin_multiplier = lat_nj_margin_multiplier_in;
    }

    void setAllOff()
    {
        lon_p_on = false;
        lon_np_on = false;
        lat_p_on = false;
        lat_np_on = false;

        lon_v_on = false;
        lon_nv_on = false;
        lat_v_on = false;
        lat_nv_on = false;

        lon_a_on = false;
        lon_na_on = false;
        lat_a_on = false;
        lat_na_on = false;

        lon_j_on = false;
        lon_nj_on = false;
        lat_j_on = false;
        lat_nj_on = false;
    }

    // This has to be called if parameters have been changed
    void prepare()
    {
        T hp, hpp;

        // Calculate max and onset
        lon_p_max = lon_p_ideal * lon_p_margin_multiplier;
        lon_np_max = lon_np_ideal * lon_np_margin_multiplier;
        lat_p_max = lat_p_ideal * lat_p_margin_multiplier;
        lat_np_max = lat_np_ideal * lat_np_margin_multiplier;

        lon_v_max = lon_v_ideal * lon_v_margin_multiplier;
        lon_nv_max = lon_nv_ideal * lon_nv_margin_multiplier;
        lat_v_max = lat_v_ideal * lat_v_margin_multiplier;
        lat_nv_max = lat_nv_ideal * lat_nv_margin_multiplier;

        lon_a_max = lon_a_ideal * lon_a_margin_multiplier;
        lon_na_max = lon_na_ideal * lon_na_margin_multiplier;
        lat_a_max = lat_a_ideal * lat_a_margin_multiplier;
        lat_na_max = lat_na_ideal * lat_na_margin_multiplier;

        lon_j_max = lon_j_ideal * lon_j_margin_multiplier;
        lon_nj_max = lon_nj_ideal * lon_nj_margin_multiplier;
        lat_j_max = lat_j_ideal * lat_j_margin_multiplier;
        lat_nj_max = lat_nj_ideal * lat_nj_margin_multiplier;

        lon_p_onset = ((T)2.0) * lon_p_ideal - lon_p_max;
        lon_np_onset = ((T)2.0) * lon_np_ideal - lon_np_max;
        lat_p_onset = ((T)2.0) * lat_p_ideal - lat_p_max;
        lat_np_onset = ((T)2.0) * lat_np_ideal - lat_np_max;

        lon_v_onset = ((T)2.0) * lon_v_ideal - lon_v_max;
        lon_nv_onset = ((T)2.0) * lon_nv_ideal - lon_nv_max;
        lat_v_onset = ((T)2.0) * lat_v_ideal - lat_v_max;
        lat_nv_onset = ((T)2.0) * lat_nv_ideal - lat_nv_max;

        lon_a_onset = ((T)2.0) * lon_a_ideal - lon_a_max;
        lon_na_onset = ((T)2.0) * lon_na_ideal - lon_na_max;
        lat_a_onset = ((T)2.0) * lat_a_ideal - lat_a_max;
        lat_na_onset = ((T)2.0) * lat_na_ideal - lat_na_max;

        lon_j_onset = ((T)2.0) * lon_j_ideal - lon_j_max;
        lon_nj_onset = ((T)2.0) * lon_nj_ideal - lon_nj_max;
        lat_j_onset = ((T)2.0) * lat_j_ideal - lat_j_max;
        lat_nj_onset = ((T)2.0) * lat_nj_ideal - lat_nj_max;

        T delta_t_inv2 = delta_t_inv * delta_t_inv;
        T delta_t_inv3 = delta_t_inv2 * delta_t_inv;

        // Figure out what position barrier derivative is at p_ideal so we can scale it to match
        // the progress weight k_p
        lon_p_f.set(((T)1.0), lon_p_onset, lon_p_max);
        lon_p_f.H(hpp, hp, lon_p_ideal);
        T k_position = k_p / hp;

        // Figure out what velocity barrier derivative is at v_ideal so we can scale it to match
        // the progress weight k_p
        lon_v_f.set(((T)1.0), lon_v_onset, lon_v_max);
        lon_v_f.H(hpp, hp, lon_v_ideal);
        T k_velocity = k_p / hp;

        lon_k_p = k_position;
        lon_k_np = k_position;
        lat_k_p = k_position;
        lat_k_np = k_position;

        lon_k_v = k_velocity / delta_t_inv;
        lon_k_nv = k_velocity / delta_t_inv;
        lat_k_v = k_velocity / delta_t_inv;
        lat_k_nv = k_velocity / delta_t_inv;

        lon_k_a = k_velocity / delta_t_inv2;
        lon_k_na = k_velocity / delta_t_inv2;
        lat_k_a = k_velocity / delta_t_inv2;
        lat_k_na = k_velocity / delta_t_inv2;

        lon_k_j = k_velocity / delta_t_inv3;
        lon_k_nj = k_velocity / delta_t_inv3;
        lat_k_j = k_velocity / delta_t_inv3;
        lat_k_nj = k_velocity / delta_t_inv3;

        lon_p_f.set(lon_k_p, lon_p_onset, lon_p_max);
        lon_np_f.set(lon_k_np, lon_np_onset, lon_np_max);
        lat_p_f.set(lat_k_p, lat_p_onset, lat_p_max);
        lat_np_f.set(lat_k_np, lat_np_onset, lat_np_max);

        lon_v_f.set(lon_k_v, lon_v_onset, lon_v_max);
        lon_nv_f.set(lon_k_nv, lon_nv_onset, lon_nv_max);
        lat_v_f.set(lat_k_v, lat_v_onset, lat_v_max);
        lat_nv_f.set(lat_k_nv, lat_nv_onset, lat_nv_max);

        lon_a_f.set(lon_k_a, lon_a_onset, lon_a_max);
        lon_na_f.set(lon_k_na, lon_na_onset, lon_na_max);
        lat_a_f.set(lat_k_a, lat_a_onset, lat_a_max);
        lat_na_f.set(lat_k_na, lat_na_onset, lat_na_max);

        lon_j_f.set(lon_k_j, lon_j_onset, lon_j_max);
        lon_nj_f.set(lon_k_nj, lon_nj_onset, lon_nj_max);
        lat_j_f.set(lat_k_j, lat_j_onset, lat_j_max);
        lat_nj_f.set(lat_k_nj, lat_nj_onset, lat_nj_max);

        lon_p_nrT = 2;
        lon_np_nrT = 2;
        lat_p_nrT = 2;
        lat_np_nrT = 2;

        lon_v_nrT = 4;
        lon_nv_nrT = 4;
        lat_v_nrT = 4;
        lat_nv_nrT = 4;

        lon_a_nrT = 6;
        lon_na_nrT = 6;
        lat_a_nrT = 6;
        lat_na_nrT = 6;

        lon_j_nrT = 8;
        lon_nj_nrT = 8;
        lat_j_nrT = 8;
        lat_nj_nrT = 8;

        lon_p_taps[0] = ((T)(1.0));
        lon_p_taps[1] = ((T)(0.0));

        lon_np_taps[0] = ((T)(-1.0));
        lon_np_taps[1] = ((T)(0.0));

        lat_p_taps[0] = ((T)(0.0));
        lat_p_taps[1] = ((T)(1.0));

        lat_np_taps[0] = ((T)(0.0));
        lat_np_taps[1] = ((T)(-1.0));

        lon_v_taps[0] = delta_t_inv * ((T)(-1.0));
        lon_v_taps[1] = ((T)(0.0));
        lon_v_taps[2] = delta_t_inv * ((T)(1.0));
        lon_v_taps[3] = ((T)(0.0));

        lon_nv_taps[0] = delta_t_inv * ((T)(1.0));
        lon_nv_taps[1] = ((T)(0.0));
        lon_nv_taps[2] = delta_t_inv * ((T)(-1.0));
        lon_nv_taps[3] = ((T)(0.0));

        lat_v_taps[0] = ((T)(0.0));
        lat_v_taps[1] = delta_t_inv * ((T)(-1.0));
        lat_v_taps[2] = ((T)(0.0));
        lat_v_taps[3] = delta_t_inv * ((T)(1.0));

        lat_nv_taps[0] = ((T)(0.0));
        lat_nv_taps[1] = delta_t_inv * ((T)(1.0));
        lat_nv_taps[2] = ((T)(0.0));
        lat_nv_taps[3] = delta_t_inv * ((T)(-1.0));

        lon_a_taps[0] = delta_t_inv2 * ((T)(1.0));
        lon_a_taps[1] = ((T)(0.0));
        lon_a_taps[2] = delta_t_inv2 * ((T)(-2.0));
        lon_a_taps[3] = ((T)(0.0));
        lon_a_taps[4] = delta_t_inv2 * ((T)(1.0));
        lon_a_taps[5] = ((T)(0.0));

        lon_na_taps[0] = delta_t_inv2 * ((T)(-1.0));
        lon_na_taps[1] = ((T)(0.0));
        lon_na_taps[2] = delta_t_inv2 * ((T)(2.0));
        lon_na_taps[3] = ((T)(0.0));
        lon_na_taps[4] = delta_t_inv2 * ((T)(-1.0));
        lon_na_taps[5] = ((T)(0.0));

        lat_a_taps[0] = ((T)(0.0));
        lat_a_taps[1] = delta_t_inv2 * ((T)(1.0));
        lat_a_taps[2] = ((T)(0.0));
        lat_a_taps[3] = delta_t_inv2 * ((T)(-2.0));
        lat_a_taps[4] = ((T)(0.0));
        lat_a_taps[5] = delta_t_inv2 * ((T)(1.0));

        lat_na_taps[0] = ((T)(0.0));
        lat_na_taps[1] = delta_t_inv2 * ((T)(-1.0));
        lat_na_taps[2] = ((T)(0.0));
        lat_na_taps[3] = delta_t_inv2 * ((T)(2.0));
        lat_na_taps[4] = ((T)(0.0));
        lat_na_taps[5] = delta_t_inv2 * ((T)(-1.0));

        lon_j_taps[0] = delta_t_inv3 * ((T)(-1.0));
        lon_j_taps[1] = ((T)(0.0));
        lon_j_taps[2] = delta_t_inv3 * ((T)(3.0));
        lon_j_taps[3] = ((T)(0.0));
        lon_j_taps[4] = delta_t_inv3 * ((T)(-3.0));
        lon_j_taps[5] = ((T)(0.0));
        lon_j_taps[6] = delta_t_inv3 * ((T)(1.0));
        lon_j_taps[7] = ((T)(0.0));

        lon_nj_taps[0] = delta_t_inv3 * ((T)(1.0));
        lon_nj_taps[1] = ((T)(0.0));
        lon_nj_taps[2] = delta_t_inv3 * ((T)(-3.0));
        lon_nj_taps[3] = ((T)(0.0));
        lon_nj_taps[4] = delta_t_inv3 * ((T)(3.0));
        lon_nj_taps[5] = ((T)(0.0));
        lon_nj_taps[6] = delta_t_inv3 * ((T)(-1.0));
        lon_nj_taps[7] = ((T)(0.0));

        lat_j_taps[0] = ((T)(0.0));
        lat_j_taps[1] = delta_t_inv3 * ((T)(-1.0));
        lat_j_taps[2] = ((T)(0.0));
        lat_j_taps[3] = delta_t_inv3 * ((T)(3.0));
        lat_j_taps[4] = ((T)(0.0));
        lat_j_taps[5] = delta_t_inv3 * ((T)(-3.0));
        lat_j_taps[6] = ((T)(0.0));
        lat_j_taps[7] = delta_t_inv3 * ((T)(1.0));

        lat_nj_taps[0] = ((T)(0.0));
        lat_nj_taps[1] = delta_t_inv3 * ((T)(1.0));
        lat_nj_taps[2] = ((T)(0.0));
        lat_nj_taps[3] = delta_t_inv3 * ((T)(-3.0));
        lat_nj_taps[4] = ((T)(0.0));
        lat_nj_taps[5] = delta_t_inv3 * ((T)(3.0));
        lat_nj_taps[6] = ((T)(0.0));
        lat_nj_taps[7] = delta_t_inv3 * ((T)(-1.0));
    }

    limitCostParameters()
    {
        setDefault(((T)1.0), PROGRESS_WEIGHT_DEFAULT);
    }
};

template <class T>
inline T speedLimitCost(const T *x, T k_v, T v_onset, T v_max, T delta_t_inv, const T *stageWeights, int nrTimesteps)
{
    int i;
    T acc, h;
    T v_i;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        v_i = (x[2 * i] - x[2 * (i - 1)]) * delta_t_inv;
        smoothBarrierFunction<T>(h, v_i, k_v, v_onset, v_max);

        acc += stageWeights[i] * h;
    }

    return (acc);
}

template <class T>
inline T speedLimitCost(const T *x, const limitCostParameters<T> *P, T delta_t_inv, const T *stageWeights,
                        int nrTimesteps)
{
    return (speedLimitCost<T>(x, P->lon_k_v, P->lon_v_onset, P->lon_v_max, delta_t_inv, stageWeights, nrTimesteps));
}

template <class T>
inline T speedLimitHessianContribution(T *H, T *g, const T *x, T k_v, T v_onset, T v_max, T delta_t_inv,
                                       const T *stageWeights, int nrTimesteps, int UB)
{
    int i;
    T acc, h, hp, hpp;
    T v_i, delta_t_inv2;

    delta_t_inv2 = delta_t_inv * delta_t_inv;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        v_i = (x[2 * i] - x[2 * (i - 1)]) * delta_t_inv;
        smoothBarrierFunctionHessian<T>(h, hp, hpp, v_i, k_v, v_onset, v_max);

        acc += stageWeights[i] * h;

        // gradient contribution
        g[2 * i] += stageWeights[i] * hp * delta_t_inv;
        if (i > 0)
            g[2 * (i - 1)] += stageWeights[i] * hp * (-delta_t_inv);

        // Hessian contribution
        H[2 * i * UB] += stageWeights[i] * hpp * delta_t_inv2;
        if (i > 0)
        {
            H[2 * (i - 1) * UB + 2] += stageWeights[i] * hpp * (-delta_t_inv2);
            H[2 * (i - 1) * UB] += stageWeights[i] * hpp * delta_t_inv2;
        }
    }

    return (acc);
}

template <class T>
inline T speedLimitHessianContribution(T *H, T *g, const T *x, const limitCostParameters<T> *P, T delta_t_inv,
                                       const T *stageWeights, int nrTimesteps, int UB)
{
    return (speedLimitHessianContribution<T>(H, g, x, P->lon_k_v, P->lon_v_onset, P->lon_v_max, delta_t_inv,
                                             stageWeights, nrTimesteps, UB));
}

//---------------------------Using functionOfFilterCost to create speed limit only costs
template <class T>
inline T speedLimitOnlyCost(const T *x, const limitCostParameters<T> *P, const T *stageWeights, int nrTimesteps)
{
    T acc;

    acc = ((T)0.0);

    // Velocity
    acc += functionOfFilterCost<T, scalarFunctionSmoothBarrier<T>>(x, P->lon_v_taps, P->lon_v_nrT, &(P->lon_v_f),
                                                                   stageWeights, nrTimesteps);

    return (acc);
}

template <class T>
inline T speedLimitOnlyHessianContribution(T *H, T *g, const T *x, const limitCostParameters<T> *P,
                                           const T *stageWeights, int nrTimesteps, int UB)
{
    T acc;

    acc = ((T)0.0);

    // Velocity
    acc += functionOfFilterHessianContribution<T, scalarFunctionSmoothBarrier<T>>(
        H, g, x, P->lon_v_taps, P->lon_v_nrT, &P->lon_v_f, stageWeights, nrTimesteps, UB);

    return (acc);
}

//---------------------------Using functionOfFilterCost to create all limit costs
template <class T, class BFT = scalarFunctionSmoothBarrier<T>>
inline T allLimitCost(const T *x, const limitCostParameters<T, BFT> *P, const T *stageWeights, const T *probEpisodeEnd,
                      int nrTimesteps)
{
    T acc;

    acc = ((T)0.0);

    // Position
    if (P->lon_p_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_p_taps, P->lon_p_nrT, &P->lon_p_f, probEpisodeEnd, nrTimesteps);
    }
    if (P->lon_np_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_np_taps, P->lon_np_nrT, &P->lon_np_f, stageWeights, nrTimesteps);
    }
    if (P->lat_p_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_p_taps, P->lat_p_nrT, &P->lat_p_f, stageWeights, nrTimesteps);
    }
    if (P->lat_np_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_np_taps, P->lat_np_nrT, &P->lat_np_f, stageWeights, nrTimesteps);
    }

    // Velocity
    if (P->lon_v_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_v_taps, P->lon_v_nrT, &P->lon_v_f, stageWeights, nrTimesteps);
    }
    if (P->lon_nv_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_nv_taps, P->lon_nv_nrT, &P->lon_nv_f, stageWeights, nrTimesteps);
    }
    if (P->lat_v_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_v_taps, P->lat_v_nrT, &P->lat_v_f, stageWeights, nrTimesteps);
    }
    if (P->lat_nv_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_nv_taps, P->lat_nv_nrT, &P->lat_nv_f, stageWeights, nrTimesteps);
    }

    // Acceleration
    if (P->lon_a_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_a_taps, P->lon_a_nrT, &P->lon_a_f, stageWeights, nrTimesteps);
    }
    if (P->lon_na_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_na_taps, P->lon_na_nrT, &P->lon_na_f, stageWeights, nrTimesteps);
    }
    if (P->lat_a_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_a_taps, P->lat_a_nrT, &P->lat_a_f, stageWeights, nrTimesteps);
    }
    if (P->lat_na_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_na_taps, P->lat_na_nrT, &P->lat_na_f, stageWeights, nrTimesteps);
    }

    // Jerk
    if (P->lon_j_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_j_taps, P->lon_j_nrT, &P->lon_j_f, stageWeights, nrTimesteps);
    }
    if (P->lon_nj_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lon_nj_taps, P->lon_nj_nrT, &P->lon_nj_f, stageWeights, nrTimesteps);
    }
    if (P->lat_j_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_j_taps, P->lat_j_nrT, &P->lat_j_f, stageWeights, nrTimesteps);
    }
    if (P->lat_nj_on)
    {
        acc += functionOfFilterCost<T, BFT>(x, P->lat_nj_taps, P->lat_nj_nrT, &P->lat_nj_f, stageWeights, nrTimesteps);
    }

    return (acc);
}

template <class T, class BFT = scalarFunctionSmoothBarrier<T>>
inline T allLimitHessianContribution(T *H, T *g, const T *x, const limitCostParameters<T, BFT> *P,
                                     const T *stageWeights, const T *probEpisodeEnd, int nrTimesteps, int UB)
{
    T acc;

    acc = ((T)0.0);

    // Velocity
    if (P->lon_p_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_p_taps, P->lon_p_nrT, &P->lon_p_f,
                                                           probEpisodeEnd, nrTimesteps, UB);
    }
    if (P->lon_np_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_np_taps, P->lon_np_nrT, &P->lon_np_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_p_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_p_taps, P->lat_p_nrT, &P->lat_p_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_np_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_np_taps, P->lat_np_nrT, &P->lat_np_f,
                                                           stageWeights, nrTimesteps, UB);
    }

    // Velocity
    if (P->lon_v_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_v_taps, P->lon_v_nrT, &P->lon_v_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lon_nv_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_nv_taps, P->lon_nv_nrT, &P->lon_nv_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_v_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_v_taps, P->lat_v_nrT, &P->lat_v_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_nv_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_nv_taps, P->lat_nv_nrT, &P->lat_nv_f,
                                                           stageWeights, nrTimesteps, UB);
    }

    // Acceleration
    if (P->lon_a_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_a_taps, P->lon_a_nrT, &P->lon_a_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lon_na_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_na_taps, P->lon_na_nrT, &P->lon_na_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_a_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_a_taps, P->lat_a_nrT, &P->lat_a_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_na_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_na_taps, P->lat_na_nrT, &P->lat_na_f,
                                                           stageWeights, nrTimesteps, UB);
    }

    // Jerk
    if (P->lon_j_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_j_taps, P->lon_j_nrT, &P->lon_j_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lon_nj_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lon_nj_taps, P->lon_nj_nrT, &P->lon_nj_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_j_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_j_taps, P->lat_j_nrT, &P->lat_j_f,
                                                           stageWeights, nrTimesteps, UB);
    }
    if (P->lat_nj_on)
    {
        acc += functionOfFilterHessianContribution<T, BFT>(H, g, x, P->lat_nj_taps, P->lat_nj_nrT, &P->lat_nj_f,
                                                           stageWeights, nrTimesteps, UB);
    }

    return (acc);
}

#endif /*LIMIT_COSTS_H*/
