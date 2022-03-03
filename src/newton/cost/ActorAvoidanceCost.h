#ifndef ACTOR_AVOIDANCE_COST_H
#define ACTOR_AVOIDANCE_COST_H

#define ACTOR_AVOIDANCE_MAX 1e30

#define ACTOR_AVOIDANCE_SAFETY_BRAKING_DEFAULT 5.0 // m/s^2
#define ACTOR_AVOIDANCE_MAX_BRAKING_DEFAULT 10.0   // m/s^2

#define ACTOR_AVOIDANCE_LONGITUDINAL_ADD_DEFAULT 5.0 // 5m front
#define ACTOR_AVOIDANCE_LONGITUDINAL_SUB_DEFAULT 3.0 // 3m back
#define ACTOR_AVOIDANCE_LATERAL_ADD_DEFAULT 2.0      // 2m side
#define ACTOR_AVOIDANCE_LATERAL_SUB_DEFAULT 2.0      // 2m side

#define ACTOR_AVOIDANCE_VELOCITY_ADD_DEFAULT 1.0  // 0.25 //1.0 //m/s
#define ACTOR_AVOIDANCE_REACTION_TIME_DEFAULT 0.3 // 0.1 //0.3 //s

#define ACTOR_AVOIDANCE_TIME_STEP_INV_DEFAULT 10.0 // 1/0.1 seconds

#define ACTOR_AVOIDANCE_B_ONSET_DEFAULT 0.0

// Three second rule says that with
// velocity 20m/s,distance should be 60m.
// velocity 30m/s,distance should be 90m.
// Applying the three second rule gives more conservative behavior at a smaller velocity set point due to
// the behaviors of reaction time and velocity margin additions
// Default longitudinal add (such as 5m) is added inside so the distances given here should be
// distance from roughly front bumper (more exactly rear axle+ACTOR_AVOIDANCE_LONGITUDINAL_ADD_DEFAULT)
// to lead vehicle closest vertex.
#define ACTOR_AVOIDANCE_SET_POINT_VELOCITY_DEFAULT 30.0 // 30.0
#define ACTOR_AVOIDANCE_SET_POINT_DISTANCE_DEFAULT 90.0 // 90.0
// These relative multipliers, relative to the forward direction are used when weights are set by set point
#define ACTOR_AVOIDANCE_BACKWARD_WEIGHT_MULTIPLIER_DEFAULT 0.125
#define ACTOR_AVOIDANCE_LEFT_WEIGHT_MULTIPLIER_DEFAULT 0.5
#define ACTOR_AVOIDANCE_RIGHT_WEIGHT_MULTIPLIER_DEFAULT 0.5
// These are only used if weights are set directly
#define ACTOR_AVOIDANCE_WEIGHT_FORWARD_DEFAULT 4.0
#define ACTOR_AVOIDANCE_WEIGHT_BACKWARD_DEFAULT 0.5
#define ACTOR_AVOIDANCE_WEIGHT_LEFT_DEFAULT 2.0
#define ACTOR_AVOIDANCE_WEIGHT_RIGHT_DEFAULT 2.0

#include <iostream>
using namespace std;
#include "../ScalarFunctionWrappers.h"
#include "../SmoothBarrierFunction.h"
#include "ProgressCost.h"

// BFT (Barrier Function Type) is the type of barrier function used for all the limits
// It needs to have an initialization function
// void set(T k_h, T x_onset, T x_max)
// where k_h is a weight, x_onset is where it starts rising (typically being zero before that) and x_max is the barrier
// position It also has to have the core cost and two derivatives of cost functions
// T f(T x) const
// T H(T& fpp, T& fp, T x) const
// where the return value is cost, and fp and fpp are first and second derivative, respectively
template <class T, class BFT = scalarFunctionSmoothBarrier<T>> class actorAvoidanceCostParametersCore
{
  public:
    T beta_s;  // Safety braking of ego
    T beta_cs; // Saftey braking of contender
    T beta_m;  // Max braking of ego
    T beta_cm; // Max braking of contender

    T x_a;         // Distance margin
    T v_a;         // Velocity margin
    T v_ca;        // Contender velocity margin
    T t_r;         // Reaction time
    T t_cr;        // Contender reaction time
    T delta_t_inv; // Inverse of the timestep
    T delta_t;     // The timestep

    T k_b;     // Weight for this direction
    T b_onset; // Onset for barrier function

    BFT f;

    void setDefault(T delta_t_inv_input = ((T)ACTOR_AVOIDANCE_TIME_STEP_INV_DEFAULT),
                    T x_a_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_ADD_DEFAULT),
                    T k_b_input = ((T)ACTOR_AVOIDANCE_WEIGHT_FORWARD_DEFAULT))
    {
        beta_s = ((T)ACTOR_AVOIDANCE_SAFETY_BRAKING_DEFAULT);
        beta_cs = ((T)ACTOR_AVOIDANCE_SAFETY_BRAKING_DEFAULT);
        beta_m = ((T)ACTOR_AVOIDANCE_MAX_BRAKING_DEFAULT);
        beta_cm = ((T)ACTOR_AVOIDANCE_MAX_BRAKING_DEFAULT);

        x_a = x_a_input;
        v_a = ((T)ACTOR_AVOIDANCE_VELOCITY_ADD_DEFAULT);
        v_ca = ((T)ACTOR_AVOIDANCE_VELOCITY_ADD_DEFAULT);
        t_r = ((T)ACTOR_AVOIDANCE_REACTION_TIME_DEFAULT);
        t_cr = ((T)ACTOR_AVOIDANCE_REACTION_TIME_DEFAULT);

        delta_t_inv = delta_t_inv_input;
        delta_t = ((T)1.0) / delta_t_inv;

        k_b = k_b_input;
        b_onset = ((T)ACTOR_AVOIDANCE_B_ONSET_DEFAULT);

        prepare();
    }

    void prepare()
    {
        f.set(k_b, b_onset, ((T)1.0));
    }

    actorAvoidanceCostParametersCore()
    {
        setDefault();
    }
};

template <class T, class BFT = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostCore(T &b, T x, T v, T xc, T vc, const actorAvoidanceCostParametersCore<T, BFT> *P)
{
    T d;

    d = xc - x;

    if (d <= ((T)0.0))
    {
        // Failed to separate case
        b = ((T)(ACTOR_AVOIDANCE_MAX));
        return (0);
    }
    if (vc >= ((T)0.0))
    {
        if (v <= ((T)0.0))
        {
            // Separating case
            b = ((T)0.0);
            return (1);
        }
        // Chasing case
        b = v * v / (P->beta_s * (((T)2.0) * d + vc * vc / P->beta_cm));
        return (2);
    }
    if (v <= ((T)0.0))
    {
        // Being chased case
        b = vc * vc / (P->beta_cs * (((T)2.0) * d + v * v / P->beta_m));
        return (3);
    }
    // Antagonists case
    b = ((T)0.5) / d * (v * v / P->beta_s + vc * vc / P->beta_cs);
    return (4);
}

// dbdx returns [db/dx db/dv]
// H returns [db^2/dx^2 db^2/dxdy db^2/dy^2]
template <class T, class BFT = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostCoreHessian(T &b, T dbdx[2], T H[3], T x, T v, T xc, T vc,
                                         const actorAvoidanceCostParametersCore<T, BFT> *P)
{
    T d, rec, v_rec, beta_rec;

    d = xc - x;

    if (d <= ((T)0.0))
    {
        // Failed to separate case
        b = ((T)(ACTOR_AVOIDANCE_MAX));
        dbdx[0] = dbdx[1] = ((T)0.0);
        H[0] = H[1] = H[2] = ((T)0.0);
        return (0);
    }
    if (vc >= ((T)0.0))
    {
        if (v <= ((T)0.0))
        {
            // Separating case
            b = ((T)0.0);
            dbdx[0] = dbdx[1] = ((T)0.0);
            H[0] = H[1] = H[2] = ((T)0.0);
            return (1);
        }
        // Chasing case
        rec = ((T)1.0) / (P->beta_s * (((T)2.0) * d + vc * vc / P->beta_cm));
        v_rec = v * rec;
        b = v * v_rec;
        beta_rec = P->beta_s * rec;

        dbdx[0] = ((T)2.0) * beta_rec * b;
        dbdx[1] = ((T)2.0) * v_rec;

        H[0] = ((T)4.0) * dbdx[0] * beta_rec;
        H[1] = ((T)2.0) * dbdx[1] * beta_rec;
        H[2] = ((T)2.0) * rec;

        return (2);
    }
    if (v <= ((T)0.0))
    {
        // Being chased case
        T rec_beta_m = ((T)1.0) / P->beta_m;
        T v_v_rec_beta_m = v * v * rec_beta_m;
        rec = ((T)1.0) / (P->beta_cs * (((T)2.0) * d + v_v_rec_beta_m));
        v_rec = vc * rec;
        b = vc * v_rec;

        T rec_beta_cs = P->beta_cs * rec;

        dbdx[0] = ((T)2.0) * rec_beta_cs * b;
        dbdx[1] = -v * dbdx[0] * rec_beta_m;

        H[0] = ((T)4.0) * dbdx[0] * rec_beta_cs;
        H[1] = -v * H[0] * rec_beta_m;
        H[2] = b * (((T)6.0) * v_v_rec_beta_m - ((T)4.0) * d) * rec_beta_cs * rec_beta_cs * rec_beta_m;

        return (3);
    }
    // Antagonists case
    rec = ((T)1.0) / d;
    v_rec = ((T)1.0) / P->beta_s;
    b = ((T)0.5) * rec * (v * v * v_rec + vc * vc / P->beta_cs);

    dbdx[0] = rec * b;
    dbdx[1] = rec * v_rec * v;

    H[0] = ((T)2.0) * rec * dbdx[0];
    H[1] = rec * dbdx[1];
    H[2] = rec * v_rec;

    return (4);
}

template <class T, class BFT = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostMargins(T &b, T x, T xmo, T xc, T vc, const actorAvoidanceCostParametersCore<T, BFT> *P)
{
    T x_hat, v_hat, xc_hat, vc_hat;
    T v, xpa, vpa, beta_m_tr, beta_cm_t_cr, vc_m_v_ca;

    v = (x - xmo) * P->delta_t_inv;
    xpa = x + P->x_a;
    vpa = v + P->v_a;
    beta_m_tr = P->beta_m * P->t_r;
    beta_cm_t_cr = P->beta_cm * P->t_cr;
    vc_m_v_ca = vc - P->v_ca;

    x_hat = xpa + (vpa + ((T)0.5) * beta_m_tr) * P->t_r;
    v_hat = vpa + beta_m_tr;
    xc_hat = xc + (vc_m_v_ca - ((T)0.5) * beta_cm_t_cr) * P->t_cr;
    vc_hat = vc_m_v_ca - beta_cm_t_cr;

    if (xc - xpa < ((T)0.0))
    {
        // Failed to separate case
        b = ((T)(ACTOR_AVOIDANCE_MAX));
        return (0);
    }

    return (actorAvoidanceCostCore<T, BFT>(b, x_hat, v_hat, xc_hat, vc_hat, P));
}

template <class T>
inline void applyLinearJacobianToHessian(T dbdx[2], T H[3], const T dbdx_loc[2], const T H_loc[3], const T A[4])
{
    T t00, t01, t10, t11;

    dbdx[0] = dbdx_loc[0] * A[0] + dbdx_loc[1] * A[2];
    dbdx[1] = dbdx_loc[0] * A[1] + dbdx_loc[1] * A[3];

    t00 = H_loc[0] * A[0] + H_loc[1] * A[2];
    t01 = H_loc[0] * A[1] + H_loc[1] * A[3];

    t10 = H_loc[1] * A[0] + H_loc[2] * A[2];
    t11 = H_loc[1] * A[1] + H_loc[2] * A[3];

    H[0] = A[0] * t00 + A[2] * t10;
    H[1] = A[0] * t01 + A[2] * t11;
    H[2] = A[1] * t01 + A[3] * t11;
}

// dbdx returns [db/dx db/dxmo]
// H returns [db^2/dx^2 db^2/dxdxmo db^2/dxmo^2]
template <class T, class BFT = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostMarginsHessian(T &b, T dbdx[2], T H[3], T x, T xmo, T xc, T vc,
                                            const actorAvoidanceCostParametersCore<T, BFT> *P)
{
    T x_hat, v_hat, xc_hat, vc_hat;
    T v, xpa, vpa, beta_m_tr, beta_cm_t_cr, vc_m_v_ca;

    v = (x - xmo) * P->delta_t_inv;
    xpa = x + P->x_a;
    vpa = v + P->v_a;
    beta_m_tr = P->beta_m * P->t_r;
    beta_cm_t_cr = P->beta_cm * P->t_cr;
    vc_m_v_ca = vc - P->v_ca;

    x_hat = xpa + (vpa + ((T)0.5) * beta_m_tr) * P->t_r;
    v_hat = vpa + beta_m_tr;
    xc_hat = xc + (vc_m_v_ca - ((T)0.5) * beta_cm_t_cr) * P->t_cr;
    vc_hat = vc_m_v_ca - beta_cm_t_cr;

    if (xc - xpa < ((T)0.0))
    {
        // Failed to separate case
        b = ((T)(ACTOR_AVOIDANCE_MAX));
        dbdx[0] = dbdx[1] = ((T)0.0);
        H[0] = H[1] = H[2] = ((T)0.0);
        return (0);
    }

    int back;
    T dbdx_loc[2], H_loc[3], A[4];
    back = actorAvoidanceCostCoreHessian<T, BFT>(b, dbdx_loc, H_loc, x_hat, v_hat, xc_hat, vc_hat, P);

    A[0] = ((T)1.0) + P->delta_t_inv * P->t_r;
    A[1] = -P->delta_t_inv * P->t_r;
    A[2] = P->delta_t_inv;
    A[3] = -P->delta_t_inv;

    applyLinearJacobianToHessian<T>(dbdx, H, dbdx_loc, H_loc, A);

    return (back);
}

// The suffixes _F _B _L _R stand for Forward, Backward, Left, Right
template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
class actorAvoidanceCostParameters
{
  public:
    actorAvoidanceCostParametersCore<T, BFT_F> forward;
    actorAvoidanceCostParametersCore<T, BFT_B> backward;
    actorAvoidanceCostParametersCore<T, BFT_L> left;
    actorAvoidanceCostParametersCore<T, BFT_R> right;

    void setDefaultByWeights(T k_p, // progress weight from progress cost
                             T delta_t_inv_input = ((T)ACTOR_AVOIDANCE_TIME_STEP_INV_DEFAULT),
                             T x_a_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_ADD_DEFAULT),
                             T x_s_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_SUB_DEFAULT),
                             T y_a_input = ((T)ACTOR_AVOIDANCE_LATERAL_ADD_DEFAULT),
                             T y_s_input = ((T)ACTOR_AVOIDANCE_LATERAL_SUB_DEFAULT),
                             T k_b_forward_input = ((T)ACTOR_AVOIDANCE_WEIGHT_FORWARD_DEFAULT),
                             T k_b_backward_input = ((T)ACTOR_AVOIDANCE_WEIGHT_BACKWARD_DEFAULT),
                             T k_b_left_input = ((T)ACTOR_AVOIDANCE_WEIGHT_LEFT_DEFAULT),
                             T k_b_right_input = ((T)ACTOR_AVOIDANCE_WEIGHT_RIGHT_DEFAULT))
    {
        forward.setDefault(delta_t_inv_input, x_a_input, k_b_forward_input);
        backward.setDefault(delta_t_inv_input, x_s_input, k_b_backward_input);
        left.setDefault(delta_t_inv_input, y_a_input, k_b_left_input);
        right.setDefault(delta_t_inv_input, y_s_input, k_b_right_input);
    }

    bool setDefaultBySetPoint(T k_p, // progress weight from progress cost
                              T delta_t_inv_input = ((T)ACTOR_AVOIDANCE_TIME_STEP_INV_DEFAULT),
                              T set_point_velocity_in = ((T)ACTOR_AVOIDANCE_SET_POINT_VELOCITY_DEFAULT),
                              T set_point_distance_in = ((T)ACTOR_AVOIDANCE_SET_POINT_DISTANCE_DEFAULT),
                              T x_a_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_ADD_DEFAULT),
                              T x_s_input = ((T)ACTOR_AVOIDANCE_LONGITUDINAL_SUB_DEFAULT),
                              T y_a_input = ((T)ACTOR_AVOIDANCE_LATERAL_ADD_DEFAULT),
                              T y_s_input = ((T)ACTOR_AVOIDANCE_LATERAL_SUB_DEFAULT),
                              T backward_weight_multiplier = ((T)ACTOR_AVOIDANCE_BACKWARD_WEIGHT_MULTIPLIER_DEFAULT),
                              T left_weight_multiplier = ((T)ACTOR_AVOIDANCE_LEFT_WEIGHT_MULTIPLIER_DEFAULT),
                              T right_weight_multiplier = ((T)ACTOR_AVOIDANCE_RIGHT_WEIGHT_MULTIPLIER_DEFAULT))
    {
        // Figure out what the forward weight is based on set point
        forward.setDefault(delta_t_inv_input, x_a_input, ((T)ACTOR_AVOIDANCE_WEIGHT_FORWARD_DEFAULT));
        bool set_point_ok = this->applySetPoint(set_point_distance_in, set_point_velocity_in, k_p);

        // Use the resulting weight with relatively multipliers for the other directions
        backward.setDefault(delta_t_inv_input, x_s_input, forward.k_b * backward_weight_multiplier);
        left.setDefault(delta_t_inv_input, y_a_input, forward.k_b * left_weight_multiplier);
        right.setDefault(delta_t_inv_input, y_s_input, forward.k_b * right_weight_multiplier);

        return (set_point_ok);
    }

    // Compute the barrier derivative at a distance, velocity pair
    // This is interesting because when that derivative matches k_p we have a steady state distance for that velocity
    T barrierDerivative(T d, T v)
    {
        // For a steady state, where lead vehicle and ego vehicle move at the same velocity, we get a
        // relation between the distance d and the normalized braking amount b via the chasing case of the function
        //
        // actorAvoidanceCostMargins<T,BFT>(b,((T)0.0),-v*forward->delta_t,d,v,&forward);
        //
        // Without additional margins, it would have been
        // actorAvoidanceCostCore<T, BFT>(b,((T)0.0),v,d,v,&forward);
        // which for the chasing case is more explicitly
        // b = v*v / (forward->beta_s * (((T)2.0) * d + v*v / forward->beta_cm));
        // or
        // b = ((T)1.0) / (forward->beta_s * (((T)2.0) * d/(v*v) + ((T)1.0)/ forward->beta_cm));
        //
        // We want the derivative of the barrier function, which can be computed by the function calls
        T b, dbdx[2], H[3];
        int back;
        back = actorAvoidanceCostMarginsHessian<T, BFT_F>(b, dbdx, H, ((T)0.0), -v * forward.delta_t, d, v, &forward);
        T h, hp, hpp;
        h = forward.f.H(hpp, hp, b);
        T barrier_derivative = hp * dbdx[0];
        if ((back == 0) || (h >= ACTOR_AVOIDANCE_MAX))
        {
            barrier_derivative = ((T)ACTOR_AVOIDANCE_MAX);
        }

        return (barrier_derivative);
    }

    // Given the progress weight k_p from the progress cost parameters, make the ideal distance d at velocity v
    // Returns true if set point is feasible, and false if it is too tight, into safety braking
    bool applySetPoint(T d, T v, T k_p, bool add_forward_add = true)
    {
        T d_padded = d;
        if (add_forward_add)
            d_padded += forward.x_a;
        // We want k_b_multiplier*barrier_derivative=k_p
        // which means that k_b_multiplier = k_p/barrier_derivative
        T barrier_derivative = this->barrierDerivative(d_padded, v);
        T k_b_multiplier = k_p / barrier_derivative;
        this->forward.k_b *= k_b_multiplier;
        this->forward.prepare();

        if (barrier_derivative >= ((T)ACTOR_AVOIDANCE_MAX))
        {
            // SetPoint too tight, is into safety braking
            return (false);
        }
        return (true);
    }

    // Find the distance at which the barrier derivative matches k_p for a given velocity v
    T findDistanceFromVelocity(T v, T k_p, bool subtract_forward_add = true, int max_iterations = 100)
    {
        T d, d_min, d_max;

        d_min = ((T)0.0);
        d_max = ((T)10000.0);
        for (int i = 0; i < max_iterations; i++)
        {
            d = ((T)0.5) * (d_min + d_max);

            T bd = this->barrierDerivative(d, v);
            if (bd < k_p)
            {
                // Derivative too small, make distance smaller
                d_max = d;
            }
            else
            {
                // Derivative too large, make distance larger
                d_min = d;
            }
        }

        if (subtract_forward_add)
            d_max -= forward.x_a;
        return (d_max);
    }

    void printDistanceToVelocityTable(T k_p, T min_velocity = ((T)0.0), T max_velocity = ((T)40.0), int nr_steps = 41,
                                      bool subtract_forward_add = true)
    {
        cout << "[" << endl;
        for (int i = 0; i < nr_steps; i++)
        {
            int nr = nr_steps - 1;
            if (nr <= 0)
                nr = 1;
            T v = min_velocity + (max_velocity - min_velocity) * ((T)i) / ((T)(nr));
            cout << "V: " << v << "->D: " << this->findDistanceFromVelocity(v, k_p, subtract_forward_add) << endl;
        }
        cout << "]" << endl;
    }

    actorAvoidanceCostParameters()
    {
        setDefaultBySetPoint(PROGRESS_WEIGHT_DEFAULT);
    }
};

template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostDirections(T &b, T x, T xmo, T y, T ymo, T xc, T yc, T vxc, T vyc,
                                        const actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> *P)
{
    T b2;
    int back, back2;

    back = actorAvoidanceCostMargins<T, BFT_F>(b, x, xmo, xc, vxc, &P->forward);
    back2 = actorAvoidanceCostMargins<T, BFT_B>(b2, -x, -xmo, -xc, -vxc, &P->backward);
    if (b2 < b)
    {
        b = b2;
        back = back2 | (1 << 3);
    }
    back2 = actorAvoidanceCostMargins<T, BFT_L>(b2, y, ymo, yc, vyc, &P->left);
    if (b2 < b)
    {
        b = b2;
        back = back2 | (2 << 3);
    }
    back2 = actorAvoidanceCostMargins<T, BFT_R>(b2, -y, -ymo, -yc, -vyc, &P->right);
    if (b2 < b)
    {
        b = b2;
        back = back2 | (3 << 3);
    }

    return (back);
}

// dbdx returns [db/dx db/dxmo]
// H returns [db^2/dx^2 db^2/dxdxmo db^2/dxmo^2]

template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostDirectionsHessianForward(T &b, T dbdx[2], T H[3], T x, T xmo, T xc, T vxc,
                                                      const actorAvoidanceCostParametersCore<T, BFT_F> *P)
{
    return (actorAvoidanceCostMarginsHessian<T, BFT_F>(b, dbdx, H, x, xmo, xc, vxc, P));
}

// dbdx returns [db/dx db/dxmo]
// H returns [db^2/dx^2 db^2/dxdxmo db^2/dxmo^2]
template <class T, class BFT_B = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostDirectionsHessianBackward(T &b, T dbdx[2], T H[3], T x, T xmo, T xc, T vxc,
                                                       const actorAvoidanceCostParametersCore<T, BFT_B> *P)
{
    int back;
    back = actorAvoidanceCostMarginsHessian<T, BFT_B>(b, dbdx, H, -x, -xmo, -xc, -vxc, P);
    dbdx[0] = -dbdx[0];
    dbdx[1] = -dbdx[1];
    return (back);
}

// dbdy returns [db/dy db/dymo]
// H returns [db^2/dy^2 db^2/dydymo db^2/dymo^2]
template <class T, class BFT_L = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostDirectionsHessianLeft(T &b, T dbdy[2], T H[3], T y, T ymo, T yc, T vyc,
                                                   const actorAvoidanceCostParametersCore<T, BFT_L> *P)
{
    return (actorAvoidanceCostMarginsHessian<T, BFT_L>(b, dbdy, H, y, ymo, yc, vyc, P));
}

// dbdy returns [db/dy db/dymo]
// H returns [db^2/dy^2 db^2/dydymo db^2/dymo^2]
template <class T, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline int actorAvoidanceCostDirectionsHessianRight(T &b, T dbdy[2], T H[3], T y, T ymo, T yc, T vyc,
                                                    const actorAvoidanceCostParametersCore<T, BFT_R> *P)
{
    int back;
    back = actorAvoidanceCostMarginsHessian<T, BFT_R>(b, dbdy, H, -y, -ymo, -yc, -vyc, P);
    dbdy[0] = -dbdy[0];
    dbdy[1] = -dbdy[1];
    return (back);
}

// The array XC is expected to hold [xc0 yc0 vxc0 vyc0 xc1 yc1 vxc1 vyc1 ......]
template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline void actorAvoidanceCostMostConstraining_i(int &mc_forward, int &mc_backward, int &mc_left, int &mc_right,
                                                 T &b_forward, T &b_backward, T &b_left, T &b_right, T x, T xmo, T y,
                                                 T ymo, const T *XC, int nrXC,
                                                 const actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> *P)
{
    int i;
    int back;
    T b_loc;

    mc_forward = mc_backward = mc_left = mc_right = -1;
    b_forward = b_backward = b_left = b_right = ((T)0.0);
    for (i = 0; i < nrXC; i++)
    {
        back = actorAvoidanceCostDirections<T, BFT_F, BFT_B, BFT_L, BFT_R>(
            b_loc, x, xmo, y, ymo, XC[4 * i], XC[4 * i + 1], XC[4 * i + 2], XC[4 * i + 3], P);
        switch ((back >> 3) & 3)
        {
        case (0): // forward was the least constraining for this vertex
            if (b_loc > b_forward)
            {
                // and it is the most constraining forward so far
                b_forward = b_loc;
                mc_forward = i;
            }
            break;
        case (1): // backward was the least constraining for this vertex
            if (b_loc > b_backward)
            {
                // and it is the most constraining backward so far
                b_backward = b_loc;
                mc_backward = i;
            }
            break;
        case (2): // left was the least constraining for this vertex
            if (b_loc > b_left)
            {
                // and it is the most constraining left so far
                b_left = b_loc;
                mc_left = i;
            }
            break;
        case (3): // right was the least constraining for this vertex
            if (b_loc > b_right)
            {
                // and it is the most constraining right so far
                b_right = b_loc;
                mc_right = i;
            }
            break;
        }
    }
}

template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline T actorAvoidanceCost_i(T x, T xmo, T y, T ymo, const T *XC, int nrXC,
                              const actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> *P)
{
    int mc_forward, mc_backward, mc_left, mc_right;
    T b, b_forward, b_backward, b_left, b_right;
    T h;

    actorAvoidanceCostMostConstraining_i<T, BFT_F, BFT_B, BFT_L, BFT_R>(mc_forward, mc_backward, mc_left, mc_right,
                                                                        b_forward, b_backward, b_left, b_right, x, xmo,
                                                                        y, ymo, XC, nrXC, P);

    b = ((T)0.0);
    if (mc_forward >= 0)
    {
        h = P->forward.f.f(b_forward);
        // smoothBarrierFunction<T>(h,b_forward ,P->forward.k_b ,P->forward.b_onset ,((T)1.0));
        b += h;
    }
    if (mc_backward >= 0)
    {
        h = P->backward.f.f(b_backward);
        // smoothBarrierFunction<T>(h,b_backward,P->backward.k_b,P->backward.b_onset,((T)1.0));
        b += h;
    }
    if (mc_left >= 0)
    {
        h = P->left.f.f(b_left);
        // smoothBarrierFunction<T>(h,b_left    ,P->left.k_b    ,P->left.b_onset    ,((T)1.0));
        b += h;
    }
    if (mc_right >= 0)
    {
        h = P->right.f.f(b_right);
        // smoothBarrierFunction<T>(h, b_right  , P->right.k_b  , P->right.b_onset  ,((T)1.0));
        b += h;
    }

    return (b);
}

template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline T actorAvoidanceHessian_i(T gx[2], T gy[2], T Hx[3], T Hy[3], T x, T xmo, T y, T ymo, const T *XC, int nrXC,
                                 const actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> *P)
{
    int mc_forward, mc_backward, mc_left, mc_right;
    T b, b_forward, b_backward, b_left, b_right;
    T h, hp, hpp;
    T g[2], H[3];

    actorAvoidanceCostMostConstraining_i<T, BFT_F, BFT_B, BFT_L, BFT_R>(mc_forward, mc_backward, mc_left, mc_right,
                                                                        b_forward, b_backward, b_left, b_right, x, xmo,
                                                                        y, ymo, XC, nrXC, P);

    b = ((T)0.0);
    gx[0] = gx[1] = gy[0] = gy[1] = ((T)0.0);
    Hx[0] = Hx[1] = Hx[2] = Hy[0] = Hy[1] = Hy[2] = ((T)0.0);

    if (mc_forward >= 0)
    {
        actorAvoidanceCostDirectionsHessianForward<T, BFT_F>(b_forward, g, H, x, xmo, XC[4 * mc_forward],
                                                             XC[4 * mc_forward + 2], &P->forward);
        h = P->forward.f.H(hpp, hp, b_forward);
        // smoothBarrierFunctionHessian<T>(h, hp, hpp, b_forward, P->forward.k_b, P->forward.b_onset, ((T)1.0));

        // Turn g and H and h,hp,hpp into contributions to gx,Hx
        // gx=dh/dx=(dh/db)*(db/dx)=hp*g
        gx[0] += hp * g[0];
        gx[1] += hp * g[1];
        // Hx=(d/dx)(dh/dx)=(d/dx)((dh/db)*(db/dx))=
        //((d/dx)(dh/db))*(db/dx)+(dh/db)*((d/dx)(db/dx))=
        //((db/dx)'*(dh2/db2))*(db/dx)+(dh/db)*(  db2/dx2    )=
        //    g'   *   hpp    *   g   +   hp  *      H=
        // hpp*g'g+hp*H
        Hx[0] += hpp * g[0] * g[0] + hp * H[0];
        Hx[1] += hpp * g[0] * g[1] + hp * H[1];
        Hx[2] += hpp * g[1] * g[1] + hp * H[2];

        b += h;
    }
    if (mc_backward >= 0)
    {
        actorAvoidanceCostDirectionsHessianBackward<T, BFT_B>(b_backward, g, H, x, xmo, XC[4 * mc_backward],
                                                              XC[4 * mc_backward + 2], &P->backward);
        h = P->backward.f.H(hpp, hp, b_backward);
        // smoothBarrierFunctionHessian<T>(h, hp, hpp, b_backward, P->backward.k_b, P->backward.b_onset, ((T)1.0));

        // Turn g and H and h,hp,hpp into contributions to gx,Hx
        // gx=dh/dx=(dh/db)*(db/dx)=hp*g
        gx[0] += hp * g[0];
        gx[1] += hp * g[1];
        // hpp*g'g+hp*H as above
        Hx[0] += hpp * g[0] * g[0] + hp * H[0];
        Hx[1] += hpp * g[0] * g[1] + hp * H[1];
        Hx[2] += hpp * g[1] * g[1] + hp * H[2];

        b += h;
    }
    if (mc_left >= 0)
    {
        actorAvoidanceCostDirectionsHessianLeft<T, BFT_L>(b_left, g, H, y, ymo, XC[4 * mc_left + 1],
                                                          XC[4 * mc_left + 3], &P->left);
        h = P->left.f.H(hpp, hp, b_left);
        // smoothBarrierFunctionHessian<T>(h, hp, hpp, b_left, P->left.k_b, P->left.b_onset, ((T)1.0));

        // Turn g and H and h,hp,hpp into contributions to gy,Hy
        // gy=dh/dy=(dh/db)*(db/dy)=hp*g
        gy[0] += hp * g[0];
        gy[1] += hp * g[1];
        // hpp*g'g+hp*H as above
        Hy[0] += hpp * g[0] * g[0] + hp * H[0];
        Hy[1] += hpp * g[0] * g[1] + hp * H[1];
        Hy[2] += hpp * g[1] * g[1] + hp * H[2];

        b += h;
    }
    if (mc_right >= 0)
    {
        actorAvoidanceCostDirectionsHessianRight<T, BFT_R>(b_right, g, H, y, ymo, XC[4 * mc_right + 1],
                                                           XC[4 * mc_right + 3], &P->right);
        h = P->right.f.H(hpp, hp, b_right);
        // smoothBarrierFunctionHessian<T>(h, hp, hpp, b_right, P->right.k_b, P->right.b_onset, ((T)1.0));

        // Turn g and H and h,hp,hpp into contributions to g,H
        // gy=dh/dy=(dh/db)*(db/dy)=hp*g
        gy[0] += hp * g[0];
        gy[1] += hp * g[1];
        // hpp*g'g+hp*H as above
        Hy[0] += hpp * g[0] * g[0] + hp * H[0];
        Hy[1] += hpp * g[0] * g[1] + hp * H[1];
        Hy[2] += hpp * g[1] * g[1] + hp * H[2];

        b += h;
    }

    return (b);
}

template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline T actorAvoidanceCost(const T *x, const T *const *XC, const int *nrXC, const T *stageWeights, int nrTimesteps,
                            const actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> *P)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        acc += stageWeights[i] * actorAvoidanceCost_i<T, BFT_F, BFT_B, BFT_L, BFT_R>(
                                     x[2 * i], x[2 * (i - 1)], x[2 * i + 1], x[2 * (i - 1) + 1], XC[i], nrXC[i], P);
    }

    return (acc);
}

template <class T, class BFT_F = scalarFunctionSmoothBarrier<T>, class BFT_B = scalarFunctionSmoothBarrier<T>,
          class BFT_L = scalarFunctionSmoothBarrier<T>, class BFT_R = scalarFunctionSmoothBarrier<T>>
inline T actorAvoidanceHessianContribution(T *H, T *g, const T *x, const T *const *XC, const int *nrXC,
                                           const T *stageWeights, int nrTimesteps, int UB,
                                           const actorAvoidanceCostParameters<T, BFT_F, BFT_B, BFT_L, BFT_R> *P)
{
    int i;
    T acc;
    T gx[2], gy[2], Hx[3], Hy[3];

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        acc += stageWeights[i] *
               actorAvoidanceHessian_i<T, BFT_F, BFT_B, BFT_L, BFT_R>(
                   gx, gy, Hx, Hy, x[2 * i], x[2 * (i - 1)], x[2 * i + 1], x[2 * (i - 1) + 1], XC[i], nrXC[i], P);

        // Turn Hx,Hy and gx,gy into contributions to H, g
        g[2 * i] += stageWeights[i] * gx[0];
        g[2 * i + 1] += stageWeights[i] * gy[0];

        H[2 * i * UB] += stageWeights[i] * Hx[0];
        H[(2 * i + 1) * UB] += stageWeights[i] * Hy[0];

        if (i > 0)
        {
            g[2 * (i - 1)] += stageWeights[i] * gx[1];
            g[2 * (i - 1) + 1] += stageWeights[i] * gy[1];

            H[2 * (i - 1) * UB + 2] += stageWeights[i] * Hx[1];
            H[2 * (i - 1) * UB] += stageWeights[i] * Hx[2];
            H[(2 * (i - 1) + 1) * UB + 2] += stageWeights[i] * Hy[1];
            H[(2 * (i - 1) + 1) * UB] += stageWeights[i] * Hy[2];
        }
    }

    return (acc);
}

#endif /*ACTOR_AVOIDANCE_COST_H*/
