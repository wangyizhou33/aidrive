#ifndef SMOOTH_BARRIERFUNCTION_H
#define SMOOTH_BARRIERFUNCTION_H

#define SMOOTH_BARRIER_FUNCTION_MAX 1e30

template <class T>
inline int smoothBarrierFunction(T &h, T x, T k_h = ((T)1.0), T x_onset = ((T)0.0), T x_max = ((T)1.0),
                                 T h_max = ((T)SMOOTH_BARRIER_FUNCTION_MAX))
{
    T xo_mx, xm_mx;

    xo_mx = x_onset - x;
    if (xo_mx >= 0)
    {
        h = ((T)0.0);
        return (0);
    }

    xm_mx = x_max - x;
    if (xm_mx <= 0)
    {
        h = ((T)h_max);
        return (2);
    }

    T s = x_max - x_onset;
    h = k_h * exp(s * (((T)1.0) / xo_mx + ((T)1.0) / xm_mx));
    return (1);
}

template <class T>
inline int smoothBarrierFunctionGradient(T &h, T &hp, T x, T k_h = ((T)1.0), T x_onset = ((T)0.0), T x_max = ((T)1.0),
                                         T h_max = ((T)SMOOTH_BARRIER_FUNCTION_MAX))
{
    T xo_mx, xm_mx, r1, r2;

    xo_mx = x_onset - x;
    if (xo_mx >= 0)
    {
        h = ((T)0.0);
        hp = ((T)0.0);
        return (0);
    }

    xm_mx = x_max - x;
    if (xm_mx <= 0)
    {
        h = ((T)h_max);
        hp = ((T)0.0);
        return (2);
    }

    T s = x_max - x_onset;
    r1 = ((T)1.0) / xo_mx;
    r2 = ((T)1.0) / xm_mx;
    h = k_h * exp(s * (r1 + r2));
    hp = s * (r1 * r1 + r2 * r2) * h;
    return (1);
}

template <class T>
inline int smoothBarrierFunctionHessian(T &h, T &hp, T &hpp, T x, T k_h = ((T)1.0), T x_onset = ((T)0.0),
                                        T x_max = ((T)1.0), T h_max = ((T)SMOOTH_BARRIER_FUNCTION_MAX))
{
    T xo_mx, xm_mx, r1, r2, r1_2, r2_2, r1_2_p_r2_2;

    xo_mx = x_onset - x;
    if (xo_mx >= 0)
    {
        h = ((T)0.0);
        hp = ((T)0.0);
        hpp = ((T)0.0);
        return (0);
    }

    xm_mx = x_max - x;
    if (xm_mx <= 0)
    {
        h = ((T)h_max);
        hp = ((T)0.0);
        hpp = ((T)0.0);
        return (2);
    }

    r1 = ((T)1.0) / xo_mx;
    r2 = ((T)1.0) / xm_mx;
    r1_2 = r1 * r1;
    r2_2 = r2 * r2;
    r1_2_p_r2_2 = r1_2 + r2_2;

    T s = x_max - x_onset;
    h = k_h * exp(s * (r1 + r2));
    hp = s * r1_2_p_r2_2 * h;
    hpp = s * (s * r1_2_p_r2_2 * r1_2_p_r2_2 + ((T)2.0) * (r1_2 * r1 + r2_2 * r2)) * h;

    return (1);
}

#endif /*SMOOTH_BARRIER_FUNCTION_H*/
