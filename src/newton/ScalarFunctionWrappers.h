#ifndef SCALAR_FUNCTION_WRAPPERS_H
#define SCALAR_FUNCTION_WRAPPERS_H

#include "BumpFunction.h"
#include "MatrixUtil.h"
#include "SmoothBarrierFunction.h"

template <class T> class scalarFunctionIdentity
{
  public:
    T f(T x) const
    {
        return (x);
    }
    T H(T &fpp, T &fp, T x) const
    {
        fp = ((T)1.0);
        fpp = ((T)0.0);
        return (x);
    }
};

template <class T> class scalarFunctionWeightedSquare
{
  public:
    T weight;
    void setWeight(T weight_in)
    {
        weight = weight_in;
    }
    T f(T x) const
    {
        return (x * x * weight);
    }
    T H(T &fpp, T &fp, T x) const
    {
        fpp = ((T)2.0) * weight;
        fp = fpp * x;
        return (x * x * weight);
    }
};

template <class T> class scalarFunctionWeightedBump
{
  public:
    T weight, width, xc;
    void set(T weight_in = ((T)1.0), T width_in = ((T)1.0), T xc_in = ((T)0.0))
    {
        weight = weight_in;
        width = width_in;
        xc = xc_in;
    }
    void setWeight(T weight_in = ((T)1.0))
    {
        weight = weight_in;
    }
    scalarFunctionWeightedBump()
    {
        set();
    }

    T f(T x) const
    {
        T h;
        bumpFunction1D<T>(h, x, xc, weight, width);
        return (h);
    }
    T H(T &fpp, T &fp, T x) const
    {
        T h;
        bumpFunctionHessian1D<T>(fpp, fp, h, x, xc, weight, width);
        return (h);
    }
};

template <class T> class scalarFunctionSmoothBarrier
{
  public:
    T k_h, x_onset, x_max, h_max;
    void set(T k_h_in, T x_onset_in, T x_max_in, T h_max_in = ((T)SMOOTH_BARRIER_FUNCTION_MAX))
    {
        k_h = k_h_in;
        x_onset = x_onset_in;
        x_max = x_max_in;
        h_max = h_max_in;
    }
    T f(T x) const
    {
        T h;
        smoothBarrierFunction<T>(h, x, k_h, x_onset, x_max, h_max);
        return (h);
    }
    T H(T &fpp, T &fp, T x) const
    {
        T h;
        smoothBarrierFunctionHessian<T>(h, fp, fpp, x, k_h, x_onset, x_max, h_max);
        return (h);
    }
};

template <class T> class scalarFunctionSquaredBarrier
{
  public:
    T k_h, x_onset, x_max, h_max;
    void set(T k_h_in, T x_onset_in, T x_max_in)
    {
        k_h = k_h_in;
        x_onset = x_onset_in;
        x_max = x_max_in;
    }
    T f(T x) const
    {
        T h;

        T xmo = x - x_onset;
        if (xmo <= ((T)0.0))
        {
            return ((T)0.0);
        }
        T arg;
        arg = xmo / (x_max - x_onset);
        h = k_h * arg * arg;

        return (h);
    }
    T H(T &fpp, T &fp, T x) const
    {
        T h;

        T xmo = x - x_onset;
        if (xmo <= ((T)0.0))
        {
            fp = ((T)0.0);
            fpp = ((T)0.0);
            return ((T)0.0);
        }
        T rec = ((T)1.0) / (x_max - x_onset);
        T arg = xmo * rec;
        h = k_h * arg * arg;
        T temp = ((T)2.0) * k_h * rec;
        fp = temp * arg;
        fpp = temp * rec;

        return (h);
    }
};

template <class T> inline T evaluatePolynomial(T x, const T *p, int degree)
{
    if (degree < 0)
        return ((T)0.0);

    T acc = p[degree];
    for (int i = degree - 1; i >= 0; i--)
    {
        acc = p[i] + x * acc;
    }

    return (acc);
}

template <class T> inline int derivativeOfPolynomial(T *dp, const T *p, int in_degree)
{
    for (int i = 1; i <= in_degree; i++)
    {
        dp[i - 1] = ((T)i) * p[i];
    }
    return (in_degree - 1);
}

template <class T, int degree> class scalarFunctionPolynomial
{
  public:
    T p[degree + 1], dp[degree + 1], dpp[degree + 1];

    scalarFunctionPolynomial()
    {
        zeroVector<T>(p, degree + 1);
        zeroVector<T>(dp, degree + 1);
        zeroVector<T>(dpp, degree + 1);
    }

    void set(const T *p_in)
    {
        copyVector<T>(p, p_in, degree + 1);
        derivativeOfPolynomial<T>(dp, p, degree);
        derivativeOfPolynomial<T>(dpp, dp, degree - 1);
    }
    T f(T x) const
    {
        return (evaluatePolynomial(x, p, degree));
    }
    T g(T &fp, T x) const
    {
        fp = evaluatePolynomial(x, dp, degree - 1);
        return (evaluatePolynomial(x, p, degree));
    }
    T H(T &fpp, T &fp, T x) const
    {
        fp = evaluatePolynomial(x, dp, degree - 1);
        fpp = evaluatePolynomial(x, dpp, degree - 2);
        return (evaluatePolynomial(x, p, degree));
    }
};

template <class T> class TwoDimensionalFunctionBump
{
  public:
    T weight, width, xc[2];
    void set(const T *xc_in, T weight_in, T width_in)
    {
        weight = weight_in;
        width = width_in;
        xc[0] = xc_in[0];
        xc[1] = xc_in[1];
    }
    T f(const T x[2]) const
    {
        T b;
        bumpFunction2D<T>(b, x[0], x[1], xc[0], xc[1], weight, width);
        return (b);
    }
    T H(T H[3], T g[2], const T x[2]) const
    {
        T b;
        bumpFunction2DHessian<T>(b, g, H, x[0], x[1], xc[0], xc[1], weight, width);
        return (b);
    }
};

#endif /*SCALAR_FUNCTION_WRAPPERS_H*/
