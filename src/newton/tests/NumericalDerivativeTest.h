#ifndef NUMERICAL_DERIVATIVE_TEST_H
#define NUMERICAL_DERIVATIVE_TEST_H

#include "../NumericalDerivative.h"
#include <cmath>

template <class T> bool isEqual(T a, T b)
{
    T eps{1e-5};

    return std::abs(a - b) < eps;
}

template <class T> class Polynomial : public differentiableFunction<T>
{
  public:
    void fun(T *f, const T *x)
    {
        f[0] = a0 + a1 * x[0] + a2 * x[0] * x[0];
    }

    T derivative(T *x)
    {
        return a1 + (T)2.0 * a2 * x[0];
    }

  private:
    // f(x) = a0 + a1*x + a2 x^2
    T a0 = (T)1.0;
    T a1 = (T)1.0;
    T a2 = (T)1.0;
};

template <class T> class Exp : public differentiableFunction<T>
{
  public:
    void fun(T *f, const T *x)
    {
        f[0] = a * std::exp(b * x[0]);
    }

    T derivative(T *x)
    {
        return b * a * std::exp(b * x[0]);
    }

  private:
    T a{1.0};
    T b{1.0};
};

template <class T> class Sin : public differentiableFunction<T>
{
  public:
    void fun(T *f, const T *x)
    {
        f[0] = a * std::sin(b * x[0]);
    }

    T derivative(T *x)
    {
        return b * a * std::cos(b * x[0]);
    }

  private:
    T a{1.0};
    T b{1.0};
};

bool testPolynomial()
{
    double J[1];
    double f[1];
    double x[1] = {2.0f};
    int N = 1;
    int M = 1;

    Polynomial<double> poly{};

    numericalDerivative(J, &poly, f, x, N, M, 1e-6);

    if (isEqual(J[0], poly.derivative(x)))
        return true;
    else
        return false;
}

bool testExponential()
{
    double J[1];
    double f[1];
    double x[1] = {2.0f};
    int N = 1;
    int M = 1;

    Exp<double> exp{};

    numericalDerivative(J, &exp, f, x, N, M, 1e-6);

    if (isEqual(J[0], exp.derivative(x)))
        return true;
    else
        return false;
}

bool testSinusoidal()
{
    double J[1];
    double f[1];
    double x[1] = {5.0f};
    int N = 1;
    int M = 1;

    Sin<double> sin{};

    numericalDerivative(J, &sin, f, x, N, M, 1e-6);

    if (isEqual(J[0], sin.derivative(x)))
        return true;
    else
        return false;
}

bool TestNumericalDerivative()
{
    bool ret{true};

    ret &= testPolynomial();
    ret &= testExponential();
    ret &= testSinusoidal();

    return ret;
}

#endif /*NUMERICAL_DERIVATIVE_TEST_H*/
