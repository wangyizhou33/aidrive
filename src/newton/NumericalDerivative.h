#ifndef NUMERICAL_DERIVATIVE_H
#define NUMERICAL_DERIVATIVE_H

#define DEFAULT_NUMERICAL_DERIVATIVE_EPS (1e-6)

#include "MatrixUtil.h"
#include <iostream>

// Help class wrapper to do numerical differentiation
template <class T> class differentiableFunction
{
  public:
    virtual void fun(T *f, const T *x) = 0;
};

// Compute the numerical derivative (Jacobian J) of a function fun with N-dimensional output and M-dimensional input at
// the input x. A vector y of size N is needed for scratch space
template <class T>
inline void numericalDerivative(T *J, differentiableFunction<T> *fw, T *f, T *x, int N, int M,
                                T eps = DEFAULT_NUMERICAL_DERIVATIVE_EPS)
{
    int i, k;

    T rec_eps = ((T)1.0) / eps;
    fw->fun(f, x); // run the function at the base location
    for (i = 0; i < M; i++)
        for (k = 0; k < N; k++)
            J[k * M + i] = f[k]; // Copy the base output values into rows of J
    for (i = 0; i < M; i++)
    {
        T temp = x[i]; // preserve coordinate
        x[i] += eps;   // perturb coordinate
        fw->fun(f, x); // run the function at the perturbed location
        for (k = 0; k < N; k++)
            J[k * M + i] = (f[k] - J[k * M + i]) * rec_eps; // Subtract and scale from output J matrix
        x[i] = temp;                                        // put coordinate back
    }
}

#endif /*NUMERICAL_DERIVATIVE_H*/
