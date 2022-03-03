#ifndef SMOOTH_BARRIER_FUNCTION_TEST_H
#define SMOOTH_BARRIER_FUNCTION_TEST_H

#include "../NumericalDerivative.h"
#include "../SmoothBarrierFunction.h"
#include "TestUtil.h"

template <class T> class SmoothBarrierFunctionDifferentiable : public differentiableFunction<T>
{
  public:
    T k_h, x_onset, x_max;
    void set(T k_h_in, T x_onset_in, T x_max_in)
    {
        k_h = k_h_in;
        x_onset = x_onset_in;
        x_max = x_max_in;
    }

    virtual void fun(T *f, const T *x)
    {
        smoothBarrierFunction<T>(f[0], x[0], k_h, x_onset, x_max);
    }
};

template <class T> class SmoothBarrierFunctionJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    T k_h, x_onset, x_max;
    void set(T k_h_in, T x_onset_in, T x_max_in)
    {
        k_h = k_h_in;
        x_onset = x_onset_in;
        x_max = x_max_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T h;
        smoothBarrierFunctionGradient<T>(h, f[0], x[0], k_h, x_onset, x_max);
    }
};

template <class T> inline T SmoothBarrierFunctionTestInstance(int N, int UB, long int &seed)
{
    // T A[100], H[100],x[10],Ax[10],y[10];

    T x = randomScalar<T>(seed);
    T x_onset = randomScalar<T>(seed);
    T x_max = randomScalar<T>(seed);
    if (x_max < x_onset)
        scalarSwap(x_max, x_onset);
    T k_h = randomScalar<T>(seed);

    T h, h2, hp2, h3, hp3, hpp3;
    int back = smoothBarrierFunction(h, x, k_h, x_onset, x_max);
    int back2 = smoothBarrierFunctionGradient(h2, hp2, x, k_h, x_onset, x_max);
    int back3 = smoothBarrierFunctionHessian(h3, hp3, hpp3, x, k_h, x_onset, x_max);

    SmoothBarrierFunctionDifferentiable<T> fun;
    fun.set(k_h, x_onset, x_max);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-8));

    SmoothBarrierFunctionJacobianDifferentiable<T> funJ;
    funJ.set(k_h, x_onset, x_max);
    T fJ[1], H[1];
    numericalDerivative<T>(H, &funJ, fJ, xv, 1, 1, (1e-8));

    T e = normDiffVector<T>(&h, &h2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&h, &h3, 1));
    e = scalarMax<T>(e, normDiffVector<T>(&hp2, &hp3, 1));
    e = scalarMax<T>(e, normDiffVector<T>(&hp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&hpp3, H, 1) / scalarMax<T>(1e-9, normOfVector(H, 1)));

    /*
    randomVector<T>(y,N,seed);
    randomVector<T>(H, N*UB, seed);

    copyVector<T>(x, y, N);

    SymmetricBandedToRegular<T>(A, H, N, UB);

    bandedLDLsolve<T>(x,H,N,UB);

    MultiplyVector<T>(Ax, A, x, N, N);

    T e = normDiffVector(Ax, y, N);
*/

    /*
    printMatrix<T>(A, N, N);
    printVector<T>(x, N);
    printVector<T>(Ax, N);
    printVector<T>(y, N);
    */

    return (e);
}

template <class T> inline bool TestSmoothBarrierFunction()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-6;        // 1e-6;
    T frac_tol = 0.9999; // 0.99;

    int N;
    int UB;

    // N = 10;
    // UB = 4;

    succ = fail = 0;
    for (N = 1; N <= 10; N++)
        for (UB = 1; UB <= 10; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = SmoothBarrierFunctionTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    e = SmoothBarrierFunctionTestInstance<T>(N, UB, pre_seed);
                    fail++;
                }
                else
                    succ++;
            }
        }

    double frac = ((double)succ) / ((double)(succ + fail));
    if (frac > frac_tol)
    {
        return (true);
    }

    return (false);
}

inline bool TestSmoothBarrierFunction()
{
    bool back;

    back = TestSmoothBarrierFunction<double>();

    return (back);
}

#endif /*SMOOTH_BARRIER_FUNCTION_TEST_H*/
