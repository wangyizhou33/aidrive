#ifndef BUMP_FUNCTION_TEST_H
#define BUMP_FUNCTION_TEST_H

#include "../BumpFunction.h"
#include "../NumericalDerivative.h"
#include "TestUtil.h"

template <class T> class BumpFunction1DDifferentiable : public differentiableFunction<T>
{
  public:
    T xc, k, w;
    void set(T xc_in, T k_in, T w_in)
    {
        xc = xc_in;
        k = k_in;
        w = w_in;
    }

    virtual void fun(T *f, const T *x)
    {
        bumpFunction1D<T>(f[0], x[0], xc, k, w);
    }
};

template <class T> class BumpFunction1DJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    T xc, k, w;
    void set(T xc_in, T k_in, T w_in)
    {
        xc = xc_in;
        k = k_in;
        w = w_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T b, bpp;
        bumpFunctionHessian1D<T>(bpp, f[0], b, x[0], xc, k, w);
    }
};

template <class T> inline T BumpFunction1DTestInstance(int N, int UB, long int &seed)
{
    T xc = ((T)0.1) * randomScalar<T>(seed);
    T x = ((T)0.1) * randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    T w = randomScalar<T>(seed);

    T b1, b2, bp2, bpp2;
    int back = bumpFunction1D<T>(b1, x, xc, k, w);
    int back2 = bumpFunctionHessian1D<T>(bpp2, bp2, b2, x, xc, k, w);

    BumpFunction1DDifferentiable<T> fun;
    fun.set(xc, k, w);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-8));

    BumpFunction1DJacobianDifferentiable<T> funJ;
    funJ.set(xc, k, w);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool BumpFunction1DTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-4;        // 1e-6;
    T frac_tol = 0.9999; // 0.99;

    int N;
    int UB;

    succ = fail = 0;
    for (N = 1; N <= 10; N++)
        for (UB = 1; UB <= 10; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = BumpFunction1DTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    e = BumpFunction1DTestInstance<T>(N, UB, pre_seed);
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

template <class T> class BumpFunctionDifferentiable : public differentiableFunction<T>
{
  public:
    T xc, yc, k, w;
    void set(T xc_in, T yc_in, T k_in, T w_in)
    {
        xc = xc_in;
        yc = yc_in;
        k = k_in;
        w = w_in;
    }

    virtual void fun(T *f, const T *x)
    {
        bumpFunction2D<T>(f[0], x[0], x[1], xc, yc, k, w);
    }
};

template <class T> class BumpFunctionJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    T xc, yc, k, w;
    void set(T xc_in, T yc_in, T k_in, T w_in)
    {
        xc = xc_in;
        yc = yc_in;
        k = k_in;
        w = w_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T b;
        bumpFunction2DGradient<T>(b, f, x[0], x[1], xc, yc, k, w);
    }
};

template <class T> inline T BumpFunctionTestInstance(int N, int UB, long int &seed)
{
    // T A[100], H[100],x[10],Ax[10],y[10];

    T x = randomScalar<T>(seed);
    T y = randomScalar<T>(seed);
    T xc = randomScalar<T>(seed);
    T yc = randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    T w = randomScalar<T>(seed);

    T b, b1, b2, b3, dbdx[2], dbdx2[2], Hsym[3], H1[4];
    int back = bumpFunction1D<T>(b, xc + sqrt((x - xc) * (x - xc) + (y - yc) * (y - yc)), xc, k, w);
    int back2 = bumpFunction2D<T>(b1, x, y, xc, yc, k, w);
    int back3 = bumpFunction2DGradient(b2, dbdx, x, y, xc, yc, k, w);
    int back4 = bumpFunction2DHessian(b3, dbdx2, Hsym, x, y, xc, yc, k, w);
    symmetricUpperTriangularToRegular<T>(H1, Hsym, 2);

    BumpFunctionDifferentiable<T> fun;
    fun.set(xc, yc, k, w);
    T xv[2], f[1], J[2];
    xv[0] = x;
    xv[1] = y;
    numericalDerivative<T>(J, &fun, f, xv, 1, 2, (1e-8));

    BumpFunctionJacobianDifferentiable<T> funJ;
    funJ.set(xc, yc, k, w);
    T fJ[2], H2[4];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 2, 2, (1e-8));

    T e = normDiffVector<T>(&b, &b1, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&b1, &b2, 1));
    e = scalarMax<T>(e, normDiffVector<T>(&b1, &b3, 1));
    e = scalarMax<T>(e, normDiffVector<T>(dbdx, dbdx2, 2));
    e = scalarMax<T>(e, normDiffVector<T>(dbdx, J, 2) / scalarMax<T>(1e-9, normOfVector(J, 2)));
    e = scalarMax<T>(e, normDiffVector<T>(H1, H2, 4) / scalarMax<T>(1e-9, normOfVector(H2, 4)));

    /*
    printMatrix<T>(A, N, N);
    printVector<T>(x, N);
    printVector<T>(Ax, N);
    printVector<T>(y, N);
    */

    return (e);
}

template <class T> inline bool BumpFunctionTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-4;        // 1e-6;
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
                T e = BumpFunctionTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    e = BumpFunctionTestInstance<T>(N, UB, pre_seed);
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

inline bool TestBumpFunction()
{
    bool back;

    back = BumpFunction1DTest<double>();
    back &= BumpFunctionTest<double>();

    return (back);
}

#endif /*BUMP_FUNCTION_TEST_H*/
