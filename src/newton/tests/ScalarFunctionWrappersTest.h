#ifndef SCALAR_FUNCTION_WRAPPERS_TEST_H
#define SCALAR_FUNCTION_WRAPPERS_TEST_H

#include "../NumericalDerivative.h"
#include "../ScalarFunctionWrappers.h"
#include "TestUtil.h"

template <class T, class SFW> class ScalarFunctionWrapperDifferentiable : public differentiableFunction<T>
{
  public:
    SFW *fh;
    void set(SFW *fh_in)
    {
        fh = fh_in;
    }

    virtual void fun(T *f, const T *x)
    {
        f[0] = fh->f(x[0]);
    }
};

template <class T, class SFW> class ScalarFunctionWrapperJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    SFW *fh;
    void set(SFW *fh_in)
    {
        fh = fh_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T b, bpp;
        b = fh->H(bpp, f[0], x[0]);
    }
};

template <class T> inline T IdentityTestInstance(int N, int UB, long int &seed)
{
    T x = randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    scalarFunctionIdentity<T> fh;

    // fh.set(k);

    T b1, b2, bp2, bpp2;
    b1 = fh.f(x);
    b2 = fh.H(bpp2, bp2, x);

    ScalarFunctionWrapperDifferentiable<T, scalarFunctionIdentity<T>> fun;
    fun.set(&fh);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-8));

    ScalarFunctionWrapperJacobianDifferentiable<T, scalarFunctionIdentity<T>> funJ;
    funJ.set(&fh);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool IdentityTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-6;        // 1e-6;
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
                T e = IdentityTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = IdentityTestInstance<T>(N, UB, pre_seed);
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

template <class T> inline T WeightedSquareTestInstance(int N, int UB, long int &seed)
{
    T x = ((T)0.1) * randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    scalarFunctionWeightedSquare<T> fh;

    fh.setWeight(k);

    T b1, b2, bp2, bpp2;
    b1 = fh.f(x);
    b2 = fh.H(bpp2, bp2, x);

    ScalarFunctionWrapperDifferentiable<T, scalarFunctionWeightedSquare<T>> fun;
    fun.set(&fh);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-9));

    ScalarFunctionWrapperJacobianDifferentiable<T, scalarFunctionWeightedSquare<T>> funJ;
    funJ.set(&fh);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool WeightedSquareTest()
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
                T e = WeightedSquareTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = WeightedSquareTestInstance<T>(N, UB, pre_seed);
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

template <class T> inline T WeightedBumpTestInstance(int N, int UB, long int &seed)
{
    T xc = ((T)0.1) * randomScalar<T>(seed);
    T x = ((T)0.1) * randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    T w = randomScalar<T>(seed);
    scalarFunctionWeightedBump<T> fh;

    fh.set(k, w, xc);

    T b1, b2, bp2, bpp2;
    b1 = fh.f(x);
    b2 = fh.H(bpp2, bp2, x);

    ScalarFunctionWrapperDifferentiable<T, scalarFunctionWeightedBump<T>> fun;
    fun.set(&fh);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-8));

    ScalarFunctionWrapperJacobianDifferentiable<T, scalarFunctionWeightedBump<T>> funJ;
    funJ.set(&fh);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool WeightedBumpTest()
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
                T e = WeightedBumpTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = WeightedBumpTestInstance<T>(N, UB, pre_seed);
                    fail++;
                }
                else
                    succ++;
            }
        }

    // cout << "Total tests " << (succ + fail) << endl;

    double frac = ((double)succ) / ((double)(succ + fail));
    if (frac > frac_tol)
    {
        return (true);
    }

    return (false);
}

template <class T> inline T WrappedSmoothBarrierTestInstance(int N, int UB, long int &seed)
{
    T x = ((T)0.1) * randomScalar<T>(seed);
    T x_onset = ((T)0.1) * randomScalar<T>(seed);
    T x_max = ((T)0.1) * randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    scalarFunctionSmoothBarrier<T> fh;

    fh.set(k, x_onset, x_max);

    T b1, b2, bp2, bpp2;
    b1 = fh.f(x);
    b2 = fh.H(bpp2, bp2, x);

    ScalarFunctionWrapperDifferentiable<T, scalarFunctionSmoothBarrier<T>> fun;
    fun.set(&fh);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-9));

    ScalarFunctionWrapperJacobianDifferentiable<T, scalarFunctionSmoothBarrier<T>> funJ;
    funJ.set(&fh);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool WrappedSmoothBarrierTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-6;        // 1e-6;
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
                T e = WrappedSmoothBarrierTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = WrappedSmoothBarrierTestInstance<T>(N, UB, pre_seed);
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

template <class T> inline T WrappedSquaredBarrierTestInstance(int N, int UB, long int &seed)
{
    T x = ((T)0.1) * randomScalar<T>(seed);
    T x_onset = ((T)0.1) * randomScalar<T>(seed);
    T x_max = ((T)0.1) * randomScalar<T>(seed);
    T k = randomScalar<T>(seed);
    scalarFunctionSquaredBarrier<T> fh;

    fh.set(k, x_onset, x_max);

    T b1, b2, bp2, bpp2;
    b1 = fh.f(x);
    b2 = fh.H(bpp2, bp2, x);

    ScalarFunctionWrapperDifferentiable<T, scalarFunctionSquaredBarrier<T>> fun;
    fun.set(&fh);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-9));

    ScalarFunctionWrapperJacobianDifferentiable<T, scalarFunctionSquaredBarrier<T>> funJ;
    funJ.set(&fh);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool WrappedSquaredBarrierTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-6;        // 1e-6;
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
                T e = WrappedSquaredBarrierTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = WrappedSquaredBarrierTestInstance<T>(N, UB, pre_seed);
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

template <class T> inline T WrappedPolynomialTestInstance(int N, int UB, long int &seed)
{
    T x = randomScalar<T>(seed);
    scalarFunctionPolynomial<T, 3> fh;
    T p[4];
    p[0] = randomScalar<T>(seed);
    p[1] = randomScalar<T>(seed);
    p[2] = randomScalar<T>(seed);
    p[3] = randomScalar<T>(seed);

    fh.set(p);

    T b1, b2, bp2, bpp2;
    b1 = fh.f(x);
    b2 = fh.H(bpp2, bp2, x);

    ScalarFunctionWrapperDifferentiable<T, scalarFunctionPolynomial<T, 3>> fun;
    fun.set(&fh);
    T xv[1], f[1], J[1];
    xv[0] = x;
    numericalDerivative<T>(J, &fun, f, xv, 1, 1, (1e-9));

    ScalarFunctionWrapperJacobianDifferentiable<T, scalarFunctionPolynomial<T, 3>> funJ;
    funJ.set(&fh);
    T fJ[1], H2[1];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 1, 1, (1e-9));

    T e = normDiffVector<T>(&b1, &b2, 1);
    e = scalarMax<T>(e, normDiffVector<T>(&bp2, J, 1) / scalarMax<T>(1e-9, normOfVector(J, 1)));
    e = scalarMax<T>(e, normDiffVector<T>(&bpp2, H2, 1) / scalarMax<T>(1e-9, normOfVector(H2, 1)));

    return (e);
}

template <class T> inline bool WrappedPolynomialTest()
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
                T e = WrappedPolynomialTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = WrappedPolynomialTestInstance<T>(N, UB, pre_seed);
                    fail++;
                }
                else
                    succ++;
            }
        }

    // cout << "Total tests " << (succ + fail) << endl;

    double frac = ((double)succ) / ((double)(succ + fail));
    if (frac > frac_tol)
    {
        return (true);
    }

    return (false);
}

inline bool TestScalarFunctionWrappers()
{
    bool back;

    back = IdentityTest<double>();
    back &= WeightedSquareTest<double>();
    back &= WeightedBumpTest<double>();
    back &= WrappedSmoothBarrierTest<double>();
    back &= WrappedSquaredBarrierTest<double>();
    back &= WrappedPolynomialTest<double>();

    return (back);
}

#endif /*SCALAR_FUNCTION_WRAPPERS_TEST_H*/
