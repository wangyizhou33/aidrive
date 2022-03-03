#ifndef FRENET_TEST_H
#define FRENET_TEST_H

#include "../Frenet.h"
#include "../NumericalDerivative.h"
#include "TestUtil.h"

template <class T, class FTT> class FrenetBasicDifferentiable : public differentiableFunction<T>
{
  public:
    FTT *ftt;
    void set(FTT *ftt_in)
    {
        ftt = ftt_in;
    }

    virtual void fun(T *f, const T *x)
    {
        ftt->transformPoint(f, x);
    }
};

template <class T, class FTT> class FrenetBasicJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    FTT *ftt;
    void set(FTT *ftt_in)
    {
        ftt = ftt_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T x_transformed[2], H_transformed[6];

        // f is 4-dimensional
        ftt->transformPointHessian(H_transformed, f, x_transformed, x);
    }
};

template <class T, class FTT = frenetTransformationLateralFunction<T, scalarFunctionPolynomial<T, 3>>>
inline T FrenetBasicTestInstance(int N, int UB, long int &seed)
{
    FTT ftt;
    T p[4];
    T x[2], v[2];

    randomVector<T>(p, 4, seed);
    ftt.get()->set(p);

    x[0] = randomScalar<T>(seed);
    x[1] = randomScalar<T>(seed);
    v[0] = randomScalar<T>(seed);
    v[1] = randomScalar<T>(seed);

    T x_transformed[2], x_transformed2[2], x_transformed3[2];
    T v_transformed[2], v_transformed2[2], J_transformed[4], H_transformed[6];
    ftt.transformPoint(x_transformed, x);
    ftt.transformPointHessian(H_transformed, J_transformed, x_transformed2, x);
    ftt.transformPointAndVelocity(x_transformed3, v_transformed, x, v);
    MultiplyVector<T>(v_transformed2, J_transformed, v, 2, 2);

    FrenetBasicDifferentiable<T, FTT> fun;
    fun.set(&ftt);
    T xv[2], f[2], J[4];
    xv[0] = x[0];
    xv[1] = x[1];
    numericalDerivative<T>(J, &fun, f, xv, 2, 2, (1e-8));

    FrenetBasicJacobianDifferentiable<T, FTT> funJ;
    funJ.set(&ftt);
    T fJ[4], H2[8];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 4, 2, (1e-8));

    T e = normDiffVector<T>(x_transformed, x_transformed2, 2);
    e = scalarMax<T>(e, normDiffVector<T>(x_transformed, x_transformed3, 2));
    e = scalarMax<T>(e, normDiffVector<T>(J, J_transformed, 4));
    e = scalarMax<T>(e, normDiffVector<T>(v_transformed, v_transformed2, 2));
    e = scalarMax<T>(e, normDiffVector<T>(&H2[0], &H_transformed[0], 1));
    e = scalarMax<T>(e, normDiffVector<T>(&H2[4], &H_transformed[1], 1));
    e = scalarMax<T>(e, normDiffVector<T>(&H2[1], &H_transformed[2], 1));
    e = scalarMax<T>(e, normDiffVector<T>(&H2[5], &H_transformed[3], 1));
    e = scalarMax<T>(e, normDiffVector<T>(&H2[2], &H_transformed[4], 1));
    e = scalarMax<T>(e, normDiffVector<T>(&H2[6], &H_transformed[5], 1));

    // e = scalarMax<T>(e, normDiffVector<T>(dbdx ,J, 2)/scalarMax<T>(1e-9,normOfVector(J,2)));
    // e = scalarMax<T>(e, normDiffVector<T>(H1,H2, 4)/scalarMax<T>(1e-9,normOfVector(H2,4)));

    return (e);
}

template <class T> inline bool FrenetBasicTest()
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
                T e = FrenetBasicTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    e = FrenetBasicTestInstance<T>(N, UB, pre_seed);
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

template <class T, int N, int UB, class FTT = frenetTransformationLateralFunction<T, scalarFunctionPolynomial<T, 3>>>
inline T FrenetConsistencyTestInstance(long int &seed)
{
    FTT ftt;
    T p[4];
    T x[2 * N];
    T g[2 * N];
    T g2[2 * N];
    T g3[2 * N];
    T g4[2 * N];
    T H[2 * N * UB];
    T H2[2 * N * UB];
    T H3[2 * N * UB];
    T H4[2 * N * UB];

    randomVector<T>(p, 4, seed);
    // zeroVector<T>(p, 4);
    // p[0] = ((T)0.0);
    // p[1] = ((T)1.0);
    ftt.get()->set(p);

    randomVector<T>(x, 2 * N, seed);
    // zeroVector<T>(x,2*N);
    // x[0] = ((T)1.0);

    randomVector<T>(g, 2 * N, seed);
    // zeroVector<T>(g,2*N);
    // g[3] = ((T)1.0);
    // g[0] = ((T)1.0);

    copyVector<T>(g2, g, 2 * N);
    copyVector<T>(g3, g, 2 * N);

    randomVector<T>(H, 2 * N * UB, seed);
    // zeroVector<T>(H,2*N*UB);
    // H[20] = ((T)1.0);
    // H[4] = ((T)1.0);

    copyVector<T>(H2, H, 2 * N * UB);
    copyVector<T>(H3, H, 2 * N * UB);
    copyVector<T>(H4, H, 2 * N * UB);

    frenetTransformHessian<T, FTT>(&ftt, H, g, x, N, UB);
    frenetTransformHessianGeneral<T, FTT>(&ftt, H2, g2, x, N, UB);
    frenetTransformHessianBruteForceAll<T, N, FTT>(&ftt, H4, g4, H3, g3, x, UB);

    T e = normDiffVector<T>(g, g2, 2 * N);
    e = scalarMax<T>(e, normDiffVector<T>(g, g4, 2 * N));
    e = scalarMax<T>(e, normDiffVector<T>(H, H2, 2 * N * UB));
    e = scalarMax<T>(e, normDiffVector<T>(H, H4, 2 * N * UB));
    e = scalarMax<T>(e, normDiffVector<T>(H2, H4, 2 * N * UB));

    return (e);
}

template <class T> inline bool FrenetConsistencyTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-12;       // 1e-6;
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
                // T e = FrenetTestInstance<T,2,2>(seed);
                T e = FrenetConsistencyTestInstance<T, 10, 4>(seed);
                if (e > tol)
                {
                    e = FrenetConsistencyTestInstance<T, 10, 4>(pre_seed);
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

template <class T, int N, class FTT> class FrenetDifferentiable : public differentiableFunction<T>
{
  public:
    int UB;
    const T *H;
    const T *g;
    FTT *ftt;
    T x_transformed[2 * N];
    T c;

    void set(FTT *ftt_in, int UB_in, const T *H_in, const T *g_in, T c_in)
    {
        ftt = ftt_in;
        UB = UB_in;
        H = H_in;
        g = g_in;
        c = c_in;
    }

    virtual void fun(T *f, const T *x)
    {
        for (int i = 0; i < N; i++)
            ftt->transformPoint(&x_transformed[2 * i], &x[2 * i]);
        // Compute a cost with Hessian H and gradient Hx+g
        f[0] = ((T)0.5) * symmetricBandedQuadraticForm(H, x_transformed, 2 * N, UB) +
               dotProduct<T>(x_transformed, g, 2 * N) + c;
    }
};

template <class T, int N, int UB, class FTT> class FrenetJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    const T *H;
    const T *g;
    FTT *ftt;
    T x_transformed[2 * N];
    T Hxpg[2 * N];
    T c;

    void set(FTT *ftt_in, const T *H_in, const T *g_in, T c_in)
    {
        ftt = ftt_in;
        H = H_in;
        g = g_in;
        c = c_in;
    }

    virtual void fun(T *f, const T *x)
    {
        int i;
        for (i = 0; i < N; i++)
            ftt->transformPoint(&x_transformed[2 * i], &x[2 * i]);
        // Compute Hx+g
        MultiplySymmetricBanded<T>(Hxpg, H, x_transformed, 2 * N, UB);
        addVectors<T>(Hxpg, Hxpg, g, 2 * N);

        T H2[2 * N * UB];

        // Transform it back to original frame
        zeroVector<T>(H2, 2 * N * UB);
        copyVector<T>(f, Hxpg, 2 * N);
        frenetTransformHessianGeneral<T, FTT>(ftt, H2, f, x, N, UB);
    }
};

template <class T, int N, int UB, class FTT = frenetTransformationLateralFunction<T, scalarFunctionPolynomial<T, 3>>>
inline T FrenetTestInstance(long int &seed)
{
    FTT ftt;
    T p[4];
    T x[2 * N];
    T x_transformed[2 * N];
    T g_copy[2 * N];
    T g[2 * N];
    T g2[2 * N];
    T g3[2 * N];
    T g4[2 * N];
    T Hxpg[2 * N];
    T H_copy[2 * N * UB];
    T H[2 * N * UB];
    T H2[2 * N * UB];
    T H3[2 * N * UB];
    T H4[2 * N * UB];
    T c;

    c = randomScalar<T>(seed);

    randomVector<T>(p, 4, seed);
    // p[0] *= ((T)0.5);
    // p[1] *= ((T)0.5);
    // p[2] *= ((T)0.5);
    p[3] *= ((T)0.5);
    // zeroVector<T>(p, 4);
    // p[0] = ((T)0.0);
    // p[1] = ((T)2.0);
    // p[2] = ((T)1.0);
    ftt.get()->set(p);

    randomVector<T>(x, 2 * N, seed);
    // zeroVector<T>(x,2*N);
    // x[0] = ((T)1.0);

    randomVector<T>(g, 2 * N, seed);
    // zeroVector<T>(g,2*N);
    // g[1] = ((T)1.0);

    randomVector<T>(H, 2 * N * UB, seed);
    // zeroVector<T>(H,2*N*UB);
    // H[0] = ((T)1.0);
    // H[2] = ((T)1.0);
    // H[3] = ((T)1.0);
    // H[0] = ((T)1.0);
    // H[1] = ((T)1.0);
    // H[2] = ((T)1.0);
    // H[3] = ((T)1.0);
    // Zero out the elements that are within bandwidth, but not part of full 2x2 block,
    // because those should never happen and are not handled
    for (int i = 1; i < 2 * N; i += 2)
        H[UB * i + UB - 1] = ((T)0.0);

    copyVector<T>(H_copy, H, 2 * N * UB);
    copyVector<T>(H2, H, 2 * N * UB);
    copyVector<T>(H3, H, 2 * N * UB);
    copyVector<T>(H4, H, 2 * N * UB);

    // Transform coordinates
    int i;
    for (i = 0; i < N; i++)
        ftt.transformPoint(&x_transformed[2 * i], &x[2 * i]);
    // Compute a cost with Hessian H and gradient Hx+g in the Frenet frame
    T cost = ((T)0.5) * symmetricBandedQuadraticForm(H, x_transformed, 2 * N, UB) +
             dotProduct<T>(x_transformed, g, 2 * N) + c;
    // Compute Hx+g
    MultiplySymmetricBanded<T>(Hxpg, H, x_transformed, 2 * N, UB);
    addVectors<T>(Hxpg, Hxpg, g, 2 * N);
    // for (i = 0; i < 2 * N; i++) g_copy[i] += dotProduct<T>(&H[UB*i],&x_transformed[i],min_of<int>(UB,2*N-i));

    copyVector<T>(g_copy, Hxpg, 2 * N);
    copyVector<T>(g2, Hxpg, 2 * N);
    copyVector<T>(g3, Hxpg, 2 * N);

    frenetTransformHessian<T, FTT>(&ftt, H_copy, g_copy, x, N, UB);
    frenetTransformHessianGeneral<T, FTT>(&ftt, H2, g2, x, N, UB);
    frenetTransformHessianBruteForceAll<T, N, FTT>(&ftt, H4, g4, H3, g3, x, UB);

    T Hsym[2 * N * 2 * N];
    SymmetricBandedToRegular<T>(Hsym, H_copy, 2 * N, UB);

    FrenetDifferentiable<T, N, FTT> fun;
    fun.set(&ftt, UB, H, g, c);
    T xv[2 * N], f[2 * N], J[2 * N];
    copyVector<T>(xv, x, 2 * N);
    numericalDerivative<T>(J, &fun, f, xv, 1, 2 * N, (1e-8));

    FrenetJacobianDifferentiable<T, N, UB, FTT> funJ;
    funJ.set(&ftt, H, g, c);
    T fJ[2 * N], Hnum[2 * N * 2 * N];
    numericalDerivative<T>(Hnum, &funJ, fJ, xv, 2 * N, 2 * N, (1e-8));

    T e = normDiffVector<T>(g_copy, g2, 2 * N);
    e = scalarMax<T>(e, normDiffVector<T>(g_copy, g4, 2 * N));
    e = scalarMax<T>(e, normDiffVector<T>(H_copy, H2, 2 * N * UB));
    e = scalarMax<T>(e, normDiffVector<T>(H_copy, H4, 2 * N * UB));
    e = scalarMax<T>(e, normDiffVector<T>(H2, H4, 2 * N * UB));

    e = scalarMax<T>(e, normDiffVector<T>(g_copy, J, 2 * N));
    e = scalarMax<T>(e, normDiffVector<T>(Hsym, Hnum, 2 * N * 2 * N));

    // e = scalarMax<T>(e, normDiffVector<T>(dbdx ,J, 2)/scalarMax<T>(1e-9,normOfVector(J,2)));
    // e = scalarMax<T>(e, normDiffVector<T>(H1,H2, 4)/scalarMax<T>(1e-9,normOfVector(H2,4)));

    return (e);
}

template <class T> inline bool FrenetTest()
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
                // T e = FrenetTestInstance<T,1,2>(seed);
                // T e = FrenetTestInstance<T,2,2>(seed);
                T e = FrenetTestInstance<T, 10, 4>(seed);
                if (e > tol)
                {
                    e = FrenetTestInstance<T, 10, 4>(pre_seed);
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

inline bool TestFrenet()
{
    bool back;

    back = FrenetBasicTest<double>();
    back &= FrenetConsistencyTest<double>();
    back &= FrenetTest<double>();

    return (back);
}

#endif /*FRENET_TEST_H*/
