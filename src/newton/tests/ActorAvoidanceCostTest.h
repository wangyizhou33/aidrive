#ifndef ACTOR_AVOIDANCE_TEST_H
#define ACTOR_AVOIDANCE_TEST_H

#include "../NumericalDerivative.h"
#include "../cost/ActorAvoidanceCost.h"
#include "TestUtil.h"

template <class T> class ActorAvoidanceCostCoreDifferentiable : public differentiableFunction<T>
{
  public:
    actorAvoidanceCostParametersCore<T> *P;
    T xc, vc;
    void set(T xc_in, T vc_in, actorAvoidanceCostParametersCore<T> *P_in)
    {
        xc = xc_in;
        vc = vc_in;
        P = P_in;
    }

    virtual void fun(T *f, const T *x)
    {
        actorAvoidanceCostCore<T>(f[0], x[0], x[1], xc, vc, P);
    }
};

template <class T> class ActorAvoidanceCostCoreJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    actorAvoidanceCostParametersCore<T> *P;
    T xc, vc;
    void set(T xc_in, T vc_in, actorAvoidanceCostParametersCore<T> *P_in)
    {
        xc = xc_in;
        vc = vc_in;
        P = P_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T b, Hsym[3];
        actorAvoidanceCostCoreHessian<T>(b, f, Hsym, x[0], x[1], xc, vc, P);
    }
};

template <class T> inline T ActorAvoidanceCostCoreTestInstance(int N, int UB, long int &seed)
{
    actorAvoidanceCostParametersCore<T> P;

    T x = randomScalar<T>(seed);
    T v = randomScalar<T>(seed);
    T xc = randomScalar<T>(seed);
    T vc = randomScalar<T>(seed);

    T b, b2, dbdx[2], Hsym[3], H1[4];
    int back = actorAvoidanceCostCore<T>(b, x, v, xc, vc, &P);
    int back2 = actorAvoidanceCostCoreHessian(b2, dbdx, Hsym, x, v, xc, vc, &P);
    symmetricUpperTriangularToRegular<T>(H1, Hsym, 2);

    ActorAvoidanceCostCoreDifferentiable<T> fun;
    fun.set(xc, vc, &P);
    T xv[2], f[1], J[2];
    xv[0] = x;
    xv[1] = v;
    numericalDerivative<T>(J, &fun, f, xv, 1, 2, (1e-8));

    ActorAvoidanceCostCoreJacobianDifferentiable<T> funJ;
    funJ.set(xc, vc, &P);
    T fJ[2], H2[4];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 2, 2, (1e-8));

    T e = normDiffVector<T>(&b, &b2, 1);
    // e = scalarMax<T>(e, normDiffVector<T>(&b1, &b2, 1));
    // e = scalarMax<T>(e, normDiffVector<T>(dbdx,dbdx2,2));
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

template <class T> inline bool ActorAvoidanceCostCoreTest()
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
                T e = ActorAvoidanceCostCoreTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    e = ActorAvoidanceCostCoreTestInstance<T>(N, UB, pre_seed);
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

template <class T> class ActorAvoidanceCostMarginsDifferentiable : public differentiableFunction<T>
{
  public:
    actorAvoidanceCostParametersCore<T> *P;
    T xc, vc;
    void set(T xc_in, T vc_in, actorAvoidanceCostParametersCore<T> *P_in)
    {
        xc = xc_in;
        vc = vc_in;
        P = P_in;
    }

    virtual void fun(T *f, const T *x)
    {
        actorAvoidanceCostMargins<T>(f[0], x[0], x[1], xc, vc, P);
    }
};

template <class T> class ActorAvoidanceCostMarginsJacobianDifferentiable : public differentiableFunction<T>
{
  public:
    actorAvoidanceCostParametersCore<T> *P;
    T xc, vc;
    void set(T xc_in, T vc_in, actorAvoidanceCostParametersCore<T> *P_in)
    {
        xc = xc_in;
        vc = vc_in;
        P = P_in;
    }

    virtual void fun(T *f, const T *x)
    {
        T b, Hsym[3];
        actorAvoidanceCostMarginsHessian<T>(b, f, Hsym, x[0], x[1], xc, vc, P);
    }
};

template <class T> inline T ActorAvoidanceCostMarginsTestInstance(int N, int UB, long int &seed)
{
    actorAvoidanceCostParametersCore<T> P;
    T x, xmo, xc, vc;

    T rs = randomScalar<T>(seed);
    if (rs < -0.5)
    {
        x = randomScalar<T>(seed);
        xmo = ((T)-30.0) * randomScalar<T>(seed);
        xc = ((T)14.0) * randomScalar<T>(seed);
        vc = ((T) + 30.0) * randomScalar<T>(seed);
    }
    else if (rs < -0.25)
    { // chasing case
        x = randomScalar<T>(seed);
        xmo = x + ((T)(0.1)); //   randomScalar<T>(seed) - ((T)(0.0));
        xc = randomScalar<T>(seed) + ((T)(35.0));
        vc = ((T) + 30.0) + randomScalar<T>(seed);
    }
    else if (randomScalar<T>(seed) < 0.0)
    { // separating case
        x = randomScalar<T>(seed);
        xmo = randomScalar<T>(seed);
        xc = ((T)14.0) * randomScalar<T>(seed);
        vc = ((T) + 30.0) * randomScalar<T>(seed);
    }
    else if (1)
    {
        x = randomScalar<T>(seed);
        xmo = ((T)-3.0) + randomScalar<T>(seed);
        xc = ((T)140.0) + randomScalar<T>(seed);
        vc = ((T)-3.0) + randomScalar<T>(seed);
    }

    T b, b2, dbdx[2], Hsym[3], H1[4];
    int back = actorAvoidanceCostMargins<T>(b, x, xmo, xc, vc, &P);
    int back2 = actorAvoidanceCostMarginsHessian(b2, dbdx, Hsym, x, xmo, xc, vc, &P);
    symmetricUpperTriangularToRegular<T>(H1, Hsym, 2);

    ActorAvoidanceCostMarginsDifferentiable<T> fun;
    fun.set(xc, vc, &P);
    T xv[2], f[1], J[2];
    xv[0] = x;
    xv[1] = xmo;
    numericalDerivative<T>(J, &fun, f, xv, 1, 2, (1e-8));

    ActorAvoidanceCostMarginsJacobianDifferentiable<T> funJ;
    funJ.set(xc, vc, &P);
    T fJ[2], H2[4];
    numericalDerivative<T>(H2, &funJ, fJ, xv, 2, 2, (1e-8));

    T e = normDiffVector<T>(&b, &b2, 1);
    // e = scalarMax<T>(e, normDiffVector<T>(&b1, &b2, 1));
    // e = scalarMax<T>(e, normDiffVector<T>(dbdx,dbdx2,2));
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

template <class T> inline bool ActorAvoidanceCostMarginsTest()
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
                T e = ActorAvoidanceCostMarginsTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    e = ActorAvoidanceCostMarginsTestInstance<T>(N, UB, pre_seed);
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

inline bool TestActorAvoidanceCost()
{
    bool back;

    back = ActorAvoidanceCostCoreTest<double>();
    back &= ActorAvoidanceCostMarginsTest<double>();

    return (back);
}

#endif /*ACTOR_AVOIDANCE_TEST_H*/
