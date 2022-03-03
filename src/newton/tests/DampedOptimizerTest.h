#ifndef BUMP_DAMPED_OPTIMIZER_TEST_H
#define BUMP_DAMPED_OPTIMIZER_TEST_H

#include "../DampedOptimizer.h"
#include "TestUtil.h"

template <class T, int N, int UB, int HL = 0> class dampedOptimizerTester : public dampedOptimizerBanded<T, N, UB, HL>
{
  public:
    T *x_loc, *g, *A, k, *H;
    void set(T *x_in, T *g_in, T *A_in, T k_in, T *H_in)
    {
        x_loc = x_in;
        g = g_in;
        A = A_in;
        k = k_in;
        H = H_in;
    }

    T xm[N], Axm[N];

    virtual T computeCost(const T *x)
    {
        subVectors(xm, x, x_loc, N);
        T temp = dotProduct<T>(xm, g, N);
        MultiplyVector<T>(Axm, A, xm, N, N);
        temp += ((T)0.5) * dotProduct<T>(Axm, xm, N);
        temp += k;

        return (temp);
    }

    virtual T computeHessian(const T *x)
    {
        subVectors(xm, x, x_loc, N);
        T temp = dotProduct<T>(xm, g, N);
        MultiplyVector<T>(Axm, A, xm, N, N);
        temp += ((T)0.5) * dotProduct<T>(Axm, xm, N);
        temp += k;

        addVectors<T>(this->getGradient(), Axm, g, N);
        copyVector<T>(this->getHessian(), H, N * UB);

        return (temp);
    }
};

template <class T, int N, int UB, int HL = 0> inline T DampedOptimizerTestInstance(long int &seed)
{
    T A[N * N], x[N], xs_mem[N + HL], *xs, g[N], k, k2;

    k = randomScalar<T>(seed);
    randomVector<T>(x, N, seed);
    randomVector<T>(xs_mem, N + HL, seed);
    xs = xs_mem + HL;
    randomVector<T>(g, N, seed);

    T preH[N * UB], H[N * UB], Q[N * N];
    randomVector<T>(preH, N * (UB / 2), seed);
    SymmetricBandedToRegular<T>(Q, preH, N, (UB / 2));
    MultiplyMatrixTranspose<T>(A, Q, Q, N, N, N);

    SymmetricRegularToSymmetricBanded<T>(H, A, N, UB);
    // SymmetricBandedToRegular<T>(A, H, N, UB);

    dampedOptimizerTester<T, N, UB, HL> opt;
    opt.set(x, g, A, k, H);

    opt.setVerboseLevel(0);
    opt.setMaxNrSteps(30);

    // copyVector<T>(xs,x, N);
    opt.runOptimization(xs);
    k2 = opt.computeCost(xs);
    T k3 = opt.computeCost(x);

    // T e = normDiffVector<T>(x, xs, N);
    T e = normOfVector<T>(opt.getGradient(), N);
    e = max_of<T>(e, normDiffVector<T>(&k, &k3, 1));
    if (k2 >= k)
        e = ((T)1e30);

    // e = scalarMax<T>(e, normDiffVector<T>(&k, &k2, 1));
    /*e = scalarMax<T>(e, normDiffVector<T>(&b1, &b3, 1));
    e = scalarMax<T>(e, normDiffVector<T>(dbdx,dbdx2,2));
    e = scalarMax<T>(e, normDiffVector<T>(dbdx ,J, 2)/scalarMax<T>(1e-9,normOfVector(J,2)));
    e = scalarMax<T>(e, normDiffVector<T>(H1,H2, 4)/scalarMax<T>(1e-9,normOfVector(H2,4)));
    */

    /*
    printMatrix<T>(A, N, N);
    printVector<T>(x, N);
    printVector<T>(Ax, N);
    printVector<T>(y, N);
    */

    return (e);
}

template <class T> inline bool DampedOptimizerTest()
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
    for (N = 1; N <= 1; N++)
        for (UB = 1; UB <= 10; UB++)
        {
            for (int i = 0; i < 100; i++)
            {
                long int pre_seed = seed;
                T e = DampedOptimizerTestInstance<T, 10, 4, 1>(seed);
                if (e > tol)
                {
                    cout << "e: " << e;
                    e = DampedOptimizerTestInstance<T, 10, 4, 1>(pre_seed);
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

inline bool TestDampedOptimizer()
{
    bool back;

    back = DampedOptimizerTest<double>();

    return (back);
}

#endif /*BUMP_DAMPED_OPTIMIZER_TEST_H*/
