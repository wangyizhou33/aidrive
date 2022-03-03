#ifndef BANDED_LDL_TEST_H
#define BANDED_LDL_TEST_H

#include "../BandedLDL.h"
#include "TestUtil.h"

template <class T> inline T BandedLDLSolveTestInstance(int N, int UB, long int &seed)
{
    T A[100], H[100], x[10], Ax[10], y[10];

    randomVector<T>(y, N, seed);
    randomVector<T>(H, N * UB, seed);

    copyVector<T>(x, y, N);

    SymmetricBandedToRegular<T>(A, H, N, UB);

    bandedLDLsolve<T>(x, H, N, UB);

    MultiplyVector<T>(Ax, A, x, N, N);

    T e = normDiffVector(Ax, y, N);

    // printMatrix<T>(A, N, N);
    // printVector<T>(x, N);
    // printVector<T>(Ax, N);
    // printVector<T>(y, N);

    return (e);
}

template <class T> inline bool BandedLDLSolveTest()
{
    long int seed = SEED_DEFAULT;
    int succ, fail;
    T tol = 1e-9;        // 1e-6;
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
                T e = BandedLDLSolveTestInstance<T>(N, UB, seed);
                if (e > tol)
                {
                    // cout << "e: " << e;
                    e = BandedLDLSolveTestInstance<T>(N, UB, pre_seed);
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

inline bool TestBandedLDL()
{
    bool back{true};

    back = BandedLDLSolveTest<double>();

    return (back);
}

#endif /*BANDED_LDL_TEST_H*/
