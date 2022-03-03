#ifndef BANDED_LDL_H
#define BANDED_LDL_H

#define BANDED_LDL_EPS_DEFAULT 1e30

#include "MatrixUtil.h"

// LDL for symmetric and band-diagonal matrix H.
// Matrix H represents is actually (N x N) but banded so only UB diagonals of the upper half (including main diagonal)
// are non-zero. N is diagonal length. UB is upper bandwidth Matrix is stored row major with only UB entries for each
// row, starting on the diagonal for each row Rows are the same length, including at the end, so H is N*UB memory
// entries
template <class T> inline void bandedLDL(T *H, int N, int UB, T eps = ((T)BANDED_LDL_EPS_DEFAULT))
{
    int i, j, iUB;
    T diag;

    for (i = 0; i < N; i++)
    {
        iUB = i * UB;
        // Get the diagonal element
        diag = H[iUB];

        // Take its reciprocal
        if (diag < eps && (-diag) < eps)
        {
            diag = ((T)1.0 / diag);
        }
        else
        {
            // Hit a very small absolute value diagonal element,
            // instead of taking its reciprocal, suppress to zero
            diag = ((T)0.0);
        }

        // Put reciprocal back on diagonal
        H[iUB] = diag;

        // Subtract outerproduct of row i times reciprocal of diagonal element from rest of matrix
        for (j = 1; j < min_of<int>(UB, N - i); j++)
        {
            subScaledVector<T>(&H[(i + j) * UB], &H[iUB + j], min_of<int>(UB - j, N - i - j), diag * H[iUB + j]);
        }

        // Scale row i by the reciprocal of diagonal element
        scaleVector<T>(&H[iUB + 1], min_of<int>(UB - 1, N - 1 - i), diag);
    }
}

// Back-substitute to solve H*x=y given the banded LDL in H.
// Substitution is done in place, so the n-dimensional vector x is both input and output.
template <class T> inline void bandedLDLbacksub(T *x, const T *H, int N, int UB)
{
    int i, j;
    T acc;

    // Back-substitute through lower triangular
    for (i = 0; i < N; i++)
    {
        acc = ((T)0.0);
        for (j = 1; j < min_of<int>(UB, i + 1); j++)
        {
            acc += H[(i - j) * UB + j] * x[i - j];
        }
        x[i] -= acc;
    }

    // Back-substitute through diagonal.
    // Since reciprocal of diagonal is stored, this is done by just multiplying by diagonal.
    for (i = 0; i < N; i++)
    {
        x[i] *= H[i * UB];
    }

    // Back-substitute through upper triangular
    for (i = N - 1; i >= 0; i--)
    {
        x[i] -= dotProduct<T>(&x[i + 1], &H[i * UB + 1], min_of<int>(UB - 1, N - 1 - i));
    }
}

// Do both the factorization and the back-substitution.
// Should not be used twice on the same matrix since the matrix gets factorized.
// If repeated use desired, use the backsub routine multiple times
template <class T> inline void bandedLDLsolve(T *x, T *H, int N, int UB, T eps = ((T)BANDED_LDL_EPS_DEFAULT))
{
    bandedLDL<T>(H, N, UB, eps);
    bandedLDLbacksub<T>(x, H, N, UB);
}

#endif /*BANDED_LDL_H*/
