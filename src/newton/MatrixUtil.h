#ifndef MATRIX_UTIL_H
#define MATRIX_UTIL_H

#include <iostream>
#include <math.h>

template <class T> inline void scalarSwap(T &a, T &b)
{
    T temp = a;
    a = b;
    b = temp;
}

// Return minimum of two values
template <class T> inline T min_of(T x, T y)
{
    if (x < y)
        return (x);
    return (y);
}

template <class T> inline T max_of(T x, T y)
{
    if (x > y)
        return (x);
    return (y);
}

template <class T> inline T scalarMax(T x, T y)
{
    return max_of(x, y);
}

template <class T> inline void zeroVector(T *x, int n)
{
    for (int i = 0; i < n; i++)
        x[i] = ((T)0.0);
}

// Copy n-dimensional vector y into x
template <class T> inline void copyVector(T *x, const T *y, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        x[i] = y[i];
    }
}

// Add n-dimensional vectors y and z and put into x. x can be the same as y or z
template <class T> inline void addVectors(T *x, const T *y, const T *z, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        x[i] = y[i] + z[i];
    }
}

// Subtract n-dimensional vectors y and z and put into x. x can be the same as y or z
template <class T> inline void subVectors(T *x, const T *y, const T *z, int n)
{
    int i;

    for (i = 0; i < n; i++)
    {
        x[i] = y[i] - z[i];
    }
}

// Return dot product between two n-dimensional vectors x and y
template <class T> inline T dotProduct(const T *x, const T *y, int n)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < n; i++)
    {
        acc += x[i] * y[i];
    }

    return (acc);
}

// Compute the L2 norm of a vector
template <class T> inline T normOfVector(const T *x, int n)
{
    return (sqrt(dotProduct<T>(x, x, n)));
}

// Scale the n-dimensional vector x by scale
template <class T> inline void scaleVector(T *x, int n, T scale)
{
    int i;

    for (i = 0; i < n; i++)
    {
        x[i] *= scale;
    }
}

// Subtract n-dimensional vector y scaled to scale*y from n dimensional vector x
template <class T> inline void subScaledVector(T *x, const T *y, int n, T scale)
{
    int i;

    for (i = 0; i < n; i++)
    {
        x[i] -= scale * y[i];
    }
}

// Multiply n x m matrix A by m dimensional vector x to make n dimensional vector Ax
template <class T> inline void MultiplyVector(T *Ax, const T *A, const T *x, int n, int m)
{
    for (int i = 0; i < n; i++)
    {
        T acc = ((T)0.0);
        for (int k = 0; k < m; k++)
        {
            acc += A[i * m + k] * x[k];
        }
        Ax[i] = acc;
    }
}

// Multiply n x m matrix A by m x o matrix B to make n x o matrix AB
template <class T> inline void MultiplyMatrix(T *AB, const T *A, const T *B, int n, int m, int o)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < o; j++)
        {
            T acc = ((T)0.0);
            for (int k = 0; k < m; k++)
            {
                acc += A[i * m + k] * B[k * o + j];
            }
            AB[i * o + j] = acc;
        }
    }
}

// Multiply by transpose of m x n matrix A by m x o matrix B to make n x o matrix AB
template <class T> inline void MultiplyMatrixTranspose(T *AtB, const T *A, const T *B, int n, int m, int o)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < o; j++)
        {
            T acc = ((T)0.0);
            for (int k = 0; k < m; k++)
            {
                acc += A[k * n + i] * B[k * o + j];
            }
            AtB[i * o + j] = acc;
        }
    }
}

template <class T> inline void SymmetricBandedToRegular(T *A, const T *H, int N, int UB)
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (i <= j)
            {
                if (j - i >= UB)
                    A[i * N + j] = ((T)0.0);
                else
                    A[i * N + j] = H[UB * i + (j - i)];
            }
            else
            {
                if (i - j >= UB)
                    A[i * N + j] = ((T)0.0);
                else
                    A[i * N + j] = H[UB * j + (i - j)];
            }
        }
    }
}

template <class T> inline void SymmetricRegularToSymmetricBanded(T *H, const T *A, int N, int UB)
{
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (i <= j)
            {
                if (j - i < UB)
                    H[UB * i + (j - i)] = A[i * N + j];
            }
        }
    }
}

template <class T> inline T symmetricBandedQuadraticForm(const T *H, const T *x, int N, int UB)
{
    int i, j;
    T acc2 = ((T)0.0);

    for (i = 0; i < N; i++)
    {
        T acc = ((T)0.0);
        for (j = 0; j < UB; j++)
        {
            if (i + j < N)
            {
                acc += H[i * UB + j] * x[i + j];
            }
        }
        for (j = 1; j < UB; j++)
        {
            if (i - j >= 0)
            {
                acc += H[(i - j) * UB + j] * x[i - j];
            }
        }
        acc2 += acc * x[i];
    }
    return (acc2);
}

template <class T> inline void MultiplySymmetricBanded(T *Hx, const T *H, const T *x, int N, int UB)
{
    int i, j;

    for (i = 0; i < N; i++)
    {
        T acc = ((T)0.0);
        for (j = 0; j < UB; j++)
        {
            if (i + j < N)
            {
                acc += H[i * UB + j] * x[i + j];
            }
        }
        for (j = 1; j < UB; j++)
        {
            if (i - j >= 0)
            {
                acc += H[(i - j) * UB + j] * x[i - j];
            }
        }
        Hx[i] = acc;
    }
}

// Transpose an n x m matrix A into At
template <class T> inline void transposeMatrix(T *At, const T *A, int n, int m)
{
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
        {
            At[j * n + i] = A[i * m + j];
        }
}

// Convert an upper triangular representation of a symmetric n x n matrix to regular full representation
// e.g. H      = [1, 2;  2, 3];
//      Hupper = [1, 2, 3];
template <class T> inline void symmetricUpperTriangularToRegular(T *H, const T *Hupper, int n)
{
    int k = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
        {
            if (i <= j)
            {
                H[i * n + j] = Hupper[k++];
            }
            else
            {
                H[i * n + j] = H[j * n + i];
            }
        }
}

template <class T> inline void printVector(const T *x, int n)
{
    std::cout << std::endl;
    std::cout << "(";
    for (int i = 0; i < n; i++)
    {
        std::cout << x[i];
        if (i < n - 1)
            std::cout << ",";
    }
    std::cout << ")" << std::endl;
}

template <class T> inline void printMatrix(const T *A, int n, int m)
{
    std::cout << std::endl;
    std::cout << "[";
    std::cout << std::endl;
    for (int i = 0; i < n; i++)
    {
        // std::cout << "[";
        for (int j = 0; j < m; j++)
        {
            std::cout << A[i * m + j];
            if (j < m - 1)
                std::cout << ",";
        }
        if (i < n - 1)
            std::cout << "|" << std::endl;
    }
    std::cout << "]";
    std::cout << std::endl;
}

#endif /*MATRIX_UTIL_H*/
