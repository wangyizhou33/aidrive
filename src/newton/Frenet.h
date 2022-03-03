#ifndef FRENET_H
#define FRENET_H

#include "MatrixUtil.h"
#include "ScalarFunctionWrappers.h"

// Class to perform the functions required to do a lateral transformation by subtracting a function p(x) from y
// LFT (Lateral Function Type) needs to have functions
// T f(T x) const
// T g(T& fp, T x) const
// T H(T& fpp, T& fp, T x) const
// that compute the function value (returned), its derivative fp and second derivative fpp
// The functions include transforming points, points and their velocity and to transform gradient and Hessian
// from the transformed coordinate system back to the original one.
template <class T, class LFT = scalarFunctionPolynomial<T, 3>> class frenetTransformationLateralFunction
{
  public:
    LFT p;

    LFT *get()
    {
        return (&p);
    }

    void transformPoint(T x_transformed[2], const T x[2])
    {
        T p_val;

        p_val = p.f(x[0]);

        x_transformed[0] = x[0];
        x_transformed[1] = x[1] - p_val;
    }

    void transformPointAndVelocity(T x_transformed[2], T v_transformed[2], const T x[2], const T v[2])
    {
        T p_val, dp_val;

        p_val = p.g(dp_val, x[0]);

        x_transformed[0] = x[0];
        x_transformed[1] = x[1] - p_val;

        v_transformed[0] = v[0];
        v_transformed[1] = v[1] - v[0] * dp_val;
    }

    // Compute the Jacobian and Hessian of the transformation, to allow using a more general Hessian
    // transformation and for testing. J is the row major 2x2 Jacobian of the mapping from x to x_transformed
    // H is the Hessian of that mapping, which is a symmetric 2x2x2 tensor (Hessian of a 2-dimensional vector)
    // and is stored as the three 2-vectors in the upper half.
    void transformPointHessian(T H_transformed[6], T J_transformed[4], T x_transformed[2], const T x[2])
    {
        T p_val, dp_val, dpp_val;

        p_val = p.H(dpp_val, dp_val, x[0]);

        x_transformed[0] = x[0];
        x_transformed[1] = x[1] - p_val;

        J_transformed[0] = ((T)1.0);
        J_transformed[1] = ((T)0.0);
        J_transformed[2] = (-dp_val);
        J_transformed[3] = ((T)1.0);

        H_transformed[0] = ((T)0.0);
        H_transformed[1] = (-dpp_val);
        H_transformed[2] = ((T)0.0);
        H_transformed[3] = ((T)0.0);
        H_transformed[4] = ((T)0.0);
        H_transformed[5] = ((T)0.0);
    }

    // H is the upper triangular banded representation in 2*UB*N entries.
    // UB is the actual bandwidth.
    // N is the number of time steps.
    // g is the 2*N gradient
    // This function performs the transformations related to coordinate pair i
    // and has to be called for each i in 0<=i<N
    void transformHessian_i(T *H, T *g, const T x[2], int i, int N, int UB)
    {
        int j;
        T p_val, dp_val, dpp_val;

        // Compute p'(x_i), p''(x_i)
        p_val = p.H(dpp_val, dp_val, x[0]);

        // Subtract p'(x_i) times row (i)_y of the Hessian from row (i)_x of the Hessian.
        H[2 * UB * i] -= dp_val * H[2 * UB * i + 1]; // Special case for the below diagonal element
        for (j = 1; j < min_of<int>(UB, 2 * (N - i)); j++)
        {
            H[UB * 2 * i + j] -= dp_val * H[UB * (2 * i + 1) + j - 1];
        }

        // Subtract p'(x_i) times column (i)_y of the Hessian from column (i)_x of the Hessian.
        for (j = 0; j < min_of<int>(UB - 1, 2 * i + 1); j++)
        {
            H[UB * (2 * i - j) + j] -= dp_val * H[UB * (2 * i - j) + j + 1];
        }

        // Subtract(g_i)_y p''(x_i)from diagonal(H_ii)_xx element i of the Hessian.
        H[2 * UB * i] -= g[2 * i + 1] * dpp_val;

        // Subtract(g_i)_y p'(x_i) from (g_i )_x
        g[2 * i] -= g[2 * i + 1] * dp_val;
    }
};

template <class T, class FTT = frenetTransformationLateralFunction<scalarFunctionPolynomial<T, 3>>>
void frenetTransformHessian(FTT *ftt, T *H, T *g, const T *x, int N, int UB)
{
    for (int i = 0; i < N; i++)
    {
        ftt->transformHessian_i(H, g, &x[2 * i], i, N, UB);
    }
}

template <class T, class FTT = frenetTransformationLateralFunction<scalarFunctionPolynomial<T, 3>>>
void frenetTransformHessianGeneral_i(FTT *ftt, T *H, T *g, const T x[2], int i, int N, int UB)
{
    int j;
    int A00, A01, A10, A11;
    T H_transformed[6], J_transformed[4], x_transformed[2];
    T temp, diagonal_hold_over;
    T HA00, HA01, HA10;

    // Compute df/dX, df^2/dX^2
    ftt->transformPointHessian(H_transformed, J_transformed, x_transformed, x);

    // Multiply (df/dX)^T from the left onto block-row i of the Hessian.
    for (j = 0; j < min_of<int>(UB >> 1, N - i); j++)
    {
        // Figure out the indexes for the four elements of the block
        A00 = (2 * UB * i + 2 * j);
        A01 = A00 + 1;
        A10 = A00 + UB - 1;
        A11 = A00 + UB;
        if (j == 0)
            A10 = A01; // Special case for diagonal block

        // Remember two elements that will otherwise get written over
        HA00 = H[A00];
        HA01 = H[A01];
        // Do the multiplication, with a special case for the diagonal block
        H[A00] = J_transformed[0] * HA00 + J_transformed[2] * H[A10];
        H[A01] = J_transformed[0] * HA01 + J_transformed[2] * H[A11];
        temp = J_transformed[1] * HA00 + J_transformed[3] * H[A10];
        if (j == 0)
            diagonal_hold_over = temp;
        else
            H[A10] = temp;
        H[A11] = J_transformed[1] * HA01 + J_transformed[3] * H[A11];
    }

    // Multiply (df/dX) from the right onto block-column i of the Hessian.
    for (j = 0; j < min_of<int>(UB >> 1, i + 1); j++)
    {
        // Figure out the indexes for the four elements of the block
        A00 = (2 * UB * (i - j) + 2 * j);
        A01 = A00 + 1;
        A10 = A00 + UB - 1;
        A11 = A00 + UB;

        // Remember two elements that will otherwise get written over
        HA00 = H[A00];
        HA10 = H[A10];
        if (j == 0)
            HA10 = diagonal_hold_over; // Special case for diagonal block
        // Do the multiplication, with a special case skip for the diagonal block
        H[A00] = HA00 * J_transformed[0] + H[A01] * J_transformed[2];
        H[A01] = HA00 * J_transformed[1] + H[A01] * J_transformed[3];
        if (j != 0)
            H[A10] = HA10 * J_transformed[0] + H[A11] * J_transformed[2];
        H[A11] = HA10 * J_transformed[1] + H[A11] * J_transformed[3];
    }

    // Add g_(F_i) * df^2/dX^2 to block diagonal element i of the Hessian
    H[2 * UB * i] += dotProduct<T>(&g[2 * i], H_transformed, 2);
    H[2 * UB * i + 1] += dotProduct<T>(&g[2 * i], H_transformed + 2, 2);
    H[2 * UB * (i + 1)] += dotProduct<T>(&g[2 * i], H_transformed + 4, 2);

    // Multiply(g_F)_i by df/dX to form(g_X)_i
    T t1, t2;
    t1 = g[2 * i];
    t2 = g[2 * i + 1];
    g[2 * i] = t1 * J_transformed[0] + t2 * J_transformed[2];
    g[2 * i + 1] = t1 * J_transformed[1] + t2 * J_transformed[3];
}

template <class T, class FTT = frenetTransformationLateralFunction<scalarFunctionPolynomial<T, 3>>>
void frenetTransformHessianGeneral(FTT *ftt, T *H, T *g, const T *x, int N, int UB)
{
    for (int i = 0; i < N; i++)
    {
        frenetTransformHessianGeneral_i<T, FTT>(ftt, H, g, &x[2 * i], i, N, UB);
    }
}

// Brute force version for testing against the above
template <class T, int N, class FTT = frenetTransformationLateralFunction<scalarFunctionPolynomial<T, 3>>>
void frenetTransformHessianBruteForceAll(FTT *ftt, T *H, T *g, T *HF, T *gF, const T *x, int UB)
{
    int i;
    T H_transformed[6], J_transformed[4], x_transformed[2];
    T dfdX[2 * N * 2 * N], H_regular[2 * N * 2 * N], H_temp[2 * N * 2 * N];

    zeroVector(dfdX, 2 * N * 2 * N);
    for (i = 0; i < N; i++)
    {
        // Compute df/dX, df^2/dX^2
        ftt->transformPointHessian(H_transformed, J_transformed, x_transformed, &x[2 * i]);
        dfdX[2 * N * 2 * i + 2 * i] = J_transformed[0];
        dfdX[2 * N * 2 * i + 2 * i + 1] = J_transformed[1];
        dfdX[2 * N * (2 * i + 1) + 2 * i] = J_transformed[2];
        dfdX[2 * N * (2 * i + 1) + 2 * i + 1] = J_transformed[3];
    }

    // Copy from symmetric banded to regular representation
    SymmetricBandedToRegular<T>(H_regular, HF, 2 * N, UB);

    // Multiply (df/dX)^T from the left onto block-row i of the Hessian.
    MultiplyMatrixTranspose<T>(H_temp, dfdX, H_regular, 2 * N, 2 * N, 2 * N);

    // Multiply (df/dX) from the right onto block-column i of the Hessian.
    MultiplyMatrix<T>(H_regular, H_temp, dfdX, 2 * N, 2 * N, 2 * N);

    // Add g_(F_i) * df^2/dX^2 to block diagonal element i of the Hessian
    for (i = 0; i < N; i++)
    {
        // Compute df/dX, df^2/dX^2
        ftt->transformPointHessian(H_transformed, J_transformed, x_transformed, &x[2 * i]);

        H_regular[2 * N * 2 * i + 2 * i] += dotProduct<T>(&gF[2 * i], H_transformed, 2);
        H_regular[2 * N * 2 * i + 2 * i + 1] += dotProduct<T>(&gF[2 * i], H_transformed + 2, 2);
        H_regular[2 * N * (2 * i + 1) + 2 * i] += dotProduct<T>(&gF[2 * i], H_transformed + 2, 2);
        H_regular[2 * N * (2 * i + 1) + 2 * i + 1] += dotProduct<T>(&gF[2 * i], H_transformed + 4, 2);
    }

    // Copy back to symmetric banded representation
    SymmetricRegularToSymmetricBanded<T>(H, H_regular, 2 * N, UB);

    // Multiply(g_F)_i by df/dX to form(g_X)_i
    MultiplyMatrixTranspose<T>(g, dfdX, gF, 2 * N, 2 * N, 1);
}

#endif /*FRENET_H*/
