#ifndef MATRIX_UTIL_TEST_H
#define MATRIX_UTIL_TEST_H

#include "../MatrixUtil.h"
#include <iostream>

bool TestScalarSwap()
{
    float af{0.0f};
    float bf{1.0f};
    int ai{0};
    int bi{1};
    scalarSwap(af, bf);
    scalarSwap(ai, bi);

    if (af == 1.0f && bf == 0.0f && ai == 1 && bi == 0)
        return true;
    else
        return false;
}

bool TestMinOf()
{
    float af{0.0f};
    float bf{1.0f};
    int ai{0};
    int bi{1};

    if (0.0f == min_of(af, bf) && 0 == min_of(ai, bi))
        return true;
    else
        return false;
}

bool TestMaxOf()
{
    float af{0.0f};
    float bf{1.0f};
    int ai{0};
    int bi{1};

    if (1.0f == max_of(af, bf) && 1 == max_of(ai, bi))
        return true;
    else
        return false;
}

bool TestScaleVector()
{
    int xi[2] = {1, 1};
    float xf[2] = {1.0f, 1.0f};
    float scale = 2.0f;

    scaleVector(xi, 2, (int)scale);
    scaleVector(xf, 2, scale);

    if (xi[0] == 2 && xi[1] == 2 && xf[0] == 2.0f && xf[1] == 2.0f)
        return true;
    else
        return false;
}

bool TestSymmetricUpperTriangularToRegular()
{
    int Hupper[3] = {1, 2, 3}; // upperband of 2x2
    int H[4] = {};
    int n = 2; // 2x2 matrix

    symmetricUpperTriangularToRegular(H, Hupper, n);

    if (H[0] == 1 && H[1] == 2 && H[2] == 2 && H[3] == 3)
        return true;
    else
        return false;
}

bool TestPrint()
{
    float vec[3] = {1.0, 2.0, 3.0};
    printVector(vec, 3);

    float mat[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    printMatrix(mat, 3, 2);

    return true;
}

bool TestMatrixUtil()
{

    bool ret{true};
    ret &= TestScalarSwap();
    ret &= TestMinOf();
    ret &= TestMaxOf();
    ret &= TestScaleVector();
    ret &= TestSymmetricUpperTriangularToRegular();
    // ret &= TestPrint();

    return ret;
}

#endif /*MATRIX_UTIL_TEST_H*/
