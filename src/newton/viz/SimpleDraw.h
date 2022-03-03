#ifndef SIMPLE_DRAW_H
#define SIMPLE_DRAW_H

#include "../MatrixUtil.h"

template <class IT> inline void simpleDrawLine(IT *img, int i1, int j1, int i2, int j2, int w, int h, IT fillval)
{
    // Sort on i
    if (i1 > i2)
    {
        scalarSwap<int>(i1, i2);
        scalarSwap<int>(j1, j2);
    }

    // Clip top
    if (i1 < 0)
    {
        // We want (j1_new-j2)/(i1_new-i2)=(j1-j2)/(i1-i2)
        // j1_new=j2+(i1_new-i2)*(j1-j2)/(i1-i2)
        if (i2 < 0)
            return;
        j1 = j2 + (0 - i2) * (j1 - j2) / (i1 - i2);
        i1 = 0;
    }
    // Clip bottom
    if (i2 >= h)
    {
        // We want (j2_new-j1)/(i2_new-i1)=(j2-j1)/(i2-i1)
        // j2_new=j1+(i2_new-i1)*(j2-j1)/(i2-i1)
        if (i1 >= h)
            return;
        j2 = j1 + (h - 1 - i1) * (j2 - j1) / (i2 - i1);
        i2 = h - 1;
    }

    // Sort on j
    if (j1 > j2)
    {
        scalarSwap<int>(i1, i2);
        scalarSwap<int>(j1, j2);
    }

    // Clip left
    if (j1 < 0)
    {
        if (j2 < 0)
            return;
        i1 = i2 + (0 - j2) * (i1 - i2) / (j1 - j2);
        j1 = 0;
    }
    // Clip right
    if (j2 >= w)
    {
        if (j1 >= w)
            return;
        i2 = i1 + (w - 1 - j1) * (i2 - i1) / (j2 - j1);
        j2 = w - 1;
    }

    // Make absolutely sure
    if (i1 < 0)
        i1 = 0;
    if (i2 < 0)
        i2 = 0;
    if (j1 < 0)
        j1 = 0;
    if (j2 < 0)
        j2 = 0;
    if (i1 >= h)
        i1 = h - 1;
    if (i2 >= h)
        i2 = h - 1;
    if (j1 >= w)
        j1 = w - 1;
    if (j2 >= w)
        j2 = w - 1;

    // Sort on i
    if (i1 > i2)
    {
        scalarSwap<int>(i1, i2);
        scalarSwap<int>(j1, j2);
    }

    int delta_i = i2 - i1;
    int delta_j = j2 - j1;
    int sum = 0;
    int i = i1;
    int j = j1;

    if (delta_j >= 0)
    {
        if (delta_i >= delta_j)
        {
            for (; i <= i2; i++)
            {
                img[i * w + j] = fillval;
                sum += delta_j;
                if (sum >= delta_i)
                {
                    sum -= delta_i;
                    j++;
                }
            }
        }
        else
        {
            for (; j <= j2; j++)
            {
                img[i * w + j] = fillval;
                sum += delta_i;
                if (sum >= delta_j)
                {
                    sum -= delta_j;
                    i++;
                }
            }
        }
    }
    else
    {
        if (delta_i >= (-delta_j))
        {
            for (; i <= i2; i++)
            {
                img[i * w + j] = fillval;
                sum += (-delta_j);
                if (sum >= delta_i)
                {
                    sum -= delta_i;
                    j--;
                }
            }
        }
        else
        {
            for (; j >= j2; j--)
            {
                img[i * w + j] = fillval;
                sum += delta_i;
                if (sum >= (-delta_j))
                {
                    sum -= (-delta_j);
                    i++;
                }
            }
        }
    }
}

template <class IT>
inline void simpleDrawLineColor(IT *img, int i1, int j1, int i2, int j2, int w, int h, IT fv1, IT fv2, IT fv3)
{
    simpleDrawLine<IT>(img, i1, j1, i2, j2, w, h, fv1);
    simpleDrawLine<IT>(img + w * h, i1, j1, i2, j2, w, h, fv2);
    simpleDrawLine<IT>(img + 2 * w * h, i1, j1, i2, j2, w, h, fv3);
}

template <class IT> inline void drawDiamond(IT *img, int i, int j, int radius, int w, int h, IT fillval)
{
    for (int io = -radius; io <= radius; io++)
    {
        int ir;
        if (io < 0)
            ir = radius + io;
        else
            ir = radius - io;
        int it = io + i;
        for (int jt = j - ir; jt <= j + ir; jt++)
        {
            if (it >= 0 && it < h && jt >= 0 && jt < w)
            {
                img[it * w + jt] = fillval;
            }
        }
    }
}

template <class IT>
inline void drawDiamondColor(IT *img, int i, int j, int radius, int w, int h, IT fv1, IT fv2, IT fv3)
{
    drawDiamond<IT>(img, i, j, radius, w, h, fv1);
    drawDiamond<IT>(img + w * h, i, j, radius, w, h, fv2);
    drawDiamond<IT>(img + 2 * w * h, i, j, radius, w, h, fv3);
}

template <class T, class IT>
inline void DrawTrajectoryColor(const T *x, T xc, T yc, T s, int N, IT *img, int w, int h, IT fv1, IT fv2, IT fv3,
                                bool line = true, bool dots = true, int dot_radius = 5)
{
    for (int i = 0; i < N; i++)
    {
        int i1, j1, i2, j2;
        i1 = h / 2 - ((int)((x[2 * i + 1] - yc) * s + ((T)0.5)));
        j1 = w / 4 + ((int)((x[2 * i] - xc) * s + ((T)0.5)));
        if (i < N - 1)
        {
            i2 = h / 2 - ((int)((x[2 * i + 3] - yc) * s + ((T)0.5)));
            j2 = w / 4 + ((int)((x[2 * i + 2] - xc) * s + ((T)0.5)));
        }

        if (line && i < N - 1)
        {
            if (i >= N - 2)
            {
                simpleDrawLineColor<IT>(img, i1, j1, i2, j2, w, h, 255, 0, 0);
            }
            else
            {
                simpleDrawLineColor<IT>(img, i1, j1, i2, j2, w, h, fv1, fv2, fv3);
            }
        }
        if (dots && i1 >= 0 && i1 < h && j1 >= 0 && j1 < w)
        {
            if (i >= N - 2)
            {
                drawDiamondColor<IT>(img, i1, j1, dot_radius, w, h, 255, 0, 0);
            }
            else
            {
                drawDiamondColor<IT>(img, i1, j1, dot_radius, w, h, fv1, fv2, fv3);
            }
        }
    }
}

#endif /*SIMPLE_DRAW_H*/
