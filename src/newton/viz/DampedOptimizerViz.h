#ifndef DAMPED_OPTIMIZER_VIZ_H
#define DAMPED_OPTIMIZER_VIZ_H

#ifdef WIN32
#include <conio.h>
#endif

#include "SimpleDisp.h"

template <class T, int N, int HL> class dampedOptimizer;

template <class T, int N, int HL = 0> class DampedOptimizerViz
{
  public:
    DampedOptimizerViz()
    {
        d = &d_act;
        m_xpdx = m_xpdx_mem + HL;
        m_dx = m_dx_mem + HL;
        first_coord = 0;
        second_coord = 1;
        first_scale = ((T)2.0);
        second_scale = ((T)2.0);
        val_scale = ((T)6.25);
        val_center = ((T)0.0);
        skip_input = false;
        mode = 2;
    }

    int init(int w_in = SIMPLE_DISP_WIDTH_DEFAULT, int h_in = SIMPLE_DISP_HEIGHT_DEFAULT,
             bool set_default_pattern = true)
    {
        return (d->init(w_in, h_in, set_default_pattern));
    }

    void runViz(const T *x, dampedOptimizer<T, N, HL> *opt)
    {
        unsigned char *img = d->get();
        int w = d->getw();
        int h = d->geth();

        // Copy including history if any
        copyVector(m_xpdx - HL, x - HL, N + HL);
        zeroVector(m_dx - HL, N + HL);

        for (;;)
        {
            int i, j;
            T base_c = opt->computeCost(m_xpdx);
            T c, cg, cH;

            if (mode == 1)
            { // 2D visualization
                for (i = 0; i < h; i++)
                {
                    for (j = 0; j < w; j++)
                    {
                        T temp1 = m_xpdx[first_coord];
                        T temp2 = m_xpdx[second_coord];

                        m_xpdx[first_coord] += first_scale * ((T)(j - (w / 2))) / ((T)(w / 2));
                        m_xpdx[second_coord] += second_scale * ((T)(i - (h / 2))) / ((T)(h / 2));

                        c = opt->computeCost(m_xpdx) - base_c;
                        unsigned char uc =
                            unsignedCharClip<T>(((c - val_center) / (100.0 * val_scale)) * ((T)128.0) + ((T)128.0));
                        unsigned char uc2 =
                            unsignedCharClip<T>(((c - val_center) / (val_scale)) * ((T)128.0) + ((T)128.0));
                        unsigned char uc3 = ((uc2 % 20) > 15) ? 0 : 255;
                        img[i * w + j] = uc;
                        img[w * h + i * w + j] = uc2;
                        img[2 * w * h + i * w + j] = uc3;

                        m_xpdx[first_coord] = temp1;
                        m_xpdx[second_coord] = temp2;
                    }
                }
            }
            if (mode == 2)
            { // 1D visualization
                for (j = 0; j < w; j++)
                {
                    T temp1 = m_xpdx[first_coord];

                    T inc = first_scale * ((T)(j - (w / 2))) / ((T)(w / 2));
                    m_xpdx[first_coord] += inc;
                    m_dx[first_coord] = inc;

                    c = opt->computeCost(m_xpdx) - base_c;
                    cg = opt->computeGradientModel(m_dx);
                    cH = opt->computeHessianModel(m_dx);

                    m_xpdx[first_coord] = temp1;
                    m_dx[first_coord] = ((T)0.0);

                    long int switch_val = (int)(((c - val_center) / (val_scale)) * ((double)h));
                    long int switch_val_g = (int)(((cg - val_center) / (val_scale)) * ((double)h));
                    long int switch_val_H = (int)(((cH - val_center) / (val_scale)) * ((double)h));

                    if (c > 1e20)
                    {
                        for (i = 0; i < h; i++)
                        {
                            img[i * w + j] = 255;
                            img[w * h + i * w + j] = 0;
                            img[2 * w * h + i * w + j] = 0;
                        }
                    }
                    else if (c > 1e8)
                    {
                        for (i = 0; i < h; i++)
                        {
                            img[i * w + j] = 255;
                            img[w * h + i * w + j] = 255;
                            img[2 * w * h + i * w + j] = 0;
                        }
                    }
                    else
                    {
                        for (i = 0; i < h; i++)
                        {
                            img[i * w + j] = (h - (h >> 2) - i < switch_val) ? 0 : 255;
                            img[w * h + i * w + j] = (h - (h >> 2) - i < switch_val) ? 0 : 255;
                            img[2 * w * h + i * w + j] = (h - (h >> 2) - i < switch_val) ? 0 : 255;
                        }
                    }

                    // Gradient drawing
                    long int icent = h - (h >> 2) - switch_val_g;
                    if (icent < 0)
                        icent = 0;
                    if (icent >= h)
                        icent = h - 1;
                    int qi;
                    for (qi = icent - 3; qi < icent + 3; qi++)
                    {
                        i = qi;
                        if (i < 0)
                            i = 0;
                        if (i >= h)
                            i = h - 1;
                        img[i * w + j] = 128;
                        img[w * h + i * w + j] = 128;
                        img[2 * w * h + i * w + j] = 0;
                    }

                    // Hessian drawing
                    icent = h - (h >> 2) - switch_val_H;
                    if (icent < 0)
                        icent = 0;
                    if (icent >= h)
                        icent = h - 1;
                    for (qi = icent - 9; qi < icent + 9; qi++)
                    {
                        i = qi;
                        if (i < 0)
                            i = 0;
                        if (i >= h)
                            i = h - 1;
                        img[i * w + j] = 0;
                        img[w * h + i * w + j] = 255;
                        // img[2 * w * h + i * w + j] = 255;
                    }

                    // Center vertical dashed line
                    if (j == (w >> 1))
                    {
                        for (i = 0; i < h; i++)
                        {
                            if ((i % 40) < 20)
                            {
                                img[i * w + j] = (h - (h >> 2) - i < switch_val) ? 0 : 0;
                                img[w * h + i * w + j] = (h - (h >> 2) - i < switch_val) ? 255 : 0;
                                img[2 * w * h + i * w + j] = (h - (h >> 2) - i < switch_val) ? 0 : 255;
                            }
                        }
                    }
                }
            }

            d->update();
            cout << "Use qwertyasdf to adjust display" << endl;
            cout << "(FC,SC|FS,SS)[VS,VC]:(" << first_coord << "," << second_coord << "|" << first_scale << ","
                 << second_scale << ")[" << val_scale << "," << val_center << "]" << endl;

            unsigned char cm;
            if (skip_input)
            {
                cm = 'p';
            }
            else
            {
#ifdef WIN32
                cm = _getch();
#else
                cm = getch();
#endif
            }

            if (cm == 'a')
            {
                first_coord++;
                if (first_coord >= N)
                    first_coord = 0;
            }
            else if (cm == 's')
            {
                first_coord--;
                if (first_coord < 0)
                    first_coord = N - 1;
            }
            else if (cm == 'd')
            {
                second_coord++;
                if (second_coord >= N)
                    second_coord = 0;
            }
            else if (cm == 'f')
            {
                second_coord--;
                if (second_coord < 0)
                    second_coord = N - 1;
            }
            else if (cm == 'q')
            {
                first_scale *= ((T)2.0);
            }
            else if (cm == 'w')
            {
                first_scale *= ((T)0.5);
            }
            else if (cm == 'e')
            {
                second_scale *= ((T)2.0);
            }
            else if (cm == 'r')
            {
                second_scale *= ((T)0.5);
            }
            else if (cm == 't')
            {
                val_scale *= ((T)2.0);
            }
            else if (cm == 'y')
            {
                val_scale *= ((T)0.5);
            }
            else if (cm == 'p')
            {
                break;
            }
            else if (cm == 'u')
            {
                skip_input = true;
                break;
            }
            else if (cm == 'm')
            {
                mode++;
                if (mode > 2)
                    mode = 1;
                break;
            }
            else
            {
                cout << "NO OP" << endl;
            }
        }
    }

    T m_xpdx_mem[N + HL];
    T *m_xpdx;
    T m_dx_mem[N + HL];
    T *m_dx;
    int first_coord, second_coord;
    T first_scale, second_scale, val_scale, val_center;
    simpleDisp *d, d_act;
    bool skip_input;
    int mode;
};

#endif /*DAMPED_OPTIMIZER_VIZ_H*/
