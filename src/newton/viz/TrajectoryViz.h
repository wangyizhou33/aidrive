#ifndef TRAJECTORY_VIZ_H
#define TRAJECTORY_VIZ_H

#ifdef WIN32
#include <conio.h>
#endif

#include "SimpleDisp.h"
#include "SimpleDraw.h"

template <class T> class TrajectoryViz
{
  public:
    TrajectoryViz()
    {
        d = &d_act;
        scale = ((T)14.0);
        xc = ((T)0.0);
        yc = ((T)0.0);
        skip_input = false;
    }

    int init(int w_in = SIMPLE_DISP_WIDTH_DEFAULT, int h_in = SIMPLE_DISP_HEIGHT_DEFAULT,
             bool set_default_pattern = true)
    {
        return (d->init(w_in, h_in, set_default_pattern));
    }

    void runViz(const T *x, int nr_trajectories, int N, int HL = 0, const T *contender = nullptr)
    {
        unsigned char *img = d->get();
        int w = d->getw();
        int h = d->geth();

        for (;;)
        {
            zeroVector<unsigned char>(img, 3 * w * h);
            for (int i = 0; i < nr_trajectories; i++)
            {
                DrawTrajectoryColor<T, unsigned char>(x + 2 * (i * (N + HL) - HL), xc, yc, scale, N + HL, img, w, h, 0,
                                                      255, 0, true, false);
                DrawTrajectoryColor<T, unsigned char>(x + 2 * (i * (N + HL) - HL), xc, yc, scale, N + HL, img, w, h,
                                                      255, 255, 255, false, true);
            }

            if (contender)
            {
                DrawTrajectoryColor<T, unsigned char>(contender, xc, yc, scale, N, img, w, h, 0, 255, 0, false, true,
                                                      20);
            }

            d->update();
            cout << "Use keys to adjust display" << endl;
            cout << "(S,xc,yc):(" << scale << "," << xc << "," << yc << ")" << endl;

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
                xc += ((T)0.1) * scale;
            }
            else if (cm == 's')
            {
                xc -= ((T)0.1) * scale;
            }
            else if (cm == 'd')
            {
                yc += ((T)0.1) * scale;
            }
            else if (cm == 'f')
            {
                yc -= ((T)0.1) * scale;
            }
            else if (cm == 'q')
            {
                scale *= ((T)2.0);
            }
            else if (cm == 'w')
            {
                scale *= ((T)0.5);
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
            else
            {
                cout << "NO OP" << endl;
            }
        }
    }

    T scale, xc, yc;
    simpleDisp *d, d_act;
    bool skip_input;
};

#endif /*TRAJECTORY_VIZ_H*/
