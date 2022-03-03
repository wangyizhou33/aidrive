#ifndef SIMPLE_DISP_H
#define SIMPLE_DISP_H

//#define SIMPLE_DISP_WIDTH_DEFAULT  640
//#define SIMPLE_DISP_HEIGHT_DEFAULT 480
#define SIMPLE_DISP_WIDTH_DEFAULT 1280
#define SIMPLE_DISP_HEIGHT_DEFAULT 960

#include "../MatrixUtil.h"
#include "SavePPM.h"
#include <stdlib.h>

#include <termios.h>
#include <unistd.h>

template <class T> inline unsigned char unsignedCharClip(T x)
{
    long int ival = (long int)x;
    if (ival < 0)
        ival = 0;
    if (ival > 255)
        ival = 255;
    return ((unsigned char)ival);
}

// equivalen to MSFT _getch()
char getch(void)
{
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

class simpleDisp
{
  public:
    simpleDisp()
    {
        mem = NULL;
        w = h = 0;
    }
    void clean()
    {
        if (mem != NULL)
        {
            free(mem);
            mem = NULL;
            w = h = 0;
        }
    }
    ~simpleDisp()
    {
        clean();
    }

    int init(int w_in = SIMPLE_DISP_WIDTH_DEFAULT, int h_in = SIMPLE_DISP_HEIGHT_DEFAULT,
             bool set_default_pattern = true)
    {
        clean();
        mem = (unsigned char *)malloc(sizeof(unsigned char) * 3 * w_in * h_in);
        if (mem == NULL)
        {
            clean();
            return (1);
        }

        w = w_in;
        h = h_in;

        if (set_default_pattern)
        {
            for (int i = 0; i < h; i++)
                for (int j = 0; j < w; j++)
                {
                    mem[i * w + j] =
                        unsignedCharClip<double>(sin(0.03 * (0.01 * (i - 100) * (i - 100) + 1.5 * (j - 100))) * 255.0);
                    mem[w * h + i * w + j] = unsignedCharClip<double>(atan(0.03 * (i + j)) * 255.0);
                    mem[2 * w * h + i * w + j] = unsignedCharClip<double>(0.0 * 255.0);
                }
        }

        return (0);
    }
    void set(const unsigned char *img)
    {
        copyVector<unsigned char>(mem, img, 3 * w * h);
    }
    unsigned char *get()
    {
        return (mem);
    }
    // int update(const char* name="SimpleDispOut")
    int update(const char *name = "SimpleDispOut.ppm")
    // int update(const char* name = "C:\\SimpleDispOut.ppm")
    {
        if (mem == NULL)
            return (1);
        return (savePPM(mem, w, h, name));
    }

    int getw()
    {
        return (w);
    }
    int geth()
    {
        return (h);
    }

    int w, h;
    unsigned char *mem;
};

#endif /*SIMPLE_DISP_H*/
