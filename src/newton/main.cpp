#include <chrono>
#include <iostream>

#include "TrajectoryOptimizer.h"

#include "viz/SimpleDisp.h"
#include "viz/TrajectoryViz.h"

#define TRAJECTORY_GRID_NR_TRAJECTORIES_DEFAULT 16 // 16
#define TRAJECTORY_GRID_X_DIVIDER_DEFAULT 4        // 4
#define TRAJECTORY_GRID_NR_TIMESTEPS_DEFAULT 30
#define TRAJECTORY_GRID_HISTORY_LENGTH_DEFAULT 3

template <class T> class TrajectoryGrid
{
  public:
    TrajectoryGrid()
    {
        mem = NULL;
        fixed = NULL;
        nrTrajectories = nrTimeSteps = historyLength = 0;
    }
    void clean()
    {
        if (mem != NULL)
        {
            free(mem);
            mem = NULL;
            nrTrajectories = nrTimeSteps = historyLength = 0;
        }
        if (fixed != NULL)
        {
            free(fixed);
            fixed = NULL;
        }
    }
    ~TrajectoryGrid()
    {
        clean();
    }

    int init(int nrTrajectories_in = TRAJECTORY_GRID_NR_TRAJECTORIES_DEFAULT,
             int nrTimeSteps_in = TRAJECTORY_GRID_NR_TIMESTEPS_DEFAULT,
             int historyLength_in = TRAJECTORY_GRID_HISTORY_LENGTH_DEFAULT)
    {
        clean();
        mem = (T *)malloc(sizeof(T) * nrTrajectories_in * (historyLength_in + nrTimeSteps_in) * 2);
        if (mem == NULL)
        {
            clean();
            return (1);
        }
        fixed = (int *)malloc(sizeof(int) * nrTimeSteps_in * 2);
        if (fixed == NULL)
        {
            clean();
            return (2);
        }

        nrTrajectories = nrTrajectories_in;
        nrTimeSteps = nrTimeSteps_in;
        historyLength = historyLength_in;

        zeroVector<T>(mem, nrTrajectories * (historyLength + nrTimeSteps) * 2);
        zeroVector<int>(fixed, nrTimeSteps * 2);

        return (0);
    }

    void makeFixGrid(T min_x, T max_x, T min_y, T max_y, T min_vx, T max_vx, T min_vy, T max_vy, T delta_t,
                     T historyVelocity, int x_divider = TRAJECTORY_GRID_X_DIVIDER_DEFAULT)
    {
        zeroVector<T>(mem, nrTrajectories * (historyLength + nrTimeSteps) * 2);
        for (int i = 0; i < nrTrajectories; i++)
        {
            int ix = (i / x_divider);
            int iy = (i % x_divider);

            T xf = ((T)ix) / ((T)(nrTrajectories / x_divider - 1));
            T yf = ((T)iy) / ((T)(x_divider - 1));
            T x = min_x + (max_x - min_x) * xf;
            T y = min_y + (max_y - min_y) * yf;
            T vx = min_vx + (max_vx - min_vx) * xf;
            T vy = min_vy + (max_vy - min_vy) * yf;

            T *tp = getTrajectory(i, nrTimeSteps - 2);
            tp[0] = x;
            tp[1] = y;
            tp[2] = x + vx * delta_t;
            tp[3] = y + vy * delta_t;

            tp = getTrajectory(i, 0);
            for (int k = -1; k >= -historyLength; k--)
            {
                tp[2 * k] = historyVelocity * delta_t * k;
                tp[2 * k + 1] = ((T)0.0);
            }
        }

        zeroVector<int>(fixed, nrTimeSteps * 2);
        fixed[2 * nrTimeSteps - 1] = fixed[2 * nrTimeSteps - 2] = fixed[2 * nrTimeSteps - 3] =
            fixed[2 * nrTimeSteps - 4] = 1;
    }

    int getNrTrajectories()
    {
        return (nrTrajectories);
    }
    int getNrTimesteps()
    {
        return (nrTimeSteps);
    }
    int getHistoryLength()
    {
        return (historyLength);
    }

    T *getTrajectory(int i, int timestep = 0)
    {
        return (mem + 2 * (historyLength + timestep + i * (historyLength + nrTimeSteps)));
    }
    int *getFixed()
    {
        return (fixed);
    }
    void setTrajectory(int i, const T *x)
    {
        copyVector<T>(getTrajectory(i, -historyLength), x - 2 * historyLength, 2 * (historyLength + nrTimeSteps));
    }

    T *mem;
    int *fixed;
    int nrTrajectories;
    int nrTimeSteps;
    int historyLength;
};

inline void TrajectoryOptimizer_exercise()
{
    TrajectoryGrid<double> tgrid;
    TrajectoryViz<double> tviz;
    trajectoryOptimizer<double, 30> topt;

    // int nrXC[30];
    double XC_mem[4 * 30];
    // double *XC[30];

    // for (int i = 0; i < 30; i++)
    // {
    //     nrXC[i] = 1;
    //     XC[i] = &XC_mem[4 * i];

    //     XC[i][0] = 10.0;
    //     XC[i][1] = 10.0;
    //     XC[i][2] = 0.0;
    //     XC[i][3] = 0.0;
    // }

    // topt.setContenderData(XC, nrXC);

    // cout << "Cost: ";
    // cout << topt.computeCost(topt.m_x) << endl;

    // topt.printTrajectory();

    // tgrid.init();

    tviz.init();
    // tviz.runViz(topt.getEgoTrajectory(),1,topt.getN(),topt.getHL());

    topt.setMaxNrSteps(30);


    // topt.run();

    // topt.printTrajectory();

    // cout << "Final Cost: ";
    // cout << topt.getFinalCost() << endl;

    // tviz.runViz(topt.getEgoTrajectory(), 1, topt.getN(), topt.getHL());

    tgrid.init();
    /*tgrid.makeFixGrid(
        ((double)20.0), ((double)50.0),
        ((double)-20.0), ((double)20.0),
        ((double)0.0), ((double)25.0),
        ((double)-30.0), ((double)30.0),
        topt.delta_t,
        ((double)0.0)
        );*/
    tgrid.makeFixGrid(((double)20.0), ((double)50.0), ((double)-20.0), ((double)20.0), ((double)0.0), ((double)25.0),
                      ((double)-30.0), ((double)30.0), topt.delta_t, ((double)0.0));
    topt.setVerboseLevel(0);
    topt.setFixed(tgrid.getFixed());
    for (int q = 0; q < tgrid.getNrTrajectories(); q++)
    {
        topt.setEgoTrajectory(tgrid.getTrajectory(q));
        topt.run();
        tgrid.setTrajectory(q, topt.getEgoTrajectory());
    }
    tviz.runViz(tgrid.getTrajectory(0), tgrid.getNrTrajectories(), tgrid.getNrTimesteps(), tgrid.getHistoryLength(),
                &XC_mem[0]);
    return;
}

int main()
{
    TrajectoryOptimizer_exercise();
    return 0;
}
