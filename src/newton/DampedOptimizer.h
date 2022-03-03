#ifndef DAMPED_OPTIMIZER_H
#define DAMPED_OPTIMIZER_H

#define DAMPED_OPTIMIZER_MAX_NR_STEPS_DEFAULT 30 // 30
#define DAMPED_OPTIMIZER_LAMBDA_INIT_DEFAULT 0.001
#define DAMPED_OPTIMIZER_LAMBDA_SUCC_MULTIPLIER_DEFAULT 0.1
#define DAMPED_OPTIMIZER_LAMBDA_FAIL_MULTIPLIER_DEFAULT 100.0
#define DAMPED_OPTIMIZER_VERBOSE_DEFAULT 3 // 0 nothing, 1 per optimization, 2 per iteration, 3 inside iteration

#include "BandedLDL.h"
#include "MatrixUtil.h"
#include "viz/DampedOptimizerViz.h"
#include <iostream>
using namespace std;

template <class T> class dampedOptimizerParameters
{
  public:
    void setDefault()
    {
        max_nr_steps = DAMPED_OPTIMIZER_MAX_NR_STEPS_DEFAULT;
        lambda_init = ((T)DAMPED_OPTIMIZER_LAMBDA_INIT_DEFAULT);
        lambda_succ_multiplier = ((T)DAMPED_OPTIMIZER_LAMBDA_SUCC_MULTIPLIER_DEFAULT);
        lambda_fail_multiplier = ((T)DAMPED_OPTIMIZER_LAMBDA_FAIL_MULTIPLIER_DEFAULT);
        eps = ((T)BANDED_LDL_EPS_DEFAULT);
        verbose = DAMPED_OPTIMIZER_VERBOSE_DEFAULT;
    }

    dampedOptimizerParameters()
    {
        setDefault();
    }

    int max_nr_steps;
    T lambda_init;
    T lambda_succ_multiplier;
    T lambda_fail_multiplier;
    T eps;
    int verbose;
};

// N is number of free variables. HL is history length (number of variables before the free variables)
template <class T, int N, int HL = 0> class dampedOptimizer
{
  public:
    dampedOptimizer()
    {
        m_xpdx = m_xpdx_mem + HL;
        viz = NULL;
        viz = &viz_act;
        viz_on = false;
    }

    virtual void setViz(DampedOptimizerViz<T, N, HL> *viz_in)
    {
        viz = viz_in;
    }
    virtual int initViz(int w_in = SIMPLE_DISP_WIDTH_DEFAULT, int h_in = SIMPLE_DISP_HEIGHT_DEFAULT,
                        bool set_default_pattern = true)
    {
        return (viz->init(w_in, h_in, set_default_pattern));
    }
    virtual void setVizOn(bool ison)
    {
        viz_on = ison;
    }

    virtual T computeCost(const T *x) = 0;
    virtual T computeHessian(const T *x) = 0;
    virtual void lambdaAugmentHessian(T lambda) = 0;
    virtual T *solveHessian() = 0;
    virtual T computeGradientModel(const T *dx) = 0;
    virtual T computeHessianModel(const T *dx) = 0;

    void optimizationStep(T *x)
    {
        const T *hinv_g;

        // Copy the history if any
        copyVector(m_xpdx - HL, x - HL, HL);

        // Make attempt
        m_current_cost = this->computeHessian(x);

        if (viz_on && viz != NULL)
        {
            viz->runViz(x, this);
        }

        if (m_optP.verbose >= 2)
        {
            cout << "Cost: " << m_current_cost;
        }

        this->lambdaAugmentHessian(m_lambda);
        hinv_g = this->solveHessian();
        subVectors<T>(m_xpdx, x, hinv_g, N);

        // Compute cost of attempt
        m_attempt_cost = this->computeCost(m_xpdx);

        if (m_optP.verbose >= 2)
        {
            cout << " Attempt Cost:" << m_attempt_cost;
        }

        // Check attempt
        if (m_attempt_cost < m_current_cost)
        {
            // Accept attempt
            copyVector<T>(x, m_xpdx, N);
            m_current_cost = m_attempt_cost;
            m_lambda *= m_optP.lambda_succ_multiplier;

            if (m_optP.verbose >= 2)
            {
                cout << " Accepted";
            }
        }
        else
        {
            // Reject attempt
            m_lambda *= m_optP.lambda_fail_multiplier;

            if (m_optP.verbose >= 2)
            {
                cout << " Rejected";
            }
        }

        if (m_optP.verbose >= 2)
        {
            cout << " Lambda:" << m_lambda << endl;
        }
    }

    void runOptimization(T *x)
    {
        int i;

        m_lambda = m_optP.lambda_init;

        for (i = 0; i < m_optP.max_nr_steps; i++)
        {
            if (m_optP.verbose >= 2)
            {
                cout << "It:" << i << " ";
            }

            this->optimizationStep(x);
        }

        /*if (viz_on && viz != NULL)
        {
            viz->runViz(x, this);
        }*/
    }

    virtual void setMaxNrSteps(int max_nr)
    {
        m_optP.max_nr_steps = max_nr;
    }
    virtual void setVerboseLevel(int verbose_level)
    {
        m_optP.verbose = verbose_level;
    }
    virtual int getVerboseLevel()
    {
        return (m_optP.verbose);
    }

    T m_lambda;
    T m_current_cost, m_attempt_cost;
    dampedOptimizerParameters<T> m_optP;
    T m_xpdx_mem[N + HL];
    T *m_xpdx;

    DampedOptimizerViz<T, N, HL> *viz, viz_act;
    bool viz_on;
};

// Copy a vector from source to dest while removing the indexes flagged by 'fixed'.
// dest can be the same as dest but does not have to be.
// Return the number of entries copied
template <class T> inline int decimateVectorByFixedVector(T *dest, const T *source, const int *fixed, int N)
{
    int j = 0;
    for (int i = 0; i < N; i++)
    {
        if (!fixed[i])
        {
            dest[j++] = source[i];
        }
    }
    return (j);
}

template <class T>
inline void expandVectorByFixedVector(T *dest, const T *source, const int *fixed, int N, int nr_free,
                                      T fillval = ((T)0.0))
{
    int j = nr_free - 1;
    for (int i = N - 1; i >= 0; i--)
    {
        if (fixed[i])
        {
            dest[i] = fillval;
        }
        else
        {
            dest[i] = source[j--];
        }
    }
}

template <class T> inline int decimateSymmetricBandedHessianByFixedVector(T *H, const int *fixed, int N, int UB)
{
    int j = 0;
    for (int i = 0; i < N; i++)
    {
        if (!fixed[i])
        {
            int source_nr = min_of<int>(UB, N - i);
            int k = decimateVectorByFixedVector(&H[UB * j], &H[UB * i], fixed + i, source_nr);
            zeroVector<T>(&H[UB * j + k], source_nr - k);
            j++;
        }
    }
    return (j);
}

template <class T, int N, int UB, int HL = 0> class dampedOptimizerBanded : public dampedOptimizer<T, N, HL>
{
  public:
    dampedOptimizerBanded()
    {
        m_fixed = NULL;
    }

    // To use, inherit and implement
    // virtual T computeCost(const T *x)=0;
    // virtual T computeHessian(const T *x)=0;
    // then call
    // void runOptimization(T *x);

    virtual void lambdaAugmentHessian(T lambda)
    {
        int i;

        for (i = 0; i < N; i++)
        {
            m_H[i * UB] += lambda;
        }
    }

    virtual T *solveHessian()
    {
        int nr_free = N;

        if (m_fixed != NULL)
        {
            nr_free = decimateVectorByFixedVector<T>(m_g, m_g, m_fixed, N);
            decimateSymmetricBandedHessianByFixedVector<T>(m_H, m_fixed, N, UB);
        }

        if (this->getVerboseLevel() >= 3)
        {
            cout << " Gradient Magnitude: " << normOfVector<T>(m_g, nr_free) << " ";
        }

        bandedLDLsolve<T>(m_g, m_H, nr_free, UB, this->m_optP.eps);

        if (m_fixed != NULL)
        {
            expandVectorByFixedVector<T>(m_g, m_g, m_fixed, N, nr_free);
        }

        return (m_g);
    }

    T *getGradient()
    {
        return (m_g);
    }
    T *getHessian()
    {
        return (m_H);
    }
    void setFixed(const int *fixed)
    {
        m_fixed = fixed;
    }

    virtual T computeGradientModel(const T *dx)
    {
        return (dotProduct<T>(dx, m_g, N));
    }
    virtual T computeHessianModel(const T *dx)
    {
        return (((T)0.5) * symmetricBandedQuadraticForm(m_H, dx, N, UB) + dotProduct<T>(dx, m_g, N));
    }

    T m_H[N * UB], m_g[N];
    const int *m_fixed;
};

#endif /*DAMPED_OPTIMIZER_H*/
