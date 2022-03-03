#ifndef PROGRESS_COST_H
#define PROGRESS_COST_H

#define PROGRESS_WEIGHT_DEFAULT 0.05 // inverse of average velocity 20m/s expected

template <class T, int N> class progressCostParameters
{
  public:
    T k_p;
    T probEpisodeEnd[N]; // Probability that episode ends exactly after stage i, not before, not after.
    T stageWeights[N];   // Probability that stage i is relevant. Running sum forward starting at i of probability of
                         // episode ending.

    // Generate stageWeights from probEpisodeEnd
    void generateStageWeights()
    {
        T acc = ((T)0.0);
        for (int i = N - 1; i >= 0; i--)
        {
            acc += probEpisodeEnd[i];
            stageWeights[i] = acc;
        }
    }

    void setDefault(T k_p_input = ((T)PROGRESS_WEIGHT_DEFAULT))
    {
        int i;
        T rec;

        rec = ((T)1.0) / ((T)N);
        for (i = 0; i < N; i++)
            probEpisodeEnd[i] = rec;

        generateStageWeights();

        k_p = k_p_input;
    }

    progressCostParameters()
    {
        setDefault();
    }
};

template <class T> inline T trajectoryProgressCost(const T *x, T k_p, const T *probEpisodeEnd, int nrTimesteps)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        acc += x[i << 1] * probEpisodeEnd[i];
    }

    return (-k_p * acc);
}

template <class T, int N> inline T trajectoryProgressCost(const T *x, const progressCostParameters<T, N> *P)
{
    return (trajectoryProgressCost<T>(x, P->k_p, P->probEpisodeEnd, N));
}

template <class T>
inline T trajectoryProgressHessianContribution(T *H, T *g, const T *x, T k_p, const T *probEpisodeEnd, int nrTimesteps)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < nrTimesteps; i++)
    {
        acc += x[i << 1] * probEpisodeEnd[i]; // cost contribution
        g[i << 1] -= k_p * probEpisodeEnd[i]; // gradient contribution
        // No Hessian contribution
    }

    return (-k_p * acc);
}

template <class T, int N>
inline T trajectoryProgressHessianContribution(T *H, T *g, const T *x, const progressCostParameters<T, N> *P)
{
    return (trajectoryProgressHessianContribution<T>(H, g, x, P->k_p, P->probEpisodeEnd, N));
}

#endif /*PROGRESS_COST_H*/
