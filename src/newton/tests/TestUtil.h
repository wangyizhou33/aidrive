#ifndef TRAJECTORY_OPT_TEST_UTIL_H
#define TRAJECTORY_OPT_TEST_UTIL_H

#define SEED_DEFAULT 9431                // 7846;
#define SIMPLE_RANDOM_ADDER_VAL 10001441 // 7877
#define SIMPLE_RANDOM_MODULO_VAL 11927   // 9973

#include "../MatrixUtil.h"
#include <iostream>

template <class T> inline void randomVectorTrivial(T *x, int N, long int &seed)
{
    for (int i = 0; i < N; i++)
    {
        seed++;
        x[i] = ((T)seed);
    }
}

template <class T> inline T randomScalar(long int &seed_val)
{
    seed_val = seed_val + ((long int)SIMPLE_RANDOM_ADDER_VAL);
    seed_val = seed_val % ((long int)SIMPLE_RANDOM_MODULO_VAL);

    T temp = ((T)seed_val) / ((T)SIMPLE_RANDOM_MODULO_VAL);

    return (((T)4.0) * (temp - ((T)0.5)));
}

template <class T> inline void randomVector(T *x, int N, long int &seed)
{
    for (int i = 0; i < N; i++)
    {
        x[i] = randomScalar<T>(seed);
    }
}

// Return the L2 norm of difference between x and y
template <class T> inline T normDiffVector(const T *x, const T *y, int n)
{
    int i;
    T acc;

    acc = ((T)0.0);
    for (i = 0; i < n; i++)
    {
        T t = x[i] - y[i];
        acc += t * t;
    }

    return (sqrt(acc));
}

#endif /*TRAJECTORY_OPT_TEST_UTIL_H*/
