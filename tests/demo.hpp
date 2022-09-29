#ifndef DEMO_HPP
#define DEMO_HPP

#include <cmath>
#include <numeric>

namespace demo
{

template <typename T>
T add(T a, T b)
{
    return a + b;
}

template <typename T>
T subtract(T a, T b)
{
    return a - b;
}

template <typename T>
bool isZero(T in)
{
    return in <= std::numeric_limits<T>::epsilon() &&
           in >= -std::numeric_limits<T>::epsilon();
}

template <typename T>
T divide(T num, T den)
{
    T ret{};

    if (isZero(den))
    {
        throw std::invalid_argument("received 0 as denominator");
    }
    else
    {
        ret = num / den;
    }
    return ret;
}

template <typename T>
T multiply(T a, T b)
{
    return a * b;
}

template int multiply<int>(int a, int b); // explicit instantiation.

} // namespace demo

#endif // DEMO_HPP