#ifndef BUMP_FUNCTION_H
#define BUMP_FUNCTION_H

template <class T> inline int bumpFunction1D(T &b, T x, T xc, T k, T w)
{
    T xmxc, den, rec;

    xmxc = x - xc;
    den = ((T)4.0) / (w * w) * xmxc * xmxc - ((T)1.0);

    if (den < ((T)0.0))
    {
        rec = ((T)1.0) / den;
        b = k * exp(rec);
        return (1);
    }

    b = ((T)0.0);
    return (0);
}

template <class T> inline int bumpFunctionHessian1D(T &bpp, T &bp, T &b, T x, T xc, T k, T w)
{
    T xmxc, den, rec;

    xmxc = x - xc;
    T four_rec_w2 = ((T)4.0) / (w * w);
    den = four_rec_w2 * xmxc * xmxc - ((T)1.0);

    if (den < ((T)0.0))
    {
        rec = ((T)1.0) / den;
        b = k * exp(rec);

        T eight_rec_w2_r = ((T)2.0) * four_rec_w2 * rec;
        T temp = b * rec * eight_rec_w2_r;
        bp = -temp * xmxc;
        bpp = temp * (eight_rec_w2_r * (((T)2.0) + rec) * xmxc * xmxc - ((T)1.0));

        return (1);
    }

    b = ((T)0.0);
    bp = ((T)0.0);
    bpp = ((T)0.0);
    return (0);
}

template <class T> inline int bumpFunction2D(T &b, T x, T y, T xc, T yc, T k, T w)
{
    T xmxc, ymyc, den, rec, m2;

    xmxc = x - xc;
    ymyc = y - yc;
    m2 = xmxc * xmxc + ymyc * ymyc;
    den = ((T)4.0) / (w * w) * m2 - ((T)1.0);

    if (den < ((T)0.0))
    {
        rec = ((T)1.0) / den;
        b = k * exp(rec);
        return (1);
    }

    b = ((T)0.0);
    return (0);
}

template <class T> inline int bumpFunction2DGradient(T &b, T dbdx[2], T x, T y, T xc, T yc, T k, T w)
{
    T xmxc, ymyc, den, rec, m2, rec_w2, coeff;

    xmxc = x - xc;
    ymyc = y - yc;
    m2 = xmxc * xmxc + ymyc * ymyc;
    rec_w2 = ((T)1.0) / (w * w);
    den = ((T)4.0) * rec_w2 * m2 - ((T)1.0);

    if (den < ((T)0.0))
    {
        rec = ((T)1.0) / den;
        b = k * exp(rec);
        coeff = ((T)(-8.0)) * rec_w2 * rec * rec * b;
        dbdx[0] = coeff * xmxc;
        dbdx[1] = coeff * ymyc;
        return (1);
    }

    b = ((T)0.0);
    dbdx[0] = ((T)0.0);
    dbdx[1] = ((T)0.0);
    return (0);
}

template <class T> inline int bumpFunction2DHessian(T &b, T dbdx[2], T H[3], T x, T y, T xc, T yc, T k, T w)
{
    T xmxc, ymyc, den, rec, m2, rec_w2, m8_rec_w2_rec, coeff, coeff2;

    xmxc = x - xc;
    ymyc = y - yc;
    m2 = xmxc * xmxc + ymyc * ymyc;
    rec_w2 = ((T)1.0) / (w * w);
    den = ((T)4.0) * rec_w2 * m2 - ((T)1.0);

    if (den < ((T)0.0))
    {
        rec = ((T)1.0) / den;
        b = k * exp(rec);
        m8_rec_w2_rec = ((T)(-8.0)) * rec_w2 * rec;
        coeff = m8_rec_w2_rec * rec * b;
        dbdx[0] = coeff * xmxc;
        dbdx[1] = coeff * ymyc;

        coeff2 = m8_rec_w2_rec * (((T)2.0) + rec);
        H[0] = coeff * (coeff2 * xmxc * xmxc + ((T)1.0));
        H[1] = coeff * (coeff2 * xmxc * ymyc);
        H[2] = coeff * (coeff2 * ymyc * ymyc + ((T)1.0));
        return (1);
    }

    b = ((T)0.0);
    dbdx[0] = ((T)0.0);
    dbdx[1] = ((T)0.0);
    H[0] = ((T)0.0);
    H[1] = ((T)0.0);
    H[2] = ((T)0.0);

    return (0);
}

#endif /*BUMP_FUNCTION_H*/
