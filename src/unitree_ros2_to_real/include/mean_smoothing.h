//////////////////////////////////////////////////////////////////////////
/**
 *   C++ implementation of Exponential Smoothing
 *
 *   Current single ES and double ES are implemented
 *
 *   Reference:
 *   1. Wikipedia: https://en.wikipedia.org/wiki/Exponential_smoothing
 *   2. http://www.itl.nist.gov/div898/handbook/pmc/section4/pmc431.htm
 *
 *   Code developed by Shuo Jin at Dept. of MAE, CUHK, Hong Kong.
 *   Email: jerry.shuojin@gmail.com. All rights reserved.
 *
 *   Any bug report or suggestion is welcome!
 *
 *   Current Version - 0.20160113
 */
//////////////////////////////////////////////////////////////////////////

#ifndef HEADER_MEAN_SMOOTHING_H
#define HEADER_MEAN_SMOOTHING_H

#include <limits>
#include <iostream>
#include <cassert>
#include <queue>

/** Class data_vec defines a D-dimensional vector
 *  \T Data type of vector
 *  \D Dimension of vector
 */
template <typename T, size_t D>
class es_vec
{
public:
    /** Default constructor*/
    es_vec()
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] = std::numeric_limits<T>::lowest();
    }

    /** Create a vector with an initial value _v for all elements */
    es_vec(const T _v)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] = _v;
    }

    /** Copy constructor*/
    es_vec(const es_vec<T, D> &_vec)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] = _vec.esv_data[i];
    }

    /** Destructor*/
    ~es_vec()
    {
    }

public:
    /** Reset the vector with a specified value _v */
    void reset(const T _v = 0)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] = _v;
    }

    /** Init from a raw vector */
    void init(const T *_data)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] = _data[i];
    }

    /** Operator [] */
    T &operator[](const size_t _i)
    {
        return esv_data[_i];
    }

    const T &operator[](const size_t _i) const
    {
        return esv_data[_i];
    }

    /** Operator = */
    es_vec<T, D> &operator=(const es_vec<T, D> &_vec)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] = _vec.esv_data[i];
        return *this;
    }

    /** Operator += */
    es_vec<T, D> &operator+=(const es_vec<T, D> &_vec)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] += _vec[i];
        return *this;
    }

    /** Operator -= */
    es_vec<T, D> &operator-=(const es_vec<T, D> &_vec)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] -= _vec[i];
        return *this;
    }

    /** Operator *= */
    es_vec<T, D> &operator*=(const T _v)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] *= _v;
        return *this;
    }

    /** Operator /= */
    es_vec<T, D> &operator/=(const T _v)
    {
        for (size_t i = 0; i < D; ++i)
            esv_data[i] /= _v;
        return *this;
    }

    /** get the raw data vector */
    T *raw_data()
    {
        return esv_data;
    }

    /** return the 2-norm of this vector */
    const T norm() const
    {
        T norm(0);
        for (size_t i = 0; i < D; ++i)
            norm += esv_data[i] * esv_data[i];
        return sqrt(norm);
    }

    /** ************* Friend functions to provide more operators ****************************** */
    /** Operator + */
    friend es_vec<T, D> operator+(const es_vec<T, D> &_vec1, const es_vec<T, D> &_vec2)
    {
        return es_vec<T, D>(_vec1) += _vec2;
    }

    /** Operator - */
    friend es_vec<T, D> operator-(const es_vec<T, D> &_vec1, const es_vec<T, D> &_vec2)
    {
        return es_vec<T, D>(_vec1) -= _vec2;
    }

    /** Operator * */
    friend es_vec<T, D> operator*(const T _v, const es_vec<T, D> &_vec)
    {
        return es_vec<T, D>(_vec) *= _v;
    }

    friend es_vec<T, D> operator*(const es_vec<T, D> &_vec, const T _v)
    {
        return es_vec<T, D>(_vec) *= _v;
    }

    /** Operator / */
    friend es_vec<T, D> operator/(const es_vec<T, D> &_vec, const T _v)
    {
        return es_vec<T, D>(_vec) /= _v;
    }

    /** Operator == */
    friend const bool operator==(const es_vec<T, D> &_vec1, const es_vec<T, D> &_vec2)
    {
        for (size_t i = 0; i < D; ++i)
        {
            if (!_vec1[i] == _vec2[i])
                return false;
        }
        return true;
    }

    /** Operator != */
    friend const bool operator!=(const es_vec<T, D> &_vec1, const es_vec<T, D> &_vec2)
    {
        for (size_t i = 0; i < D; ++i)
        {
            if (!_vec1[i] == _vec2[i])
                return true;
        }
        return false;
    }

    /** Operator << */
    friend std::ostream &operator<<(std::ostream &_os, const es_vec<T, D> &_vec)
    {
        _os << "[";
        for (size_t i = 0; i < D - 1; ++i)
            _os << _vec[i] << " ";
        _os << _vec[D - 1] << "]";
        return _os;
    }

private:
    T esv_data[D];
};

/** Single Exponential Smoothing Method
 *  \T Data type float/double/double double...
 *  \D Dimension of this class
 */
template <typename T, size_t D>
class mean_smoothing
{
public:
    /** Default constructor */
    mean_smoothing(): window(15), ses_counter(0), sum(0)
    {
        curr_smoothed_ob.reset();
    }

    /** Destructor */
    ~mean_smoothing()
    {
    }

public:
    /** Set the value of smoothing constant in the range (0, 1) */
    void set_window(const T _val)
    {
        assert(_val > 0);
        window = _val;
    }

    /** Push a new vec and pop its smoothed value as the return */
    const es_vec<T, D> push_to_pop(const es_vec<T, D> _curr_raw_ob)
    {
        ++ses_counter;
        sum += _curr_raw_ob;
        last_obs.push(_curr_raw_ob);

        if (window < ses_counter)
        {
            sum -= last_obs.front();
            last_obs.pop();
            curr_smoothed_ob = sum / window;
        }
        else
        {
            curr_smoothed_ob = sum / ses_counter;
        }

        return curr_smoothed_ob;
    }

    /** Get the current counter */
    const size_t counter() const { return ses_counter; }

    /** Reset this class to initial state */
    void reset()
    {
        curr_smoothed_ob.reset();
        window = 15;
        ses_counter = 0;
        sum = 0;
    }

private:
    es_vec<T, D> curr_smoothed_ob;
    T window;
    size_t ses_counter;
    es_vec<T, D> sum;
    std::queue <es_vec<T, D>> last_obs;
};

#endif // HEADER_MEAN_SMOOTHING_H