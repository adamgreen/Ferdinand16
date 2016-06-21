/*  Copyright (C) 2016  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef VECTOR_H_
#define VECTOR_H_

#include <assert.h>
#include <stdint.h>


template <class T>
class Vector
{
public:
    Vector(T x, T y, T z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Vector()
    {
        clear();
    }

    static Vector<T> createFromSwizzledSource(Vector<int16_t>& swizzle, Vector<T>& v)
    {
        Vector<T> newVector;

        for (int i = 0 ; i < 3 ; i++)
        {
            int16_t swizzleFrom = swizzle[i];
            bool    inverse = false;

            if (swizzleFrom < 0)
            {
                swizzleFrom = -swizzleFrom;
                inverse = true;
            }

            newVector[i] = inverse ? -v[swizzleFrom - 1] : v[swizzleFrom - 1];
        }

        return newVector;
    }

    void clear()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    T& operator [](int i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            assert(i >= 0 && i <= 2);
            return z;
        }
    }

    float magnitude()
    {
        return sqrtf(x*x + y*y + z*z);
    }

    void normalize()
    {
        float magInverse = 1.0f / magnitude();
        x *= magInverse;
        y *= magInverse;
        z *= magInverse;
    }

    T dotProduct(Vector<T>& v)
    {
        return x * v.x + y * v.y + z * v.z;
    }

    Vector<T> crossProduct(Vector<T>& v)
    {
        return Vector<T>(y*v.z - z*v.y,
                         z*v.x - x*v.z,
                         x*v.y - y*v.x);
    }

    Vector<T> multiply(T scalar)
    {
        return Vector<T>(x * scalar, y * scalar, z * scalar);
    }

    Vector<T> subtract(Vector<T>& v)
    {
        return Vector<T>(x - v.x, y - v.y, z - v.z);
    }

    T x;
    T y;
    T z;
};

#endif /* VECTOR_H_ */
