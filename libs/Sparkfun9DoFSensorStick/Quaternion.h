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
#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "Vector.h"

class Quaternion
{
public:
    Quaternion(float w, float x, float y, float z)
    {
        this->w = w;
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Quaternion()
    {
        clear();
    }

    static Quaternion createFromBasisVectors(Vector<float>& row0, Vector<float>& row1, Vector<float>& row2)
    {
        // Convert the basis vectors (rotation matrix) into a normalized quaternion.
        float w = 0.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float trace = row0.x + row1.y + row2.z;
        if (trace > 0.0f)
        {
            w = sqrt(trace + 1.0f) / 2.0f;
            float lambda = 1.0f / (4.0f * w);
            x = lambda * (row2.y - row1.z);
            y = lambda * (row0.z - row2.x);
            z = lambda * (row1.x - row0.y);
        }
        else
        {
            if (row0.x > row1.y && row0.x > row2.z)
            {
                // m00 is the largest value on diagonal.
                x = sqrt(row0.x - row1.y - row2.z + 1.0f) / 2.0f;
                float lambda = 1.0f / (4.0f * x);
                w = lambda * (row2.y - row1.z);
                y = lambda * (row0.y + row1.x);
                z = lambda * (row0.z + row2.x);
            }
            else if (row1.y > row0.x && row1.y > row2.z)
            {
                // m11 is the largest value on diagonal.
                y = sqrt(row1.y - row0.x - row2.z + 1.0f) / 2.0f;
                float lambda = 1.0f / (4.0f * y);
                w = lambda * (row0.z - row2.x);
                x = lambda * (row0.y + row1.x);
                z = lambda * (row1.z + row2.y);
            }
            else
            {
                // Only get here if m22 is the largest value on diagonal.
                z = sqrt(row2.z - row0.x - row1.y + 1.0f) / 2.0f;
                float lambda = 1.0f / (4.0f * z);
                w = lambda * (row1.x - row0.y);
                x = lambda * (row0.z + row2.x);
                y = lambda * (row1.z + row2.y);
            }
        }
        return Quaternion(w, x, y, z);
    }

    void clear()
    {
        w = 1.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    float magnitude()
    {
        return sqrtf(w*w + x*x + y*y + z*z);
    }

    void normalize()
    {
        float magInverse = 1.0f / magnitude();
        w *= magInverse;
        x *= magInverse;
        y *= magInverse;
        z *= magInverse;
    }

    float dotProduct(Quaternion& q)
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    void flip()
    {
        w = -w;
        x = -x;
        y = -y;
        z = -z;
    }

    Quaternion subtract(Quaternion& q)
    {
        return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
    }

    Quaternion add(Quaternion& q)
    {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }

    float w;
    float x;
    float y;
    float z;
};

#endif /* QUATERNION_H_ */
