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
#ifndef MATRIX_4x4_H_
#define MATRIX_4x4_H_

#include <dsp.h>
#include "Quaternion.h"

class Matrix4x4
{
public:
    Matrix4x4(float m00, float m01, float m02, float m03,
              float m10, float m11, float m12, float m13,
              float m20, float m21, float m22, float m23,
              float m30, float m31, float m32, float m33)
    {
        m_00 = m00;
        m_01 = m01;
        m_02 = m02;
        m_03 = m03;

        m_10 = m10;
        m_11 = m11;
        m_12 = m12;
        m_13 = m13;

        m_20 = m20;
        m_21 = m21;
        m_22 = m22;
        m_23 = m23;

        m_30 = m30;
        m_31 = m31;
        m_32 = m32;
        m_33 = m33;
    }

    Matrix4x4()
    {
        clear();
    }

    void clear()
    {
        memset(m_data, 0, sizeof(m_data));
    }

    Quaternion multiply(Quaternion& q)
    {
        Quaternion prod;

        prod.w = m_00*q.w + m_01*q.x + m_02*q.y + m_03*q.z;
        prod.x = m_10*q.w + m_11*q.x + m_12*q.y + m_13*q.z;
        prod.y = m_20*q.w + m_21*q.x + m_22*q.y + m_23*q.z;
        prod.z = m_30*q.w + m_31*q.x + m_32*q.y + m_33*q.z;

        return prod;
    }

    Matrix4x4 multiply(Matrix4x4& m)
    {
        Matrix4x4 prod;

        prod.m_00 = m_00*m.m_00 + m_01*m.m_10 + m_02*m.m_20 + m_03*m.m_30;
        prod.m_01 = m_00*m.m_01 + m_01*m.m_11 + m_02*m.m_21 + m_03*m.m_31;
        prod.m_02 = m_00*m.m_02 + m_01*m.m_12 + m_02*m.m_22 + m_03*m.m_32;
        prod.m_03 = m_00*m.m_03 + m_01*m.m_13 + m_02*m.m_23 + m_03*m.m_33;

        prod.m_10 = m_10*m.m_00 + m_11*m.m_10 + m_12*m.m_20 + m_13*m.m_30;
        prod.m_11 = m_10*m.m_01 + m_11*m.m_11 + m_12*m.m_21 + m_13*m.m_31;
        prod.m_12 = m_10*m.m_02 + m_11*m.m_12 + m_12*m.m_22 + m_13*m.m_32;
        prod.m_13 = m_10*m.m_03 + m_11*m.m_13 + m_12*m.m_23 + m_13*m.m_33;

        prod.m_20 = m_20*m.m_00 + m_21*m.m_10 + m_22*m.m_20 + m_23*m.m_30;
        prod.m_21 = m_20*m.m_01 + m_21*m.m_11 + m_22*m.m_21 + m_23*m.m_31;
        prod.m_22 = m_20*m.m_02 + m_21*m.m_12 + m_22*m.m_22 + m_23*m.m_32;
        prod.m_23 = m_20*m.m_03 + m_21*m.m_13 + m_22*m.m_23 + m_23*m.m_33;

        prod.m_30 = m_30*m.m_00 + m_31*m.m_10 + m_32*m.m_20 + m_33*m.m_30;
        prod.m_31 = m_30*m.m_01 + m_31*m.m_11 + m_32*m.m_21 + m_33*m.m_31;
        prod.m_32 = m_30*m.m_02 + m_31*m.m_12 + m_32*m.m_22 + m_33*m.m_32;
        prod.m_33 = m_30*m.m_03 + m_31*m.m_13 + m_32*m.m_23 + m_33*m.m_33;

        return prod;
    }

    Matrix4x4 multiplyTransposed(Matrix4x4& m)
    {
        Matrix4x4 prod;

        prod.m_00 = m_00*m.m_00 + m_01*m.m_01 + m_02*m.m_02 + m_03*m.m_03;
        prod.m_01 = m_00*m.m_10 + m_01*m.m_11 + m_02*m.m_12 + m_03*m.m_13;
        prod.m_02 = m_00*m.m_20 + m_01*m.m_21 + m_02*m.m_22 + m_03*m.m_23;
        prod.m_03 = m_00*m.m_30 + m_01*m.m_31 + m_02*m.m_32 + m_03*m.m_33;

        prod.m_10 = m_10*m.m_00 + m_11*m.m_01 + m_12*m.m_02 + m_13*m.m_03;
        prod.m_11 = m_10*m.m_10 + m_11*m.m_11 + m_12*m.m_12 + m_13*m.m_13;
        prod.m_12 = m_10*m.m_20 + m_11*m.m_21 + m_12*m.m_22 + m_13*m.m_23;
        prod.m_13 = m_10*m.m_30 + m_11*m.m_31 + m_12*m.m_32 + m_13*m.m_33;

        prod.m_20 = m_20*m.m_00 + m_21*m.m_01 + m_22*m.m_02 + m_23*m.m_03;
        prod.m_21 = m_20*m.m_10 + m_21*m.m_11 + m_22*m.m_12 + m_23*m.m_13;
        prod.m_22 = m_20*m.m_20 + m_21*m.m_21 + m_22*m.m_22 + m_23*m.m_23;
        prod.m_23 = m_20*m.m_30 + m_21*m.m_31 + m_22*m.m_32 + m_23*m.m_33;

        prod.m_30 = m_30*m.m_00 + m_31*m.m_01 + m_32*m.m_02 + m_33*m.m_03;
        prod.m_31 = m_30*m.m_10 + m_31*m.m_11 + m_32*m.m_12 + m_33*m.m_13;
        prod.m_32 = m_30*m.m_20 + m_31*m.m_21 + m_32*m.m_22 + m_33*m.m_23;
        prod.m_33 = m_30*m.m_30 + m_31*m.m_31 + m_32*m.m_32 + m_33*m.m_33;

        return prod;
    }

    Matrix4x4 inverse()
    {
        Matrix4x4               result;
        arm_matrix_instance_f32 instanceIn = {4, 4, (float*)m_data};
        arm_matrix_instance_f32 instanceOut = {4, 4, (float*)result.m_data};

        arm_mat_inverse_f32(&instanceIn, &instanceOut);

        return result;
    }

    Matrix4x4 add(Matrix4x4& m)
    {
        Matrix4x4 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01 + m.m_01;
        sum.m_02 = m_02 + m.m_02;
        sum.m_03 = m_03 + m.m_03;

        sum.m_10 = m_10 + m.m_10;
        sum.m_11 = m_11 + m.m_11;
        sum.m_12 = m_12 + m.m_12;
        sum.m_13 = m_13 + m.m_13;

        sum.m_20 = m_20 + m.m_20;
        sum.m_21 = m_21 + m.m_21;
        sum.m_22 = m_22 + m.m_22;
        sum.m_23 = m_23 + m.m_23;

        sum.m_30 = m_30 + m.m_30;
        sum.m_31 = m_31 + m.m_31;
        sum.m_32 = m_32 + m.m_32;
        sum.m_33 = m_33 + m.m_33;

        return sum;
    }

    // This function is an optimization of the general add() method which can be used when parameter m is known to only
    // contain zeroes in its non-diagonal elements.
    Matrix4x4 addDiagonal(Matrix4x4& m)
    {
        Matrix4x4 sum;

        sum.m_00 = m_00 + m.m_00;
        sum.m_01 = m_01;
        sum.m_02 = m_02;
        sum.m_03 = m_03;

        sum.m_10 = m_10;
        sum.m_11 = m_11 + m.m_11;
        sum.m_12 = m_12;
        sum.m_13 = m_13;

        sum.m_20 = m_20;
        sum.m_21 = m_21;
        sum.m_22 = m_22 + m.m_22;
        sum.m_23 = m_23;

        sum.m_30 = m_30;
        sum.m_31 = m_31;
        sum.m_32 = m_32;
        sum.m_33 = m_33 + m.m_33;

        return sum;
    }

    Matrix4x4 subtract(Matrix4x4& m)
    {
        Matrix4x4 sum;

        sum.m_00 = m_00 - m.m_00;
        sum.m_01 = m_01 - m.m_01;
        sum.m_02 = m_02 - m.m_02;
        sum.m_03 = m_03 - m.m_03;

        sum.m_10 = m_10 - m.m_10;
        sum.m_11 = m_11 - m.m_11;
        sum.m_12 = m_12 - m.m_12;
        sum.m_13 = m_13 - m.m_13;

        sum.m_20 = m_20 - m.m_20;
        sum.m_21 = m_21 - m.m_21;
        sum.m_22 = m_22 - m.m_22;
        sum.m_23 = m_23 - m.m_23;

        sum.m_30 = m_30 - m.m_30;
        sum.m_31 = m_31 - m.m_31;
        sum.m_32 = m_32 - m.m_32;
        sum.m_33 = m_33 - m.m_33;

        return sum;
    }

    union
    {
        float m_data[4][4];
        struct
        {
            float m_00;
            float m_01;
            float m_02;
            float m_03;

            float m_10;
            float m_11;
            float m_12;
            float m_13;

            float m_20;
            float m_21;
            float m_22;
            float m_23;

            float m_30;
            float m_31;
            float m_32;
            float m_33;
        };
    };
};

#endif /* MATRIX_4x4_H_ */
