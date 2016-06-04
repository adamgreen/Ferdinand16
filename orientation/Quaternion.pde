/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
PMatrix3D quaternionToMatrix(float[] q)
{
  float w = q[0];
  float x = q[1];
  float y = q[2];
  float z = q[3];
  
  float x2 = x * 2;
  float y2 = y * 2;
  float z2 = z * 2;
  float wx2 = w * x2;
  float wy2 = w * y2;
  float wz2 = w * z2;
  float xx2 = x * x2;
  float xy2 = x * y2;
  float xz2 = x * z2;
  float yy2 = y * y2;
  float yz2 = y * z2;
  float zz2 = z * z2;

  return new PMatrix3D(1.0f - yy2 - zz2,        xy2 - wz2,        xz2 + wy2, 0.0f,
                       xy2 + wz2, 1.0f - xx2 - zz2,        yz2 - wx2, 0.0f,
                       xz2 - wy2,        yz2 + wx2, 1.0f - xx2 - yy2, 0.0f,
                       0.0f,             0.0f,             0.0f, 1.0f);
}

void quaternionNormalize(float[] q)
{
  float magnitude = sqrt(q[0] * q[0] +
                         q[1] * q[1] +  
                         q[2] * q[2] +  
                         q[3] * q[3]);
  q[0] /= magnitude;
  q[1] /= magnitude;
  q[2] /= magnitude;
  q[3] /= magnitude;
}

void quaternionAdd(float[] q1, float[] q2)
{
  q1[0] += q2[0];
  q1[1] += q2[1];
  q1[2] += q2[2];
  q1[3] += q2[3];
}

void quaternionSubtract(float[] q1, float[] q2)
{
  q1[0] -= q2[0];
  q1[1] -= q2[1];
  q1[2] -= q2[2];
  q1[3] -= q2[3];
}

void quaternionDoubleAdd(double[] q1, float[] q2)
{
  q1[0] += (double)q2[0];
  q1[1] += (double)q2[1];
  q1[2] += (double)q2[2];
  q1[3] += (double)q2[3];
}

void quaternionDoubleAddSquared(double[] q1, float[] q2)
{
  q1[0] += (double)q2[0] * (double)q2[0];
  q1[1] += (double)q2[1] * (double)q2[1];
  q1[2] += (double)q2[2] * (double)q2[2];
  q1[3] += (double)q2[3] * (double)q2[3];
}

void quaternionStats(double[] mean, double[] variance, double[] sum, double[] sumSquared, int samples)
{
  for (int i = 0 ; i < mean.length ; i++)
  {
    mean[i] = sum[i] / samples;
    variance[i] = (sumSquared[i] - ((sum[i] * sum[i]) / samples)) / (samples - 1);
  }
}

void quaternionPrint(double[] m)
{
  println(m[0] + ", " + m[1] + ", " + m[2] + ", " + m[3]);
}

float[] matrixToQuaternion(PMatrix3D matrix)
{
  // Convert rotation matrix into normalized quaternion.
  float w = 0.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float trace = matrix.m00 + matrix.m11 + matrix.m22;
  if (trace > 0.0f)
  {
    w = sqrt(trace + 1.0f) / 2.0f;
    float lambda = 1.0f / (4.0f * w);
    x = lambda * (matrix.m21 - matrix.m12);
    y = lambda * (matrix.m02 - matrix.m20);
    z = lambda * (matrix.m10 - matrix.m01);
    return quaternion(w, x, y, z);
  }
  else
  {
    if (matrix.m00 > matrix.m11 && matrix.m00 > matrix.m22)
    {
      // m00 is the largest value on diagonal.
      x = sqrt(matrix.m00 - matrix.m11 - matrix.m22 + 1.0f) / 2.0f;
      float lambda = 1.0f / (4.0f * x);
      w = lambda * (matrix.m21 - matrix.m12);
      y = lambda * (matrix.m01 + matrix.m10);
      z = lambda * (matrix.m02 + matrix.m20);
      return quaternion(w, x, y, z);
    }
    else if (matrix.m11 > matrix.m00 && matrix.m11 > matrix.m22)
    {
      // m11 is the largest value on diagonal.
      y = sqrt(matrix.m11 - matrix.m00 - matrix.m22 + 1.0f) / 2.0f;
      float lambda = 1.0f / (4.0f * y);
      w = lambda * (matrix.m02 - matrix.m20);
      x = lambda * (matrix.m01 + matrix.m10);
      z = lambda * (matrix.m12 + matrix.m21);
      return quaternion(w, x, y, z);
    }
    else
    {
      // Only get here if m22 is the largest value on diagonal.
      z = sqrt(matrix.m22 - matrix.m00 - matrix.m11 + 1.0f) / 2.0f;
      float lambda = 1.0f / (4.0f * z);
      w = lambda * (matrix.m10 - matrix.m01);
      x = lambda * (matrix.m02 + matrix.m20);
      y = lambda * (matrix.m12 + matrix.m21);
      return quaternion(w, x, y, z);
    }
  }
}

float[] quaternion(float w, float x, float y, float z)
{
  float[] resultQuaternion = new float[4];
  resultQuaternion[0] = w;
  resultQuaternion[1] = x;
  resultQuaternion[2] = y;
  resultQuaternion[3] = z;
  return resultQuaternion;
}

float quaternionDot(float[] q1, float[] q2)
{
  return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]; 
}

void quaternionFlip(float[] q)
{
  q[0] = -q[0];
  q[1] = -q[1];
  q[2] = -q[2];
  q[3] = -q[3];
}

