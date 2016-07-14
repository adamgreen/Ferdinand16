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
class HeadingSensorCalibration
{
  public IntVector   accelMin;
  public IntVector   accelMax;
  public IntVector   magMin;
  public IntVector   magMax;
  public FloatVector gyroCoefficientA;
  public FloatVector gyroCoefficientB;
  public FloatVector gyroScale;
  public IntVector   declinationCorrection;
  public IntVector   mountingCorrection;
};

class HeadingSensor
{
  HeadingSensor(Serial port, HeadingSensorCalibration calibration, Heading sampleCounts)
  {
    m_calibration = calibration;
    calibrate(calibration);
    
    m_averages = new MovingAverage[6];
    m_averages[0] = new MovingAverage(sampleCounts.m_accelX);
    m_averages[1] = new MovingAverage(sampleCounts.m_accelY);
    m_averages[2] = new MovingAverage(sampleCounts.m_accelZ);
    m_averages[3] = new MovingAverage(sampleCounts.m_magX);
    m_averages[4] = new MovingAverage(sampleCounts.m_magY);
    m_averages[5] = new MovingAverage(sampleCounts.m_magZ);
    
    // Clear out any data that we might be in the middle of.
    m_port = port;
    m_port.clear();
    String dummy = m_port.readStringUntil('\n');
    m_port.bufferUntil('\n');
  }

  void calibrate(HeadingSensorCalibration c)
  {
    m_midpoint = new FloatHeading((c.accelMin.x + c.accelMax.x) / 2.0f,
                                  (c.accelMin.y + c.accelMax.y) / 2.0f,
                                  (c.accelMin.z + c.accelMax.z) / 2.0f,
                                  (c.magMin.x + c.magMax.x) / 2.0f,
                                  (c.magMin.y + c.magMax.y) / 2.0f,
                                  (c.magMin.z + c.magMax.z) / 2.0f,
                                  0.0f,
                                  0.0f,
                                  0.0f,
                                  0.0f);
    m_scale = new FloatHeading((c.accelMax.x - c.accelMin.x) / 2.0f,
                               (c.accelMax.y - c.accelMin.y) / 2.0f,
                               (c.accelMax.z - c.accelMin.z) / 2.0f,
                               (c.magMax.x - c.magMin.x) / 2.0f,
                               (c.magMax.y - c.magMin.y) / 2.0f,
                               (c.magMax.z - c.magMin.z) / 2.0f,
                               1.0f,
                               1.0f,
                               1.0f,
                               1.0f);
    m_declinationCorrection = angleFromDegreeMinuteSecond(c.declinationCorrection);
    m_mountingCorrection = angleFromDegreeMinuteSecond(c.mountingCorrection);
  }
  
  float angleFromDegreeMinuteSecond(IntVector angle)
  {
    if (angle.x >= 0.0f)
      return radians(angle.x + angle.y / 60.0f + angle.z / 3600.0f);
    else
      return radians(angle.x - angle.y / 60.0f - angle.z / 3600.0f);
  }

  boolean update()
  {
    String line = m_port.readString();
    if (line == null)
      return false;
      
    String[] tokens = splitTokens(line, ",\n");
    if (tokens.length == 17 || tokens.length == 16)
    {
      int nextToken = 0;
      
      if (tokens.length == 17 && tokens[0].equals("R"))
      {
        m_resetRequested = true;
        nextToken++;
      }
      
      int firstToken = nextToken;
      m_currentRaw.m_accelX = int(tokens[nextToken++]);
      m_currentRaw.m_accelY = int(tokens[nextToken++]);
      m_currentRaw.m_accelZ = int(tokens[nextToken++]);
      m_currentRaw.m_magX = int(tokens[nextToken++]);
      m_currentRaw.m_magY = int(tokens[nextToken++]);
      m_currentRaw.m_magZ = int(tokens[nextToken++]);
      m_currentRaw.m_gyroX = int(tokens[nextToken++]);
      m_currentRaw.m_gyroY = int(tokens[nextToken++]);
      m_currentRaw.m_gyroZ = int(tokens[nextToken++]);
      m_currentRaw.m_gyroTemperature = int(tokens[nextToken++]);
      
      for (int i = 0 ; i < m_averages.length ; i++)
        m_averages[i].update(int(tokens[firstToken + i]));
   
      m_max = m_max.max(m_currentRaw);
      m_min = m_min.min(m_currentRaw);
      
      // Skip time tokens.
      nextToken++; nextToken++;

      // Extract and store orientation quaternion calculated by embedded device.
      float w = float(tokens[nextToken++]);
      float x = float(tokens[nextToken++]);
      float y = float(tokens[nextToken++]);
      float z = float(tokens[nextToken++]);
      m_embeddedQuaternion = quaternion(w, x, y, z);

      return true;
    }
    
    return false;
  }
    
  Heading getCurrentRaw()
  {
    return m_currentRaw;
  }
  
  Heading getCurrentMovingAverage()
  {
    return new Heading(m_averages[0].getAverage(),
                       m_averages[1].getAverage(),
                       m_averages[2].getAverage(),
                       m_averages[3].getAverage(),
                       m_averages[4].getAverage(),
                       m_averages[5].getAverage(),
                       m_currentRaw.m_gyroX,
                       m_currentRaw.m_gyroY,
                       m_currentRaw.m_gyroZ,
                       m_currentRaw.m_gyroTemperature);
  }
  
  FloatHeading getCurrentFiltered()
  {
    FloatVector gyro = getCalibratedGyroData();
    return new FloatHeading((m_averages[0].getAverage() - m_midpoint.m_accelX) / m_scale.m_accelX,
                            (m_averages[1].getAverage() - m_midpoint.m_accelY) / m_scale.m_accelY,
                            (m_averages[2].getAverage() - m_midpoint.m_accelZ) / m_scale.m_accelZ,
                            (m_averages[3].getAverage() - m_midpoint.m_magX) / m_scale.m_magX,
                            (m_averages[4].getAverage() - m_midpoint.m_magY) / m_scale.m_magY,
                            (m_averages[5].getAverage() - m_midpoint.m_magZ) / m_scale.m_magZ,
                            gyro.x,
                            gyro.y,
                            gyro.z,
                            m_currentRaw.m_gyroTemperature);
  }
  
  FloatVector getCalibratedGyroData()
  {
    FloatVector result = new FloatVector();
    result.x = m_currentRaw.m_gyroX - (m_currentRaw.m_gyroTemperature * m_calibration.gyroCoefficientA.x + m_calibration.gyroCoefficientB.x);
    result.y = m_currentRaw.m_gyroY - (m_currentRaw.m_gyroTemperature * m_calibration.gyroCoefficientA.y + m_calibration.gyroCoefficientB.y);
    result.z = m_currentRaw.m_gyroZ - (m_currentRaw.m_gyroTemperature * m_calibration.gyroCoefficientA.z + m_calibration.gyroCoefficientB.z);
    result.x *= radians(1.0f / m_calibration.gyroScale.x);
    result.y *= radians(1.0f / m_calibration.gyroScale.y);
    result.z *= radians(1.0f / m_calibration.gyroScale.z);
    return result;
  }
  
  FloatHeading getCurrent()
  {
    FloatVector gyro = getCalibratedGyroData();
    return new FloatHeading((m_currentRaw.m_accelX - m_midpoint.m_accelX) / m_scale.m_accelX,
                            (m_currentRaw.m_accelY - m_midpoint.m_accelY) / m_scale.m_accelY,
                            (m_currentRaw.m_accelZ - m_midpoint.m_accelZ) / m_scale.m_accelZ,
                            (m_currentRaw.m_magX - m_midpoint.m_magX) / m_scale.m_magX,
                            (m_currentRaw.m_magY - m_midpoint.m_magY) / m_scale.m_magY,
                            (m_currentRaw.m_magZ - m_midpoint.m_magZ) / m_scale.m_magZ,
                            gyro.x,
                            gyro.y,
                            gyro.z,
                            m_currentRaw.m_gyroTemperature);                           
  }
  
  float[] getEmbeddedQuaternion()
  {
    return m_embeddedQuaternion;
  }
  
  float getHeading(float[] q)
  {
    // Correct compass heading for declination at location where the robot is being run.
    // Also account for how the IMU is mounted to the robot. The yaw reading is negated to
    // flip the rotations around the y-axis so that they increase as you progress from North
    // to East.
    return constrainAngle(-getYaw(q) + m_declinationCorrection + m_mountingCorrection);
  }

  float constrainAngle(float angle)
  {
    if (angle < -PI)
    {
      return angle + TWO_PI;
    }
    else if (angle > PI)
    {
      return angle - TWO_PI;
    }
    
    return angle;
  }
  
  float getYaw(float[] q)
  {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
  
    return atan2(2*(x*z+y*w), 1-2*(x*x+y*y));
  }
  
  float getPitch(float[] q)
  {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
  
    return asin(-2*(y*z-x*w));
  }
    
  float getRoll(float[] q)
  {
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
  
    return atan2(2*(x*y+z*w), 1-2*(x*x+z*z));
  }

  Heading getMin()
  {
    return m_min;
  }
  
  Heading getMax()
  {
    return m_max;
  }
  
  boolean wasResetRequested()
  {
    boolean curr = m_resetRequested;
    m_resetRequested = false;
    return curr;
  }
  
  HeadingSensorCalibration m_calibration;
  Serial  m_port;
  Heading m_currentRaw = new Heading();
  Heading m_min = new Heading(0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
                              0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF,
                              0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF);
  Heading m_max = new Heading(0x80000000, 0x80000000, 0x80000000,
                              0x80000000, 0x80000000, 0x80000000,
                              0x80000000, 0x80000000, 0x80000000, 0x80000000);
  FloatHeading    m_midpoint;
  FloatHeading    m_scale;
  float           m_declinationCorrection;
  float           m_mountingCorrection;
  float[]         m_embeddedQuaternion = new float[4];
  MovingAverage[] m_averages;
  boolean         m_resetRequested = false;
};

