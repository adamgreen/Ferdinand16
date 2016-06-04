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
class Heading
{
  int m_accelX;
  int m_accelY;
  int m_accelZ;
  int m_magX;
  int m_magY;
  int m_magZ;
  int m_gyroX;
  int m_gyroY;
  int m_gyroZ;
  int m_gyroTemperature;
  
  Heading()
  {
  }
  
  Heading(int accelX, int accelY, int accelZ, int magX, int magY, int magZ, int gyroX, int gyroY, int gyroZ, int gyroTemperature)
  {
    m_accelX = accelX;
    m_accelY = accelY;
    m_accelZ = accelZ;
    m_magX = magX;
    m_magY = magY;
    m_magZ = magZ;
    m_gyroX = gyroX;
    m_gyroY = gyroY;
    m_gyroZ = gyroZ;
    m_gyroTemperature = gyroTemperature;
  }
  
  Heading max(Heading other)
  {
    Heading max = new Heading();
    if (m_accelX > other.m_accelX)
      max.m_accelX = m_accelX;
    else
      max.m_accelX = other.m_accelX;
    if (m_accelY > other.m_accelY)
      max.m_accelY = m_accelY;
    else
      max.m_accelY = other.m_accelY;
    if (m_accelZ > other.m_accelZ)
      max.m_accelZ = m_accelZ;
    else
      max.m_accelZ = other.m_accelZ;

    if (m_magX > other.m_magX)
      max.m_magX = m_magX;
    else
      max.m_magX = other.m_magX;
    if (m_magY > other.m_magY)
      max.m_magY = m_magY;
    else
      max.m_magY = other.m_magY;
    if (m_magZ > other.m_magZ)
      max.m_magZ = m_magZ;
    else
      max.m_magZ = other.m_magZ;

    if (m_gyroX > other.m_gyroX)
      max.m_gyroX = m_gyroX;
    else
      max.m_gyroX = other.m_gyroX;
    if (m_gyroY > other.m_gyroY)
      max.m_gyroY = m_gyroY;
    else
      max.m_gyroY = other.m_gyroY;
    if (m_gyroZ > other.m_gyroZ)
      max.m_gyroZ = m_gyroZ;
    else
      max.m_gyroZ = other.m_gyroZ;
    
    if (m_gyroTemperature > other.m_gyroTemperature)
      max.m_gyroTemperature = m_gyroTemperature;
    else
      max.m_gyroTemperature = other.m_gyroTemperature;
      
    return max;
  }
  
  Heading min(Heading other)
  {
    Heading min = new Heading();
    if (m_accelX < other.m_accelX)
      min.m_accelX = m_accelX;
    else
      min.m_accelX = other.m_accelX;
    if (m_accelY < other.m_accelY)
      min.m_accelY = m_accelY;
    else
      min.m_accelY = other.m_accelY;
    if (m_accelZ < other.m_accelZ)
      min.m_accelZ = m_accelZ;
    else
      min.m_accelZ = other.m_accelZ;

    if (m_magX < other.m_magX)
      min.m_magX = m_magX;
    else
      min.m_magX = other.m_magX;
    if (m_magY < other.m_magY)
      min.m_magY = m_magY;
    else
      min.m_magY = other.m_magY;
    if (m_magZ < other.m_magZ)
      min.m_magZ = m_magZ;
    else
      min.m_magZ = other.m_magZ;

    if (m_gyroX < other.m_gyroX)
      min.m_gyroX = m_gyroX;
    else
      min.m_gyroX = other.m_gyroX;
    if (m_gyroY < other.m_gyroY)
      min.m_gyroY = m_gyroY;
    else
      min.m_gyroY = other.m_gyroY;
    if (m_gyroZ < other.m_gyroZ)
      min.m_gyroZ = m_gyroZ;
    else
      min.m_gyroZ = other.m_gyroZ;

    if (m_gyroTemperature < other.m_gyroTemperature)
      min.m_gyroTemperature = m_gyroTemperature;
    else
      min.m_gyroTemperature = other.m_gyroTemperature;
      
    return min;
  }
  
  void print()
  {
    System.out.print(m_accelX + "," +
              m_accelY + "," +
              m_accelZ + "," +
              m_magX + "," +
              m_magY + "," +
              m_magZ + "," +
              m_gyroX + "," +
              m_gyroY + "," +
              m_gyroZ + "," +
              m_gyroTemperature);
  }
};

