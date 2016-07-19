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
class DoubleHeading
{
  double m_accelX;
  double m_accelY;
  double m_accelZ;
  double m_magX;
  double m_magY;
  double m_magZ;
  double m_gyroX;
  double m_gyroY;
  double m_gyroZ;
  double m_gyroTemperature;
  
  DoubleHeading()
  {
  }
  
  DoubleHeading(double accelX, double accelY, double accelZ, double magX, double magY, double magZ, double gyroX, double gyroY, double gyroZ, double gyroTemperature)
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

  void add(Heading other)
  {
    m_accelX += other.m_accelX;
    m_accelY += other.m_accelY;
    m_accelZ += other.m_accelZ;
    m_magX += other.m_magX;
    m_magY += other.m_magY;
    m_magZ += other.m_magZ;
    m_gyroX += other.m_gyroX;
    m_gyroY += other.m_gyroY;
    m_gyroZ += other.m_gyroZ;
    m_gyroTemperature += other.m_gyroTemperature;
  }

  void addSquared(Heading other)
  {
    m_accelX += other.m_accelX * other.m_accelX;
    m_accelY += other.m_accelY * other.m_accelY;
    m_accelZ += other.m_accelZ * other.m_accelZ;
    m_magX += other.m_magX * other.m_magX;
    m_magY += other.m_magY * other.m_magY;
    m_magZ += other.m_magZ * other.m_magZ;
    m_gyroX += other.m_gyroX * other.m_gyroX;
    m_gyroY += other.m_gyroY * other.m_gyroY;
    m_gyroZ += other.m_gyroZ * other.m_gyroZ;
    m_gyroTemperature += other.m_gyroTemperature * other.m_gyroTemperature;
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

