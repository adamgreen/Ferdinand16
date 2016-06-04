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
class FloatHeading
{
  float m_accelX;
  float m_accelY;
  float m_accelZ;
  float m_magX;
  float m_magY;
  float m_magZ;
  float m_gyroX;
  float m_gyroY;
  float m_gyroZ;
  float m_gyroTemperature;
  
  FloatHeading()
  {
  }
  
  FloatHeading(float accelX, float accelY, float accelZ, float magX, float magY, float magZ, float gyroX, float gyroY, float gyroZ, float gyroTemperature)
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

