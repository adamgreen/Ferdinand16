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
class MovingAverage
{
  MovingAverage(int sampleCount)
  {
    m_sampleCount = sampleCount;
    m_samples = new int[sampleCount];
    m_index = 0;
    m_sum = 0;
  }
  
  void update(int newestSample)
  {
    int oldestSample = m_samples[m_index];
    m_samples[m_index] = newestSample;
    m_index = (m_index + 1) % m_sampleCount;
    
    m_sum -= oldestSample;
    m_sum += newestSample;
  }
  
  int getAverage()
  {
    return m_sum / m_sampleCount;
  }
  
  int[] m_samples;
  int   m_sampleCount;
  int   m_index;
  int   m_sum;
};

