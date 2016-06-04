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
class FloatVector
{
  public FloatVector(float x, float y, float z)
  {
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public FloatVector()
  {
    this.x = 0.0f;
    this.y = 0.0f;
    this.z = 0.0f;
  }
  
  public float x;
  public float y;
  public float z;
};
