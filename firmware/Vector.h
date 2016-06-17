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

    void clear()
    {
        x = 0;
        y = 0;
        z = 0;
    }

    T x;
    T y;
    T z;
};

#endif /* VECTOR_H_ */
