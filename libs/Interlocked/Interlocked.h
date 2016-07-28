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
#ifndef INTERLOCKED_H_
#define INTERLOCKED_H_

#include <mbed.h>


__STATIC_INLINE uint32_t interlockedIncrement(volatile uint32_t* p)
{
    uint32_t value;

    do
    {
        value = __LDREXW(p);
        value++;
    } while (__STREXW(value, p));

    return value;
}

__STATIC_INLINE uint32_t interlockedDecrement(volatile uint32_t* p)
{
    uint32_t value;

    do
    {
        value = __LDREXW(p);
        value--;
    } while (__STREXW(value, p));

    return value;
}

#endif // INTERLOCKED_H_
