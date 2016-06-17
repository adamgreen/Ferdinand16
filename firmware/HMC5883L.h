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
#ifndef HMC5883L_H_
#define HMC5883L_H_

#include <mbed.h>
#include "Vector.h"
#include "SensorBase.h"


class HMC5883L : public SensorBase
{
public:
    HMC5883L(I2C* pI2C, int address = 0x3C);

    void getVector(Vector<int16_t>* pVector);

protected:
    void initMagnetometer();
    void waitForDataReady();
};

#endif /* HMC5883L_H_ */
