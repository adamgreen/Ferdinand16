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
#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

#include <mbed.h>


class SensorBase
{
public:
    SensorBase(I2C* pI2C, int address);

    int didInitFail() { return m_failedInit; }
    int didIoFail() { return m_failedIo; }

protected:
    void writeRegister(char registerAddress, char value);
    void readRegister(char registerAddress, char* pBuffer);
    void readRegisters(char registerAddress, void* pBuffer, size_t bufferSize);

    I2C* m_pI2C;
    int  m_failedInit;
    int  m_failedIo;
    int  m_address;
};

#endif /* SENSOR_BASE_H_ */
