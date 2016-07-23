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
#include "SensorBase.h"


SensorBase::SensorBase(I2C* pI2C, int address)
{
    m_pI2C = pI2C;
    m_address = address;
    m_failedIo = 0;
    m_failedInit = 0;
}

void SensorBase::writeRegister(char registerAddress, char value)
{
    char dataToSend[2] = { registerAddress, value };

    m_failedIo = m_pI2C->write(m_address, dataToSend, sizeof(dataToSend), false);
}

void SensorBase::readRegister(char registerAddress, char* pBuffer)
{
    readRegisters(registerAddress, pBuffer, sizeof(*pBuffer));
}

void SensorBase::readRegisters(char registerAddress, void* pBuffer, size_t bufferSize)
{
    m_failedIo = m_pI2C->write(m_address, &registerAddress, sizeof(registerAddress), true);
    if (m_failedIo)
        return;
    m_failedIo = m_pI2C->read(m_address, (char*)pBuffer, bufferSize, false);
}
