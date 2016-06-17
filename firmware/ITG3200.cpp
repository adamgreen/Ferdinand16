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
#include "ITG3200.h"


/* Gyro I2C Registers */
#define WHO_AM_I            0x00
#define SMPLRT_DIV          0x15
#define DLPF_FS             0x16
#define INT_CFG             0x17
#define INT_STATUS          0x1A
#define TEMP_OUT_H          0x1B
#define TEMP_OUT_L          0x1C
#define GYRO_XOUT_H         0x1D
#define GYRO_XOUT_L         0x1E
#define GYRO_YOUT_H         0x1F
#define GYRO_YOUT_L         0x20
#define GYRO_ZOUT_H         0x21
#define GYRO_ZOUT_L         0x22
#define PWR_MGM             0x3E

/* DLPF bits */
#define FS_SEL_SHIFT        3
#define FS_SEL_MASK         (0x3 << FS_SEL_SHIFT)
#define FS_SEL_2000         (3 << FS_SEL_SHIFT)
#define DLPF_CFG_MASK       0x7
#define DLPF_CFG_256HZ      0
#define DLPF_CFG_188HZ      1
#define DLPF_CFG_98HZ       2
#define DLPF_CFG_42HZ       3
#define DLPF_CFG_20HZ       4
#define DLPF_CFG_10HZ       5
#define DLPF_CFG_5HZ        6

/* INT_CFG bits */
#define ACTL                (1 << 7)
#define OPEN                (1 << 6)
#define LATCH_INT_EN        (1 << 5)
#define INT_ANYRD_2CLEAR    (1 << 4)
#define ITG_RDY_EN          (1 << 2)
#define RAW_RDY_EN          (1 << 0)

/* INT_STATUS  bits */
#define ITG_RDY             (1 << 2)
#define RAW_DATA_RDY        (1 << 0)

/* PWR_MGM bits */
#define H_RESET                 (1 << 7)
#define SLEEP                   (1 << 6)
#define STBY_XG                 (1 << 5)
#define STBY_YG                 (1 << 4)
#define STBY_ZG                 (1 << 3)
#define CLK_SEL_MASK            7
#define CLK_SEL_INT             0
#define CLK_SEL_PLL_X           1
#define CLK_SEL_PLL_Y           2
#define CLK_SEL_PLL_Z           3
#define CLK_SEL_PLL_EXT_32768HZ 4
#define CLK_SEL_PLL_EXT_19_2MHZ 5


ITG3200::ITG3200(I2C* pI2C, int address /* = 0xA6 */) : SensorBase(pI2C, address)
{
    initGyro();
}

void ITG3200::initGyro()
{
    do
    {
        writeRegister(PWR_MGM, H_RESET);
        if (m_failedIo)
            break;
        writeRegister(INT_CFG, LATCH_INT_EN | ITG_RDY_EN | RAW_RDY_EN);
        if (m_failedIo)
            break;
        writeRegister(PWR_MGM, CLK_SEL_PLL_X);
        if (m_failedIo)
            break;
        waitForPllReady();
        writeRegister(SMPLRT_DIV, (1000 / 100) - 1);
        if (m_failedIo)
            break;
        writeRegister(DLPF_FS, FS_SEL_2000 | DLPF_CFG_42HZ);
        if (m_failedIo)
            break;
    }
    while (0);

    if (m_failedIo)
        m_failedInit = 1;
}

void ITG3200::waitForPllReady()
{
    char intStatus = 0;

    do
    {
        readRegister(INT_STATUS, &intStatus);
    } while ((intStatus & ITG_RDY) == 0);
}

void ITG3200::getVector(Vector<int16_t>* pVector, int16_t* pTemperature)
{
    char bigEndianDataWithTemp[8];

    readRegisters(TEMP_OUT_H, bigEndianDataWithTemp, sizeof(bigEndianDataWithTemp));
    if (m_failedIo)
        return;

    // Data returned is big endian so byte swap.
    *pTemperature = (bigEndianDataWithTemp[0] << 8) | bigEndianDataWithTemp[1];
    pVector->x = (bigEndianDataWithTemp[2] << 8) | bigEndianDataWithTemp[3];
    pVector->y = (bigEndianDataWithTemp[4] << 8) | bigEndianDataWithTemp[5];
    pVector->z = (bigEndianDataWithTemp[6] << 8) | bigEndianDataWithTemp[7];
}

void ITG3200::waitForDataReady()
{
    char intStatus = 0;

    do
    {
        readRegister(INT_STATUS, &intStatus);
    } while ((intStatus & RAW_DATA_RDY) == 0);
}

