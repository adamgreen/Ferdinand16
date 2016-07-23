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
#include "HMC5883L.h"


/* Magnetometer I2C Registers */
#define CONFIG_A            0x00
#define CONFIG_B            0x01
#define MODE                0x02
#define DATA_OUT_X_MSB      0x03
#define DATA_OUT_X_LSB      0x04
#define DATA_OUT_Z_MSB      0x05
#define DATA_OUT_Z_LSB      0x06
#define DATA_OUT_Y_MSB      0x07
#define DATA_OUT_Y_LSB      0x08
#define STATUS              0x09
#define ID_A                0x0A
#define ID_B                0x0B
#define ID_C                0x0C

/* CONFIG_A bits */
#define SAMPLES_SHIFT           5
#define SAMPLES_MASK            (3 << SAMPLES_SHIFT)
#define SAMPLES_1               (0 << SAMPLES_SHIFT)
#define SAMPLES_2               (1 << SAMPLES_SHIFT)
#define SAMPLES_4               (2 << SAMPLES_SHIFT)
#define SAMPLES_8               (3 << SAMPLES_SHIFT)
#define RATE_SHIFT              2
#define RATE_MASK               (7 << RATE_SHIFT)
#define RATE_0_75               (0 << RATE_SHIFT)
#define RATE_1_5                (1 << RATE_SHIFT)
#define RATE_3                  (2 << RATE_SHIFT)
#define RATE_7_5                (3 << RATE_SHIFT)
#define RATE_15                 (4 << RATE_SHIFT)
#define RATE_30                 (5 << RATE_SHIFT)
#define RATE_75                 (6 << RATE_SHIFT)
#define MEASUREMENT_MASK        3
#define MEASUREMENT_NORMAL      0
#define MEASUREMENT_POS_BIAS    1
#define MEASUREMENT_NEG_BIAS    2

/* CONFIG_B bits */
#define GAIN_SHIFT              5
#define GAIN_MASK               (7 << GAIN_SHIFT)
#define GAIN_0_88GA             (0 << GAIN_SHIFT)
#define GAIN_1_3GA              (1 << GAIN_SHIFT)
#define GAIN_1_9GA              (2 << GAIN_SHIFT)
#define GAIN_2_5GA              (3 << GAIN_SHIFT)
#define GAIN_4_0GA              (4 << GAIN_SHIFT)
#define GAIN_4_7GA              (5 << GAIN_SHIFT)
#define GAIN_5_6GA              (6 << GAIN_SHIFT)
#define GAIN_8_1GA              (7 << GAIN_SHIFT)

/* MODE bits */
#define MODE_MASK               3
#define MODE_CONTINUOUS         0
#define MODE_SINGLE             1
#define MODE_IDLE1              2
#define MODE_IDLE2              3

/* STATUS bits */
#define STATUS_LOCK             (1 << 1)
#define STATUS_RDY              (1 << 0)

HMC5883L::HMC5883L(I2C* pI2C, int address /* = 0x3C */) : SensorBase(pI2C, address)
{
    initMagnetometer();
}

void HMC5883L::initMagnetometer()
{
    do
    {
        writeRegister(CONFIG_A, SAMPLES_8 | MEASUREMENT_NORMAL);
        if (m_failedIo)
            break;
        writeRegister(CONFIG_B, GAIN_1_3GA);
        if (m_failedIo)
            break;
        // Prime the read by issuing the first single shot read.
        writeRegister(MODE, MODE_SINGLE);
        if (m_failedIo)
            break;
    }
    while (0);

    if (m_failedIo)
        m_failedInit = 1;
}

void HMC5883L::getVector(Vector<int16_t>* pVector)
{
    uint8_t            bigEndianData[6];

    readRegisters(DATA_OUT_X_MSB, &bigEndianData, sizeof(bigEndianData));
    if (m_failedIo)
        return;

    // Data returned from sensor is in big endian byte order with an axis order of X, Z, Y
    pVector->x = (bigEndianData[0] << 8) | bigEndianData[1];
    pVector->z = (bigEndianData[2] << 8) | bigEndianData[3];
    pVector->y = (bigEndianData[4] << 8) | bigEndianData[5];

    // Prime for the next read by issuing the next single shot read.
    writeRegister(MODE, MODE_SINGLE);
}
