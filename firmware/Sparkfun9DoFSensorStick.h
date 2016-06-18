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
#ifndef SPARKFUN_9DOF_SENSOR_STICK_H_
#define SPARKFUN_9DOF_SENSOR_STICK_H_

#include <mbed.h>
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"
#include "Quaternion.h"
#include "Vector.h"

typedef struct SensorValues
{
    Vector<int16_t> accel;
    Vector<int16_t> mag;
    Vector<int16_t> gyro;
    int16_t         gyroTemperature;
} SensorValues;

typedef struct SensorCalibratedValues
{
    Vector<float> accel;
    Vector<float> mag;
    Vector<float> gyro;
} SensorCalibratedValues;

typedef struct SensorCalibration
{
  Vector<int16_t>   accelMin;
  Vector<int16_t>   accelMax;
  Vector<int16_t>   magMin;
  Vector<int16_t>   magMax;
  Vector<float>     gyroCoefficientA;
  Vector<float>     gyroCoefficientB;
  Vector<float>     gyroScale;
} SensorCalibration;


class Sparkfun9DoFSensorStick
{
public:
    Sparkfun9DoFSensorStick(PinName sdaPin, PinName sclPin, const SensorCalibration* pCalibration = NULL);

    void calibrate(const SensorCalibration* pCalibration);

    SensorValues           getRawSensorValues();
    SensorCalibratedValues calibrateSensorValues(const SensorValues* pRawValues);
    Quaternion             getOrientation(SensorCalibratedValues* pCalibratedValues);

    int didInitFail() { return m_failedInit; }
    int didIoFail() { return m_failedIo; }
    float getIdleTimePercent() { return m_idleTimePercent; }

protected:
    void tickHandler();

    Timer                   m_totalTimer;
    Timer                   m_idleTimer;
    SensorCalibration       m_calibration;
    Ticker                  m_ticker;
    I2C                     m_i2c;
    ADXL345                 m_accel;
    HMC5883L                m_mag;
    ITG3200                 m_gyro;
    SensorCalibratedValues  m_midpoints;
    SensorCalibratedValues  m_scales;
    float                   m_idleTimePercent;
    volatile int            m_failedInit;
    volatile int            m_failedIo;
    volatile uint32_t       m_currentSample;
    uint32_t                m_lastSample;
    SensorValues            m_sensorValues;

};

#endif /* SPARKFUN_9DOF_SENSOR_STICK_H_ */
