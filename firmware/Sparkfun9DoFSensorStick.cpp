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
#include <math.h>
#include "Sparkfun9DoFSensorStick.h"


Sparkfun9DoFSensorStick::Sparkfun9DoFSensorStick(PinName sdaPin,
                                                 PinName sclPin,
                                                 const SensorCalibration* pCalibration /* = NULL */) :
    m_i2c(sdaPin, sclPin),
    m_accel(&m_i2c),
    m_mag(&m_i2c),
    m_gyro(&m_i2c)
{
    m_i2c.frequency(400000);
    m_failedIo = 0;
    m_failedInit = 0;
    m_currentSample = 0;
    m_lastSample = 0;
    calibrate(pCalibration);

    m_failedInit = m_accel.didInitFail() || m_mag.didInitFail() || m_gyro.didInitFail();
    m_failedIo = m_failedInit;
    if (!m_failedInit)
        m_ticker.attach_us(this, &Sparkfun9DoFSensorStick::tickHandler, 1000000 / 100);

    m_idleTimePercent = 0.0f;
    m_totalTimer.start();
    m_idleTimer.start();
}

void Sparkfun9DoFSensorStick::calibrate(const SensorCalibration* pCalibration)
{
    if (!pCalibration)
        return;
    m_calibration = *pCalibration;

    m_midpoints.accel.x = (m_calibration.accelMin.x + m_calibration.accelMax.x) / 2.0f;
    m_midpoints.accel.y = (m_calibration.accelMin.y + m_calibration.accelMax.y) / 2.0f;
    m_midpoints.accel.z = (m_calibration.accelMin.z + m_calibration.accelMax.z) / 2.0f;
    m_midpoints.mag.x = (m_calibration.magMin.x + m_calibration.magMax.x) / 2.0f;
    m_midpoints.mag.y = (m_calibration.magMin.y + m_calibration.magMax.y) / 2.0f;
    m_midpoints.mag.z = (m_calibration.magMin.z + m_calibration.magMax.z) / 2.0f;

    m_scales.accel.x = (m_calibration.accelMax.x - m_calibration.accelMin.x) / 2.0f;
    m_scales.accel.y = (m_calibration.accelMax.y - m_calibration.accelMin.y) / 2.0f;
    m_scales.accel.z = (m_calibration.accelMax.z - m_calibration.accelMin.z) / 2.0f,
    m_scales.mag.x = (m_calibration.magMax.x - m_calibration.magMin.x) / 2.0f;
    m_scales.mag.y = (m_calibration.magMax.y - m_calibration.magMin.y) / 2.0f;
    m_scales.mag.z = (m_calibration.magMax.z - m_calibration.magMin.z) / 2.0f;

    // Convert gyro scale to radians and pre-divide to improve performance at runtime.
    m_calibration.gyroScale.x = ((1.0f / m_calibration.gyroScale.x) * (float)M_PI) / 180.0f;
    m_calibration.gyroScale.y = ((1.0f / m_calibration.gyroScale.y) * (float)M_PI) / 180.0f;
    m_calibration.gyroScale.z = ((1.0f / m_calibration.gyroScale.z) * (float)M_PI) / 180.0f;
}


void Sparkfun9DoFSensorStick::tickHandler()
{
    // Assume I/O failed unless we complete everything successfully and then clear this flag.
    int failedIo = 1;

    do
    {
        m_accel.getVector(&m_sensorValues.accel);
        if (m_accel.didIoFail())
            break;

        m_mag.getVector(&m_sensorValues.mag);
        if (m_mag.didIoFail())
            break;

        m_gyro.getVector(&m_sensorValues.gyro, &m_sensorValues.gyroTemperature);
        if (m_gyro.didIoFail())
            break;

        m_currentSample++;

        // If we got here then all reads were successful.
        failedIo = 0;
    } while (0);

    m_failedIo = failedIo;
}


SensorValues Sparkfun9DoFSensorStick::getRawSensorValues()
{
    uint32_t     currentSample;
    SensorValues sensorValues;

    // Wait for next sample to become available.
    m_idleTimer.reset();
    do
    {
        currentSample = m_currentSample;
    } while (currentSample == m_lastSample);
    m_lastSample = currentSample;

    m_idleTimePercent = ((float)m_idleTimer.read_us() * 100.0f) / (float)m_totalTimer.read_us();
    m_totalTimer.reset();

    __disable_irq();
        memcpy(&sensorValues, &m_sensorValues, sizeof(sensorValues));
    __enable_irq();

    return sensorValues;
}

SensorCalibratedValues Sparkfun9DoFSensorStick::calibrateSensorValues(const SensorValues* pRawValues)
{
    SensorCalibratedValues calibratedValues;

    calibratedValues.accel.x = (pRawValues->accel.x - m_midpoints.accel.x) / m_scales.accel.x;
    calibratedValues.accel.y = (pRawValues->accel.y - m_midpoints.accel.y) / m_scales.accel.y;
    calibratedValues.accel.z = (pRawValues->accel.z - m_midpoints.accel.z) / m_scales.accel.z;

    calibratedValues.mag.x = (pRawValues->mag.x - m_midpoints.mag.x) / m_scales.mag.x;
    calibratedValues.mag.y = (pRawValues->mag.y - m_midpoints.mag.y) / m_scales.mag.y;
    calibratedValues.mag.z = (pRawValues->mag.z - m_midpoints.mag.z) / m_scales.mag.z;

    calibratedValues.gyro.x = pRawValues->gyro.x - (pRawValues->gyroTemperature * m_calibration.gyroCoefficientA.x + m_calibration.gyroCoefficientB.x);
    calibratedValues.gyro.y = pRawValues->gyro.y - (pRawValues->gyroTemperature * m_calibration.gyroCoefficientA.y + m_calibration.gyroCoefficientB.y);
    calibratedValues.gyro.z = pRawValues->gyro.z - (pRawValues->gyroTemperature * m_calibration.gyroCoefficientA.z + m_calibration.gyroCoefficientB.z);

    calibratedValues.gyro.x *= m_calibration.gyroScale.x;
    calibratedValues.gyro.y *= m_calibration.gyroScale.y;
    calibratedValues.gyro.z *= m_calibration.gyroScale.z;

    return calibratedValues;
}

Quaternion Sparkfun9DoFSensorStick::getOrientation(SensorCalibratedValues* pCalibratedValues)
{
    // Setup gravity (down) and north vectors.
    // UNDONE: Swizzling should be controlled by config.ini
    // NOTE: The fields are swizzled to make the axis on the device match the axis on the screen.
    Vector<float> down(pCalibratedValues->accel.y, pCalibratedValues->accel.z, pCalibratedValues->accel.x);
    Vector<float> north(-pCalibratedValues->mag.x, pCalibratedValues->mag.z, pCalibratedValues->mag.y);

    // Project the north vector onto the earth surface plane, for which gravity is the surface normal.
    //  north.dotProduct(downNormalized) = north.magnitude * cos(theta)  NOTE: downNormalized.magnitude = 1.0f
    //   The result of this dot product is the length of the north vector when projected onto the gravity vector since
    //   the magnitude of the north vector is the hypotenuse of a right angle triangle and the unit gravity vector
    //   is the side adjacent to angle theta (the angle between gravity and north vectors).
    //  northProjectedToGravityNormal = downNormalized.multiply(north.dotProduct(downNormalized)
    //   Multiply the unit gravity vector by the magnitude previously calculated to get the vector representing the
    //   north vector after it has been projected onto the gravity vector.
    //  north = north.subtract(northProjectedToGravityNormal)
    //   Follow this projected vector down the surface normal to the plane representing the earth's surface.
    down.normalize();
    Vector<float> northProjectedToGravityNormal = down.multiply(north.dotProduct(down));
    north = north.subtract(northProjectedToGravityNormal);
    north.normalize();

    // To create a rotation matrix, we need all 3 basis vectors so calculate the vector which
    // is orthogonal to both the down and north vectors (ie. the normalized cross product).
    Vector<float> west = north.crossProduct(down);
    west.normalize();
    Quaternion rotationQuaternion = Quaternion::createFromBasisVectors(north, down, west);

    return rotationQuaternion;
}
