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
/* Test harness for my heading class. */
#include <assert.h>
#include <mbed.h>
#include "ConfigFile.h"
#include "DmaSerial.h"
#include "FlashFileSystem.h"
#include "files.h"
#include "Sparkfun9DoFSensorStick.h"


static volatile bool g_resetRequested = false;

#if !MRI_ENABLE
static DmaSerial g_serial(USBTX, USBRX);
#endif // !MRI_ENABLE


// Function prototypes.
static void serialReceiveISR();
static SensorCalibration readConfigurationFile();

extern Quaternion g_gyro;
extern Matrix4x4  g_A;
extern Quaternion g_initialX;
extern Quaternion g_xPredicted;
extern Matrix4x4  g_initialP;
extern Matrix4x4  g_PPredicted;
extern Matrix4x4  g_K;
extern Quaternion g_z;
extern Matrix4x4  g_P;

static void dumpMatrix(const char* pName, Matrix4x4* pMatrix)
{
    printf("%s = \n", pName);
    for (int row = 0 ; row < 4 ; row++)
    {
        printf("    %.8g,%.8g,%.8g,%.8g\n",
               pMatrix->m_data[row][0],
               pMatrix->m_data[row][1],
               pMatrix->m_data[row][2],
               pMatrix->m_data[row][3]);
    }
}

static void dumpQuaternion(const char* pName, Quaternion* pQuaternion)
{
    printf("%s = ", pName);
    printf("%.8g,%.8g,%.8g,%.8g\n", pQuaternion->w, pQuaternion->x, pQuaternion->y, pQuaternion->z);
}

int main()
{
    static Timer timer;
    timer.start();
#if !MRI_ENABLE
    g_serial.baud(230400);
    g_serial.attach(serialReceiveISR);
#endif // !MRI_ENABLE

    static FlashFileSystem fileSystem("flash", g_fileSystemData);
    if (!fileSystem.IsMounted())
        error("Encountered error mounting FLASH file system.\n");


    static SensorCalibration calibration = readConfigurationFile();
    static Sparkfun9DoFSensorStick sensorStick(p9, p10, &calibration);
    if (sensorStick.didInitFail())
        error("Encountered I2C I/O error during Sparkfun 9DoF Sensor Stick init.\n");

    for (;;)
    {
        char buffer[256];
        int  length;
        bool wasResetRequested = g_resetRequested;

        if (wasResetRequested)
        {
            sensorStick.reset();
            g_resetRequested = false;
        }

        SensorValues sensorValues = sensorStick.getRawSensorValues();
        if (sensorStick.didIoFail())
            error("Encountered I2C I/O error during fetch of Sparkfun 9DoF Sensor Stick readings.\n");
        SensorCalibratedValues calibratedValues = sensorStick.calibrateSensorValues(&sensorValues);
        Quaternion orientation = sensorStick.getOrientation(&calibratedValues);

        int elapsedTime = timer.read_us();
        timer.reset();
        length = snprintf(buffer, sizeof(buffer), "%s%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%f,%f,%f,%f\n",
                          wasResetRequested ? "R," : "",
                          sensorValues.accel.x, sensorValues.accel.y, sensorValues.accel.z,
                          sensorValues.mag.x, sensorValues.mag.y, sensorValues.mag.z,
                          sensorValues.gyro.x, sensorValues.gyro.y, sensorValues.gyro.z,
                          sensorValues.gyroTemperature,
                          elapsedTime,
                          sensorStick.getIdleTimePercent(),
                          orientation.w, orientation.x, orientation.y, orientation.z);
        assert( length < (int)sizeof(buffer) );

#if MRI_ENABLE
        printf("%s", buffer);
#else
        g_serial.dmaTransmit(buffer, length);
#endif

#define DUMP_MATRIX(X) dumpMatrix(#X, &X);
#define DUMP_QUATERNION(X) dumpQuaternion(#X, &X);
        if (wasResetRequested)
        {
            // Must send R again to have these dumped.
            while (!g_resetRequested)
            {
            }

            DUMP_QUATERNION(g_gyro);
            DUMP_MATRIX(g_A);
            DUMP_QUATERNION(g_initialX);
            DUMP_QUATERNION(g_xPredicted);
            DUMP_MATRIX(g_initialP);
            DUMP_MATRIX(g_PPredicted);
            DUMP_MATRIX(g_K);
            DUMP_QUATERNION(g_z);
            DUMP_MATRIX(g_P);

            for(;;)
            {
            }
        }
    }

    return 0;
}

#if !MRI_ENABLE
static void serialReceiveISR()
{
    while (g_serial.readable())
    {
        int byte = g_serial.getc();
        if (byte == 'R')
        {
            g_resetRequested = true;
        }
    }
}
#endif // !MRI_ENABLE

static SensorCalibration readConfigurationFile()
{
    ConfigFile        configFile;
    if (configFile.open("/flash/config.ini"))
        error("Encountered error opening /flash/config.ini\n");

    SensorCalibration calibration;
    if (!configFile.getIntVector("compass.accelerometer.min", &calibration.accelMin))
        error("Failed to read compass.accelerometer.min\n");
    if (!configFile.getIntVector("compass.accelerometer.max", &calibration.accelMax))
        error("Failed to read compass.accelerometer.max\n");
    if (!configFile.getIntVector("compass.magnetometer.min", &calibration.magMin))
        error("Failed to read compass.magnetometer.min\n");
    if (!configFile.getIntVector("compass.magnetometer.max", &calibration.magMax))
        error("Failed to read compass.magnetometer.max\n");
    if (!configFile.getFloatVector("compass.gyro.coefficient.A", &calibration.gyroCoefficientA))
        error("Failed to read compass.gyro.coefficient.A\n");
    if (!configFile.getFloatVector("compass.gyro.coefficient.B", &calibration.gyroCoefficientB))
        error("Failed to read compass.gyro.coefficient.B\n");
    if (!configFile.getFloatVector("compass.gyro.scale", &calibration.gyroScale))
        error("Failed to read compass.gyro.scale\n");
    if (!configFile.getIntVector("compass.accelerometer.swizzle", &calibration.accelSwizzle))
        error("Failed to read compass.accelerometer.swizzle\n");
    if (!configFile.getIntVector("compass.magnetometer.swizzle", &calibration.magSwizzle))
        error("Failed to read compass.magnetometer.swizzle\n");
    if (!configFile.getIntVector("compass.gyro.swizzle", &calibration.gyroSwizzle))
        error("Failed to read compass.gyro.swizzle\n");
    if (!configFile.getFloat("compass.initial.variance", &calibration.initialVariance))
        error("Failed to read compass.initial.variance\n");
    if (!configFile.getFloat("compass.gyro.variance", &calibration.gyroVariance))
        error("Failed to read compass.gyro.variance\n");
    if (!configFile.getFloat("compass.accelerometer.magnetometer.variance", &calibration.accelMagVariance))
        error("Failed to read compass.accelerometer.magnetometer.variance\n");

    return calibration;
}
