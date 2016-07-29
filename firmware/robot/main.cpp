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
/* Main code for my Robo-Magellan robot. */
#include <assert.h>
#include <mbed.h>
#include <mri.h>
#include <ConfigFile.h>
#include <DebugSerial.h>
#include <DmaSerial.h>
#include <FlashFileSystem.h>
#include <MotorController.h>
#include <Sparkfun9DoFSensorStick.h>
#include "files.h"


// The robot can be running in one of these modes.
enum RunMode
{
    // Reset the Kalman filter, etc. Then switch to running mode.
    RUN_MODE_RESET,
    // Running the main robot behaviour loop.
    RUN_MODE_RUNNING,
    // Enter a debug mode that communicates with user through GDB via semi-hosting API.
    RUN_MODE_DEBUG,
    // Enter a calibration routine which rotates back and forth for PID model calculation.
    RUN_MODE_CALIBRATE
};

static RunMode          g_runMode = RUN_MODE_CALIBRATE;
static bool             g_haveGlobalConstructorsRun = false;
static MotorController  g_motorController(p28, p27);
static DebugSerial<256> g_debugSerial;
static int              g_debug = 0;
static Timer            g_motorUpdateTimer;

// Function prototypes.
static SensorCalibration readConfigurationFile();
static void              runMode(Sparkfun9DoFSensorStick* pSensorStick);
static void              readOrientationAndHeadingAngle(Sparkfun9DoFSensorStick* pSensorStick,
                                                        Quaternion* pOrientation,
                                                        float* pHeadingAngle);
static void              debugMode(Sparkfun9DoFSensorStick* pSensorStick);
static void              calibrateMode(Sparkfun9DoFSensorStick* pSensorStick);
static void              stopMotors();


int main()
{
    g_haveGlobalConstructorsRun = true;

    static FlashFileSystem fileSystem("flash", g_fileSystemData);
    if (!fileSystem.IsMounted())
        error("Encountered error mounting FLASH file system.\n");

    static SensorCalibration calibration = readConfigurationFile();
    static Sparkfun9DoFSensorStick sensorStick(p9, p10, &calibration);
    if (sensorStick.didInitFail())
        error("Encountered I2C I/O error during Sparkfun 9DoF Sensor Stick init.\n");

    g_motorUpdateTimer.start();
    for (;;)
    {
        if (g_debug)
            g_runMode = RUN_MODE_DEBUG;

        switch (g_runMode)
        {
        case RUN_MODE_RESET:
            sensorStick.reset();
            g_motorUpdateTimer.reset();
            g_runMode = RUN_MODE_RUNNING;
            break;
        case RUN_MODE_RUNNING:
            runMode(&sensorStick);
            break;
        case RUN_MODE_DEBUG:
            debugMode(&sensorStick);
            g_debug = 0;
            break;
        case RUN_MODE_CALIBRATE:
            calibrateMode(&sensorStick);
            break;
        }
    }

    return 0;
}

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
    if (!configFile.getFloatVector("compass.declinationCorrection", &calibration.declinationCorrection))
        error("Failed to read compass.declinationCorrection\n");
    if (!configFile.getFloatVector("compass.mountingCorrection", &calibration.mountingCorrection))
        error("Failed to read compass.mountingCorrection\n");

    return calibration;
}

static void runMode(Sparkfun9DoFSensorStick* pSensorStick)
{
    // Update motors at 50Hz (20 msec period).
    if (g_motorUpdateTimer.read_ms() >= 20)
    {
        g_motorUpdateTimer.reset();
        g_motorController.setWheelSpeeds(100, 100);
    }

    Quaternion orientation;
    float headingAngle;
    readOrientationAndHeadingAngle(pSensorStick, &orientation, &headingAngle);

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%f,%.1f\n", headingAngle * 180.0f / M_PI, pSensorStick->getIdleTimePercent());
    g_debugSerial.outputStringToGdb(buffer);
}

static void readOrientationAndHeadingAngle(Sparkfun9DoFSensorStick* pSensorStick,
                                           Quaternion* pOrientation,
                                           float* pHeadingAngle)
{
    SensorValues sensorValues = pSensorStick->getRawSensorValues();
    if (pSensorStick->didIoFail())
        error("Encountered I2C I/O error during fetch of Sparkfun 9DoF Sensor Stick readings.\n");
    SensorCalibratedValues calibratedValues =  pSensorStick->calibrateSensorValues(&sensorValues);
    *pOrientation =  pSensorStick->getOrientation(&calibratedValues);
    *pHeadingAngle =  pSensorStick->getHeading(pOrientation);
}

static void debugMode(Sparkfun9DoFSensorStick* pSensorStick)
{
    stopMotors();

    printf("debug: ");
    char buffer[32];
    fgets(buffer, sizeof(buffer), stdin);
    printf("echo: %s", buffer);

    g_runMode = RUN_MODE_RESET;
}

// Time to allow the Kalman filter to stabilize before running the PID calibration routine, in milliseconds.
#define CALIBRATE_STABILIZE_TIME 15000
// During calibration, oscillate between two limits (clockwise and counterclockwise) which are this far from original
// heading at the start of the test. This angle is in degrees.
#define CALIBRATE_ANGLE 45
// Motor speed to use during oscillations. This is in encoder ticks per second.
#define CALIBRATE_SPEED 100
// Number of rotation oscillations to perform during test.
#define CALIBRATE_OSCILLATION_ITERATIONS 10

static void calibrateMode(Sparkfun9DoFSensorStick* pSensorStick)
{
    float      headingAngle = 0.0f;
    Quaternion orientation;
    char       buffer[128];

    // Reset and allow the Kalman filter to stabilize.
    g_debugSerial.outputStringToGdb("Starting PID calibration routine...\n");
    Timer timer;
    timer.start();
    pSensorStick->reset();
    g_debugSerial.outputStringToGdb("Stabilizing the Kalman filter...\n");
    while (timer.read_ms() < CALIBRATE_STABILIZE_TIME)
    {
        readOrientationAndHeadingAngle(pSensorStick, &orientation, &headingAngle);
        snprintf(buffer, sizeof(buffer), "%f\n", headingAngle * 180.0f / M_PI);
        g_debugSerial.outputStringToGdb(buffer);
    }

    // Calculate angle limits for oscillation based on current heading Angle.
    float minAngle = Sparkfun9DoFSensorStick::constrainAngle(headingAngle - (CALIBRATE_ANGLE * M_PI / 180.0f));
    float maxAngle = Sparkfun9DoFSensorStick::constrainAngle(headingAngle + (CALIBRATE_ANGLE * M_PI / 180.0f));
    enum Direction
    {
        CLOCKWISE,
        COUNTERCLOCKWISE
    } direction = CLOCKWISE;

    // Run oscillating rotations.
    snprintf(buffer, sizeof(buffer), "Starting oscillations between %f and %f...\n", minAngle * 180.0f / M_PI,
                                                                                     maxAngle * 180.0f / M_PI);
    g_debugSerial.outputStringToGdb(buffer);
    uint32_t oscillationCount = 0;
    uint32_t sampleCount = 0;
    timer.reset();
    while (oscillationCount < CALIBRATE_OSCILLATION_ITERATIONS)
    {
        // Allow Kalman filter to run at 100Hz.
        readOrientationAndHeadingAngle(pSensorStick, &orientation, &headingAngle);

        // Only need to do anything with the data at 50Hz (1/2 Kalman rate) when we can actually
        // update the motor controller.
        sampleCount++;
        if (sampleCount & 1)
            continue;

        float angleDelta;
        switch (direction)
        {
        case CLOCKWISE:
            // Keep rotating if delta is >= 90 degrees. Once below 90, only rotate while delta is negative.
            angleDelta = Sparkfun9DoFSensorStick::constrainAngle(headingAngle - maxAngle);
            if (fabsf(angleDelta) < M_PI/2 && angleDelta >= 0.0f)
            {
                direction = COUNTERCLOCKWISE;
            }
            break;
        case COUNTERCLOCKWISE:
            // Keep rotating if delta is >= 90 degrees. Once below 90, only rotate while delta is positive.
            angleDelta = Sparkfun9DoFSensorStick::constrainAngle(headingAngle - minAngle);
            if (fabsf(angleDelta) < M_PI/2 && angleDelta <= 0.0f)
            {
                direction = CLOCKWISE;
                oscillationCount++;
            }
            break;
        }

        int32_t speedDelta = 0;
        switch (direction)
        {
        case CLOCKWISE:
            speedDelta = CALIBRATE_SPEED;
            break;
        case COUNTERCLOCKWISE:
            speedDelta = -CALIBRATE_SPEED;
            break;
        }
        int32_t leftSpeed = speedDelta;
        int32_t rightSpeed = -speedDelta;
        g_motorController.setWheelSpeeds(leftSpeed, rightSpeed);

        snprintf(buffer, sizeof(buffer), "%u,%f,%ld\n", timer.read_ms(), headingAngle * 180.0f / M_PI, speedDelta);
        g_debugSerial.outputStringToGdb(buffer);
    }
    g_debugSerial.outputStringToGdb("PID Calibration completed.\n");

    // Enter debug mode after leaving calibration mode.
    __debugbreak();
    g_debug = 1;
}


/* These are hooks that get called by the MRI debug monitor whenever it takes/releases control of the CPU.
   The implementation of this code makes sure that the motors are turned off when entering debug monitor control and
   then allows the existing main loop code to recover once it starts running again. */
extern "C" void __mriPlatform_EnteringDebuggerHook(void)
{
    // Don't do anything until the global constructors have been executed to setup the motor and debugger serial port.
    if (!g_haveGlobalConstructorsRun)
        return;

    // Allow any outbound DMA traffic on debug serial port to complete before passing control to MRI.
    g_debugSerial.flush();
    stopMotors();
}

static void stopMotors()
{
    char stopMotorsCommand[] = "GOSPD 0 0\r";

    // Issue dmaTransmit() directly rather than calling setWheelSpeeds() since that API uses snprintf() which isn't
    // safe to call from an ISR (it makes use of dynamic allocations).
    g_motorController.dmaTransmit(stopMotorsCommand, sizeof(stopMotorsCommand) - 1);
    g_motorController.txFlush();
}

extern "C" void __mriPlatform_LeavingDebuggerHook(void)
{
    // Just let the main loop recover and startup the motors again.
    // Clear any interrupts that might have been marked as pending against the debug serial port while MRI was
    // communicating with GDB.
    g_debugSerial.clearPendingInterrupt();
}
