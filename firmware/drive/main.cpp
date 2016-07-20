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
/* Test harness for controlling the Parallax DHB-10 Dual Motor Controller
   with the LPC1768. */
#include <mbed.h>
#include <MotorController.h>


MotorController g_motorController(p28, p27);
Timer           g_timer;


int main()
{
    EncoderCounts lastEncoderCounts = {0, 0};

    int nextSendTime = 0;
    g_timer.start();
    while (g_timer.read_ms() < 2000)
    {
        // Only want to interact with the motor controller at 50Hz.
        while (g_timer.read_ms() - nextSendTime < 0)
        {
        }

        g_motorController.setWheelSpeeds(100, 100);
        EncoderCounts currEncoderCounts = g_motorController.getEncoderCounts();
        if (nextSendTime > 500)
        {
            int32_t leftDelta = currEncoderCounts.left - lastEncoderCounts.left;
            int32_t rightDelta = currEncoderCounts.right - lastEncoderCounts.right;
            if (leftDelta <= 0 || rightDelta <= 0)
            {
                g_motorController.setWheelSpeeds(0, 0);
                g_motorController.txFlush();

                fprintf(stderr, "error: Encoder counts didn't increase as expected. %ld, %ld\n", leftDelta, rightDelta);
                exit(-1);
            }
        }
        lastEncoderCounts = currEncoderCounts;
        nextSendTime += 20;
    }

    // Stop the motors.
    g_motorController.setWheelSpeeds(0, 0);
    g_motorController.txFlush();

    return 0;
}
