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
/* Reads in PID oscillation test for self-integrating process and
   calculates the system model parameters.
*/
#include <stdio.h>
#include <stdint.h>
#include <math.h>


int main(int argc, char**argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: pidCalc filename.csv\n");
        return 1;
    }
    const char* pFilename = argv[1];

    FILE* pFile = fopen(pFilename, "r");
    if (!pFile)
    {
        fprintf(stderr, "error: Failed to open %s\n", pFilename);
        return 1;
    }

    // Skip heading line.
    char buffer[128];
    char* pResult = fgets(buffer, sizeof(buffer), pFile);
    if (!pResult)
    {
        fprintf(stderr, "error: Failed to read header line from %s\n", pFilename);
        return 1;
    }
    printf("%s", buffer);

    enum SlopeState
    {
        SLOPE1_START_SEARCH,
        SLOPE1_END_SEARCH,
        SLOPE2_START_SEARCH,
        SLOPE2_END_SEARCH
    };
    SlopeState slopeState = SLOPE1_START_SEARCH;
    struct Stats
    {
        float      startTime;
        float      startAngle;
        float      sum;
        uint32_t   count;
    };
    Stats slope1 = {0.0f, 0.0f, 0.0f, 0};
    Stats slope2 = {0.0f, 0.0f, 0.0f, 0};
    Stats deadTime = {0.0f, 0.0f, 0.0f, 0};
    int32_t minSpeed = 0x7FFFFFFF;
    int32_t maxSpeed = 0x80000000;

    // Set previous angle to a value low enough that first element will be detected as start of slope1.
    float prevAngle = -100.0f;
    int32_t prevSpeed = 100;
    while (1)
    {
        pResult = fgets(buffer, sizeof(buffer), pFile);
        if (!pResult)
            break;

        uint32_t intTime = 0;
        float    angle = 0.0f;
        int32_t  speed = 0;
        sscanf(buffer, "%u,%f,%d", &intTime, &angle, &speed);

        float time = (float)intTime / 1000.0f;
        angle = angle * M_PI / 180.0f;
        printf("%.3f,%f,%d\n", time, angle, speed);

        // Track the min and maximum speed used during test.
        if (speed < minSpeed)
            minSpeed = speed;
        if (speed > maxSpeed)
            maxSpeed = speed;

        // Find the ends of slope1 & slope2.
        switch (slopeState)
        {
        case SLOPE1_START_SEARCH:
            if (angle > prevAngle)
            {
                slope1.startTime = time;
                slope1.startAngle = angle;
                if (deadTime.startTime != 0.0f)
                {
                    deadTime.sum += time - deadTime.startTime;
                    deadTime.count++;
                    printf("deadTime: %f\n", time - deadTime.startTime);
                }
                slopeState = SLOPE1_END_SEARCH;
                printf("slope1 start\n");
            }
            break;
        case SLOPE1_END_SEARCH:
            if (prevSpeed > speed)
            {
                slope1.sum += (angle - slope1.startAngle) / (time - slope1.startTime);
                slope1.count++;
                deadTime.startTime = time;
                slopeState = SLOPE2_START_SEARCH;
                printf("slope1 end: %f\n", (angle - slope1.startAngle) / (time - slope1.startTime));
            }
            break;
        case SLOPE2_START_SEARCH:
            if (angle < prevAngle)
            {
                slope2.startTime = time;
                slope2.startAngle = angle;
                if (deadTime.startTime != 0.0f)
                {
                    deadTime.sum += time - deadTime.startTime;
                    deadTime.count++;
                    printf("deadTime: %f\n", time - deadTime.startTime);
                }
                slopeState = SLOPE2_END_SEARCH;
                printf("slope2 start\n");
            }
            break;
        case SLOPE2_END_SEARCH:
            if (prevSpeed < speed)
            {
                slope2.sum += (angle - slope2.startAngle) / (time - slope2.startTime);
                slope2.count++;
                deadTime.startTime = time;
                slopeState = SLOPE1_START_SEARCH;
                printf("slope2 end: %f\n", (angle - slope2.startAngle) / (time - slope2.startTime));
            }
            break;
        }
        prevAngle = angle;
        prevSpeed = speed;
    }

    float slope1Val = slope1.sum / (float)slope1.count;
    float slope2Val = slope2.sum / (float)slope2.count;
    float deadTimeVal = deadTime.sum / (float)deadTime.count;
    int32_t CO1 = maxSpeed;
    int32_t CO2 = minSpeed;
    float Kp_star = (slope2Val - slope1Val) / (float)(CO2 - CO1);
    float Tc = 3.0f * deadTimeVal;
    float Kc = (1.0f / Kp_star) * ((2 * Tc + deadTimeVal) / powf(Tc + deadTimeVal, 2.0f));
    float Ti = 2 * Tc + deadTimeVal;

    printf("\n\n==Stats==\n");
    printf("slope1 = %f\n", slope1Val);
    printf("slope2 = %f\n", slope2Val);
    printf("CO1 = %d\n", CO1);
    printf("CO2 = %d\n", CO2);
    printf("Kp* = %f\n", Kp_star);
    printf("Өp = %f\n", deadTimeVal);
    printf("Tc = %f\n", Tc);
    printf("Kc = %f\n", Kc);
    printf("Ti = %f\n", Ti);

    return 0;
}
