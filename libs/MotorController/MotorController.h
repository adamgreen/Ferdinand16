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
/* Class for driving the Arlo Robot Platform at specified speeds. */
#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <assert.h>
#include <DmaSerial.h>

// Timeout values used as MotorController::getResponse() timeoutInMilliseconds parameter.
#define INFINITE_TIMEOUT (~0UL)
#define TWENTY_MILLISECOND_TIMEOUT 20


struct EncoderCounts
{
    uint32_t left;
    uint32_t right;
};

class MotorController : public DmaSerial
{
public:
    MotorController(PinName txPin, PinName rxPin);

    void          setWheelSpeeds(int32_t left, int32_t right);
    EncoderCounts getEncoderCounts();
    uint32_t      getErrorCount() { return m_errorCount; }

protected:
    void sendCommand(char* pCommand);
    int  getResponse(char* pResponse, size_t responseSize, uint32_t timeoutInMilliseconds);
    int  getNextByteWithTimeout(uint32_t timeoutInMilliseconds);
    void serialReceiveHandler();

    char*                  m_pResponseCurr;
    char*                  m_pResponseEnd;
    Timer                  m_timer;
    volatile EncoderCounts m_encoderCounts;
    char                   m_command[32];
    char                   m_response[32];
    uint32_t               m_errorCount;
};

#endif // MOTOR_CONTROLLER_H_
