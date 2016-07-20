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
#include "MotorController.h"


MotorController::MotorController(PinName txPin, PinName rxPin) : DmaSerial(txPin, rxPin)
{
    m_errorCount = ~0UL;
    memset((void*)&m_encoderCounts, 0, sizeof(m_encoderCounts));

    m_timer.start();
    baud(19200);
    strcpy(m_command, "TXPIN CH2\r");
    memset(m_response, 0, sizeof(m_response));
    sendCommand(m_command);
    int responseCode = getResponse(m_response, sizeof(m_response), TWENTY_MILLISECOND_TIMEOUT);
    if (responseCode == -1 || m_response[0] != '\0')
    {
        fprintf(stderr, "error: Failed switching to full duplex mode. (%d) %s\n", responseCode, m_response);
        exit(-1);
    }

    m_pResponseCurr = m_response;
    m_pResponseEnd = m_response + sizeof(m_response);
    attach<MotorController>(this, &MotorController::serialReceiveHandler);
}

void MotorController::setWheelSpeeds(int32_t left, int32_t right)
{
    // Wait for previous async transmit to complete before modifying the buffer.
    txFlush();
    snprintf(m_command, sizeof(m_command), "GOSPD %ld %ld\rDIST\r", left, right);
    sendCommand(m_command);
}

EncoderCounts MotorController::getEncoderCounts()
{
    EncoderCounts encoderCounts;

    __disable_irq();
        memcpy(&encoderCounts, (const void*)&m_encoderCounts, sizeof(encoderCounts));
    __enable_irq();

    return encoderCounts;
}


void MotorController::sendCommand(char* pCommand)
{
    size_t length = strlen(pCommand);
    dmaTransmit(pCommand, length);
}

int MotorController::getResponse(char* pResponse, size_t responseSize, uint32_t timeoutInMilliseconds)
{
    char* pEnd = pResponse + responseSize - 1;

    assert ( responseSize > 0 );
    m_timer.reset();
    while (pResponse < pEnd)
    {
        int byte = getNextByteWithTimeout(timeoutInMilliseconds);
        if (byte == -1)
            return -1;
        if (byte == '\r')
            break;
        *pResponse++ = byte;
    }
    *pResponse = '\0';
    return 0;
}

int MotorController::getNextByteWithTimeout(uint32_t timeoutInMilliseconds)
{
    while (timeoutInMilliseconds == INFINITE_TIMEOUT || (uint32_t)m_timer.read_ms() < timeoutInMilliseconds)
    {
        if (readable())
            return getc();
    }

    // Get here if we have timed out.
    return -1;
}

void MotorController::serialReceiveHandler()
{
    while (readable())
    {
        char byte = getc();
        *m_pResponseCurr++ = byte;
        if (m_pResponseCurr >= m_pResponseEnd)
        {
            // This response overflowed the buffer so reset and count it as an error.
            m_errorCount++;
            m_pResponseCurr = m_response;
        }
        else if (byte == '\r')
        {
            // A complete response has just completed so parse it.
            m_pResponseCurr = m_response;
            if (m_response[0] != '\r')
            {
                char* pCurr = m_response;
                m_encoderCounts.left = strtoul(pCurr, &pCurr, 10);
                m_encoderCounts.right = strtoul(pCurr, &pCurr, 10);
            }
        }
    }
}
