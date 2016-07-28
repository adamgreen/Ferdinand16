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
#ifndef DEBUG_SERIAL_H_
#define DEBUG_SERIAL_H_

#include <assert.h>
#include <mbed.h>
#include <mri.h>
#include <GPDMA.h>
#include <Interlocked.h>


template <uint32_t BUFFER_SIZE>
class DebugSerial
{
public:
    DebugSerial();

    void outputStringToGdb(const char* pOutput);
    void flush();
    void clearPendingInterrupt();

protected:
    void findMriUart(void);
    bool isInterruptEnabled(IRQn_Type IRQn);
    void hijackMriSerialInterrupt();
    static void serialReceiveHandler() { s_pDebug->serialReceive(); };
    void serialReceive();
    void dmaTransmit(const void* pData, size_t dataLength);

    static DebugSerial*     s_pDebug;
    LPC_UART_TypeDef*       m_pSerial;
    LPC_GPDMACH_TypeDef*    m_pChannel;
    uint32_t                m_channelMask;
    volatile uint32_t       m_acksExpected;
    int                     m_serialIndex;
    bool                    m_transferInProgress;
    char                    m_buffer[BUFFER_SIZE];
};

template <uint32_t BUFFER_SIZE>
DebugSerial<BUFFER_SIZE>* DebugSerial<BUFFER_SIZE>::s_pDebug = NULL;


template <uint32_t BUFFER_SIZE>
DebugSerial<BUFFER_SIZE>::DebugSerial()
{
    static const uint32_t fifoEnable = (1 << 0);
    static const uint32_t dmaEnable = (1 << 3);

    m_acksExpected = 0;

    findMriUart();
    hijackMriSerialInterrupt();

    // Allocate a GPDMA channel to use for this UART.
    int channelIndex = allocateDmaChannel(GPDMA_CHANNEL_LOW);
    assert ( channelIndex != -1 );
    m_pChannel = dmaChannelFromIndex(channelIndex);
    m_channelMask = 1 << channelIndex;

    // Enable DMA for UART.
    m_pSerial->FCR = fifoEnable | dmaEnable;
    m_transferInProgress = false;

    // Make sure that GPDMA block is enabled.
    enableGpdmaPower();
    enableGpdmaInLittleEndianMode();

    // Make sure that GPDMA is configured for UART and not TimerMatch.
    LPC_SC->DMAREQSEL &= ~(3 << (m_serialIndex * 2));
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::findMriUart(void)
{
    static LPC_UART_TypeDef* const uartConfigurations[] =
    {
        (LPC_UART_TypeDef*)LPC_UART0,
        (LPC_UART_TypeDef*)LPC_UART1,
        (LPC_UART_TypeDef*)LPC_UART2,
        (LPC_UART_TypeDef*)LPC_UART3
    };

    // Default to UART0 if not able to find it below.
    m_serialIndex = 0;

    // Find UART being used by MRI which is the only UART interrupt which should be enabled at priority level 0.
    for (int i = 0 ; i < 4 ; i++)
    {
        IRQn irq = (IRQn)(UART0_IRQn + i);
        if (NVIC_GetPriority(irq) == 0 && isInterruptEnabled(irq))
        {
            m_serialIndex = i;
            break;
        }
   }
    m_pSerial = uartConfigurations[m_serialIndex];
}

template <uint32_t BUFFER_SIZE>
bool DebugSerial<BUFFER_SIZE>::isInterruptEnabled(IRQn_Type IRQn)
{
    return NVIC->ISER[((uint32_t)(IRQn) >> 5)] & (1 << ((uint32_t)(IRQn) & 0x1F));
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::hijackMriSerialInterrupt()
{
    assert ( !s_pDebug );
    s_pDebug = this;

    // Want fault handler, including DebugMon to keep priority 0.
    // Want UART utilized by MRI to be one priority level lower (level 1).
    // Want all other interrupts lower than the UART interrupt (level 2+).
    // These priority levels allow a CTRL+C in GDB to break into interrupt handlers and throwing a debug exception from
    // the UART interrupt (happens when CTRL+C is sent from GDB) doesn't result in Hard Fault escalation since
    // DebugMon is higher priority than the UART ISR.
    NVIC_SetPriority(SVCall_IRQn, 2);
    NVIC_SetPriority(PendSV_IRQn, 2);
    NVIC_SetPriority(SysTick_IRQn, 2);

    static const int CAN_IRQn = 34;
    int              irq;
    for (irq = WDT_IRQn ; irq <= CAN_IRQn ; irq++)
        NVIC_SetPriority((IRQn)irq, 2);

    irq = UART0_IRQn + m_serialIndex;
    NVIC_SetPriority((IRQn)irq, 1);
    NVIC_SetVector((IRQn)irq, (uint32_t)serialReceiveHandler);
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::serialReceive(void)
{
    static const uint8_t receiverDataReadyBit = 1 << 0;

    uint8_t byteReceived = 0x00;
    if (m_pSerial->LSR & receiverDataReadyBit)
        byteReceived = m_pSerial->RBR;

    if (m_acksExpected > 0 && (byteReceived == '+' || byteReceived == '-'))
    {
        // Just received ack/nack for a debug output string.
        interlockedDecrement(&m_acksExpected);
    }
    else
    {
        // Unexpected data received so break into debugger.
        flush();
        m_acksExpected = 0;
        __debugbreak();
    }

    return;
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::outputStringToGdb(const char* pOutput)
{
    static const char hexDigits[] = "0123456789ABCDEF";

    const uint8_t* pSrc = (uint8_t*)pOutput;
    char*          pDest = m_buffer;
    size_t         bytesLeft = BUFFER_SIZE;

    // Packet contains overhead of '$' + 'O' + '#' + 2 hex checksum digits,
    if (bytesLeft < 1 + 1 + 1 + 2)
        return;

    // Wait for previous transmit (if any) to complete since we are using the same m_buffer.
    flush();

    *pDest++ = '$';
    *pDest++ = 'O';
    bytesLeft -= 2;

    uint8_t checksum = 'O';
    while (*pSrc && bytesLeft > 2 + 1 + 2)
    {
        uint8_t byte = *pSrc++;

        uint8_t hexDigit = hexDigits[(byte >> 4)];
        checksum += hexDigit;
        *pDest++ = hexDigit;

        hexDigit = hexDigits[byte & 0xF];
        checksum += hexDigit;
        *pDest++ = hexDigit;

        bytesLeft -= 2;
    }

    assert ( bytesLeft >= 1 + 2 );
    *pDest++ = '#';
    *pDest++ = hexDigits[(checksum >> 4)];
    *pDest++ = hexDigits[checksum & 0xF];

    interlockedIncrement(&m_acksExpected);
    dmaTransmit(m_buffer, pDest - m_buffer);
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::dmaTransmit(const void* pData, size_t dataLength)
{
    uint32_t             uartTx = DMA_PERIPHERAL_UART0TX_MAT0_0 + (m_serialIndex * 2);

    // Clear error and terminal complete interrupts for this UART's GPDMA channel.
    LPC_GPDMA->DMACIntTCClear = m_channelMask;
    LPC_GPDMA->DMACIntErrClr  = m_channelMask;

    // Prep GPDMA channel to send bytes via the UART.
    m_pChannel->DMACCSrcAddr  = (uint32_t)pData;
    m_pChannel->DMACCDestAddr = (uint32_t)&m_pSerial->THR;
    m_pChannel->DMACCLLI      = 0;
    m_pChannel->DMACCControl  = DMACCxCONTROL_I | DMACCxCONTROL_SI |
                     (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_SBSIZE_SHIFT) |
                     (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_DBSIZE_SHIFT) |
                     (dataLength & DMACCxCONTROL_TRANSFER_SIZE_MASK);

    // Enable GPDMA channel.
    m_pChannel->DMACCConfig = DMACCxCONFIG_ENABLE |
                   (uartTx << DMACCxCONFIG_DEST_PERIPHERAL_SHIFT) |
                   DMACCxCONFIG_TRANSFER_TYPE_M2P |
                   DMACCxCONFIG_IE |
                   DMACCxCONFIG_ITC;

    // Flag that we have a transaction in progress.
    m_transferInProgress = true;
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::flush()
{
    while (m_transferInProgress && (LPC_GPDMA->DMACIntStat & m_channelMask) == 0)
    {
    }
    m_transferInProgress = false;
}

template <uint32_t BUFFER_SIZE>
void DebugSerial<BUFFER_SIZE>::clearPendingInterrupt()
{
    NVIC_ClearPendingIRQ((IRQn)(UART0_IRQn + m_serialIndex));
}

#endif // DEBUG_SERIAL_H_
