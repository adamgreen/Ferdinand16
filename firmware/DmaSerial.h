/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef DMA_SERIAL_H_
#define DMA_SERIAL_H_

#include "dma.h"

class DmaSerial : public Serial
{
public:
    DmaSerial(PinName tx, PinName rx, const char* pName = NULL) : Serial(tx, rx, pName)
    {
        static const uint32_t fifoEnable = (1 << 0);
        static const uint32_t dmaEnable = (1 << 3);

        // Enable DMA for UART.
        _serial.uart->FCR = fifoEnable | dmaEnable;
        m_transferInProgress = false;

        // Make sure that GPDMA block is enabled.
        enableGpdmaPower();
        enableGpdmaInLittleEndianMode();

        // Make sure that GPDMA is configured for UART and not TimerMatch.
        LPC_SC->DMAREQSEL &= ~(3 << (_serial.index * 2));
    }

    void dmaTransmit(void* pData, size_t dataLength)
    {
        LPC_GPDMACH_TypeDef* pChannel6 = LPC_GPDMACH6;
        uint32_t             uartTx = DMA_PERIPHERAL_UART0TX_MAT0_0 + (_serial.index * 2);

        // Wait for previous transmit (is any) to complete.
        while (m_transferInProgress && (LPC_GPDMA->DMACIntStat & (1 << 6)) != (1 << 6))
        {
        }
        m_transferInProgress = false;

        // Clear error and terminal complete interrupts for channel 6.
        LPC_GPDMA->DMACIntTCClear = (1 << 6);
        LPC_GPDMA->DMACIntErrClr  = (1 << 6);

        // Prep Channel6 to send bytes via the UART.
        pChannel6->DMACCSrcAddr  = (uint32_t)pData;
        pChannel6->DMACCDestAddr = (uint32_t)&_serial.uart->THR;
        pChannel6->DMACCLLI      = 0;
        pChannel6->DMACCControl  = DMACCxCONTROL_I | DMACCxCONTROL_SI |
                         (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_SBSIZE_SHIFT) |
                         (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_DBSIZE_SHIFT) |
                         (dataLength & DMACCxCONTROL_TRANSFER_SIZE_MASK);

        // Enable Channel6.
        pChannel6->DMACCConfig = DMACCxCONFIG_ENABLE |
                       (uartTx << DMACCxCONFIG_DEST_PERIPHERAL_SHIFT) |
                       DMACCxCONFIG_TRANSFER_TYPE_M2P |
                       DMACCxCONFIG_IE |
                       DMACCxCONFIG_ITC;

        // Flag that we have a transaction in progress.
        m_transferInProgress = true;
    }
protected:
    bool m_transferInProgress;
};

#endif // DMA_SERIAL_H_
