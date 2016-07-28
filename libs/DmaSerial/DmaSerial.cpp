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
#include <assert.h>
#include <DmaSerial.h>
#include <GPDMA.h>

DmaSerial::DmaSerial(PinName tx, PinName rx, const char* pName /* = NULL */) : Serial(tx, rx, pName)
{
    static const uint32_t fifoEnable = (1 << 0);
    static const uint32_t dmaEnable = (1 << 3);

    // Allocate a GPDMA channel to use for this UART.
    int channelIndex = allocateDmaChannel(GPDMA_CHANNEL_LOW);
    assert ( channelIndex != -1 );
    m_pChannel = dmaChannelFromIndex(channelIndex);
    m_channelMask = 1 << channelIndex;

    // Enable DMA for UART.
    _serial.uart->FCR = fifoEnable | dmaEnable;
    m_transferInProgress = false;

    // Make sure that GPDMA block is enabled.
    enableGpdmaPower();
    enableGpdmaInLittleEndianMode();

    // Make sure that GPDMA is configured for UART and not TimerMatch.
    LPC_SC->DMAREQSEL &= ~(3 << (_serial.index * 2));
}

void DmaSerial::dmaTransmit(void* pData, size_t dataLength)
{
    uint32_t             uartTx = DMA_PERIPHERAL_UART0TX_MAT0_0 + (_serial.index * 2);

    // Wait for previous transmit (if any) to complete.
    txFlush();

    // Clear error and terminal complete interrupts for this UART's GPDMA channel.
    LPC_GPDMA->DMACIntTCClear = m_channelMask;
    LPC_GPDMA->DMACIntErrClr  = m_channelMask;

    // Prep GPDMA channel to send bytes via the UART.
    m_pChannel->DMACCSrcAddr  = (uint32_t)pData;
    m_pChannel->DMACCDestAddr = (uint32_t)&_serial.uart->THR;
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

void DmaSerial::txFlush()
{
    while (m_transferInProgress && (LPC_GPDMA->DMACIntStat & m_channelMask) == 0)
    {
    }
    m_transferInProgress = false;
}
