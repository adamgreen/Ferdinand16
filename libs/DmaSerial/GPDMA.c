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
#include <cmsis.h>
#include <stdio.h>
#include <string.h>
#include "GPDMA.h"

static uint32_t g_dmaChannelsInUse;

int allocateDmaChannel(DmaDesiredChannel desiredChannel)
{
    switch (desiredChannel)
    {
    case GPDMA_CHANNEL_HIGH:
        for (int i = GPDMA_CHANNEL_HIGHEST ; i <= GPDMA_CHANNEL_LOWEST ; i++)
        {
            uint32_t mask = (1 << i);
            if ((mask & g_dmaChannelsInUse) == 0)
            {
                g_dmaChannelsInUse |= mask;
                return i;
            }
        }
        return -1;
    case GPDMA_CHANNEL_LOW:
        // Reserve GPDMA_CHANNEL_LOWEST for memory to memory operations.
        for (int i = GPDMA_CHANNEL_LOWEST - 1; i >= GPDMA_CHANNEL_HIGHEST ; i--)
        {
            uint32_t mask = (1 << i);
            if ((mask & g_dmaChannelsInUse) == 0)
            {
                g_dmaChannelsInUse |= mask;
                return i;
            }
        }
        return -1;
    default:
        if (((1 << desiredChannel) & g_dmaChannelsInUse) == 0)
        {
            return desiredChannel;
        }
        return -1;
    }
    return -1;
}

void freeDmaChannel(int channel)
{
    if (channel >= GPDMA_CHANNEL_HIGHEST && channel <= GPDMA_CHANNEL_LOWEST)
    {
        g_dmaChannelsInUse &= ~(1 << channel);
    }
}

LPC_GPDMACH_TypeDef* dmaChannelFromIndex(int index)
{
    switch (index)
    {
    case GPDMA_CHANNEL0:
        return LPC_GPDMACH0;
    case GPDMA_CHANNEL1:
        return LPC_GPDMACH1;
    case GPDMA_CHANNEL2:
        return LPC_GPDMACH2;
    case GPDMA_CHANNEL3:
        return LPC_GPDMACH3;
    case GPDMA_CHANNEL4:
        return LPC_GPDMACH4;
    case GPDMA_CHANNEL5:
        return LPC_GPDMACH5;
    case GPDMA_CHANNEL6:
        return LPC_GPDMACH6;
    case GPDMA_CHANNEL7:
        return LPC_GPDMACH7;
    default:
        return NULL;
    }
}



static DmaInterruptHandler* g_pDmaHandlers = NULL;

void DMA_IRQHandler(void)
{
    uint32_t dmaInterruptStatus = LPC_GPDMA->DMACIntStat;
    DmaInterruptHandler* pCurr = g_pDmaHandlers;
    DmaInterruptHandler* pNext = NULL;

    while (pCurr && dmaInterruptStatus)
    {
        pNext = pCurr->pNext;

        uint32_t handledInterrupts = pCurr->handler(pCurr->pContext, dmaInterruptStatus);
        dmaInterruptStatus &= ~handledInterrupts;

        pCurr = pNext;
    }
}

int addDmaInterruptHandler(DmaInterruptHandler* pHandler)
{
    int wasListEmpty = (g_pDmaHandlers == NULL);

    pHandler->pNext = g_pDmaHandlers;
    g_pDmaHandlers = pHandler;

    if (wasListEmpty)
    {
        // Enable GPDMA interrupt.
        NVIC_EnableIRQ(DMA_IRQn);
    }

    return wasListEmpty;
}

int removeDmaInterruptHandler(DmaInterruptHandler* pHandler)
{
    DmaInterruptHandler* pPrev = NULL;
    DmaInterruptHandler* pCurr = g_pDmaHandlers;

    while (pCurr && pCurr != pHandler)
    {
        pPrev = pCurr;
        pCurr = pCurr->pNext;
    }
    if (!pCurr)
    {
        // Handler was not found in list to remove.
        return 0;
    }
    if (!pPrev)
    {
        // List will now be empty.
        NVIC_DisableIRQ(DMA_IRQn);
        g_pDmaHandlers = NULL;
        return 1;
    }
    pPrev->pNext = pHandler->pNext;
    pHandler->pNext = NULL;
    return 0;
}


static void initDmaMemCopy(void);
static uint32_t dmaMemCopyInterruptHandler(void* pContext, uint32_t dmaInterruptStatus);

static const DmaMemCopyCallback* g_pMemCopyCallback = NULL;
static LPC_GPDMACH_TypeDef*      g_pChannelMemCopy = NULL;
static uint32_t                  g_channelMemCopy;
static int                       g_haveInitForMemCopy = 0;
static DmaInterruptHandler       g_dmaMemCopyHandler = { &dmaMemCopyInterruptHandler, NULL, NULL };



int dmaMemCopy(void* pDest, const void* pSrc, size_t size, const DmaMemCopyCallback *pCallback)
{
    initDmaMemCopy();

    if (!g_pMemCopyCallback)
    {
        // Kick off the DMA transfer to perform the copy since the DMA channel is free.
        g_pMemCopyCallback = pCallback;
        uint32_t memcopyChannelMask = 1 << g_channelMemCopy;

        LPC_GPDMA->DMACIntTCClear = memcopyChannelMask;
        LPC_GPDMA->DMACIntErrClr  = memcopyChannelMask;

        g_pChannelMemCopy->DMACCSrcAddr  = (uint32_t)pSrc;
        g_pChannelMemCopy->DMACCDestAddr = (uint32_t)pDest;
        g_pChannelMemCopy->DMACCLLI      = 0;
        g_pChannelMemCopy->DMACCControl  = DMACCxCONTROL_I | DMACCxCONTROL_SI | DMACCxCONTROL_DI |
                         (DMACCxCONTROL_WIDTH_WORD << DMACCxCONTROL_SWIDTH_SHIFT) |
                         (DMACCxCONTROL_WIDTH_WORD << DMACCxCONTROL_DWIDTH_SHIFT) |
                         (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_SBSIZE_SHIFT) |
                         (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_DBSIZE_SHIFT) |
                         (size & DMACCxCONTROL_TRANSFER_SIZE_MASK);

        // Enable DMA memory copy channel.
        g_pChannelMemCopy->DMACCConfig = DMACCxCONFIG_ENABLE |
                       DMACCxCONFIG_TRANSFER_TYPE_M2M |
                       DMACCxCONFIG_IE |
                       DMACCxCONFIG_ITC;

        return 1;
    }
    else
    {
        // DMA channel wasn't free so use CPU to perform the copy.
        memcpy(pDest, pSrc, size);
        pCallback->handler(pCallback->pContext);
        return 0;
    }
}

static void initDmaMemCopy(void)
{
    if (g_haveInitForMemCopy)
    {
        return;
    }

    // Allocate DMA channel for copying memory.
    g_channelMemCopy = allocateDmaChannel(GPDMA_CHANNEL_MEM2MEM);
    g_pChannelMemCopy = dmaChannelFromIndex(g_channelMemCopy);

    // Add handler for handling these DMA interrupts.
    addDmaInterruptHandler(&g_dmaMemCopyHandler);

    g_haveInitForMemCopy = 1;
}

uint32_t dmaMemCopyInterruptHandler(void* pContext, uint32_t dmaInterruptStatus)
{
    uint32_t memcopyChannelMask = 1 << g_channelMemCopy;

    if ((dmaInterruptStatus & memcopyChannelMask) == 0)
    {
        return 0;
    }

    // Callback into the client application to let them know that the memcpy has completed.
    assert ( g_pMemCopyCallback );
    g_pMemCopyCallback->handler(g_pMemCopyCallback->pContext);
    g_pMemCopyCallback = NULL;

    // Flag that we have handled this interrupt.
    LPC_GPDMA->DMACIntTCClear = memcopyChannelMask;
    return memcopyChannelMask;
}

void uninitDmaMemCopy(void)
{
    // Shouldn't be called while there is still a DMA mem copy in progress.
    assert ( !g_pMemCopyCallback );

    if (!g_haveInitForMemCopy)
    {
        return;
    }
    removeDmaInterruptHandler(&g_dmaMemCopyHandler);
    freeDmaChannel(g_channelMemCopy);
    g_haveInitForMemCopy = 0;
}



__attribute__((section("AHBSRAM0"),aligned)) static uint8_t g_dmaHeap0[16 * 1024];
static uint8_t*                                             g_pDmaHeap0 = g_dmaHeap0;
__attribute__((section("AHBSRAM1"),aligned)) static uint8_t g_dmaHeap1[16 * 1024];
static uint8_t*                                             g_pDmaHeap1 = g_dmaHeap1;

void* dmaHeap0Alloc(uint32_t size)
{
    uint8_t* p = g_pDmaHeap0;
    // Keep heap 8-byte aligned.
    size = (size + 7) & ~7;
    g_pDmaHeap0 += size;
    return p;
}

void* dmaHeap1Alloc(uint32_t size)
{
    uint8_t* p = g_pDmaHeap1;
    // Keep heap 8-byte aligned.
    size = (size + 7) & ~7;
    g_pDmaHeap1 += size;
    return p;
}
