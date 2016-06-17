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
#ifndef DMA_H_
#define DMA_H_

#define DMACCxCONTROL_TRANSFER_SIZE_MASK    0xFFF
#define DMACCxCONTROL_SBSIZE_SHIFT          12
#define DMACCxCONTROL_DBSIZE_SHIFT          15
#define DMACCxCONTROL_BURSTSIZE_1           0
#define DMACCxCONTROL_BURSTSIZE_4           1
#define DMACCxCONTROL_BURSTSIZE_8           2
#define DMACCxCONTROL_BURSTSIZE_16          3
#define DMACCxCONTROL_BURSTSIZE_32          4
#define DMACCxCONTROL_BURSTSIZE_64          5
#define DMACCxCONTROL_BURSTSIZE_128         6
#define DMACCxCONTROL_BURSTSIZE_256         7
#define DMACCxCONTROL_SWIDTH_SHIFT          18
#define DMACCxCONTROL_DWIDTH_SHIFT          21
#define DMACCxCONTROL_WIDTH_BYTE            0
#define DMACCxCONTROL_WIDTH_HALFWORD        1
#define DMACCxCONTROL_WIDTH_WORD            2
#define DMACCxCONTROL_SI                    (1 << 26)
#define DMACCxCONTROL_DI                    (1 << 27)
#define DMACCxCONTROL_I                     (1 << 31)

#define DMACCxCONFIG_ENABLE                 (1 << 0)
#define DMACCxCONFIG_SRC_PERIPHERAL_SHIFT   1
#define DMACCxCONFIG_DEST_PERIPHERAL_SHIFT  6
#define DMACCxCONFIG_TRANSFER_TYPE_SHIFT    11
#define DMACCxCONFIG_IE                     (1 << 14)
#define DMACCxCONFIG_ITC                    (1 << 15)
#define DMACCxCONFIG_ACTIVE                 (1 << 17)
#define DMACCxCONFIG_HALT                   (1 << 18)

#define DMA_PERIPHERAL_SSP0_TX              0
#define DMA_PERIPHERAL_SSP0_RX              1
#define DMA_PERIPHERAL_SSP1_TX              2
#define DMA_PERIPHERAL_SSP1_RX              3
#define DMA_PERIPHERAL_ADC                  4
#define DMA_PERIPHERAL_I2S0                 5
#define DMA_PERIPHERAL_I2S1                 6
#define DMA_PERIPHERAL_DAC                  7
#define DMA_PERIPHERAL_UART0TX_MAT0_0       8
#define DMA_PERIPHERAL_UART0RX_MAT0_1       9
#define DMA_PERIPHERAL_UART1TX_MAT1_0       10
#define DMA_PERIPHERAL_UART1RX_MAT1_1       11
#define DMA_PERIPHERAL_UART2TX_MAT2_0       12
#define DMA_PERIPHERAL_UART2RX_MAT2_1       13
#define DMA_PERIPHERAL_UART3TX_MAT3_0       14
#define DMA_PERIPHERAL_UART3RX_MAT3_1       15

#define DMACCxCONFIG_TRANSFER_TYPE_M2M      (0 << DMACCxCONFIG_TRANSFER_TYPE_SHIFT)
#define DMACCxCONFIG_TRANSFER_TYPE_M2P      (1 << DMACCxCONFIG_TRANSFER_TYPE_SHIFT)
#define DMACCxCONFIG_TRANSFER_TYPE_P2M      (2 << DMACCxCONFIG_TRANSFER_TYPE_SHIFT)
#define DMACCxCONFIG_TRANSFER_TYPE_P2P      (3 << DMACCxCONFIG_TRANSFER_TYPE_SHIFT)


static __INLINE void enableGpdmaPower(void)
{
    LPC_SC->PCONP |= (1 << 29);
}

static __INLINE void enableGpdmaInLittleEndianMode(void)
{
    LPC_GPDMA->DMACConfig = 1;
}

#endif /* DMA_H_ */
