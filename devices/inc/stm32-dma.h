/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2024 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef stm32_dma_h
#define stm32_dma_h

#define HAVE_DMA

//STM32H735
#if DMA_T_VERSION == 1

#define NUM_DMA_CHANNELS	8
#define NUM_DMAMUX_CHANNELS	16

struct dmamux_t
{
	uint32_t	CCR[NUM_DMAMUX_CHANNELS];
	uint32_t	field_40[16];
	uint32_t	CSR;
	uint32_t	CFR;
	uint32_t	field_88[30];
	uint32_t	RGCR[4];
	uint32_t	RGSR;
	uint32_t	RGCFR;
	uint32_t	RGSR2;
	uint32_t	RGCFR2;
};

enum dma_scr_t
{
	DMA_SCR_MBURST_SINGLE	= 0x00000000,
	DMA_SCR_MBURST_INCR4	= 0x00800000,
	DMA_SCR_MBURST_INCR8	= 0x01000000,
	DMA_SCR_MBURST_INCR16	= 0x01800000,

	DMA_SCR_PBURST_SINGLE	= 0x00000000,
	DMA_SCR_PBURST_INCR4	= 0x00200000,
	DMA_SCR_PBURST_INCR8	= 0x00400000,
	DMA_SCR_PBURST_INCR16	= 0x00600000,

	DMA_SCR_TRBUFF			= 0x00100000,
	DMA_SCR_CT				= 0x00080000,
	DMA_SCR_DBM				= 0x00040000,

	DMA_SCR_PL_LOW			= 0x00000000,
	DMA_SCR_PL_MED			= 0x00010000,
	DMA_SCR_PL_HIGH			= 0x00020000,
	DMA_SCR_PL_VHIGH		= 0x00030000,

	DMA_SCR_PINCOS			= 0x00008000,

	DMA_SCR_MSIZE_8			= 0x00000000,
	DMA_SCR_MSIZE_16		= 0x00002000,
	DMA_SCR_MSIZE_32		= 0x00004000,

	DMA_SCR_PSIZE_8			= 0x00000000,
	DMA_SCR_PSIZE_16		= 0x00000800,
	DMA_SCR_PSIZE_32		= 0x00001000,

	DMA_SCR_MINC			= 0x00000400,
	DMA_SCR_PINC			= 0x00000200,
	DMA_SCR_CIRC			= 0x00000100,

	DMA_SCR_DIR_P2M			= 0x00000000,
	DMA_SCR_DIR_M2P			= 0x00000040,
	DMA_SCR_DIR_M2M			= 0x00000080,

	DMA_SCR_PFCTRL			= 0x00000020,

	DMA_SCR_TCIE			= 0x00000010,
	DMA_SCR_HTIE			= 0x00000008,
	DMA_SCR_TEIE			= 0x00000004,
	DMA_SCR_DMEIE			= 0x00000002,

	DMA_SCR_EN				= 0x00000001
};

struct dmastream_t
{
	uint32_t		CR;
	uint32_t		NDTR;
	volatile void*	PAR;
	volatile void*	M0AR;
	volatile void*	M1AR;
	uint32_t		FCR;
};

struct dma_t
{
	uint32_t	LISR;
	uint32_t	HISR;
	uint32_t	LIFCR;
	uint32_t	HIFCR;

	dmastream_t	streams[NUM_DMA_CHANNELS];
};

#else

#error Undefined or unspecified DMA_T_VERSION

#endif	//version check

#endif	//include guard
