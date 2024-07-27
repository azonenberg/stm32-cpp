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

#ifndef stm32_mdma_h
#define stm32_mdma_h

#define HAVE_MDMA

//STM32H735
#if MDMA_T_VERSION == 1

#define NUM_MDMA_CHANNELS 16

enum mdma_cr_t
{
	MDMA_CR_SWRQ			= 0x0001'0000,
	MDMA_CR_EN				= 0x0000'0001
};

enum mdma_isr_t
{
	MDMA_ISR_CRQA			= 0x0001'0000,
	MDMA_ISR_TCIF			= 0x0000'0010,
	MDMA_ISR_CTCIF			= 0x0000'0002
};

enum mdma_tcr_t
{
	MDMA_TCR_BWM			= 0x8000'0000,
	MDMA_TCR_SWRM			= 0x4000'0000,

	MDMA_TCR_TRGM_LINK		= 0x3000'0000,
	MDMA_TCR_TRGM_REP_BLOCK	= 0x2000'0000,
	MDMA_TCR_TRGM_BLOCK		= 0x1000'0000,
	MDMA_TCR_PKE			= 0x0200'0000,
	MDMA_TCR_DEST_INC_32	= 0x0000'0800,
	MDMA_TCR_SRC_INC_16		= 0x0000'0100,
	MDMA_TCR_DEST_SIZE_32	= 0x0000'0080,
	MDMA_TCR_SRC_SIZE_16	= 0x0000'0010,
	MDMA_TCR_DEST_INC		= 0x0000'0008,
	MDMA_TCR_SRC_INC		= 0x0000'0002,
};

enum mdma_tbr_t
{
	MDMA_TBR_DEST_AXI		= 0x0000'0000,
	MDMA_TBR_DEST_TCM		= 0x0002'0000,
	MDMA_TBR_SRC_TCM		= 0x0001'0000
};

typedef struct
{
	uint32_t	TCR;		//verify
	uint32_t	BNDTR;
	uint32_t	SAR;
	uint32_t	DAR;
	uint32_t	BRUR;
	uint32_t	LAR;
	uint32_t	TBR;
	uint32_t	field_1c;
	uint32_t	MAR;
	uint32_t	MDR;
} mdma_linkedlist_t;

typedef struct
{
	uint32_t	ISR;
	uint32_t	IFCR;
	uint32_t	ESR;
	uint32_t	CR;
	uint32_t	TCR;
	uint32_t	BNDTR;
	uint32_t	SAR;
	uint32_t	DAR;
	uint32_t	BRUR;
	uint32_t	LAR;
	uint32_t	TBR;
	uint32_t	field_2c;
	uint32_t	MAR;
	uint32_t	MDR;
	uint32_t	field_38;
	uint32_t	field_3c;
} mdma_chan_t;

typedef struct
{
	uint32_t GISR0;
	uint32_t field_04[15];

	mdma_chan_t	channels[NUM_MDMA_CHANNELS];
} mdma_t;

#else

#error Undefined or unspecified MDMA_T_VERSION

#endif	//version check

#endif	//include guard
