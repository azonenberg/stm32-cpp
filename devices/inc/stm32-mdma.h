/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2025 Andrew D. Zonenberg                                                                          *
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

//STM32H735, H750
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

	MDMA_TCR_TRGM_MASK		= 0x3000'0000,
	MDMA_TCR_TRGM_LINK		= 0x3000'0000,
	MDMA_TCR_TRGM_REP_BLOCK	= 0x2000'0000,
	MDMA_TCR_TRGM_BLOCK		= 0x1000'0000,
	MDMA_TCR_TRGM_BUFFER	= 0x0000'0000,

	MDMA_TCR_PKE			= 0x0200'0000,

	MDMA_TCR_DEST_INC_8		= 0x0000'0000,
	MDMA_TCR_DEST_INC_16	= 0x0000'0400,
	MDMA_TCR_DEST_INC_32	= 0x0000'0800,
	MDMA_TCR_DEST_INC_64	= 0x0000'0c00,
	MDMA_TCR_DEST_INC_MASK	= 0x0000'0c00,

	MDMA_TCR_SRC_INC_8		= 0x0000'0000,
	MDMA_TCR_SRC_INC_16		= 0x0000'0100,
	MDMA_TCR_SRC_INC_32		= 0x0000'0200,
	MDMA_TCR_SRC_INC_64		= 0x0000'0300,
	MDMA_TCR_SRC_INC_MASK	= 0x0000'0300,

	MDMA_TCR_DEST_SIZE_8	= 0x0000'0000,
	MDMA_TCR_DEST_SIZE_16	= 0x0000'0040,
	MDMA_TCR_DEST_SIZE_32	= 0x0000'0080,
	MDMA_TCR_DEST_SIZE_64	= 0x0000'00c0,
	MDMA_TCR_DEST_SIZE_MASK	= 0x0000'00c0,

	MDMA_TCR_SRC_SIZE_8		= 0x0000'0000,
	MDMA_TCR_SRC_SIZE_16	= 0x0000'0010,
	MDMA_TCR_SRC_SIZE_32	= 0x0000'0020,
	MDMA_TCR_SRC_SIZE_64	= 0x0000'0030,
	MDMA_TCR_SRC_SIZE_MASK	= 0x0000'0030,

	MDMA_TCR_DEST_INC		= 0x0000'0008,
	MDMA_TCR_DEST_DEC		= 0x0000'000c,
	MDMA_TCR_DEST_INCMASK	= 0x0000'000c,
	MDMA_TCR_DEST_FIX		= 0x0000'0000,

	MDMA_TCR_SRC_INC		= 0x0000'0002,
	MDMA_TCR_SRC_DEC		= 0x0000'0003,
	MDMA_TCR_SRC_MODEMASK	= 0x0000'0003,
	MDMA_TCR_SRC_FIX		= 0x0000'0000,
};

enum mdma_tbr_t
{
	MDMA_TBR_DEST_AXI		= 0x0000'0000,
	MDMA_TBR_DEST_TCM		= 0x0002'0000,
	MDMA_TBR_SRC_TCM		= 0x0001'0000,
	MDMA_TBR_SRC_AXI		= 0x0000'0000
};

struct mdma_linkedlist_t
{
	uint32_t					TCR;
	uint32_t					BNDTR;
	volatile void*				SAR;
	volatile void*				DAR;
	uint32_t					BRUR;
	volatile mdma_linkedlist_t*	LAR;
	uint32_t					TBR;
	uint32_t					field_1c;	//reserved
	uint32_t					MAR;
	uint32_t					MDR;
} ;

struct mdma_chan_t
{
	uint32_t					ISR;
	uint32_t					IFCR;
	uint32_t					ESR;
	uint32_t					CR;
	volatile mdma_linkedlist_t	state;
	uint32_t					field_38;	//padding
	uint32_t					field_3c;
};

struct mdma_t
{
	uint32_t GISR0;
	uint32_t field_04[15];

	mdma_chan_t	channels[NUM_MDMA_CHANNELS];
};

#else

#error Undefined or unspecified MDMA_T_VERSION

#endif	//version check

#endif	//include guard
