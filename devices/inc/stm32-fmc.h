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

#ifndef stm32_fmc_h
#define stm32_fmc_h

#define HAVE_FMC

//STM32H735
#if FMC_T_VERSION == 1

typedef struct
{
	//Configuration and timing registers
	uint32_t	BCR1;
	uint32_t	BTR1;
	uint32_t	BCR2;
	uint32_t	BTR2;
	uint32_t	BCR3;
	uint32_t	BTR3;
	uint32_t	BCR4;
	uint32_t	BTR4;

	uint32_t	field_20[56];

	uint32_t	field_100;
	uint32_t	BWTR1;
	uint32_t	field_108;
	uint32_t	BWTR2;
	uint32_t	field_110;
	uint32_t	BWTR3;
	uint32_t	field_118;
	uint32_t	BWTR4;
} fmc_t;

enum fmc_bcr_t
{
	//only valid in bank 1
	FMC_BCR_FMCEN		= 0x8000'0000,
	FMC_BCR_CCLKEN		= 0x0010'0000,
	FMC_BCR_BMAP_SWAP	= 0x0100'0000,

	FMC_BCR_CBURSTRW	= 0x0008'0000,
	FMC_BCR_WAITEN		= 0x0000'2000,
	FMC_BCR_WREN		= 0x0000'1000,
	FMC_BCR_WAITCFG		= 0x0000'0800,
	FMC_BCR_BURSTEN		= 0x0000'0100,
	FMC_BCR_WIDTH_16	= 0x0000'0010,
	FMC_BCR_TYPE_SRAM	= 0x0000'0000,
	FMC_BCR_TYPE_PSRAM	= 0x0000'0004,
	FMC_BCR_MUXEN		= 0x0000'0002,
	FMC_BCR_MBKEN		= 0x0000'0001,

	FMC_RESERVED_BITS	= 0x0000'0080
};

#else

#error Undefined or unspecified FMC_T_VERSION

#endif	//version check

#endif	//include guard
