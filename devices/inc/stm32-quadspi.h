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

#ifndef stm32_quadspi_h
#define stm32_quadspi_h

#define HAVE_QUADSPI

//STM32L431
#if QUADSPI_T_VERSION == 1

typedef struct
{
	uint32_t	CR;
	uint32_t	DCR;
	uint32_t	SR;
	uint32_t	FCR;
	uint32_t	DLR;
	uint32_t	CCR;
	uint32_t	AR;
	uint32_t	ABR;
	uint32_t	DR;
	uint32_t	PSMKR;
	uint32_t	PSMAR;
	uint32_t	PIR;
	uint32_t	LPTR;
} quadspi_t;

enum quadspi_cr_t
{
	QUADSPI_FTHRES_MASK	= 0xf00,
	QUADSPI_ABORT		= 0x02,
	QUADSPI_ENABLE 		= 0x01
};

enum quadspi_sr_t
{
	QUADSPI_BUSY	= 0x20,
	QUADSPI_TOF		= 0x10,
	QUADSPI_SMF		= 0x08,
	QUADSPI_FTF		= 0x04,
	QUADSPI_TCF		= 0x02,
	QUADSPI_TEF		= 0x01
};

enum quadspi_dcr_t
{
	QUADSPI_CSHT_MASK	= 0x700
};

enum quadspi_ccr_t
{
	QUADSPI_DDRM					= 0x80000000,
	QUADSPI_FMODE_MASK				= 0x0c000000,
	QUADSPI_FMODE_INDIRECT_WRITE	= 0x00000000,
	QUADSPI_FMODE_INDIRECT_READ		= 0x04000000,
	QUADSPI_FMODE_MEMORY_MAP		= 0x0c000000,
	QUADSPI_DMODE_MASK				= 0x03000000,
	QUADSPI_DCYC_MASK				= 0x007c0000,
	QUADSPI_ABSIZE_MASK				= 0x00030000,
	QUADSPI_ABMODE_MASK				= 0x0000c000,
	QUADSPI_ADSIZE_MASK				= 0x00003000,
	QUADSPI_ADMODE_MASK				= 0x00000c00,
	QUADSPI_IMODE_MASK				= 0x00000300,
	QUADSPI_INSN_MASK				= 0x000000ff
};

#else

#error Undefined or unspecified QUADSPI_T_VERSION

#endif	//version check

#endif	//include guard
