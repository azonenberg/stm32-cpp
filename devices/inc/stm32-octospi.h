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

#ifndef stm32_octospi_h
#define stm32_octospi_h

#define HAVE_OCTOSPI

//STM32H735
#if OCTOSPI_T_VERSION == 1

typedef struct
{
	uint32_t		CR;
	uint32_t		field_4;
	uint32_t		DCR1;
	uint32_t		DCR2;
	uint32_t		DCR3;
	uint32_t		DCR4;
	uint32_t		field_18[2];
	uint32_t		SR;
	uint32_t		FCR;
	uint32_t		field_28[6];
	uint32_t		DLR;
	uint32_t		field_44;
	uint32_t		AR;
	uint32_t		field_4c;
	uint32_t		DR;
	uint32_t		field_54[11];
	uint32_t		PSMKR;
	uint32_t		field_84;
	uint32_t		PSMAR;
	uint32_t		field_8c;
	uint32_t		PIR;
	uint32_t		field_94[27];
	uint32_t		CCR;
	uint32_t		field_104;
	uint32_t		TCR;
	uint32_t		field_10c;
	uint32_t		IR;
	uint32_t		field_114[3];
	uint32_t		ABR;
	uint32_t		field_124[3];
	uint32_t		LPTR;
	uint32_t		field_134[3];
	uint32_t		WPCCR;
	uint32_t		field_144;
	uint32_t		WPTCR;
	uint32_t		field_14c;
	uint32_t		WPIR;
	uint32_t		field_154[3];
	uint32_t		WPABR;
	uint32_t		field_164[7];
	uint32_t		WCCR;
	uint32_t		field_184;
	uint32_t		WTCR;
	uint32_t		field_18c;
	uint32_t		WIR;
	uint32_t		field_194[3];
	uint32_t		WABR;
	uint32_t		field_1a4[23];
	uint32_t		HLCR;
} octospi_t;

enum octospi_cr
{
	OCTOSPI_FMODE_MASK				= 0x30000000,
	OCTOSPI_FMODE_INDIRECT_WRITE	= 0x00000000,
	OCTOSPI_FMODE_INDIRECT_READ		= 0x10000000,
	OCTOSPI_FMODE_AUTO_POLL			= 0x20000000,
	OCTOSPI_FMODE_MMAP				= 0x30000000,

	OCTOSPI_FTHRES_MASK				= 0x00001f00,

	OCTOSPI_TCEN					= 0x00000008,
	OCTOSPI_DMAEN					= 0x00000004,
	OCTOSPI_ABORT					= 0x00000002,
	OCTOSPI_EN						= 0x00000001
};

enum octospi_dcr1
{
	OCTOSPI_MEM_TYPE_STANDARD		= 0x02000000,

	OCTOSPI_CSHT_MASK				= 0x00003f00
};

enum octospi_sr
{
	OCTOSPI_BUSY					= 0x00000020,
	OCTOSPI_TOF						= 0x00000010,
	OCTOSPI_SMF						= 0x00000008,
	OCTOSPI_FTF						= 0x00000004,
	OCTOSPI_TCF						= 0x00000002,
	OCTOSPI_TEF						= 0x00000001
};

enum octospi_ccr
{
	OCTOSPI_SIOO					= 0x80000000,
	OCTOSPI_DQSE					= 0x20000000,
	OCTOSPI_DDTR					= 0x08000000,
	OCTOSPI_DMODE_MASK				= 0x07000000,
	OCTOSPI_ABSIZE_MASK				= 0x00300000,
	OCTOSPI_ABDTR					= 0x00080000,
	OCTOSPI_ABMODE_MASK				= 0x00070000,
	OCTOSPI_ADSIZE_MASK				= 0x00003000,
	OCTOSPI_ADDTR					= 0x00000800,
	OCTOSPI_ADMODE_MASK				= 0x00000700,
	OCTOSPI_ISIZE_MASK				= 0x00000030,
	OCTOSPI_IDTR					= 0x00000008,
	OCTOSPI_IMODE_MASK				= 0x00000007
};

enum octospi_tcr
{
	OCTOSPI_SSHIFT					= 0x40000000,
	OCTOSPI_DHQC					= 0x10000000,
	OCTOSPI_DCYC_MASK				= 0x0000001f
};

typedef struct
{
	uint32_t		CR;
	uint32_t		P1CR;
	uint32_t		P2CR;
} octospim_t;

//STM32MP257
#elif OCTOSPI_T_VERSION == 2

typedef struct
{
	uint32_t		CR;
	uint32_t		field_4;
	uint32_t		DCR1;
	uint32_t		DCR2;
	uint32_t		DCR3;
	uint32_t		DCR4;
	uint32_t		field_18[2];
	uint32_t		SR;
	uint32_t		FCR;
	uint32_t		field_28[6];
	uint32_t		DLR;
	uint32_t		field_44;
	uint32_t		AR;
	uint32_t		field_4c;
	uint32_t		DR;
	uint32_t		field_54[11];
	uint32_t		PSMKR;
	uint32_t		field_84;
	uint32_t		PSMAR;
	uint32_t		field_8c;
	uint32_t		PIR;
	uint32_t		field_94[27];
	uint32_t		CCR;
	uint32_t		field_104;
	uint32_t		TCR;
	uint32_t		field_10c;
	uint32_t		IR;
	uint32_t		field_114[3];
	uint32_t		ABR;
	uint32_t		field_124[3];
	uint32_t		LPTR;
	uint32_t		field_134[3];
	uint32_t		WPCCR;
	uint32_t		field_144;
	uint32_t		WPTCR;
	uint32_t		field_14c;
	uint32_t		WPIR;
	uint32_t		field_154[3];
	uint32_t		WPABR;
	uint32_t		field_164[7];
	uint32_t		WCCR;
	uint32_t		field_184;
	uint32_t		WTCR;
	uint32_t		field_18c;
	uint32_t		WIR;
	uint32_t		field_194[3];
	uint32_t		WABR;
	uint32_t		field_1a4[23];
	uint32_t		HLCR;
	uint32_t		field_204[122];
	uint32_t		HWCFGR2;
	uint32_t		HWCFGR;
	uint32_t		VERR;
	uint32_t		IDR;
	uint32_t		MIDR;
} octospi_t;

enum octospi_cr
{
	OCTOSPI_FMODE_MASK				= 0x30000000,
	OCTOSPI_FMODE_INDIRECT_WRITE	= 0x00000000,
	OCTOSPI_FMODE_INDIRECT_READ		= 0x10000000,
	OCTOSPI_FMODE_AUTO_POLL			= 0x20000000,
	OCTOSPI_FMODE_MMAP				= 0x30000000,

	OCTOSPI_FTHRES_MASK				= 0x00001f00,

	OCTOSPI_TCEN					= 0x00000008,
	OCTOSPI_DMAEN					= 0x00000004,
	OCTOSPI_ABORT					= 0x00000002,
	OCTOSPI_EN						= 0x00000001
};

enum octospi_dcr1
{
	OCTOSPI_MEM_TYPE_STANDARD		= 0x02000000,

	OCTOSPI_CSHT_MASK				= 0x00003f00
};

enum octospi_sr
{
	OCTOSPI_BUSY					= 0x00000020,
	OCTOSPI_TOF						= 0x00000010,
	OCTOSPI_SMF						= 0x00000008,
	OCTOSPI_FTF						= 0x00000004,
	OCTOSPI_TCF						= 0x00000002,
	OCTOSPI_TEF						= 0x00000001
};

enum octospi_ccr
{
	OCTOSPI_SIOO					= 0x80000000,
	OCTOSPI_DQSE					= 0x20000000,
	OCTOSPI_DDTR					= 0x08000000,
	OCTOSPI_DMODE_MASK				= 0x07000000,
	OCTOSPI_ABSIZE_MASK				= 0x00300000,
	OCTOSPI_ABDTR					= 0x00080000,
	OCTOSPI_ABMODE_MASK				= 0x00070000,
	OCTOSPI_ADSIZE_MASK				= 0x00003000,
	OCTOSPI_ADDTR					= 0x00000800,
	OCTOSPI_ADMODE_MASK				= 0x00000700,
	OCTOSPI_ISIZE_MASK				= 0x00000030,
	OCTOSPI_IDTR					= 0x00000008,
	OCTOSPI_IMODE_MASK				= 0x00000007
};

enum octospi_tcr
{
	OCTOSPI_SSHIFT					= 0x40000000,
	OCTOSPI_DHQC					= 0x10000000,
	OCTOSPI_DCYC_MASK				= 0x0000001f
};

typedef struct
{
	uint32_t		CR;
	uint32_t		field_4[251];
	uint32_t		HWCFGR;
	uint32_t		VERR;
	uint32_t		IDR;
	uint32_t		SIDR;
} octospim_t;

#else

#error Undefined or unspecified OCTOSPI_T_VERSION

#endif	//version check

#endif	//include guard
