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

#ifndef stm32_ramcfg_h
#define stm32_ramcfg_h

#define HAVE_RAMCFG

//STM32MP257
#if RAMCFG_T_VERSION == 1

struct ramcfg_t
{
	uint32_t	SYSRAMCR;
	uint32_t	field_4;
	uint32_t	SYSRAMISR;
	uint32_t	field_0c[7];
	uint32_t	SYSRAMERKEYR;
	uint32_t	field_2c[21];

	uint32_t	SRAM1CR;
	uint32_t	field_84;
	uint32_t	SRAM1ISR;
	uint32_t	field_8c[7];
	uint32_t	SRAM1ERKEYR;
	uint32_t	field_ac[21];

	uint32_t	SRAM2CR;
	uint32_t	field_104;
	uint32_t	SRAM2ISR;
	uint32_t	field_10c[7];
	uint32_t	SRAM2ERKEYR;
	uint32_t	field_102c[21];

	uint32_t	RETRAMCR;
	uint32_t	RETRAMIER;
	uint32_t	RETRAMISR;
	uint32_t	RETRAMSEAR;
	uint32_t	RETRAMDEAR;
	uint32_t	RETRAMICR;
	uint32_t	field_198[3];
	uint32_t	RETRAMECCKEYR;
	uint32_t	RETRAMERKEYR;
	uint32_t	field_1ac;
	uint32_t	RETRAMCCR1;
	uint32_t	RETRAMCCR2;
	uint32_t	RETRAMCRSR;
	uint32_t	RETRAMCSR;
	uint32_t	RETRAMCCSR;
	uint32_t	field_1c4[15];

	uint32_t	LPSRAM1CR;
	//TODO after here
};

#else

#error Undefined or unspecified RAMCFG_T_VERSION

#endif	//version check

#endif	//include guard
