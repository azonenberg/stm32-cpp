/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2026 Andrew D. Zonenberg                                                                          *
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

#ifndef stm32_adc_h
#define stm32_adc_h

#define HAVE_DAC

//STM32L431
#if DAC_T_VERSION == 1

typedef struct
{
	uint32_t	CR;
	uint32_t	SWTRGR;
	uint32_t	DHR12R1;
	uint32_t	DHR12L1;
	uint32_t	DHR8R1;
	uint32_t	DHR12R2;
	uint32_t	DHR12L2;
	uint32_t	DHR8R2;
	uint32_t	DHR12RD;
	uint32_t	DHR12LD;
	uint32_t	DHR8RD;
	uint32_t	DOR1;
	uint32_t	DOR2;
	uint32_t	SR;
	uint32_t	CCR;
	uint32_t	MCR;
	uint32_t	SHSR1;
	uint32_t	SHSR2;
	uint32_t	SHHR;
	uint32_t	SHRR;
} dac_t;

enum dac_cr
{
	//All registers are shifted up by 16 bits if we're channel 2 rather than 1
	DAC_CR_TEN	= 0x0000'0002,
	DAC_CR_EN	= 0x0000'0001
};

#else

#error Undefined or unspecified DAC_T_VERSION

#endif	//version check

#endif	//include guard
