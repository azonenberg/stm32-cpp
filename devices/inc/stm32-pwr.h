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

#ifndef stm32_pwr_h
#define stm32_pwr_h

#define HAVE_PWR

//STM32H735, H750
#if PWR_T_VERSION == 1

typedef struct
{
	uint32_t CR1;
	uint32_t CSR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t CPUCR;
	uint32_t field_14;
	uint32_t D3CR;
	uint32_t WKUPCR;
	uint32_t WKUPFR;
	uint32_t WKUPEPR;
} pwr_t;
extern volatile pwr_t PWR;

#ifdef STM32H735
	enum VoltageRange
	{
		RANGE_VOS3 = 1,	//0.95 - 1.05V
		RANGE_VOS2 = 2,	//1.05 - 1.15V
		RANGE_VOS1 = 3,	//1.15 - 1.26V
		RANGE_VOS0 = 0	//1.26 - 1.4V
	};
#elif defined(STM32H750)
	enum VoltageRange
	{
		RANGE_VOS3 = 1,	//0.95 - 1.26V
		RANGE_VOS2 = 2,	//1.05 - 1.26V
		RANGE_VOS1 = 3,	//1.15 - 1.26V
		RANGE_VOS0 = 0	//1.26 - 1.4V (PWR_D3CR suggests reserved and unavailable?? but seems rev V parts have it)
	};
#else
	#error Unknown STM32H7 PWR version
#endif

enum pwr_csr1
{
	PWR_CSR1_ACTVOSRDY = 0x2000
};

enum pwr_d3cr
{
	PWR_D3CR_VOSRDY = 0x2000,

	PWR_D3CR_VOSMASK = 0xc000
};

#else

#error Undefined or unspecified PWR_T_VERSION

#endif	//version check

#endif	//include guard
