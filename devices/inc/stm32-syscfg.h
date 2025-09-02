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

#ifndef stm32_syscfg_h
#define stm32_syscfg_h

#define HAVE_SYSCFG

//STM32H735, STM32H750
#if SYSCFG_T_VERSION == 1

#define HAVE_PKG

typedef struct
{
	uint32_t field_0;
	uint32_t PMCR;
	uint32_t EXTICR1;
	uint32_t EXTICR2;
	uint32_t EXTICR3;
	uint32_t EXTICR4;
	uint32_t CFGR;
	uint32_t field_1c;
	uint32_t CCCSR;
	uint32_t CCVR;
	uint32_t CCCR;
	uint32_t field_2c;
	uint32_t ADC2ALT;			//reserved / not present in H750
	uint32_t field_34[60];
	uint32_t PKGR;
	uint32_t field_128[118];
	uint32_t UR0;
	uint32_t field_304;
	uint32_t UR2;
	uint32_t UR3;
	uint32_t UR4;
	uint32_t UR5;
	uint32_t UR6;
	uint32_t UR7;
	uint32_t UR8;				//reserved / not present in H735
	uint32_t UR9;				//reserved / not present in H735
	uint32_t UR10;				//reserved / not present in H735
	uint32_t UR11;
	uint32_t UR12;
	uint32_t UR13;
	uint32_t UR14;
	uint32_t UR15;
	uint32_t UR16;
	uint32_t UR17;
} syscfg_t;

enum syscfg_pmcr
{
	ETH_MODE_RMII	= 0x00800000
};


#else

#error Undefined or unspecified SYSCFG_T_VERSION

#endif	//version check

#endif	//include guard
