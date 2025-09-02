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

#ifndef stm32_rcc_h
#define stm32_rcc_h

//STM32H735, H750
#if RCC_T_VERSION == 1

typedef struct
{
	uint32_t CR;
	uint32_t HSICFGR;
	uint32_t CRCCR;
	uint32_t CSICFGR;
	uint32_t CFGR;
	uint32_t field_14;
	uint32_t D1CFGR;
	uint32_t D2CFGR;
	uint32_t D3CFGR;
	uint32_t field_24;
	uint32_t PLLCKSELR;
	uint32_t PLLCFGR;
	uint32_t PLL1DIVR;
	uint32_t PLL1FRACR;
	uint32_t PLL2DIVR;
	uint32_t PLL2FRACR;
	uint32_t PLL3DIVR;
	uint32_t PLL3FRACR;
	uint32_t field_48;
	uint32_t D1CCIPR;
	uint32_t D2CCIP1R;
	uint32_t D2CCIP2R;
	uint32_t D3CCIPR;
	uint32_t field_5c;
	uint32_t CIER;
	uint32_t CIFR;
	uint32_t CICR;
	uint32_t field_6c;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t field_78;
	uint32_t AHB3RSTR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB4RSTR;
	uint32_t APB3RSTR;
	uint32_t APB1LRSTR;
	uint32_t APB1HRSTR;
	uint32_t APB2RSTR;
	uint32_t APB4RSTR;
	uint32_t GCR;
	uint32_t field_a4;
	uint32_t D3AMR;
	uint32_t field_ac[9];
	uint32_t RSR;
	uint32_t AHB3ENR;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB4ENR;
	uint32_t APB3ENR;
	uint32_t APB1LENR;
	uint32_t APB1HENR;
	uint32_t APB2ENR;
	uint32_t APB4ENR;
	uint32_t field_f8;
	uint32_t AHB3LPENR;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB4LPENR;
	uint32_t APB3LPENR;
	uint32_t APB1LLPENR;
	uint32_t APB1HLPENR;
	uint32_t APB2LPENR;
	uint32_t APB4LPENR;
	uint32_t field_120[4];
	uint32_t C1_RSR;
	uint32_t C1_AHB3ENR;
	uint32_t C1_AHB1ENR;
	uint32_t C1_AHB2ENR;
	uint32_t C1_AHB4ENR;
	uint32_t C1_APB3ENR;
	uint32_t C1_APB1LENR;
	uint32_t C1_APB1HENR;
	uint32_t C1_APB2ENR;
	uint32_t C1_APB4ENR;
	uint32_t field_158;
	uint32_t C1_AHB3LPENR;
	uint32_t C1_AHB1LPENR;
	uint32_t C1_AHB2LPENR;
	uint32_t C1_AHB4LPENR;
	uint32_t C1_APB3LPENR;
	uint32_t C1_APB1LLPENR;
	uint32_t C1_APB1HLPENR;
	uint32_t C1_APB2LPENR;
	uint32_t C1_APB4LPENR;
} rcc_t;

#else

#error Undefined or unspecified RCC_T_VERSION

#endif	//version check

#endif	//include guard
