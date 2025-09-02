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

#ifndef stm32_tim_h
#define stm32_tim_h

#define HAVE_TIM

//STM32H735
#if TIM_T_VERSION == 1

typedef struct
{
	uint32_t	CR1;
	uint32_t	CR2;
	uint32_t	SMCR;
	uint32_t	DIER;
	uint32_t	SR;
	uint32_t	EGR;
	uint32_t	CCMR1;
	uint32_t	CCMR2;
	uint32_t	CCER;
	uint32_t	CNT;
	uint32_t	PSC;
	uint32_t	ARR;
	uint32_t	RCR;
	uint32_t	CCR1;
	uint32_t	CCR2;
	uint32_t	CCR3;
	uint32_t	CCR4;
	uint32_t	BDTR;
	uint32_t	DCR;
	uint32_t	DMAR;
	uint32_t	field_50;
	uint32_t	CCMR3;
	uint32_t	CCR5;
	uint32_t	CCR6;
	uint32_t	AF1;
	uint32_t	AF2;
	uint32_t	TISEL;
} tim_t;

#else

#error Undefined or unspecified TIM_T_VERSION

#endif	//version check

#endif	//include guard
