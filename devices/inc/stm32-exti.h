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

#ifndef stm32_exti_h
#define stm32_exti_h

#define HAVE_EXTI

//STM32L431
#if EXTI_T_VERSION == 1

typedef struct
{
	uint32_t	IMR1;
	uint32_t	EMR1;
	uint32_t	RTSR1;
	uint32_t	FTSR1;
	uint32_t	SWIER1;
	uint32_t	PR1;
	uint32_t	IMR2;
	uint32_t	EMR2;
	uint32_t	RTSR2;
	uint32_t	FTSR2;
	uint32_t	SWIER2;
	uint32_t	PR2;
} exti_t;

//STM32L031
#elif EXTI_T_VERSION == 2

typedef struct
{
	uint32_t	IMR;
	uint32_t	EMR;
	uint32_t	RTSR;
	uint32_t	FTSR;
	uint32_t	SWIER;
	uint32_t	PR;
} exti_t;

//STM32MP257
#elif EXTI_T_VERSION == 3

typedef struct
{
	uint32_t	RTSR1;
	uint32_t	FTSR1;
	uint32_t	SWIER1;
	uint32_t	RPR1;
	uint32_t	FPR1;
	uint32_t	SECCFGR1;
	uint32_t	PRIVCFGR1;
	uint32_t	field_1c;
	uint32_t	RTSR2;
	uint32_t	FTSR2;
	uint32_t	SWIER2;
	uint32_t	RPR2;
	uint32_t	FPR2;
	uint32_t	SECCFGR2;
	uint32_t	PRIVCFGR2;
	uint32_t	field_3c;
	uint32_t	RTSR3;
	uint32_t	FTSR3;
	uint32_t	SWIER3;
	uint32_t	RPR3;
	uint32_t	FPR3;
	uint32_t	SECCFGR3;
	uint32_t	PRIVCFGR3;
	uint32_t	field_5c;
	uint32_t	EXTICR1;
	uint32_t	EXTICR2;
	uint32_t	EXTICR3;
	uint32_t	EXTICR4;
	uint32_t	LOCKR;
	uint32_t	field_74[3];
	uint32_t	C1IMR1;
	uint32_t	field_84[3];
	uint32_t	C1IMR2;
	uint32_t	field_94[3];
	uint32_t	IMR3;
	uint32_t	field_a4[7];
	uint32_t	C2IMR1;
	uint32_t	C2EMR1;
	uint32_t	field_c8;
	uint32_t	field_cc;
	uint32_t	C2IMR2;
	uint32_t	field_d4[3];
	uint32_t	C2IMR3;
	uint32_t	C2EMR3;
	uint32_t	field_e8[38];
	uint32_t	CIDCFGR[85];
	uint32_t	field_2d4[11];
	uint32_t	C1CIDCFRGR;
	uint32_t	C2CIDCFRGR;
	uint32_t	field_308[46];
	uint32_t	HWCFGR13;
	//TODO: finish adding fields
} exti_t;	//EXTI1, is EXTI2 different?

#else

#error Undefined or unspecified EXTI_T_VERSION

#endif	//version check

#endif	//include guard
