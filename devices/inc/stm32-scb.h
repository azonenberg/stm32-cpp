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

#ifndef stm32_scb_h
#define stm32_scb_h

#include "stm32-mpu.h"

//ARMv8-M (currently Cortex-M33)
#if SCB_T_VERSION == 3

//so far we don't implement the nonsecure alias
typedef struct
{
	uint32_t	CPUID;
	uint32_t	ICSR;
	uint32_t	VTOR;
	uint32_t	AIRCR;
	uint32_t	SCR;	//ed10
	uint32_t	CCR;
	uint8_t		SHP[12];
	uint32_t	SHCSR;
	uint32_t	CFSR;	//ed28, contains MMFSR / BFSR / UFSR
	uint32_t	HFSR;
	uint32_t	field_30;
	uint32_t	MMFAR;
	uint32_t	BFAR;
	uint32_t	AFSR;
	uint32_t	ID_PFR0;
	uint32_t	ID_PFR1;
	uint32_t	ID_DFR0;
	uint32_t	ID_AFR0;
	uint32_t	ID_MMFR[4];
	uint32_t	ID_ISAR[6];
	uint32_t	CLIDR;
	uint32_t	CTR;
	uint32_t	CCSIDR;
	uint32_t	CSSELR;
	uint32_t	CPACR;
	uint32_t	NSACR;
	mpu_t		_MPU;
	uint32_t	field_edc8;
} scb_t;

#else

//ARMv7-M
typedef struct
{
	uint32_t	CPUID;
	uint32_t	ICSR;
	uint32_t	VTOR;
	uint32_t	AIRCR;
	uint32_t	SCR;
	uint32_t	CCR;
	uint8_t		SHP[12];
	uint32_t	SHCR;
	uint32_t	CFSR;
	uint32_t	HFSR;
	uint32_t	DFSR;
	uint32_t	MMFAR;
	uint32_t	BFAR;
	uint32_t	AFSR;
	uint32_t	PFR[2];
	uint32_t	DFR;
	uint32_t	ADR;
	uint32_t	MMFR[4];
	uint32_t	ISAR[5];
	uint32_t	field_e000ed74;
#if SCB_T_VERSION == 2
	uint32_t	CLIDR;
	uint32_t	CTR;
	uint32_t	CCSIDR;
	uint32_t	CCSELR;
	uint32_t	CPACR;
	uint32_t	field_e000ed8c;
	mpu_t		_MPU;
	uint32_t	field_e000eda4[22];
	uint32_t	DEMCR;
	uint32_t	field_e000ee00[64];
	uint32_t	STIR;
	uint32_t	field_e000ef04[19];
	uint32_t	ICIALLU;
	uint32_t	field_e000ef54;
	uint32_t	ICIMVAU;
	uint32_t	DCIMVAC;
	uint32_t	DCISW;
	uint32_t	DCCMVAU;
	uint32_t	DCCMVAC;
	uint32_t	DCCSW;
	uint32_t	DCCIMVAC;
	uint32_t	DCCISW;
	uint32_t	BPIALL;
	uint32_t	field_e000ef7c;
	uint32_t	field_e000ef80;
	uint32_t	field_e000ef84;
	uint32_t	field_e000ef88;
	uint32_t	field_e000ef8c;
	uint32_t	CM7_ITCMCR;
	uint32_t	CM7_DTCMCR;
	uint32_t	CM7_AHBPCR;
	uint32_t	CM7_CACR;
	uint32_t	CM7_AHBSCR;
	uint32_t	field_e000efa4;
	uint32_t	CM7_ABFSR;

	//more after this
#endif
} scb_t;

#endif

#endif	//include guard
