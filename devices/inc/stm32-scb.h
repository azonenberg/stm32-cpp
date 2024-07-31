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
	uint32_t	CSSELR;
	uint32_t	CPACR;
	//more after this
#endif
} scb_t;

#endif	//include guard
