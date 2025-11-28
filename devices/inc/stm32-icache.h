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

#ifndef stm32_icache_h
#define stm32_icache_h

#define HAVE_ICACHE

//STM32MP257
#if ICACHE_T_VERSION == 1

enum icache_cr_t
{
	ICACHE_CR_MISSMRST				= 0x80000,
	ICACHE_CR_HITMRST				= 0x40000,
	ICACHE_CR_MISSMEN				= 0x20000,
	ICACHE_CR_HITMEN				= 0x10000,
	ICACHE_CR_MODE_ASSOCIATIVE		= 0x4,
	ICACHE_CR_CACHEINV				= 0x2,
	ICACHE_CR_EN					= 0x1
};

enum icache_sr_t
{
	ICACHE_SR_BUSYF					= 0x1
};

enum icache_hwcfgr_t
{
	ICACHE_HWCFGR_ECC				= 0x8000'0000,
};

struct icache_region_t
{
	uint32_t		CRR;
};

struct icache_t
{
	uint32_t 		CR;
	uint32_t 		SR;
	uint32_t 		IER;
	uint32_t 		FCR;
	uint32_t 		HMONR;
	uint32_t 		MMONR;
	uint32_t		field_18[2];
	icache_region_t regions[4];
	uint32_t		field_30[240];
	uint32_t		HWCFGR;
	uint32_t		VERR;
	uint32_t		IPIDR;
	uint32_t		SIDR;
};

#else

#error Undefined or unspecified ICACHE_T_VERSION

#endif	//version check

#endif	//include guard
