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

#ifndef stm32_dcache_h
#define stm32_dcache_h

#define HAVE_DCACHE

//STM32MP257
#if DCACHE_T_VERSION == 1

enum dcache_cr_t
{
	DCACHE_CR_WMISSMRST				= 0x0080'0000,
	DCACHE_CR_WHITMRST				= 0x0040'0000,
	DCACHE_CR_WMISSMEN				= 0x0020'0000,
	DCACHE_CR_WHITMEN				= 0x0010'0000,

	DCACHE_CR_RMISSMRST				= 0x0008'0000,
	DCACHE_CR_RHITMRST				= 0x0004'0000,
	DCACHE_CR_RMISSMEN				= 0x0002'0000,
	DCACHE_CR_RHITMEN				= 0x0001'0000,

	DCACHE_CR_ALLPERF_RST			= DCACHE_CR_WMISSMRST | DCACHE_CR_WHITMRST | DCACHE_CR_RMISSMRST | DCACHE_CR_RHITMRST,
	DCACHE_CR_ALLPERF_EN			= DCACHE_CR_WMISSMEN | DCACHE_CR_WHITMEN | DCACHE_CR_RMISSMEN | DCACHE_CR_RHITMEN,

	DCACHE_CR_CACHEINV				= 0x2,
	DCACHE_CR_EN					= 0x1
};

enum dcache_sr_t
{
	DCACHE_SR_BUSYCMDF				= 0x8,
	DCACHE_SR_BUSYF					= 0x1
};
enum dcache_hwcfgr_t
{
	DCACHE_HWCFGR_ECC				= 0x8000'0000,
};

struct dcache_t
{
	uint32_t 		CR;
	uint32_t 		SR;
	uint32_t 		IER;
	uint32_t 		FCR;
	uint32_t 		RHMONR;
	uint32_t 		RMMONR;
	uint32_t		field_18[2];
	uint32_t		WHMONR;
	uint32_t		WMMONR;
	uint32_t		CMDRSADDRR;
	uint32_t		CMDREADDRR;
	uint32_t		field_30[240];
	uint32_t		HWCFGR;
	uint32_t		VERR;
	uint32_t		IPIDR;
	uint32_t		SIDR;
};
#else

#error Undefined or unspecified DCACHE_T_VERSION

#endif	//version check

#endif	//include guard
