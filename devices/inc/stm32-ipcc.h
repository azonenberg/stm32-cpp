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

#ifndef stm32_ipcc_h
#define stm32_ipcc_h

#define HAVE_IPCC

//STM32MP257
#if IPCC_T_VERSION == 1

struct ipcc_t
{
	uint32_t	C1CR;
	uint32_t	C1MR;
	uint32_t	C1SCR;
	uint32_t	C1TOC2SR;
	uint32_t	C2CR;
	uint32_t	C2MR;
	uint32_t	C2SCR;
	uint32_t	C2TOC1SR;
	uint32_t	C1SECCFGR;
	uint32_t	C1PRIVCFGR;
	uint32_t	C1CIDCFGR;
	uint32_t	C2SECCFGR;
	uint32_t	C2PRIVCFGR;
	uint32_t	C2CIDCFGR;
	uint32_t	field_9c[213];
	uint32_t	IPCC1_HWCFGR;
	uint32_t	IPCC2_HWCFGR;
	uint32_t	VERR;
	uint32_t	PIDR;
	uint32_t	SIDR;
};

#else

#error Undefined or unspecified IPCC_T_VERSION

#endif	//version check

#endif	//include guard
