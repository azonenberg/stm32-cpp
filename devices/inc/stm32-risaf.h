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

#ifndef stm32_risaf_h
#define stm32_risaf_h

#define HAVE_RISAF

//STM32MP257
#if RISAF_T_VERSION == 1

enum risaf_cfgr_t
{
	RISAF_CFGR_SEC		= 0x100,
	RISAF_CFGR_BREN		= 0x01
};

enum risaf_iaesr_t
{
	RISAF_IAESR_IANRW	= 0x80,
	RISAF_IAESR_IASEC	= 0x20,
	RISAF_IAESR_IAPRIV	= 0x10
};

enum risaf_sr_t
{
	RISAF_SR_ENCDIS		= 0x4
};

enum risaf_iasr_t
{
	RISAF_IASR_CAEF		= 1,
	RISAF_IASR_IAEF0	= 2,
	RISAF_IASR_IAEF1	= 4
};

struct risafrgn_t
{
	uint32_t	CFGR;
	uint32_t	STARTR;
	uint32_t	ENDR;
	uint32_t	CIDCFGR;
	uint32_t	ACFGR;
	uint32_t	ASTARTR;
	uint32_t	AENDR;
	uint32_t	ANESTR;
	uint32_t	BCFGR;
	uint32_t	BSTARTR;
	uint32_t	BENDR;
	uint32_t	BNESTR;
	uint32_t	field_30[4];
};

/*
	NOTE: not all regions are implemented in all RISAFs.
	RISAF1 has 4, RISAF2 has 4, RISAF5 has 2, only RISAF4 has the full 15 implemented
 */
struct risaf_t
{
	uint32_t	CR;
	uint32_t	SR;
	uint32_t	IASR;
	uint32_t	IACR;
	uint32_t	field_10[4];
	uint32_t	IAESR0;
	uint32_t	IADDR0;
	uint32_t	IAESR1;
	uint32_t	IADDR1;
	uint32_t	KEYR[4];
	risafrgn_t	regions[15];
	uint32_t	field_3f0[764];
	uint32_t	HWCFGR;
	uint32_t	VERR;
	uint32_t	IPIDR;
	uint32_t	SIDR;
};

#else

#error Undefined or unspecified RISAF_T_VERSION

#endif	//version check

#endif	//include guard
