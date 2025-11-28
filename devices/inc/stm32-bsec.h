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

#ifndef stm32_bsec_h
#define stm32_bsec_h

#define HAVE_BSEC

//STM32MP257
#if BSEC_T_VERSION == 1

//Fuse addresses
enum shadowed_fuseaddr_t
{
	BSEC_VIRGIN				= 0,	//Device is blank from the fab and not yet factory programmed
	BSEC_ID_0				= 5,	//Unique ID word 0
	BSEC_ID_1				= 6,	//Unique ID word 1
	BSEC_ID_2				= 7,	//Unique ID word 2
	BSEC_RPN				= 9,	//Device part number
	BSEC_BOOTROM_CONFIG_7	= 16,	//Miscellaneous bitfields
	BSEC_PKG				= 122,	//Device package
};

enum nonshadowed_fuseaddr_t
{
	BSEC_FSBLA_COUNT		= 12,	//Monotonic counter for FSBL-A
	BSEC_REV_ID				= 102	//Device revision ID
};

struct bsec_t
{
	uint32_t FVR[376];
	uint32_t field_5e0[136];
	uint32_t SPLOCK[12];
	uint32_t field_830[4];
	uint32_t SWLOCK[12];
	uint32_t field_870[4];
	uint32_t SRLOCK[12];
	uint32_t field_8b0[4];
	uint32_t OTPVLDR[12];
	uint32_t field_8f0[20];
	uint32_t SFSR[12];
	uint32_t field_970[165];
	uint32_t OTPCR;
	uint32_t WDR;
	uint32_t field_c0c[125];
	uint32_t SCRATCHR[4];
	uint32_t LOCKR;
	uint32_t JTAGINR;
	uint32_t JTAGOUTR;
	uint32_t field_e1c;
	uint32_t DENR;
	uint32_t UNMAPR;
	uint32_t field_e28[6];
	uint32_t SR;
	uint32_t OTPSR;
	uint32_t field_e48[62];
	uint32_t WOSCR[8];
	uint32_t field_f60[34];
	uint32_t HRCR;
	uint32_t WRCR;
	uint32_t field_ff0;
	uint32_t VERR;
	uint32_t IPIDR;
	uint32_t SIDR;
};

#else

#error Undefined or unspecified BSEC_T_VERSION

#endif	//version check

#endif	//include guard
