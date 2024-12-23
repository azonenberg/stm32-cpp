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

#ifndef stm32_crypt_h
#define stm32_crypt_h

#define HAVE_CRYP

//STM32H735
#if CRYPT_T_VERSION == 1

typedef struct
{
	uint32_t	CR;
	uint32_t	SR;
	uint32_t	DIN;
	uint32_t	DOUT;
	uint32_t	DMACR;
	uint32_t	IMSCR;
	uint32_t	RISR;
	uint32_t	MISR;
	uint32_t	K0LR;
	uint32_t	K0RR;
	uint32_t	K1LR;
	uint32_t	K1RR;
	uint32_t	K2LR;
	uint32_t	K2RR;
	uint32_t	K3LR;
	uint32_t	K3RR;
	uint32_t	IV0LR;
	uint32_t	IV0RR;
	uint32_t	IV1LR;
	uint32_t	IV1RR;
	uint32_t	CSGCMCCM0R;
	uint32_t	CSGCMCCM1R;
	uint32_t	CSGCMCCM2R;
	uint32_t	CSGCMCCM3R;
	uint32_t	CSGCMCCM4R;
	uint32_t	CSGCMCCM5R;
	uint32_t	CSGCMCCM6R;
	uint32_t	CSGCMCCM7R;
	uint32_t	CSGCM0R;
	uint32_t	CSGCM1R;
	uint32_t	CSGCM2R;
	uint32_t	CSGCM3R;
	uint32_t	CSGCM4R;
	uint32_t	CSGCM5R;
	uint32_t	CSGCM6R;
	uint32_t	CSGCM7R;
} cryp_t;

enum cryp_cr
{
	CRYP_ALG_AES_GCM	= 0x80000,
	CRYP_EN				= 0x8000,
	CRYP_GCM_PHASE_INIT	= 0x0,
	CRYP_GCM_PHASE_AAD	= 0x10000,
	CRYP_GCM_PHASE_DATA	= 0x20000,
	CRYP_GCM_PHASE_TAG	= 0x30000,
	CRYP_GCM_PHASE_MASK	= 0x30000,
	CRYP_BSWAP_BYTE		= 0x80,
	CRYP_DECRYPT		= 0x0004,
	CRYP_KEY_128		= 0x0,

	CRYP_FFLUSH			= 0x4000
};

enum cryp_sr
{
	CRYP_BUSY 	= 0x10,
	CRYP_OFNE 	= 0x4,
	CRYP_IFEM	= 0x1
};

enum crypt_dmacr
{
	CRYP_DMACR_DOEN	= 0x02,
	CRYP_DMACR_DIEN	= 0x01
};

#else

#error Undefined or unspecified CRYPT_T_VERSION

#endif	//version check

#endif	//include guard
