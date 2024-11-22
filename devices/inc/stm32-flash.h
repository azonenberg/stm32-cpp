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

#ifndef stm32_flash_h
#define stm32_flash_h

#define HAVE_FLASH

//STM32H735
#if FLASH_T_VERSION == 1

typedef struct
{
	uint32_t ACR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t CR;
	uint32_t SR;
	uint32_t CCR;
	uint32_t OPTCR;
	uint32_t OPTSR_CUR;
	uint32_t OPTSR_PRG;
	uint32_t OPTCCR;
	uint32_t PRAR_CUR;
	uint32_t PRAR_PRG;
	uint32_t SCAR_CUR;
	uint32_t SCAR_PRG;
	uint32_t WPSN_CUR;
	uint32_t WPSN_PRG;
	uint32_t BOOT_CUR;
	uint32_t BOOT_PRG;
	uint32_t padding1[2];
	uint32_t CRCCR;
	uint32_t CRCSADDR;
	uint32_t CRCEADDR;
	uint32_t CRCDATAR;
	uint32_t ECC_FAR;
	uint32_t padding2[3];
	uint32_t OPTSR2_CUR;
	uint32_t OPTSR2_PRG;
} flash_t;

enum flash_cr
{
	FLASH_CR_LOCK			= 0x1,
	FLASH_CR_STRT			= 0x80,

	FLASH_CR_PSIZE_MASK		= 0x30,
	FLASH_CR_PSIZE_X8		= 0x00,
	FLASH_CR_PSIZE_X16		= 0x10,
	FLASH_CR_PSIZE_X32		= 0x20,
	FLASH_CR_PSIZE_X64		= 0x30,

	FLASH_CR_SECTOR_MASK	= 0x700,

	FLASH_CR_FW				= 0x40,
	FLASH_CR_SER			= 0x4,
	FLASH_CR_PG				= 0x2
};

enum flash_sr
{
	FLASH_SR_BUSY			= 0x1,

	FLASH_SR_DBECCERR		= 0x04000000,

	FLASH_SR_ERR_MASK		= 0x7EE0400
};

#define HAVE_FLASH_ECC

//STM32L431
#elif FLASH_T_VERSION == 2

typedef struct
{
	uint32_t ACR;
	uint32_t PDKEYR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t ECCR;
	uint32_t OPTR;
	uint32_t PCROP1SR;
	uint32_t PCROP1ER;
	uint32_t WRP1AR;
	uint32_t WRP1BR;
} flash_t;

enum flash_acr
{
	FLASH_ACR_DCEN			= 0x400,
	FLASH_ACR_ICEN 			= 0x200,
	FLASH_ACR_PREFETCHEN	= 0x100,
};

enum flash_cr
{
	FLASH_CR_LOCK			= 0x80000000,
	FLASH_CR_STRT			= 0x10000,

	FLASH_CR_SECTOR_MASK	= 0x7f8,

	FLASH_CR_PER			= 0x2,
	FLASH_CR_PG				= 0x1
};

enum flash_sr
{
	FLASH_SR_BUSY			= 0x10000,

	FLASH_SR_ERR_MASK		= 0xC3FA
};

enum flash_eccr
{
	FLASH_ECCR_ECCD			= 0x80000000,
	FLASH_ECCR_ECCC			= 0x40000000
};

#define HAVE_FLASH_ECC

//STM32L031
#elif FLASH_T_VERSION == 3

typedef struct
{
	uint32_t ACR;
	uint32_t PECR;
	uint32_t PDKEYR;
	uint32_t PKEYR;
	uint32_t PRGKEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t OPTR;
	uint32_t WRPROT1;
	uint32_t WRPROT2;
} flash_t;

enum flash_acr
{
	FLASH_ACR_PREFETCHEN	= 0x2,
	FLASH_ACR_LATENCY		= 0x1
};

enum flash_pecr
{
	FLASH_PECR_ERASE		= 0x200,
	FLASH_PECR_PROG			= 0x008,
	FLASH_PECR_PRGLOCK		= 0x002,
	FLASH_PECR_PELOCK		= 0x001
};

enum flash_sr
{
	FLASH_SR_ERR_MASK		= 0x10600,
	FLASH_SR_EOP			= 0x2,
	FLASH_SR_BUSY			= 0x1
};

#else

#error Undefined or unspecified FLASH_T_VERSION

#endif	//version check

#endif	//include guard
