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

#ifndef Flash_h
#define Flash_h

#include <stm32.h>

/**
	@brief Flash memory

	All functions are static because there's only one flash subsystem in the device.
 */
class Flash
{
public:

	#ifdef STM32L031
	static void SetConfiguration(int hclkFreqMHz, VoltageRange range);
	#endif

	#ifdef STM32L4
	static void SetConfiguration(int hclkFreqMHz, VoltageRange range);
	#endif

	#ifdef STM32H7
	static void SetConfiguration(int axiClockFreqMHz, VoltageRange range);
	#endif

	#ifdef HAVE_FLASH_ECC
	static bool CheckForECCFaults()
	{ return (FLASH.SR & FLASH_SR_DBECCERR) != 0; }

	static void ClearECCFaults()
	{ FLASH.CCR |= FLASH_SR_DBECCERR; }

	static uint32_t GetFaultAddress()
	{ return 0x08000000 + FLASH.ECC_FAR*32; }
	#endif

	#ifdef STM32F7

	enum VoltageRange
	{
		RANGE_1V8,	//1.8V - 2.1V
		RANGE_2V1,	//2.1V - 2.4V
		RANGE_2V4,	//2.4V - 2.7V
		RANGE_2V7,	//2.7V and up
	};

	static void SetConfiguration(bool cacheEnable, bool prefetchEnable, int cpuFreqMHz, VoltageRange range);

	#endif

	static bool BlockErase(uint8_t* address);
	static bool Write(uint8_t* address, const uint8_t* data, uint32_t len);

protected:
	static void Unlock()
	{
		#if FLASH_T_VERSION == 3
			//enable access to FLASH_PECR
			FLASH.PKEYR = 0x89abcdef;
			FLASH.PKEYR = 0x02030405;

			//eanble actual flash programming
			FLASH.PRGKEYR = 0x8c9daebf;
			FLASH.PRGKEYR = 0x13141516;
		#else
			FLASH.KEYR = 0x45670123;
			FLASH.KEYR = 0xCDEF89AB;
		#endif
	}

	static void Lock()
	{
		#if FLASH_T_VERSION == 3
			FLASH.PECR |= FLASH_PECR_PRGLOCK;
			FLASH.PECR |= FLASH_PECR_PELOCK;
		#else
			FLASH.CR |= FLASH_CR_LOCK;
		#endif
	}

	#ifdef STM32H735
	static uint32_t m_maxPsize;
	#endif
};

#endif
