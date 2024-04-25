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

#ifndef rtc_h
#define rtc_h

#ifdef HAVE_RTC

#include <peripheral/RCC.h>
#include <time.h>

/**
	@brief The realtime clock (and backup SRAM)

	RTC has pclk for APB and ker_clk, where does ker_clk come from? have to check RCC
	can clock from LSE, divided HSE, LSI
 */
class RTC
{
public:

	#ifdef STM32H735
	/**
		@brief Set up kernel clock in the RCC to use divided HSE

		@param prediv	Pre-divider value for kernel clock. Must be between 2 and 63, and Fhse / prediv must be <1 MHz
	 */
	static void SetClockFromHSE(uint8_t prediv)
	{
		//Set RTCPRE in RCC_CFGR to configure the divider
		RCC.CFGR = (RCC.CFGR & ~0x3f00) | (prediv << 8);

		//Set DBP in PWR_CR1 to 1 to enable change
		PWR.CR1 |= 0x100;

		//Dummy read of CR1 (per RM0468 page 284) to confirm write has taken effect
		[[maybe_unused]]
		volatile int unused = PWR.CR1;

		//Set RTCSEL in RCC_BDCR to configure the RTC clock to come from the HSE, then enable rtc_ck
		RCC.BDCR = (RCC.BDCR & ~0x300) | 0x8300;
	}

	static void SetPrescaleAndTime(uint16_t predivA, uint16_t predivS, const tm& now, uint16_t frac);

	static void GetTime(tm& now, uint16_t& frac);

	static void Unlock()
	{
		_RTC.WPR = 0xca;
		_RTC.WPR = 0x53;
	}

	static void Lock()
	{ _RTC.WPR = 0x00; }

	#endif

	volatile uint8_t* GetBackupMemory()
	{ return reinterpret_cast<volatile uint8_t*>(&_RTC.BKP[0]); }
};

#endif

#endif
