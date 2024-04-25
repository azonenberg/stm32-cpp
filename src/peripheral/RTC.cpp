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

#include <stm32.h>
#include "RTC.h"

#ifdef STM32H735

/**
	@brief Sets additional prescalers

	predivA ranges from 1 to 256, output clocks the sub-second counter
	predivS ranges from 1 to 32768

	For now, convention is that we use 10 kHz fractional ticks
	TODO: this isn't compatible with 32 kHz external clock, so we need settings for that?
 */
void RTC::SetPrescaleAndTime(uint16_t predivA, uint16_t predivS, const tm& now, uint16_t frac)
{
	//RM0468 51.4.4 prescalers are offset by 1
	uint32_t predivAoff = predivA - 1;
	uint32_t predivSoff = predivS - 1;

	Unlock();

	//Set INIT 1 in ISR
	_RTC.ISR |= 0x80;

	//Poll INITF until set
	while( (_RTC.ISR & 0x40) != 0x40)
	{}

	//Enable 24 hour hardware clock
	_RTC.CR &= ~0x40;

	//Program the prescaler
	_RTC.PRER = (predivAoff << 16) | predivSoff;

	//Set time and date (can't modify sub seconds it looks?)
	_RTC.TR =
		( (now.tm_hour / 10) << 20) |
		( (now.tm_hour % 10) << 16) |
		( (now.tm_min / 10) << 12) |
		( (now.tm_min % 10) << 8) |
		( (now.tm_sec / 10) << 4) |
		(now.tm_sec % 10);

	int year = now.tm_year - 100;
	_RTC.DR =
		( (year / 10) << 20) |
		( (year % 10) << 16) |
		((now.tm_wday + 1) << 13) |
		((now.tm_mon / 10) << 12) |
		((now.tm_mon % 10) << 8) |
		((now.tm_mday / 10) << 4) |
		(now.tm_mday % 10);

	//Turn on bypass mode
	_RTC.CR |= 0x20;

	//Clear INIT bit
	_RTC.ISR &= ~0x80;

	//Wait for INITS to be set (RTC is out of reset so we can modify sub second value)
	while( (_RTC.ISR & 0x40) != 0)
	{}

	//Wait for SHPF to be clear (no shift pending)
	while( (_RTC.ISR & 0x8) != 0)
	{}

	//Shift sub second clock counter to match the offset
	//By default, the sub-second counter is 9999 (down counting).
	//We want to decrement until it matches the current fractional divide (see 51.4.11).
	//Add 1 second to compensate for the subtraction
	uint32_t shift = 9999 - frac;
	_RTC.SHIFTR = shift | 0x80000000;

	//Wait for shift to complete
	while( (_RTC.ISR & 0x8) != 0)
	{}

	Lock();
}

/**
	@brief Gets the current time as a libc-formatted time structure plus fractional seconds
 */
void RTC::GetTime(tm& now, uint16_t& frac)
{
	//Initial read of timestamp registers
	uint32_t ssr = _RTC.SSR;
	uint32_t tr = _RTC.TR;
	uint32_t dr = _RTC.DR;

	//If the sub-second register wrapped around since we read the initial values, we got carries
	//and the previous values are invalid. If so, read again.
	//(no need to check a third time since this process takes way less than a second)
	uint32_t ssrnext = _RTC.SSR;
	if(ssrnext < ssr)
	{
		ssr = ssrnext;
		tr = _RTC.TR;
		dr = _RTC.DR;
	}

	//Crack the time/date fields
	//ignore PM bit since we always use 24-hour clock
	now.tm_hour =
		( ((tr >> 20) & 0x3) * 10) +	//hour tens
		( (tr >> 16) & 0xf );			//hour ones

	now.tm_min =
		( ((tr >> 12) & 0x7) * 10) +	//minute tens
		( (tr >> 8) & 0xf );			//minute ones

	now.tm_sec =
		( ((tr >> 4) & 0x7) * 10) +		//second tens
		(tr & 0xf );					//second ones

	//Crack the date fields
	now.tm_year = 100 +					//assume year is between 2000 and 2099, RTC is only 2 digit year
										//(did we learn nothing from y2k??? the most significant byte is free...)
		( ( (dr >> 20) & 0xf)*10 ) +	//year tens
		( (dr >> 16) & 0xf);			//year ones

	now.tm_wday = ((dr >> 13) & 0x7) - 1;	//day of week

	now.tm_mon =
		((dr >> 12) & 0x1) * 10 +		//month tens
		( (dr >> 8) & 0xf );			//month ones

	now.tm_mday =
		((dr >> 4) & 0x3) * 10 +		//day tens
		(dr & 0xf );					//day ones
	now.tm_isdst = 0;

	//No parsing needed for fractional value just mask off high bits
	ssr &= 0xffff;

	//Handle wrapped seconds right after NTP sync
	const uint32_t fullscale = 9999;
	if(ssr > fullscale)
	{
		frac = fullscale*2 - ssr;

		//Renormalize time (but not date)
		//This can lead to midnight on the 1st wrapping to 23:59:59 on the 0th if we happen to NTP sync exactly then.
		//This is probably fine for now?
		if(now.tm_sec == 0)
		{
			now.tm_sec = 59;
			if(now.tm_min == 0)
			{
				now.tm_min = 59;
				if(now.tm_hour == 0)
				{
					now.tm_hour = 23;
					now.tm_mday --;
				}
				else
					now.tm_hour --;
			}
			else
				now.tm_min --;
		}
		else
			now.tm_sec --;
	}
	else
		frac = fullscale - ssr;
}

#endif
