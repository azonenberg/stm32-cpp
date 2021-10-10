/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2021 Andrew D. Zonenberg                                                                          *
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

#include <stm32fxxx.h>
#include "Flash.h"

#ifdef STM32F7

/**
	@brief Configures the flash cache, latency, and  prefetching for the specified operating frequency/voltage range
 */
void Flash::SetConfiguration(bool cacheEnable, bool prefetchEnable, int cpuFreqMHz, VoltageRange range)
{
	//Calculate the number of wait states to use
	//Based on table 7 of RM0410, section 3.3.2
	int waitStates = 0;
	switch(range)
	{
		case RANGE_1V8:
			if(cpuFreqMHz <= 20)
				waitStates = 0;
			else if(cpuFreqMHz <= 40)
				waitStates = 1;
			else if(cpuFreqMHz <= 60)
				waitStates = 2;
			else if(cpuFreqMHz <= 80)
				waitStates = 3;
			else if(cpuFreqMHz <= 100)
				waitStates = 4;
			else if(cpuFreqMHz <= 120)
				waitStates = 5;
			else if(cpuFreqMHz <= 140)
				waitStates = 6;
			else if(cpuFreqMHz <= 160)
				waitStates = 7;
			else
				waitStates = 8;
			break;

		case RANGE_2V1:
			if(cpuFreqMHz <= 22)
				waitStates = 0;
			else if(cpuFreqMHz <= 44)
				waitStates = 1;
			else if(cpuFreqMHz <= 66)
				waitStates = 2;
			else if(cpuFreqMHz <= 88)
				waitStates = 3;
			else if(cpuFreqMHz <= 110)
				waitStates = 4;
			else if(cpuFreqMHz <= 132)
				waitStates = 5;
			else if(cpuFreqMHz <= 154)
				waitStates = 6;
			else if(cpuFreqMHz <= 176)
				waitStates = 7;
			else if(cpuFreqMHz <= 198)
				waitStates = 8;
			else
				waitStates = 9;
			break;

		case RANGE_2V4:
			if(cpuFreqMHz <= 24)
				waitStates = 0;
			else if(cpuFreqMHz <= 48)
				waitStates = 1;
			else if(cpuFreqMHz <= 72)
				waitStates = 2;
			else if(cpuFreqMHz <= 96)
				waitStates = 3;
			else if(cpuFreqMHz <= 120)
				waitStates = 4;
			else if(cpuFreqMHz <= 144)
				waitStates = 5;
			else if(cpuFreqMHz <= 168)
				waitStates = 6;
			else if(cpuFreqMHz <= 192)
				waitStates = 7;
			else
				waitStates = 8;
			break;

		case RANGE_2V7:
		default:
			if(cpuFreqMHz <= 30)
				waitStates = 0;
			else if(cpuFreqMHz <= 60)
				waitStates = 1;
			else if(cpuFreqMHz <= 90)
				waitStates = 2;
			else if(cpuFreqMHz <= 120)
				waitStates = 3;
			else if(cpuFreqMHz <= 150)
				waitStates = 4;
			else if(cpuFreqMHz <= 180)
				waitStates = 5;
			else if(cpuFreqMHz <= 210)
				waitStates = 6;
			else
				waitStates = 7;
			break;
	}

	//Configure the access control register
	FLASH.ACR = waitStates;
	if(cacheEnable)
		FLASH.ACR |= FLASH_ACR_ARTEN;
	if(prefetchEnable)
		FLASH.ACR |= FLASH_ACR_PREFETCHEN;

	//TODO: set largest PSIZE possible with the selected voltage
}

#endif
