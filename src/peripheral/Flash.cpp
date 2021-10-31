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

uint32_t Flash::m_maxPsize = FLASH_CR_PSIZE_X8;

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

	//Set largest PSIZE possible with the selected voltage
	switch(range)
	{
		case RANGE_1V8:
			m_maxPsize = FLASH_CR_PSIZE_X8;
			break;

		case RANGE_2V1:
		case RANGE_2V4:
			m_maxPsize = FLASH_CR_PSIZE_X16;
			break;

		case RANGE_2V7:
		default:
			m_maxPsize = FLASH_CR_PSIZE_X32;
			break;
	}
}

#endif

/**
	@brief Erases the block of flash containing the specified address
 */
bool Flash::BlockErase(uint8_t* address)
{
	//Sector number isn't simple math because sector size isn't uniform!
	uint32_t addr = reinterpret_cast<uint32_t>(address);
	int numSector = 0;

	#ifdef STM32F777

		//TODO: dual bank support
		static const uint32_t sectorEndsSingleBank[12] =
		{
			0x08007fff,	//32 kB
			0x0800ffff,	//32 kB
			0x08017fff,	//32 kB
			0x0801ffff,	//32 kB
			0x0803ffff,	//128 kB
			0x0807ffff,	//256 kB
			0x080bffff,	//256 kB
			0x080fffff,	//256 kB
			0x0813ffff,	//256 kB
			0x0817ffff,	//256 kB
			0x081bffff,	//256 kB
			0x081fffff,	//256 kB
		};

		numSector = -1;
		for(int i=0; i<12; i++)
		{
			if(addr < sectorEndsSingleBank[i])
			{
				numSector = i;
				break;
			}
		}
		if( (addr < 0x08000000) || (numSector < 0) )
			return false;

	#else

		//not implemented for this target device
		return false;
	#endif

	//Block until flash is free
	while(FLASH.SR & FLASH_SR_BUSY)
	{}

	//Enable writing
	Unlock();

	//Set maximum PSIZE to get fast erase
	FLASH.CR = (FLASH.CR & ~FLASH_CR_PSIZE_MASK) | m_maxPsize;

	//Select the sector being erased
	FLASH.CR = (FLASH.CR & ~FLASH_CR_SECTOR_MASK) | (numSector << 3) | FLASH_CR_SER;

	//Do the erase
	FLASH.CR |= FLASH_CR_STRT;

	//Block until flash is free
	asm("dmb st");
	while(FLASH.SR & FLASH_SR_BUSY)
	{}

	//Re-lock the control register
	FLASH.CR |= FLASH_CR_LOCK;

	//Check for errors
	if(FLASH.SR & FLASH_SR_ERR_MASK)
		return false;
	return true;
}

bool Flash::Write(uint8_t* address, const uint8_t* data, uint32_t len)
{
	//Block until flash is free
	while(FLASH.SR & FLASH_SR_BUSY)
	{}

	//Enable writing
	Unlock();

	//Set PSIZE to byte regardless of our maximum
	//TODO: enable fast writes for large, aligned data?
	FLASH.CR = (FLASH.CR & ~FLASH_CR_PSIZE_MASK) | FLASH_CR_PSIZE_X8;

	//Write the data one byte at a time
	for(uint32_t i=0; i<len; i++)
	{
		//Enable programming
		FLASH.CR |= FLASH_CR_PG;

		//Do the actual write
		address[i] = data[i];

		//Wait until done, then check for errors
		asm("dmb st");
		while(FLASH.SR & FLASH_SR_BUSY)
		{}
		if(FLASH.SR & FLASH_SR_ERR_MASK)
			return false;
	}

	//Done
	//Re-lock the control register
	FLASH.CR |= FLASH_CR_LOCK;

	return true;
}
