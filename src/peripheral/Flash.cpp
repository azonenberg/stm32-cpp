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

#include <stm32.h>
#include "Flash.h"
#include <string.h>

#ifndef NO_INTERNAL_FLASH

#if defined(STM32H735) || defined(STM32H750)
uint32_t Flash::m_maxPsize = FLASH_CR_PSIZE_X8;
#endif

#ifdef STM32L031

void Flash::SetConfiguration(int hclkFreqMHz, VoltageRange range)
{
	//Calculate the number of wait states to use
	//Based on table 14 of RM0377, page 67
	int latency = 0;
	switch(range)
	{
		case RANGE_VOS1:
			if(hclkFreqMHz > 16)
				latency = 1;
			break;

		case RANGE_VOS2:
			if(hclkFreqMHz > 8)
				latency = 1;
			break;

		case RANGE_VOS3:
			//in lowest range max freq is 4.2 MHz and wait states don't help
			break;
	}

	//Actually configure the flash
	//Enable prefetch and set requested number of wait states
	FLASH.ACR = FLASH_ACR_PREFETCHEN;
	if(latency)
	{
		FLASH.ACR |= FLASH_ACR_LATENCY;

		//Read back to verify write took effect
		while( (FLASH.ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY)
		{}
	}
}

#endif

#ifdef STM32L4

void Flash::SetConfiguration(int hclkFreqMHz, VoltageRange range)
{
	//Calculate the number of wait states to use
	//Based on table 9 of RM0394, section 3.3.3
	unsigned int waitStates = 0;
	if(range == RANGE_VOS1)
	{
		if(hclkFreqMHz <= 16)
			waitStates = 0;
		else if(hclkFreqMHz <= 32)
			waitStates = 1;
		else if(hclkFreqMHz <= 48)
			waitStates = 2;
		else if(hclkFreqMHz <= 64)
			waitStates = 3;
		else //if(hclkFreqMHz <= 80)
			waitStates = 4;
	}
	else //if(range == RANGE_VOS2)
	{
		if(hclkFreqMHz <= 6)
			waitStates = 0;
		else if(hclkFreqMHz <= 12)
			waitStates = 1;
		else if(hclkFreqMHz <= 18)
			waitStates = 2;
		else //if(hclkFreqMHz <= 26)
			waitStates = 3;
	}

	//Actually configure the flash
	//Enable prefetch and I/D caches, set requested number of wait states
	FLASH.ACR = FLASH_ACR_PREFETCHEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | waitStates;

	//Read back to verify write took effect
	while( (FLASH.ACR & 0x7) != waitStates)
	{}
}

#endif

#ifdef STM32H7

/**
	@brief Configures the flash cache, latency, and prefetching for the specified operating frequency/voltage range
 */
void Flash::SetConfiguration(int axiClockFreqMHz, VoltageRange range)
{
	#ifdef STM32H735
		//Calculate the number of wait states to use
		//Based on table 16 of RM0468, section 4.3.8
		unsigned int waitStates = 0;
		switch(range)
		{
			case RANGE_VOS0:
				if(axiClockFreqMHz <= 70)
					waitStates = 0;
				else if(axiClockFreqMHz <= 140)
					waitStates = 1;
				else if(axiClockFreqMHz <= 210)
					waitStates = 2;
				else //if(axiClockFreqMHz <= 275)
					waitStates = 3;
				break;

			case RANGE_VOS1:
				if(axiClockFreqMHz <= 67)
					waitStates = 0;
				else if(axiClockFreqMHz <= 133)
					waitStates = 1;
				else// if(axiClockFreqMHz <= 200)
					waitStates = 2;
				break;

			case RANGE_VOS2:
				if(axiClockFreqMHz <= 50)
					waitStates = 0;
				else if(axiClockFreqMHz <= 100)
					waitStates = 1;
				else// if(axiClockFreqMHz <= 150)
					waitStates = 2;
				break;

			case RANGE_VOS3:
			default:
				if(axiClockFreqMHz <= 35)
					waitStates = 0;
				else if(axiClockFreqMHz <= 70)
					waitStates = 1;
				else// if(axiClockFreqMHz <= 85)
					waitStates = 2;
				break;
		}

		//Configure the access control register
		//Wait states and programming delay are the same
		FLASH.ACR = waitStates | (waitStates << 4);

		//Read back to verify write took effect
		while( (FLASH.ACR & 0xf) != waitStates)
		{}

		//No hardware limits on PSIZE in STM32H7
		m_maxPsize = FLASH_CR_PSIZE_X64;

	#elif defined(STM32H750)

		//Calculate the number of wait states to use
		//Based on table 17 of RM0433, section 4.3.8
		//page 160
		unsigned int waitStates = 0;
		unsigned int wrHighFreq = 0;
		switch(range)
		{
			case RANGE_VOS0:
				if(axiClockFreqMHz <= 70)
					waitStates = 0;
				else if(axiClockFreqMHz <= 140)
				{
					waitStates = 1;
					wrHighFreq = 1;
				}
				else if(axiClockFreqMHz <= 210)
				{
					waitStates = 2;
					wrHighFreq = 2;
				}
				else if(axiClockFreqMHz < 225)
				{
					waitStates = 3;
					wrHighFreq = 2;
				}
				else //if(axiClockFreqMHz <= 240)
				{
					waitStates = 4;
					wrHighFreq = 2;
				}
				break;

			case RANGE_VOS1:
				if(axiClockFreqMHz <= 70)
					waitStates = 0;
				else if(axiClockFreqMHz <= 140)
				{
					waitStates = 1;
					wrHighFreq = 1;
				}
				else if(axiClockFreqMHz <= 210)
				{
					waitStates = 2;
					wrHighFreq = 2;
				}
				else// if(axiClockFreqMHz <= 225)
				{
					waitStates = 3;
					wrHighFreq = 2;
				}
				break;

			case RANGE_VOS2:
				if(axiClockFreqMHz <= 55)
					waitStates = 0;
				else if(axiClockFreqMHz <= 110)
				{
					waitStates = 1;
					wrHighFreq = 1;
				}
				else if(axiClockFreqMHz <= 165)
				{
					waitStates = 2;
					wrHighFreq = 1;
				}
				else if(axiClockFreqMHz < 225)
				{
					waitStates = 3;
					wrHighFreq = 2;
				}
				else// if(axiClockFreqMHz == 225)
				{
					waitStates = 4;
					wrHighFreq = 2;
				}
				break;

			case RANGE_VOS3:
			default:
				if(axiClockFreqMHz <= 45)
					waitStates = 0;
				else if(axiClockFreqMHz <= 90)
				{
					waitStates = 1;
					wrHighFreq = 1;
				}
				else if(axiClockFreqMHz <= 135)
				{
					waitStates = 2;
					wrHighFreq = 1;
				}
				else if(axiClockFreqMHz <= 180)
				{
					waitStates = 3;
					wrHighFreq = 2;
				}
				else// if(axiClockFreqMHz <= 225)
				{
					waitStates = 4;
					wrHighFreq = 2;
				}
				break;
		}

		//Configure the access control register
		FLASH.ACR = waitStates | (wrHighFreq << 4);

		//Read back to verify write took effect
		while( (FLASH.ACR & 0xf) != waitStates)
		{}

		//No hardware limits on PSIZE in STM32H7
		m_maxPsize = FLASH_CR_PSIZE_X64;

	#else

		#error unimplemented

	#endif
}

#endif

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
	//Sector number isn't simple math because sector size isn't uniform on all devices!
	uint32_t addr = reinterpret_cast<uint32_t>(address);
	[[maybe_unused]] int numSector = 0;

	const uint32_t flashStart = 0x08000000;

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
		if( (addr < flashStart) || (numSector < 0) )
			return false;

	#elif defined(STM32H735)

		//Clear any errors we had going in
		FLASH.CCR |= FLASH_SR_ERR_MASK;

		//uniform 128 kB sectors
		static const uint32_t sectorEnds[8] =
		{
			0x0801ffff,
			0x0803ffff,
			0x0805ffff,
			0x0807ffff,
			0x0809ffff,
			0x080bffff,
			0x080dffff,
			0x080fffff
		};

		numSector = -1;
		for(int i=0; i<8; i++)
		{
			if(addr < sectorEnds[i])
			{
				numSector = i;
				break;
			}
		}
		if( (addr < flashStart) || (numSector < 0) )
			return false;

	#elif defined(STM32L431)

		//bail if address is out of range
		if(addr < flashStart)
			return false;

		//uniform 2 kB sectors
		uint32_t sectorStart = addr & ~0x7ff;
		uint32_t flashOffset = sectorStart - flashStart;
		numSector = flashOffset / 2048;

	#elif defined(STM32L031)

		//bail if address is out of range
		if(addr < flashStart)
			return false;

	#else

		(void)addr;
		(void)numSector;

		//not implemented for this target device
		while(1)
		{}
	#endif

	//H7 dual bank flash?
	#if FLASH_T_VERSION == 4

		//TODO: support second flash bank
		while(FLASH.SR1 & FLASH_SR_BUSY)
		{}

	#else
		//Block until flash is free
		while(FLASH.SR & FLASH_SR_BUSY)
		{}
	#endif

	//Clear any errors we had going in
	#if defined(STM32L431) || defined(STM32L031)
		FLASH.SR = FLASH_SR_ERR_MASK;
	#elif defined(STM32H735)
		FLASH.CCR |= FLASH_SR_ERR_MASK;
	#else
		//not implemented for this target device
		while(1)
		{}
	#endif

	//Enable writing
	Unlock();

	//Set maximum PSIZE to get fast erase
	#ifdef STM32H735
		FLASH.CR = (FLASH.CR & ~FLASH_CR_PSIZE_MASK) | m_maxPsize;
	#endif

	//Select the sector being erased
	#if defined(STM32F777)
		FLASH.CR = (FLASH.CR & ~FLASH_CR_SECTOR_MASK) | (numSector << 3) | FLASH_CR_SER;
	#elif defined(STM32L431)
		FLASH.CR = (FLASH.CR & ~FLASH_CR_SECTOR_MASK) | (numSector << 3) | FLASH_CR_PER;
	#elif defined(STM32H735)
		FLASH.CR = (FLASH.CR & ~FLASH_CR_SECTOR_MASK) | (numSector << 8) | FLASH_CR_SER;
	#elif defined(STM32L031)
		FLASH.PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;
	#else
		//not implemented for this target device
		while(1)
		{}
	#endif

	#ifdef STM32L031

		//Do the erase
		*reinterpret_cast<volatile uint32_t*>(address) = 0;

		//Block until flash is free
		asm("dmb st");
		while(FLASH.SR & FLASH_SR_BUSY)
		{}

		//Clear EOP bit if set
		if(FLASH.SR & FLASH_SR_EOP)
			FLASH.SR = FLASH_SR_EOP;

	#elif FLASH_T_VERSION == 4

		//TODO: support second flash bank

		//Do the erase
		FLASH.CR1 |= FLASH_CR_STRT;

		//Block until flash is free
		asm("dmb st");
		while(FLASH.SR1 & FLASH_SR_BUSY)
		{}

	#else
		//Do the erase
		FLASH.CR |= FLASH_CR_STRT;

		//Block until flash is free
		asm("dmb st");
		while(FLASH.SR & FLASH_SR_BUSY)
		{}
	#endif

	//Clear the erase bit
	#ifdef STM32L431
		FLASH.CR &= ~FLASH_CR_PER;
	#elif defined(STM32L031)
		FLASH.PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG);
	#elif defined(STM32H735)
		FLASH.CR &= ~FLASH_CR_SER;
	#endif

	//Re-lock the control register
	Lock();

	//Check for errors
	#if FLASH_T_VERSION == 4
		//TODO: support second flash bank
		if(FLASH.SR1 & FLASH_SR_ERR_MASK)
			return false;
	#else
		if(FLASH.SR & FLASH_SR_ERR_MASK)
			return false;
	#endif

	return true;
}

bool Flash::Write(uint8_t* address, const uint8_t* data, uint32_t len)
{
	//Block until flash is free
	#if FLASH_T_VERSION == 4
		//TODO: support second flash bank
		while(FLASH.SR1 & FLASH_SR_BUSY)
		{}
	#else
		while(FLASH.SR & FLASH_SR_BUSY)
		{}
	#endif

	//Enable writing
	Unlock();

	#ifdef STM32L431

		//Clear program error
		FLASH.SR |= FLASH_SR_ERR_MASK;

		//Write the data in blocks of 8 bytes, force write after last one
		uint32_t tmp[2];
		for(uint32_t i=0; i<len; i+=8)
		{
			//Put stuff in write buffer
			uint32_t blocksize = 8;
			if(i+8 > len)
			{
				blocksize = len - i;
				tmp[0] = 0;
				tmp[1] = 0;
			}
			memcpy((void*)(&tmp[0]), data+i, blocksize);

			//Enable programming
			FLASH.CR |= FLASH_CR_PG;

			//Push to flash, must use 32-bit AHB transactions (no partial blocks allowed)
			*reinterpret_cast<volatile uint32_t*>(address + i) = tmp[0];
			*reinterpret_cast<volatile uint32_t*>(address + i+4) = tmp[1];

			//Wait until done, then check for errors
			asm("dmb st");
			while(FLASH.SR & FLASH_SR_BUSY)
			{}
			if(FLASH.SR & FLASH_SR_ERR_MASK)
			{
				FLASH.SR |= FLASH_SR_ERR_MASK;

				//Clear program enable before returning
				FLASH.CR &= ~FLASH_CR_PG;
				Lock();

				return false;
			}
		}

	#elif defined(STM32L031)

		//Clear program error
		FLASH.SR |= FLASH_SR_ERR_MASK;

		//Write the data in blocks of 4 bytes, force write after last one
		uint32_t tmp;
		for(uint32_t i=0; i<len; i+=4)
		{
			//Put stuff in write buffer
			uint32_t blocksize = 4;
			if(i+4 > len)
			{
				blocksize = len - i;
				tmp = 0;
			}
			memcpy((void*)(&tmp), data+i, blocksize);

			//Enable programming
			//FLASH.PECR |= FLASH_CR_PROG;

			//Push to flash, must use 32-bit AHB transactions (no partial blocks allowed)
			*reinterpret_cast<volatile uint32_t*>(address + i) = tmp;

			//Wait until done, then check for errors
			asm("dmb st");
			while(FLASH.SR & FLASH_SR_BUSY)
			{}
			if(FLASH.SR & FLASH_SR_ERR_MASK)
			{
				FLASH.SR |= FLASH_SR_ERR_MASK;

				//Clear program enable before returning
				//FLASH.CR &= ~FLASH_CR_PROG;
				Lock();

				return false;
			}
		}

	#elif defined(STM32H735)

		//Set PSIZE to our maximum
		FLASH.CR = (FLASH.CR & ~FLASH_CR_PSIZE_MASK) | m_maxPsize;

		//Clear any errors we had going in
		FLASH.CCR |= FLASH_SR_ERR_MASK;

		//Write the data in blocks of 32 bytes; force write after the last one
		uint32_t tmp[8];
		for(uint32_t i=0; i<len; i+=32)
		{
			//Enable programming
			FLASH.CR |= FLASH_CR_PG;

			//Put stuff in write buffer
			uint32_t blocksize = 32;
			if(i+32 > len)
			{
				blocksize = len - i;
				memset(tmp, 0, sizeof(tmp));
			}
			memcpy((void*)(&tmp[0]), data+i, blocksize);

			//Write the buffer to flash
			*reinterpret_cast<volatile uint32_t*>(address + i) = tmp[0];
			*reinterpret_cast<volatile uint32_t*>(address + i+4) = tmp[1];
			*reinterpret_cast<volatile uint32_t*>(address + i+8) = tmp[2];
			*reinterpret_cast<volatile uint32_t*>(address + i+12) = tmp[3];
			*reinterpret_cast<volatile uint32_t*>(address + i+16) = tmp[4];
			*reinterpret_cast<volatile uint32_t*>(address + i+20) = tmp[5];
			*reinterpret_cast<volatile uint32_t*>(address + i+24) = tmp[6];
			*reinterpret_cast<volatile uint32_t*>(address + i+28) = tmp[7];

			//Wait until done, then check for errors
			asm("dmb st");
			while(FLASH.SR & FLASH_SR_BUSY)
			{}
			if(FLASH.SR & FLASH_SR_ERR_MASK)
			{
				FLASH.CCR |= FLASH_SR_ERR_MASK;

				//Clear program enable before returning
				FLASH.CR &= ~FLASH_CR_PG;
				Lock();
				return false;
			}
		}

	#elif defined(STM32H750)

		//unimplemented
		while(1)
		{}

	#else

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
			{
				//Clear program enable before returning
				FLASH.CR &= ~FLASH_CR_PG;

				Lock();
				return false;
			}
		}

	#endif

	//Done
	//Re-lock the control register and exit program mode
	#ifdef STM32L031
		//no special bit to clear
	#elif FLASH_T_VERSION == 4
		//TODO: second flash bank support
		FLASH.CR1 &= ~FLASH_CR_PG;
	#else
		FLASH.CR &= ~FLASH_CR_PG;
	#endif
	Lock();

	return true;
}

#endif
