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

#ifndef ICACHE_h
#define ICACHE_h

#ifdef HAVE_ICACHE

//@brief STM32MP2 etc ICACHE block
class ICACHE
{
public:

	///@brief Wait until we're not busy
	static void WaitIdle()
	{
		while(IsBusy())
		{}
	}

	///@brief Check if the cache is enabled
	static bool IsEnabled()
	{ return (_ICACHE.CR & ICACHE_CR_EN) ? true : false; }

	///@brief Turn the cache on
	static void Enable()
	{
		WaitIdle();
		_ICACHE.CR |= ICACHE_CR_EN;
	}

	///@brief Invalidate all lines in the cache
	static void InvalidateAll()
	{
		_ICACHE.CR |= ICACHE_CR_CACHEINV;
		WaitIdle();
	}

	///@brief Check if the cache is busy
	static bool IsBusy()
	{ return (_ICACHE.SR & ICACHE_SR_BUSYF) != 0; }

	///@brief Return the current number of cache hits (saturates at max)
	static uint32_t PerfGetHitCount()
	{ return _ICACHE.HMONR; }

	///@brief Return the current number of cache misses (note this is only 16 bits and saturates at max)
	static uint16_t PerfGetMissCount()
	{ return _ICACHE.MMONR; }

	///@brief Reset the hit counter
	static void PerfResetHitCount()
	{
		_ICACHE.CR |= ICACHE_CR_HITMRST;
		_ICACHE.CR &= ~ICACHE_CR_HITMRST;
	}

	///@brief Reset the miss counter
	static void PerfResetMissCount()
	{
		_ICACHE.CR |= ICACHE_CR_MISSMRST;
		_ICACHE.CR &= ~ICACHE_CR_MISSMRST;
	}

	///@brief Enable the hit counter
	static void PerfEnableHitCount()
	{ _ICACHE.CR |= ICACHE_CR_HITMEN;  }

	///@brief Enable the miss counter
	static void PerfEnableMissCount()
	{ _ICACHE.CR |= ICACHE_CR_MISSMEN; }

	///@brief Get width of the AHB slave interface
	static uint32_t GetAHBSWidth()
	{
		switch( (_ICACHE.HWCFGR >> 26) & 3)
		{
			case 0:	return 32;
			case 1: return 64;
			case 2: return 128;
			default: return 0;
		}
	}

	///@brief Get width of the AHB master2 interface
	static uint32_t GetAHBM2Width()
	{
		switch( (_ICACHE.HWCFGR >> 23) & 3)
		{
			case 0:	return 0;
			case 1: return 32;
			case 2: return 64;
			default: return 0;
		}
	}

	///@brief Get width of the AHB master1 interface
	static uint32_t GetAHBM1Width()
	{
		switch( (_ICACHE.HWCFGR >> 21) & 3)
		{
			case 0:	return 0;
			case 1: return 32;
			case 2: return 64;
			default: return 128;
		}
	}

	///@brief Get the number of available remap regions
	static uint32_t GetRemapRegions()
	{ return (_ICACHE.HWCFGR >> 16) & 7; }

	///@brief Get the size of each remap region, in MB
	static uint32_t GetRemapRegionSize()
	{ return 1 << ( (_ICACHE.HWCFGR >> 12) & 0xf); }

	///@brief Get width of the cache line, in bytes
	static uint32_t GetCacheLineWidth()
	{
		switch( (_ICACHE.HWCFGR >> 9) & 3)
		{
			case 1:	return 8;
			case 2: return 16;
			case 3: return 32;
			default: return 0;
		}
	}

	///@brief Get the total cache size, in kB
	static uint32_t GetCacheSize()
	{ return 1 << ( (_ICACHE.HWCFGR >> 4) & 7 ); }

	///@brief Get the number of ways of associacivity
	static uint32_t GetNumWays()
	{ return 1 << ( _ICACHE.HWCFGR & 3 ); }
};

#endif

#endif
