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

#ifndef DCACHE_h
#define DCACHE_h

#ifdef HAVE_DCACHE

//@brief STM32MP2 etc DCACHE block
class DCACHE
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
	{ return (_DCACHE.CR & DCACHE_CR_EN) ? true : false; }

	///@brief Turn the cache on
	static void Enable()
	{
		WaitIdle();
		_DCACHE.CR |= DCACHE_CR_EN;
	}

	///@brief Invalidate all lines in the cache
	static void InvalidateAll()
	{
		_DCACHE.CR |= DCACHE_CR_CACHEINV;
		WaitIdle();
	}

	//TODO: clean/invalidate address range

	///@brief Check if the cache is busy
	static bool IsBusy()
	{ return (_DCACHE.SR & (DCACHE_SR_BUSYF | DCACHE_SR_BUSYCMDF)) != 0; }

	///@brief Reset all performance counters
	static void PerfResetAll()
	{
		_DCACHE.CR |= DCACHE_CR_ALLPERF_RST;
		_DCACHE.CR &= ~DCACHE_CR_ALLPERF_RST;
	}

	///@brief Enable all performance counters
	static void PerfEnableAll()
	{ 	_DCACHE.CR |= DCACHE_CR_ALLPERF_EN; }

	///@brief Return the current number of read cache hits (saturates at max)
	static uint32_t PerfGetReadHitCount()
	{ return _DCACHE.RHMONR; }

	///@brief Return the current number of read cache misses (note this is only 16 bits and saturates at max)
	static uint16_t PerfGetReadMissCount()
	{ return _DCACHE.RMMONR; }

	///@brief Return the current number of write cache hits (saturates at max)
	static uint32_t PerfGetWriteHitCount()
	{ return _DCACHE.WHMONR; }

	///@brief Return the current number of write cache misses (note this is only 16 bits and saturates at max)
	static uint16_t PerfGetWriteMissCount()
	{ return _DCACHE.WMMONR; }

	///@brief Get width of the AHB master interface
	static uint32_t GetAHBMWidth()
	{
		switch( (_DCACHE.HWCFGR >> 23) & 3)
		{
			case 0:	return 0;
			case 1: return 32;
			case 2: return 64;
			default: return 0;
		}
	}

	///@brief Get width of the cache line, in bytes
	static uint32_t GetCacheLineWidth()
	{
		switch( (_DCACHE.HWCFGR >> 9) & 3)
		{
			case 1:	return 8;
			case 2: return 16;
			case 3: return 32;
			default: return 0;
		}
	}

	///@brief Get the total cache size, in kB
	static uint32_t GetCacheSize()
	{ return 1 << ( (_DCACHE.HWCFGR >> 4) & 7 ); }

	///@brief Get the number of ways of associacivity
	static uint32_t GetNumWays()
	{ return 1 << ( _DCACHE.HWCFGR & 3 ); }
};

#endif

#endif
