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

#ifndef fmc_h
#define fmc_h

#ifdef HAVE_FMC

/**
	@file
	@author Andrew D. Zonenberg
	@brief Flexible memory controller
 */

/**
	@brief Driver for the STM32H7 (and possibly other) flexible memory controller
 */
class FMCBank
{
public:

	/**
		@brief Initialize the FMC
	 */
	FMCBank([[maybe_unused]] volatile fmc_t* fmc, uint8_t bank)
	{
		//TODO: support other banks
		if(bank != 0)
		{
			while(1)
			{}
		}

		//Default configuration
		_FMC.BTR1 =
			(0 << 24) |		//data latency 2 clocks
			(1 << 20) |		//clock frequency fmc_ker_clk / 2
			(0 << 16);		//no bus turnaround delay added

		_FMC.BWTR1 =
			(0 << 16);		//no bus turnaround delay added

		_FMC.BCR1 = FMC_BCR_FMCEN | FMC_BCR_MBKEN | FMC_RESERVED_BITS;
	}

	/**
		@brief Makes the FMC clock run even when not issuing a memory transaciton
	 */
	void EnableFreeRunningClock(bool enable = true)
	{
		if(enable)
			_FMC.BCR1 |= FMC_BCR_CCLKEN;
		else
			_FMC.BCR1 &= ~FMC_BCR_CCLKEN;
	}

	/**
		@brief Enable the memory to be written
	 */
	void EnableWrites(bool enable = true)
	{
		if(enable)
			_FMC.BCR1 |= FMC_BCR_WREN;
		else
			_FMC.BCR1 &= ~FMC_BCR_WREN;
	}

	/**
		@brief Sets the memory as synchronous
	 */
	void SetSynchronous(bool enable = true)
	{
		if(enable)
			_FMC.BCR1 |= (FMC_BCR_CBURSTRW | FMC_BCR_BURSTEN);
		else
			_FMC.BCR1 &= ~(FMC_BCR_CBURSTRW | FMC_BCR_BURSTEN);
	}

	/**
		@brief Enables address/data bus multiplexing
	 */
	void SetAddressDataMultiplex(bool enable = true)
	{
		if(enable)
			_FMC.BCR1 |= FMC_BCR_MUXEN;
		else
			_FMC.BCR1 &= ~FMC_BCR_MUXEN;
	}

	/**
		@brief Set the bus width
	 */
	void SetBusWidth(fmc_bcr_t width)
	{
		_FMC.BCR1 = (_FMC.BCR1 & ~FMC_BCR_WIDTH_MASK) | width;
	}

	/**
		@brief Set the memory type
	 */
	void SetMemoryType(fmc_bcr_t type)
	{
		_FMC.BCR1 = (_FMC.BCR1 & ~FMC_BCR_TYPE_MASK) | type;
	}

	/**
		@brief Puts the PSRAM bank in bank 1 so it's treated as device memory by default
	 */
	void SetPsramBankAs1(bool enable = true)
	{
		if(enable)
			_FMC.BCR1 |= FMC_BCR_BMAP_SWAP;
		else
			_FMC.BCR1 &= ~FMC_BCR_BMAP_SWAP;
	}

	/**
		@brief Enable wait states for synchronous memory
	 */
	void EnableSynchronousWaitStates(bool enable = true)
	{
		if(enable)
			_FMC.BCR1 |= FMC_BCR_WAITEN;
		else
			_FMC.BCR1 &= ~FMC_BCR_WAITEN;
	}

	/**
		@brief Configure timing of the wait state
	 */
	void SetEarlyWaitState(bool early = true)
	{
		if(!early)
			_FMC.BCR1 |= FMC_BCR_WAITCFG;
		else
			_FMC.BCR1 &= ~FMC_BCR_WAITCFG;
	}


protected:
};

#endif

#endif
