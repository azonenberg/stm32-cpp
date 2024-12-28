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

#ifndef itm_h
#define itm_h

/**
	@brief The Instrumentation Trace Macrocell
 */
class ITM
{
public:

	/**
		@brief Unlock the ITM
	 */
	static void Unlock()
	{ _ITM.LAR = ITM_LAR_UNLOCK; }

	/**
		@brief Enable the ITM (also turns on trace support globally)
	 */
	static void Enable()
	{
		Unlock();

		//Enable tracing system wide (turns on DWT and ITM)
		SCB.DEMCR |= DEMCR_TRCENA;

		#ifdef STM32L431
		DBGMCU.CR |= DBGMCU_CR_TRACE_IOEN;

		//Turn off TPIU formatting (TODO make this nicer)
		*reinterpret_cast<volatile uint32_t*>(0xe0040304) = 0x100;
		#endif

		//Enable the ITM itself
		_ITM.TCR |= ITM_TCR_ITMENA;
	}

	/**
		@brief Enable an ITM stimulus channel
	 */
	static void EnableChannel(uint8_t chan)
	{ _ITM.TER[chan / 32] |= (1 << (chan % 32)); }

	/**
		@brief Enable forwarding of hardware events from the DWT to the ITM and TPIU
	 */
	static void EnableDwtForwarding()
	{ _ITM.TCR |= ITM_TCR_TXENA; }

	/**
		@brief Send a byte of data out a stimulus channel
	 */
	static void SendByte(uint8_t chan, uint8_t val)
	{
		while(_ITM.STIM[chan] == 0)
		{}

		*reinterpret_cast<volatile uint8_t*>(&_ITM.STIM[chan]) = val;
	}

};

#endif
