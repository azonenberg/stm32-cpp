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
#include "MDMA.h"
#include "RCC.h"

#ifdef HAVE_MDMA

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Globals

///@brief The global MDMA instance
MDMA g_mdma;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Channel allocator

/**
	@brief Initialize the free list (all channels unallocated) and enables the controller
 */
MDMA::MDMA()
{
	RCCHelper::Enable(&_MDMA);

	for(int i=0; i<NUM_MDMA_CHANNELS; i++)
	{
		m_freeChannels.Push(i);
		m_channels[i].Initialize(i);
	}
}

/**
	@brief Allocate an MDMA channel if one is available, otherwise return null
 */
MDMAChannel* MDMA::AllocateChannel()
{
	if(m_freeChannels.IsEmpty())
		return nullptr;

	auto idx = m_freeChannels.Pop();
	auto chan = &m_channels[idx];
	chan->ConfigureDefaults();
	return chan;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MDMAChannel

/**
	@brief Resets a channel to the default state
 */
void MDMAChannel::ConfigureDefaults()
{
	auto& chan = _MDMA.channels[m_index];

	//no need to clear ISR
	//no need to clear IFCR
	//no need to clear ESR
	chan.CR = 0;
	chan.TCR = 0;
	chan.BNDTR = 0;
	chan.SAR = 0;
	chan.DAR = 0;
	chan.BRUR = 0;
	chan.LAR = 0;
	chan.TBR = 0;
	chan.MAR = 0;
	chan.MDR = 0;
}

#endif
