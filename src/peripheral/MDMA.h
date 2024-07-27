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

#ifndef mdma_h
#define mdma_h

#ifdef HAVE_MDMA

#include <embedded-utils/FIFO.h>

/**
	@brief A single channel of the MDMA
 */
class MDMAChannel
{
public:
	MDMAChannel()
	{}

	/**
		@brief
	 */
	uint32_t GetIndex()
	{ return m_index; }

	//Internals, only used by MDMA class (not part of public API)
public:
	/**
		@brief Initializes the channel after creation
	 */
	void Initialize(uint32_t index)
	{ m_index = index; }

	void ConfigureDefaults();

protected:
	uint32_t	m_index;
};

/**
	@brief Top level wrapper for the MDMA

	Mostly just an allocator for DMA channels.

	Exactly one instance of this class should be created, at global scope, if the MDMA is being used.
 */
class MDMA
{
public:
	MDMA();

	MDMAChannel* AllocateChannel();

	/**
		@brief Returns a channel to the free list
	 */
	void FreeChannel(MDMAChannel* chan)
	{ m_freeChannels.Push(chan->GetIndex()); }

protected:
	FIFO<uint8_t, NUM_MDMA_CHANNELS> m_freeChannels;

	MDMAChannel m_channels[NUM_MDMA_CHANNELS];
};

extern MDMA g_mdma;

#endif

#endif
