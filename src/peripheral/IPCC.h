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

#ifndef IOPCC_h
#define IOPCC_h

#ifdef HAVE_IPCC

/**
	@brief A pointer that is always 64 bits in storage even if the native pointer size is smaller

	Used for data exchange between 32 and 64 bit cores sharing a common physical address space in the low 4GB
 */
template<class T>
class PaddedPointer
{
public:
	PaddedPointer()
	{}

	PaddedPointer(volatile T* p)
	{ Set(p); }

	volatile T* Get()
	{ return reinterpret_cast<T*>(m_ptr); }

	void Set(volatile T* p)
	{ m_ptr = reinterpret_cast<uint64_t>(p); }

	T* operator->()
	{ return const_cast<T*>(Get()); }

protected:
	volatile uint64_t m_ptr;
};

class IPCC
{
public:

	///@brief Create the IPCC object
	IPCC(volatile ipcc_t* ipcc)
		: m_ipcc(ipcc)
	{
		RCCHelper::Enable(ipcc);
	}

	///@brief Check if the channel is free in the primary to secondary direction (should be low half)
	bool IsPrimaryToSecondaryChannelFree(uint32_t mask)
	{ return (m_ipcc->C1TOC2SR & mask) == 0; }

	///@brief Check if the channel is free in the secondary to primary direction (should be low half)
	bool IsSecondaryToPrimaryChannelFree(uint32_t mask)
	{ return (m_ipcc->C2TOC1SR & mask) == 0; }

	///@brief Marks the channel as occupied in the primary to secondary direction (should be high half)
	void SetPrimaryToSecondaryChannelBusy(uint32_t setmask)
	{ m_ipcc->C1SCR = setmask; }

	///@brief Marks the channel as free in the primary to secondary direction (should be low half)
	void SetPrimaryToSecondaryChannelFree(uint32_t clearmask)
	{ m_ipcc->C2SCR = clearmask; }

	///@brief Marks the channel as occupied in the secondary to primary direction (should be high half)
	void SetSecondaryToPrimaryChannelBusy(uint32_t setmask)
	{ m_ipcc->C2SCR = setmask; }

	///@brief Marks the channel as free in the secondary to primary direction (should be low half)
	void SetSecondaryToPrimaryChannelFree(uint32_t clearmask)
	{ m_ipcc->C1SCR = clearmask; }

	///@brief Initialize it (done on the primary side)
	void Initialize()
	{
		//Set all channels to secure privileged with no CID filtering
		m_ipcc->C1CIDCFGR = 0;
		m_ipcc->C1SECCFGR = 0xffff;
		m_ipcc->C1PRIVCFGR = 0xffff;

		m_ipcc->C2CIDCFGR = 0;
		m_ipcc->C2SECCFGR = 0xffff;
		m_ipcc->C2PRIVCFGR = 0xffff;

		//Disable all interrupts
		m_ipcc->C1CR = 0;
		m_ipcc->C2CR = 0;

		//Ignore mask in CxMR because we're not using interrupts yet
	}

protected:
	PaddedPointer<ipcc_t> m_ipcc;
};

#endif

#endif
