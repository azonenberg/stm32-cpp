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

#ifndef RISAF_h
#define RISAF_h

#ifdef HAVE_RISAF

///@brief Resource Isolation Slave Unit for Address space, Full
class RISAF
{
public:

	RISAF(volatile risaf_t* risaf)
		: m_risaf(risaf)
	{
		//Enable the RCC clock used for the RISAF block.
		//NOTE: The RISAF does not have its own clock enable! Configuring it turns on the clock to whatever
		//peripheral it's in front of (e.g. for RISAF5 it's the PCIe block).
		RCCHelper::Enable(risaf);
	}

	/**
		@brief Configure a base region

		@param idx			Region index
		@param baseAddress	Start offset of the region (relative to start of peripheral's address space)
		@param endAddress	End offset of the region (relative to start of peripheral's address space)
		@param cidWriteMask	Bitmask of CIDs to give write access
		@param cidReadMask	Bitmask of CIDs to give read access
		@param cidPrivMask	Bitmask of CIDs to restrict to privileged mode only
							(if unset, both privileged and unprivileged access are allowed)
		@param secureMode	If true, accesses from any CID must be secure.
							If false, accesses from any CID must be nonsecure.
	 */
	void ConfigureBaseRegion(
		size_t idx,
		uint32_t baseAddress,
		uint32_t endAddress,
		uint8_t cidWriteMask,
		uint8_t cidReadMask,
		uint8_t cidPrivMask,
		bool secureMode)
	{
		m_risaf->regions[idx].STARTR = baseAddress;
		m_risaf->regions[idx].ENDR = endAddress;
		m_risaf->regions[idx].CIDCFGR = (cidWriteMask << 16) | cidReadMask;
		m_risaf->regions[idx].CFGR = (cidPrivMask << 16);
		if(secureMode)
			m_risaf->regions[idx].CFGR |= RISAF_CFGR_SEC;
		m_risaf->regions[idx].CFGR |= RISAF_CFGR_BREN;	//must enable after configuring
	}

protected:
	volatile risaf_t* m_risaf;
};

#endif

#endif
