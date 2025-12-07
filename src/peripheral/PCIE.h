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

#ifndef PCIE_h
#define PCIE_h

#ifdef HAVE_PCIE

///@brief PCIe controller
class PCIE
{
public:

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// System configuration and setup

	enum RefclkType
	{
		REFCLK_EXT_100MHZ,	//100 MHz on PCIE_CLKIN_P/N
		REFCLK_INT_25MHZ	//25 MHz from RCC (on ck_ker_usb3pciephy, flexclkgen channel 34)
	};

	static void Initialize(RefclkType refclkType, bool autoTrainToFullSpeed);
	static void WaitForLinkUp();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Link status



	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Runtime helpers

	/**
		@brief Send a vendor-specific DLLP (type 0x30)

		@param payload 24 bit DLLP payload (high 8 bits ignored)
	 */
	static void SendVendorSpecificDLLP(uint32_t payload)
	{
		auto portLogic = GetPortLogic();
		portLogic->VENDOR_SPEC_DLLP = (payload << 8) | PCIE_DLLP_TYPE_VENDOR_SPEC;
		portLogic->PORT_LINK_CTRL |= PCIE_VENDOR_SPECIFIC_DLLP_REQ;
	}

	static void UnlockConfig()
	{ GetPortLogic()->MISC_CONTROL_1 |= PCIE_DBI_RO_WR_EN; }

	static void LockConfig()
	{ GetPortLogic()->MISC_CONTROL_1 &= ~PCIE_DBI_RO_WR_EN; }

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ATU configuration

	static void SetupOutboundATURegion(
		size_t iregion,
		pcie_tlptype_t tlptype,
		uint32_t cpuaddr,
		uint32_t cpulimit,
		uint32_t targetaddr);

	static void SetupInboundATURegion(
		size_t iregion,
		pcie_tlptype_t tlptype,
		uint32_t cpuaddr,
		uint32_t cpulimit,
		uint32_t targetaddr);

	static void ClearInboundATURegion(size_t iregion);

protected:
	static volatile pcie_atu_cfg_t* GetATU()
	{ return reinterpret_cast<volatile pcie_atu_cfg_t*>(reinterpret_cast<volatile uint8_t*>(&_PCIE) + 0x30'0000); }

	static volatile pcie_portlogic_t* GetPortLogic()
	{ return reinterpret_cast<volatile pcie_portlogic_t*>(reinterpret_cast<volatile uint8_t*>(&_PCIE) + 0x700); }

	//no pcie member, assume we have only one instance so we use it implicitly
};

#endif

#endif
