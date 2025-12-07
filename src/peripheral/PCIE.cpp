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
#include <peripheral/PCIE.h>
#include <peripheral/RCC.h>

#ifdef HAVE_PCIE

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initial hardware configuration

/**
	@brief Initialize the PCIe controller

	@param refclkType				Select source and frequency of the SERDES reference clock
	@param autoTrainToFullSpeed		Automatically send a SpeedChange request to 5 GT/s after link up if set
 */
void PCIE::Initialize(RefclkType refclkType, bool autoTrainToFullSpeed)
{
	//Turn on clocks to the PCIe PHY and protocol stack
	RCCHelper::Enable(&_COMBOPHY);
	RCCHelper::Enable(&_PCIE);

	switch(refclkType)
	{
		/*
			SSC off, refout buffer off, no input divider, SSC refclk is 100 MHz, multiplier 25, use ext clock
			this also gives 2500 MHz VCO
		 */
		case REFCLK_EXT_100MHZ:
			//g_log("Clock setup: using external 100 MHz REF_P/N\n");
			SYSCFG.COMBOPHYCR1 = 0x33;
			break;

		/*
			poweron default config

			SSC off, refout buffer off, no input divider, SSC refclk is 25 MHz, multiplier 100, use RCC clock
			I think this translates to 25 MHz * 100 = 2500 MHz VCO, presumably DDR PHY
		 */
		case REFCLK_INT_25MHZ:
			//g_log("Clock setup: using internal 25 MHz from RCC ck_ker_usb3pciephy\n");
			SYSCFG.COMBOPHYCR1 = 0xc8;
			break;

		default:
			//g_log(Logger::ERROR, "Unrecognized PCIe clock source\n");
			while(1)
			{}
			break;
	}

	//Don't isolate the COMBOPHY, run in PCIe mode
	//Spread spectrum disabled
	SYSCFG.COMBOPHYCR2 = 0x0000'8000;

	//COMBOPHYCR3 is I/O buffer control, leave this default for now
	//COMBOPHYCR4 is I/O buffer control, leave default for now
	//COMBOPHYCR5 is I/O buffer control, leave default for now

	//Request RTUNE calibration
	SYSCFG.COMBOPHYCR3 = SYSCFG.COMBOPHYCR3 | 0x4000'0000;
	while( (SYSCFG.COMBOPHYSR & 1) != 1)
	{}
	//g_log("Termination resistor calibration complete\n");
	SYSCFG.COMBOPHYCR3 = SYSCFG.COMBOPHYCR3 & ~0x4000'0000;

	//Wait for PHY initialization
	while( (SYSCFG.COMBOPHYSR & 2) != 0)
	{}
	//g_log("PHY ready\n");

	//Request train to full speed if user wants it
	if(autoTrainToFullSpeed)
	{
		auto portLogic = GetPortLogic();
		portLogic->GEN2_CTRL |= PCIE_GEN2_CTRL_DIRECT_SPEED_CHANGE;
	}

	//Configure the PCIe control register: bus/device 00:00, LTSSM enabled
	SYSCFG.PCIECR = 0x404;

	//Disable AXI bus bandwidth limiters
	//TODO: do we need this? we have BW to spare it looks
	SYSCFG.ICNPCIBWLCR = 0;
	SYSCFG.ICNCPU1BWLCR = 0;
	SYSCFG.ICNE2EBWRCR = 0;
}

void PCIE::WaitForLinkUp()
{
	while( (SYSCFG.PCIESR1 & 0x24) != 0x24)
	{}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iATU stuff

/**
	@brief Setup an iATU outbound region

	@param iregion		Region index. Ranges from 0 to 3 on the STM32MP2, other IP configs might be different
	@param tlptype		TLP type to translate the requests to
	@param cpuaddr		Start address on the CPU side (not an offset)
	@param cpulimit		End address on the CPU side (not an offset)
	@param targetaddr	Destination address on the peripheral side
 */
void PCIE::SetupOutboundATURegion(
	size_t iregion,
	pcie_tlptype_t tlptype,
	uint32_t cpuaddr,
	uint32_t cpulimit,
	uint32_t targetaddr)
{
	auto atu = PCIE::GetATU();
	auto& region = atu[iregion].outbound;

	//TLP header, leave other fields zeroed
	region.region_ctrl_1		= tlptype;

	//Hook up addresses
	region.lwr_base_addr		= cpuaddr;
	region.limit_addr			= cpulimit;
	region.lwr_target_addr		= targetaddr;

	//Tie off high half of all 64-bit fields. We're a 64-bit system but nothing in the pcie address space is above 4GB
	region.upper_base_addr		= 0x0000'0000;
	region.upper_limit_addr		= 0x0000'0000;
	region.upper_target_addr	= 0x0000'0000;

	//region_ctrl_3 is only used for SR-IO, zero it
	region.region_ctrl_3		= 0x0000'0000;

	//Enable the region after configuring it
	region.region_ctrl_2	= PCIE_REGION_CTL_2_REGION_EN;

	//Read back and wait for write to commit
	while( (region.region_ctrl_2 & PCIE_REGION_CTL_2_REGION_EN) == 0)
	{}
}

/**
	@brief Setup an iATU inbound region

	@param iregion		Region index. Ranges from 0 to 3 on the STM32MP2, other IP configs might be different
	@param tlptype		TLP type to translate the requests to
	@param cpuaddr		Start address on the CPU side (not an offset)
	@param cpulimit		End address on the CPU side (not an offset)
	@param targetaddr	Destination address on the peripheral side
 */
void PCIE::SetupInboundATURegion(
	size_t iregion,
	pcie_tlptype_t tlptype,
	uint32_t cpuaddr,
	uint32_t cpulimit,
	uint32_t targetaddr)
{
	auto atu = PCIE::GetATU();
	auto& region = atu[iregion].inbound;

	//TLP header, leave other fields zeroed
	region.region_ctrl_1		= tlptype;

	//Hook up addresses
	region.lwr_base_addr		= cpuaddr;
	region.limit_addr			= cpulimit;
	region.lwr_target_addr		= targetaddr;

	//Tie off high half of all 64-bit fields. We're a 64-bit system but nothing in the pcie address space is above 4GB
	region.upper_base_addr		= 0x0000'0000;
	region.upper_limit_addr		= 0x0000'0000;
	region.upper_target_addr	= 0x0000'0000;

	//region_ctrl_3 is only used for SR-IO, zero it
	region.region_ctrl_3		= 0x0000'0000;

	//Enable the region after configuring it
	region.region_ctrl_2	= PCIE_REGION_CTL_2_REGION_EN;

	//Read back and wait for write to commit
	while( (region.region_ctrl_2 & PCIE_REGION_CTL_2_REGION_EN) == 0)
	{}
}

/**
	@brief Clear an inbound iATU region to empty
 */
void PCIE::ClearInboundATURegion(size_t iregion)
{
	auto atu = PCIE::GetATU();
	auto& region = atu[iregion].inbound;

	region.region_ctrl_1 = 0x0000'0000;
}

#endif
