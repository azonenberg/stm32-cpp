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

#ifdef HAVE_PCIE

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
