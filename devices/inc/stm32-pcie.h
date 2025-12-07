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

#ifndef stm32_pcie_h
#define stm32_pcie_h

#define HAVE_PCIE

//STM32MP257
#if PCIE_T_VERSION == 1

struct pcie_type0_cfg_t
{
	uint32_t	bar2;
	uint32_t	bar3;
	uint32_t	bar4;
	uint32_t	bar5;
	uint32_t	legacy_cardbus_cis;
	uint16_t	subsysVendor;
	uint16_t	subsysID;
	uint32_t	expansionRomBase;
	uint8_t		capsPtr;
	uint8_t		padding_caps[3];
	uint32_t	field_14;
	uint8_t		irqLine;
	uint8_t		irqPin;
	uint8_t		minGnt;
	uint8_t		maxLat;
};

struct pcie_type1_cfg_t
{
	uint8_t		primaryBus;
	uint8_t		secondaryBus;
	uint8_t		subordinateBus;
	uint8_t		secondaryLatency;
	uint8_t		ioBase;
	uint8_t		ioLimit;
	uint16_t	secondaryStatus;
	uint16_t	memoryBase;
	uint16_t	memoryLimit;
	uint16_t	prefetchableBase;
	uint16_t	prefetchableLimit;
	uint32_t	prefetchableBaseHigh;
	uint32_t	prefetchableLimitHigh;
	uint16_t	ioBaseHigh;
	uint16_t	ioLimitHigh;
	uint8_t		capsPtr;
	uint8_t		padding_caps[3];
	uint32_t	expansionRomBase;
	uint8_t		irqLine;
	uint8_t		irqPin;
	uint16_t	bridgeCtl;
};

//PCIe type base configuration space
struct pcie_cfg_t
{
	//Base fields common to all types
	uint16_t	vendor;
	uint16_t	device;
	uint16_t	command;
	uint16_t	status;
	uint32_t	class_revision;
	uint8_t		cacheLineSize;
	uint8_t		latencyTimer;
	uint8_t		headerType;
	uint8_t		bist;
	uint32_t	bar0;
	uint32_t	bar1;

	//Type specific fields
	union
	{
		pcie_type0_cfg_t	type0;
		pcie_type1_cfg_t	type1;
	};

	//extensions come later
};

enum pcie_tlptype_t
{
	PCIE_TLP_TYPE_NORMAL	= 0,
	PCIE_TLP_TYPE_IO		= 2,
	PCIE_TLP_TYPE_CONFIG	= 4
};

enum pcie_region_ctrl_2_t
{
	PCIE_REGION_CTL_2_REGION_EN	= 0x8000'0000
};

struct pcie_atublock_t
{
	uint32_t	region_ctrl_1;
	uint32_t	region_ctrl_2;
	uint32_t	lwr_base_addr;
	uint32_t	upper_base_addr;	//high half of base address, leave zero
	uint32_t	limit_addr;
	uint32_t	lwr_target_addr;
	uint32_t	upper_target_addr;	//high half of target address, leave zero
	uint32_t	region_ctrl_3;		//only used for SRIOV, ignore/leave zero
	uint32_t	upper_limit_addr;	//high half of limit address, leave zero
	uint32_t	field_24[55];
};

struct pcie_atu_cfg_t
{
	pcie_atublock_t	outbound;
	pcie_atublock_t	inbound;
};

struct pcie_vc_t
{
	uint32_t	P_RX_Q_CTRL;
	uint32_t	NP_RX_Q_CTRL;
	uint32_t	CPL_RX_Q_CTRL;
};

enum dllptype_t
{
	PCIE_DLLP_TYPE_VENDOR_SPEC = 0x30
};

enum pcie_port_link_ctrl_t
{
	PCIE_VENDOR_SPECIFIC_DLLP_REQ		= 0x1
};

enum pcie_misc_control_1_t
{
	PCIE_DBI_RO_WR_EN					= 0x1
};

enum pcie_gen2_ctrl_t
{
	PCIE_GEN2_CTRL_DIRECT_SPEED_CHANGE	= 0x2'0000
};

struct pcie_portlogic_t
{
	uint32_t	ACK_LATENCY_TIMER;
	uint32_t	VENDOR_SPEC_DLLP;
	uint32_t	PORT_FORCE;
	uint32_t	ACK_F_ASPM_CTRL;
	uint32_t	PORT_LINK_CTRL;
	uint32_t	LANE_SKEW;
	uint32_t	TIMER_CTRL_MAX_FUNC_NUM;
	uint32_t	SYMBOL_TIMER_FILTER_1;
	uint32_t	FILTER_MASK_2;
	uint32_t	AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL;
	uint32_t	PL_DEBUG0;
	uint32_t	PL_DEBUG1;
	uint32_t	TX_P_FC_CREDIT_STATUS;
	uint32_t	TX_NP_FC_CREDIT_STATUS;
	uint32_t	TX_CPL_FC_CREDIT_STATUS;
	uint32_t	QUEUE_STATUS;
	uint32_t	VC_TX_ARBI_1;
	uint32_t	VC_TX_ARBI_2;
	pcie_vc_t	vcqctl[2];		//TODO how many VCs in our instance?
	uint32_t	field_760[53];
	uint32_t	GEN2_CTRL;
	uint32_t	PHY_STATUS;
	uint32_t	PHY_CONTROL;
	uint32_t	field_818;
	uint32_t	TRGT_MAP_CTRL;
	uint32_t	field_820[33];
	uint32_t	CLOCK_GATING_CTRL;
	uint32_t	GEN3_RELATED;
	uint32_t	GEN3_EQ_LOCAL_FS_LF;
	uint32_t	GEN3_EQ_PSET_COEFF_MAP[11];
	uint32_t	GEN3_EQ_PSET_INDEX;
	uint32_t	field_8a0;
	uint32_t	GEN3_EQ_COEFF_LEGALITY_STATUS;
	uint32_t	GEN3_EQ_CONTROL;
	uint32_t	GEN3_EQ_FB_MODE_DIR_CHANGE;
	uint32_t	field_8b0;
	uint32_t	ORDER_RULE_CTRL;
	uint32_t	PIPE_LOOPBACK_CONTROL;
	uint32_t	MISC_CONTROL_1;
};

enum pcie_captype_t
{
	PCIE_CAPTYPE_PCIE	= 0x10,
	PCIE_CAPTYPE_MSI	= 0x05,
	PCIE_CAPTYPE_PM		= 0x01
};

enum pcie_ptype_t
{
	PCIE_PTYPE_ENDPOINT		= 0,
	PCIE_PTYPE_ROOT_OF_ROOT	= 4
};

//PCIe base capability structure
struct pcie_cap_t
{
	//Common to all extended caps
	uint8_t		type;
	uint8_t		nextcap;

	//Generic capability register, type dependent
	uint16_t	capreg;
};

enum pcie_devcaps_t
{
	PCIE_DEVCAPS_FLR	= 0x1000'0000
};

//PCIe capability structure
struct pcie_pcie_cap_t
{
	pcie_cap_t	base;
	uint32_t	device_caps;
	uint32_t	device_ctrl_status;
	uint32_t	link_caps;
};

struct pcie_t
{
	//Base configuration registers
	pcie_cfg_t	base;
};

#else

#error Undefined or unspecified PCIE_T_VERSION

#endif	//version check

#endif	//include guard
