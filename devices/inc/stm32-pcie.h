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

/*
struct pcie_atu_cfg_t
{
};
*/

struct pcie_t
{
	//Base configuration registers
	pcie_cfg_t	base;

	//TODO: ATU
};

#else

#error Undefined or unspecified PCIE_T_VERSION

#endif	//version check

#endif	//include guard
