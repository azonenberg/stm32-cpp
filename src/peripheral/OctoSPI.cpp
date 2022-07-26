/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2022 Andrew D. Zonenberg                                                                          *
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

/**
	@file
	@brief	Implementation of OctoSPI
 */
#include <stm32.h>
#include <algorithm>
#include <peripheral/RCC.h>
#include <peripheral/OctoSPI.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

OctoSPI::OctoSPI(volatile octospi_t* lane, uint32_t sizeBytes, uint8_t prescale)
	: m_lane(lane)
{
	RCCHelper::Enable(lane);

	//5 bit device size field, offset by 1
	//TODO: can this be done in a less ugly fashion?
	int nbits = 0;
	for(int i=0; i<32; i++)
	{
		if( (sizeBytes >> i) & 1)
			nbits = i;
	}
	nbits --;

	//Wait until not busy, then disable
	while(lane->SR & OCTOSPI_BUSY)
	{}
	lane->CR = 0;

	//Default configuration
	lane->DCR1 =
		OCTOSPI_MEM_TYPE_STANDARD |
		(nbits << 16);
	lane->DCR2 = prescale - 1;
	lane->DCR3 = 0;
	lane->DCR4 = 0;
	lane->FCR = OCTOSPI_TOF | OCTOSPI_SMF | OCTOSPI_TCF | OCTOSPI_TEF;
	lane->CCR = 0;
	lane->TCR = 0;
	lane->IR = 0;
	lane->CR = OCTOSPI_EN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mode setting

/**
	@brief Sets the number of cycles to deselect the device for during back-to-back transactions
 */
void OctoSPI::SetDeselectTime(uint8_t ncycles)
{
	ncycles = min(ncycles, (uint8_t)64);
	ncycles --;

	m_lane->DCR1 = (m_lane->DCR1 & ~OCTOSPI_CSHT_MASK) | (ncycles);
}

/**
	@brief Enables or disables DDR mode for all fields of the transaction.
 */
void OctoSPI::SetDoubleRateMode(bool ddr)
{
	uint32_t mask = OCTOSPI_DDTR | OCTOSPI_ABDTR | OCTOSPI_ADDTR | OCTOSPI_IDTR;
	if(ddr)
		m_lane->CCR |= mask;
	else
		m_lane->CCR &= ~mask;
}

/**
	@brief Configures the instruction field
 */
void OctoSPI::SetInstructionMode(mode_t mode, uint8_t nbytes)
{
	nbytes --;
	nbytes = min(nbytes, (uint8_t)3);

	m_lane->CCR &= ~(OCTOSPI_ISIZE_MASK | OCTOSPI_IMODE_MASK);
	m_lane->CCR |= (nbytes << 4) | (mode);
}

/**
	@brief Configures the address field
 */
void OctoSPI::SetAddressMode(mode_t mode, uint8_t nbytes)
{
	nbytes --;
	nbytes = min(nbytes, (uint8_t)3);

	m_lane->CCR &= ~(OCTOSPI_ADSIZE_MASK | OCTOSPI_ADMODE_MASK);
	m_lane->CCR |= (nbytes << 12) | (mode << 8);
}

/**
	@brief Configures the alternate bytes field
 */
void OctoSPI::SetAltBytesMode(mode_t mode, uint8_t nbytes)
{
	nbytes --;
	nbytes = min(nbytes, (uint8_t)3);

	m_lane->CCR &= ~(OCTOSPI_ABSIZE_MASK | OCTOSPI_ABMODE_MASK);
	m_lane->CCR |= (nbytes << 20) | (mode << 16);
}

/**
	@brief Configures the data field
 */
void OctoSPI::SetDataMode(mode_t mode)
{
	m_lane->CCR &= ~OCTOSPI_DMODE_MASK;
	m_lane->CCR |= (mode << 24);
}

/**
	@brief Sets the number of dummy cycles used by the memory
 */
void OctoSPI::SetDummyCycleCount(uint8_t ncycles)
{
	ncycles &= 0x1f;
	m_lane->TCR = (m_lane->TCR & ~OCTOSPI_DCYC_MASK) | ncycles;
}

/**
	@brief Enables or disables DQS
 */
void OctoSPI::SetDQSEnable(bool enable)
{
	if(enable)
		m_lane->CCR |= OCTOSPI_DQSE;
	else
		m_lane->CCR &= ~OCTOSPI_DQSE;
}

/**
	@brief Sets the interrupt full threshold for the FIFO
 */
void OctoSPI::SetFifoThreshold(uint8_t threshold)
{
	m_lane->CR = (m_lane->CR & ~OCTOSPI_FTHRES_MASK) | (threshold << 8);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Indirect bus access

/**
	@brief Simple blocking write operation

	@param insn		Instruction to send (if enabled)
	@param addr		Address to send (if enabled)
	@param data		Data to send
	@param len		Number of bytes of data to send
 */
void OctoSPI::BlockingWrite(uint32_t insn, uint32_t addr, const uint8_t* data, uint32_t len)
{
	//Block until the previous operation completes
	while(m_lane->SR & OCTOSPI_BUSY)
	{}

	m_lane->CR = (m_lane->CR & ~OCTOSPI_FMODE_MASK) | OCTOSPI_FMODE_INDIRECT_WRITE;
	m_lane->DLR = len - 1;
	m_lane->AR = addr;
	m_lane->IR = insn;

	//for now, simple dumb bytewise copy
	volatile uint8_t* buf = reinterpret_cast<volatile uint8_t*>(&m_lane->DR);
	for(uint32_t i=0; i<len; i++)
		*buf = data[i];
}

/**
	@brief Simple blocking read operation

	@param insn		Instruction to send (if enabled)
	@param addr		Address to send (if enabled)
	@param data		Data to send
	@param len		Number of bytes of data to send
 */
void OctoSPI::BlockingRead(uint32_t insn, uint32_t addr, uint8_t* data, uint32_t len)
{
	//Block until the previous operation completes
	while(m_lane->SR & OCTOSPI_BUSY)
	{}

	m_lane->CR = (m_lane->CR & ~OCTOSPI_FMODE_MASK) | OCTOSPI_FMODE_INDIRECT_READ;
	m_lane->DLR = len - 1;
	m_lane->AR = addr;
	m_lane->IR = insn;

	//for now, simple dumb bytewise copy
	volatile uint8_t* buf = reinterpret_cast<volatile uint8_t*>(&m_lane->DR);
	for(uint32_t i=0; i<len; i++)
		data[i] = *buf;
}
