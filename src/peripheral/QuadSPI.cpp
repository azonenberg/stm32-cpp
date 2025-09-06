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

/**
	@file
	@brief	Implementation of QuadSPI
 */
#include <stm32.h>
#include <algorithm>
#include <peripheral/RCC.h>
#include <peripheral/QuadSPI.h>

#ifdef HAVE_QUADSPI

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

QuadSPI::QuadSPI(volatile quadspi_t* lane, uint32_t sizeBytes, uint8_t prescale)
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
	Abort();
	WaitIdle();
	lane->CR = 0;

	//Default configuration
	lane->CR = (prescale-1) << 24;
	lane->DCR = (nbits << 16);
	lane->CCR = 0;
	lane->FCR = QUADSPI_TOF | QUADSPI_SMF | QUADSPI_TCF | QUADSPI_TEF;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mode setting

/**
	@brief Sets the number of cycles to deselect the device for during back-to-back transactions
 */
void QuadSPI::SetDeselectTime(uint8_t ncycles)
{
	ncycles = min(ncycles, (uint8_t)64);
	ncycles --;

	m_lane->DCR = (m_lane->DCR & ~QUADSPI_CSHT_MASK) | (ncycles << 8);
}

/**
	@brief Enables or disables DDR mode for all fields of the transaction other than the instruction.
 */
void QuadSPI::SetDoubleRateMode(bool ddr)
{
	if(ddr)
		m_ccrBase |= QUADSPI_DDRM;
	else
		m_ccrBase &= ~QUADSPI_DDRM;
}

/**
	@brief Configures the instruction field
 */
void QuadSPI::SetInstructionMode(mode_t mode)
{
	m_ccrBase = (m_ccrBase & ~QUADSPI_IMODE_MASK) | (mode << 8);
}

/**
	@brief Configures the address field
 */
void QuadSPI::SetAddressMode(mode_t mode, uint8_t nbytes)
{
	nbytes --;
	nbytes = min(nbytes, (uint8_t)3);

	m_ccrBase &= ~(QUADSPI_ADSIZE_MASK | QUADSPI_ADMODE_MASK);
	m_ccrBase |= (nbytes << 12) | (mode << 10);
}

/**
	@brief Configures the alternate bytes field
 */
void QuadSPI::SetAltBytesMode(mode_t mode, uint8_t nbytes)
{
	nbytes --;
	nbytes = min(nbytes, (uint8_t)3);

	m_ccrBase &= ~(QUADSPI_ABSIZE_MASK | QUADSPI_ABMODE_MASK);
	m_ccrBase |= (nbytes << 16) | (mode << 14);
}

/**
	@brief Configures the data field
 */
void QuadSPI::SetDataMode(mode_t mode)
{
	m_ccrBase &= ~QUADSPI_DMODE_MASK;
	m_ccrBase |= (mode << 24);
}

/**
	@brief Sets the number of dummy cycles used by the memory
 */
void QuadSPI::SetDummyCycleCount(uint8_t ncycles)
{
	ncycles &= 0x1f;
	m_ccrBase = (m_ccrBase & ~QUADSPI_DCYC_MASK) | (ncycles << 18);
}

/**
	@brief Sets the interrupt full threshold for the FIFO
 */
void QuadSPI::SetFifoThreshold(uint8_t threshold)
{
	m_lane->CR = (m_lane->CR & ~QUADSPI_FTHRES_MASK) | (threshold << 8);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Indirect bus access

/**
	@brief Send a single byte command then restore config
 */
void QuadSPI::SendSingleByteCommand(uint8_t cmd)
{
	WaitIdle();

	//Send the command
	m_lane->CCR =
		QUADSPI_FMODE_INDIRECT_WRITE |
		(MODE_SINGLE << 8) |
		cmd;
}

/**
	@brief Simple blocking write operation

	@param insn		Instruction to send (if enabled)
	@param addr		Address to send (if enabled)
	@param data		Data to send
	@param len		Number of bytes of data to send
 */

void QuadSPI::BlockingWrite(uint32_t insn, uint32_t addr, const uint8_t* data, uint32_t len)
{
	//Block until the previous operation completes
	WaitIdle();

	m_lane->DLR = len - 1;
	m_lane->CCR = m_ccrBase | QUADSPI_FMODE_INDIRECT_WRITE | insn;
	m_lane->AR = addr;

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
void QuadSPI::BlockingRead(uint32_t insn, uint32_t addr, uint8_t* data, uint32_t len)
{
	//Block until the previous operation completes
	WaitIdle();

	m_lane->DLR = len - 1;
	m_lane->CCR = m_ccrBase | QUADSPI_FMODE_INDIRECT_READ | insn;
	m_lane->AR = addr;

	//for now, simple dumb bytewise copy
	volatile uint8_t* buf = reinterpret_cast<volatile uint8_t*>(&m_lane->DR);
	for(uint32_t i=0; i<len; i++)
		data[i] = *buf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory map the QUADSPI

void QuadSPI::SetMemoryMapMode(uint32_t insn)
{
	WaitIdle();

	m_lane->CCR = m_ccrBase | QUADSPI_FMODE_MEMORY_MAP | insn;
}

#endif
