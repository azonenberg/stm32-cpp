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

#include <stm32.h>
#include <ctype.h>
#include <string.h>
#include <peripheral/RCC.h>
#include <peripheral/SPI.h>

#ifdef HAVE_SPI

/**
	@brief Initialize a SPI lane

	@param lane			The peripheral to use
	@param fullDuplex	True in normal full-duplex mode (MOSI+MISO)
						False in half-duplex mode (MOSI used bidirectionally)
	@param baudDiv		Baud rate divisor from APB clock (must be power of two)
 */
SPI::SPI(volatile spi_t* lane, bool fullDuplex, uint16_t baudDiv)
	: m_lane(lane)
	, m_fullDuplex(fullDuplex)
{
	RCCHelper::Enable(lane);

	//8-bit word size, set RXNE as soon as we have a byte in the FIFO
	//TODO: make word size configurable?
	lane->CR2 = (7 << 8) | SPI_FRXTH;

	//Turn on the peripheral in master mode.
	//To prevent problems, we need to have the internal CS# pulled high.
	//TODO: support slave mode
	lane->CR1 = SPI_MASTER | SPI_SOFT_CS | SPI_INTERNAL_CS;
	lane->CR1 |= SPI_ENABLE;

	//Calculate correct init value for baud rate divisor
	switch(baudDiv)
	{
		case 2:
			break;
		case 4:
			lane->CR1 |= 0x8;
			break;
		case 8:
			lane->CR1 |= 0x10;
			break;
		case 16:
			lane->CR1 |= 0x18;
			break;
		case 32:
			lane->CR1 |= 0x20;
			break;
		case 64:
			lane->CR1 |= 0x28;
			break;
		case 128:
			lane->CR1 |= 0x30;
			break;

		//use max value for invalid divisor
		case 256:
		default:
			lane->CR1 |= 0x38;
			break;
	}

	//Enable bidirectional mode if requested
	if(!fullDuplex)
		lane->CR1 |= SPI_BIDI_MODE;
}

void SPI::BlockingWrite(uint8_t data)
{
	//In half-duplex mode, select output mode
	if(!m_fullDuplex)
		m_lane->CR1 |= SPI_BIDI_OE;

	//If FIFO is full, block
	while( (m_lane->SR & SPI_TX_FIFO_MASK) == SPI_TX_FIFO_MASK)
	{}

	//Send it
	m_lane->DR = data;

	//If there's anything in the RX buffer, discard it
	DiscardRxData();
}

uint8_t SPI::BlockingRead()
{
	//Wait for previous events to complete
	WaitForWrites();
	DiscardRxData();

	//In half-duplex mode, select input mode
	if(!m_fullDuplex)
	{
		m_lane->CR1 &= !SPI_BIDI_OE;
		m_lane->CR1 |= SPI_RX_ONLY;
	}

	//Write a dummy byte
	m_lane->DR = 0x0;

	//Wait for data to be ready
	while( (m_lane->SR & SPI_RX_NOT_EMPTY) == 0)
	{}

	//Done, return it
	return m_lane->DR;
}

/**
	@brief 	Block until all pending writes have completed
 */
void SPI::WaitForWrites()
{
	//Wait for busy flag to clear
	while(m_lane->SR & SPI_BUSY)
	{}
}

/**
	@brief Discards any data in the RX FIFO
 */
void SPI::DiscardRxData()
{
	volatile int unused;
	while(m_lane->SR & SPI_RX_NOT_EMPTY)
		unused = m_lane->DR;
}

/**
	@brief Set the SPI clock inversion mode (CPOL)
 */
void SPI::SetClockInvert(bool invert)
{
	//Disable the peripheral
	m_lane->CR1 &= ~SPI_ENABLE;

	if(invert)
		m_lane->CR1 |= SPI_CPOL;
	else
		m_lane->CR1 &= ~SPI_CPOL;

	//Re-enable it
	m_lane->CR1 |= SPI_ENABLE;
}

#endif
