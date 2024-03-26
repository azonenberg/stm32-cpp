/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2024 Andrew D. Zonenberg                                                                          *
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
SPI::SPI(volatile spi_t* lane, bool fullDuplex, uint16_t baudDiv, bool masterMode)
	: m_lane(lane)
	, m_fullDuplex(fullDuplex)
{
	RCCHelper::Enable(lane);

	#ifdef STM32H735

		//TODO: what do we write to TSER, TSIZE in CR2?

		//8 bit word size
		lane->CFG1 = 7;

		switch(baudDiv)
		{
			case 2:
				break;

			case 4:
				lane->CFG1 |= (1 << 28);
				break;

			case 8:
				lane->CFG1 |= (2 << 28);
				break;

			case 16:
				lane->CFG1 |= (3 << 28);
				break;

			case 32:
				lane->CFG1 |= (4 << 28);
				break;

			case 64:
				lane->CFG1 |= (5 << 28);
				break;

			case 128:
				lane->CFG1 |= (6 << 28);
				break;

			//use max divisor if invalid is selected
			case 256:
			default:
				lane->CFG1 |= (7 << 28);
				break;
		}

		//Set master mode with CS# in output mode
		//(we don't have to configure the alt mode on CS# but this keeps it from detecting false mode faults)
		if(masterMode)
			lane->CFG2 = SPI_MASTER | SPI_SSOE;
		else
			lane->CFG2 = 0;

		//must be done after making all other config changes
		lane->CR1 |= SPI_ENABLE;

	#else //this is for STM32L031 and F777?

		//8-bit word size, set RXNE as soon as we have a byte in the FIFO
		//TODO: make word size configurable?
		lane->CR2 = (7 << 8) | SPI_FRXTH;

		//Turn on the peripheral in master mode.
		//To prevent problems, we need to have the internal CS# pulled high.
		if(masterMode)
			lane->CR1 = SPI_MASTER | SPI_SOFT_CS | SPI_INTERNAL_CS;
		else
			lane->CR1 = 0;

		//ok to do before baud changes
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

	#endif

}

/**
	@brief Check if read data is available
 */
bool SPI::PollReadDataReady()
{
	if((m_lane->SR & SPI_RX_NOT_EMPTY) != 0)
		return true;
	return false;
}

/**
	@brief Send a byte of data in response to a message from the host
 */
void SPI::NonblockingWriteDevice(uint8_t data)
{
	#ifdef STM32H735
		m_lane->TXDR = data;
	#else
		m_lane->DR = data;
	#endif
}

void SPI::BlockingWrite(uint8_t data)
{
	#ifdef STM32H735

		//TODO: half duplex support

		//If FIFO is full, block
		while( (m_lane->SR & SPI_TX_FIFO_MASK) == 0)
		{}

		//Send it
		m_lane->TXDR = data;
		m_lane->CR1 |= SPI_START;

	#elif defined(STM32L031)

		//In half-duplex mode, select output mode
		if(!m_fullDuplex)
			m_lane->CR1 |= SPI_BIDI_OE;

		//If FIFO is full, block
		while( (m_lane->SR & SPI_TX_EMPTY) != 0)
		{}

		//Send it
		m_lane->DR = data;

	#else

		//In half-duplex mode, select output mode
		if(!m_fullDuplex)
			m_lane->CR1 |= SPI_BIDI_OE;

		//If FIFO is full, block
		while( (m_lane->SR & SPI_TX_FIFO_MASK) == SPI_TX_FIFO_MASK)
		{}

		//Send it
		m_lane->DR = data;

	#endif

	//If there's anything in the RX buffer, discard it
	DiscardRxData();
}

uint8_t SPI::BlockingRead()
{
	//Wait for previous events to complete
	WaitForWrites();
	DiscardRxData();

	#ifdef STM32H735

		//TODO: half duplex support

		//Write a dummy byte
		m_lane->TXDR = 0;
		m_lane->CR1 |= SPI_START;

		//Wait for lane to be ready
		while( (m_lane->SR & SPI_RX_NOT_EMPTY) == 0)
		{}

		//Done, return it
		return m_lane->RXDR;

	#else

		//In half-duplex mode, select input mode
		if(!m_fullDuplex)
		{
			m_lane->CR1 &= ~SPI_BIDI_OE;
			m_lane->CR1 |= SPI_RX_ONLY;
		}

		//Write a dummy byte
		m_lane->DR = 0x0;

		//Wait for data to be ready
		while( (m_lane->SR & SPI_RX_NOT_EMPTY) == 0)
		{}

		//Done, return it
		return m_lane->DR;
	#endif
}

uint8_t SPI::BlockingReadDevice()
{
	#ifdef STM32H735

		//Wait for lane to be ready
		while( (m_lane->SR & SPI_RX_NOT_EMPTY) == 0)
		{}

		//Done, return it
		return m_lane->RXDR;

	#else

		//Wait for data to be ready
		while( (m_lane->SR & SPI_RX_NOT_EMPTY) == 0)
		{}

		//Done, return it
		return m_lane->DR;
	#endif
}

/**
	@brief 	Block until all pending writes have completed
 */
void SPI::WaitForWrites()
{
	#ifdef STM32H735

		//Wait for busy flag to clear
		while( (m_lane->SR & SPI_TX_EMPTY) == 0)
		{}

	#else

		//Wait for busy flag to clear
		while(m_lane->SR & SPI_BUSY)
		{}

	#endif
}

/**
	@brief Discards any data in the RX FIFO
 */
void SPI::DiscardRxData()
{
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

	volatile int unused;

	#ifdef STM32H735

		//Wait for busy flag to clear
		while(m_lane->SR & SPI_RX_NOT_EMPTY)
			unused = m_lane->RXDR;

	#else
		while(m_lane->SR & SPI_RX_NOT_EMPTY)
			unused = m_lane->DR;
	#endif

	#pragma GCC diagnostic pop
}

/**
	@brief Set the SPI clock inversion mode (CPOL)
 */
void SPI::SetClockInvert(bool invert)
{
	#ifdef STM32H735

		//Disable the peripheral
		m_lane->CR1 &= ~SPI_ENABLE;

		//Set clock invert flag appropriately
		if(invert)
			m_lane->CFG2 |= SPI_CPOL;
		else
			m_lane->CFG2 &= ~SPI_CPOL;

		//Re-enable
		m_lane->CR1 |= SPI_ENABLE;

	#else

		//Disable the peripheral
		m_lane->CR1 &= ~SPI_ENABLE;

		if(invert)
			m_lane->CR1 |= SPI_CPOL;
		else
			m_lane->CR1 &= ~SPI_CPOL;

		//Re-enable it
		m_lane->CR1 |= SPI_ENABLE;

	#endif
}

#endif
