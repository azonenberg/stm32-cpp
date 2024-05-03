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

#ifndef uart_h
#define uart_h

#include <stm32.h>
#include <peripheral/RCC.h>
#include <util/StringHelpers.h>
#include <util/BufferedCharacterDevice.h>

#ifdef HAVE_UART

/**
	@file
	@author Andrew D. Zonenberg
	@brief UART driver
 */

/**
	@brief Driver for a UART
 */
template<size_t rxbufsize, size_t txbufsize>
class UART : public BufferedCharacterDevice<rxbufsize, txbufsize>
{
public:

	UART(volatile usart_t* lane, uint32_t baud_div = 181)
	 : UART(lane, lane, baud_div)
	{}

	__attribute__((noinline))
	UART(volatile usart_t* txlane, volatile usart_t* rxlane, uint32_t baud_div)
		: m_txlane(txlane)
		, m_rxlane(rxlane)
	{
		//Turn on the UART
		RCCHelper::Enable(txlane);
		RCCHelper::Enable(rxlane);

		//Set baud rates
		m_txlane->BRR = baud_div;
		if(m_txlane != m_rxlane)
			m_rxlane->BRR = baud_div;

		//Wipe config register to default states
		m_txlane->CR3 = 0x0;
		m_txlane->CR2 = 0x0;
		m_txlane->CR1 = 0x0;
		if(m_txlane != m_rxlane)
		{
			m_rxlane->CR3 = 0x0;
			m_rxlane->CR2 = 0x0;
			m_rxlane->CR1 = 0x0;
		}

		//Configure TX/RX lanes appropriately
		m_txlane->CR1 |= USART_CR1_TE | USART_CR1_UE;
		m_rxlane->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
	}

	//TX side
	virtual void PrintBinary(char ch) override
	{
		//Block if the FIFO has no free space
		while(BufferedCharacterDevice<rxbufsize, txbufsize>::m_txFifo.IsFull())
		{}

		//Write to the FIFO
		BufferedCharacterDevice<rxbufsize, txbufsize>::m_txFifo.Push(ch);

		//Enable TX interrupt
		m_txlane->CR1 |= USART_CR1_TXEIE;
	}

	//TX interrupt handler
	virtual void OnIRQTxEmpty()
	{
		if(BufferedCharacterDevice<rxbufsize, txbufsize>::m_txFifo.IsEmpty())
			m_txlane->CR1 &= ~USART_CR1_TXEIE;
		else
			m_txlane->TDR = BufferedCharacterDevice<rxbufsize, txbufsize>::m_txFifo.Pop();
	}

	//RX interrupt handler
	virtual void OnIRQRxData()
	{ BufferedCharacterDevice<rxbufsize, txbufsize>::m_rxFifo.Push(m_rxlane->RDR); }

	///@brief Forces a flush, even if interrupts are disabled
	void __attribute__((noinline)) BlockingFlush()
	{
		while(!BufferedCharacterDevice<rxbufsize, txbufsize>::m_txFifo.IsEmpty())
		{
			if(m_txlane->ISR & USART_ISR_TXE)
				OnIRQTxEmpty();
		}
	}

protected:
	volatile usart_t* m_txlane;
	volatile usart_t* m_rxlane;
};

#endif

#endif
