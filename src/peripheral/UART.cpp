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

#ifdef HAVE_UART

#include <stm32.h>
#include <ctype.h>
#include <string.h>
#include <peripheral/UART.h>
#include <peripheral/RCC.h>
#include <util/StringHelpers.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UART driver class

/**
	@brief Initializes a UART.
 */
UART::UART(volatile usart_t* txlane, volatile usart_t* rxlane, uint32_t baud_div)
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
	m_txlane->CR1 |= 0x9;
	m_rxlane->CR1 |= 0x25;
}

void UART::PrintBinary(char ch)
{
	m_txlane->TDR = ch;

	while(0 == (m_txlane->ISR & USART_ISR_TXE))
	{}
}

#endif
