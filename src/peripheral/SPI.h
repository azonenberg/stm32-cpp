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

#ifndef spi_h
#define spi_h

#ifdef HAVE_SPI

#include <util/FIFO.h>

class SPIEvent
{
public:
	SPIEvent(uint8_t t=0, uint8_t d=0)
	: type(t)
	, data(d)
	{}

	enum
	{
		TYPE_DATA,
		TYPE_CS
	};
	uint8_t type;

	uint8_t	data;

	SPIEvent& operator=(const SPIEvent& rhs) =default;

	void operator=(const SPIEvent& rhs) volatile
	{
		type = rhs.type;
		data = rhs.data;
	}
};

//TODO: consider refactoring this to be a CharacterDevice?
class SPI
{
public:
	SPI(volatile spi_t* lane, bool fullDuplex, uint16_t baudDiv, bool masterMode = true);
	void SetBaudDiv(uint16_t baudDiv);

	bool PollReadDataReady();

	void BlockingWrite(uint8_t data);
	void WaitForWrites();
	uint8_t BlockingRead();
	uint8_t BlockingReadDevice();
	void NonblockingWriteDevice(uint8_t data);
	void BlockingWriteDevice(const uint8_t* data, uint32_t len);
	void DiscardRxData();

	void SetClockInvert(bool invert);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Interrupt driven API, for now only fully supports device-mode operation

	void EnableRxInterrupt()
	{ m_lane->CR2 |= SPI_RXNEIE; }

	void NonblockingWriteFifo(const uint8_t* data, uint32_t len);

	bool HasNextTxByte()
	{ return !m_txFifo.IsEmpty(); }

	uint8_t GetNextTxByte()
	{ return m_txFifo.Pop(); }

	SPIEvent GetEvent()
	{ return m_rxFifo.Pop(); }

	bool HasEvents()
	{ return !m_rxFifo.IsEmpty(); }

	//Interrupt handlers
	void OnIRQRxData(uint8_t value)
	{ m_rxFifo.Push(SPIEvent(SPIEvent::TYPE_DATA, value) ); }

	void OnIRQCSEdge(bool value)
	{ m_rxFifo.Push(SPIEvent(SPIEvent::TYPE_CS, value)); }

protected:
	volatile spi_t*	m_lane;
	bool m_fullDuplex;
	bool m_lastWasWrite;

	FIFO<SPIEvent, 32> m_rxFifo;
	FIFO<uint8_t, 32> m_txFifo;
};

#endif

#endif
