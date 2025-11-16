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

#ifndef SPIServer_h
#define SPIServer_h

#include <peripheral/SPI.h>
#include <embedded-utils/FIFO.h>

#ifdef HAVE_SPI

/**
	@brief Base class for an SPI peripheral
 */
class SPIServer
{
public:
	SPIServer(SPI<64, 64>& spi)
	: m_spi(spi)
	{}

	void Poll();

protected:

	///@brief Called on CS# falling edge
	void OnFallingEdge()
	{
		m_nbyte = 0;
		m_command = 0;
	}

	///@brief Called when a SPI byte arrives
	void OnByte(uint8_t b)
	{
		if(m_nbyte == 0)
			OnCommand(b);
		else
			OnDataByte(b);

		m_nbyte ++;
	}

	virtual void OnDataByte(uint8_t b);
	virtual void OnCommand(uint8_t b) =0;

	///@brief Our SPI device
	SPI<64, 64>& m_spi;

	///@brief Command ID sent in the last request
	uint8_t m_command;

	///@brief Data byte index
	uint16_t m_nbyte;
};

#endif
#endif
