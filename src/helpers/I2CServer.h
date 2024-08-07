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

#ifndef I2CServer_h
#define I2CServer_h

#include <peripheral/I2C.h>
#include <embedded-utils/FIFO.h>

/**
	@brief Base class for an I2C peripheral
 */
class I2CServer
{
public:
	I2CServer(I2C& i2c)
	: m_i2c(i2c)
	, m_regid(0)
	, m_readActive(false)
	, m_writeIndex(0)
	{}

	void Poll();

protected:
	void OnAddressMatch();
	void PollIncomingData();

	void SendReply8(uint8_t data)
	{ m_txbuf.Push(data); }

	void SendReply16(uint16_t data)
	{
		m_txbuf.Push(static_cast<uint8_t>(data >> 8));
		m_txbuf.Push(static_cast<uint8_t>(data & 0xff));
	}

	/**
		@brief Called when a new address byte matching us is received
	 */
	virtual void OnRequestStart() =0;

	/**
		@brief Called when a new read request is received
	 */
	virtual void OnRequestRead() =0;

	virtual void OnWriteData(uint8_t data);

	///@brief Our I2C device
	I2C& m_i2c;

	///@brief Register ID sent in the last request
	uint32_t m_regid;

	///@brief Transmit buffer
	FIFO<uint8_t, 32> m_txbuf;

	///@brief True if a read transaction is in progress
	bool m_readActive;

	///@brief Write byte index
	uint8_t m_writeIndex;
};

#endif
