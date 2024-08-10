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

#ifndef i2c_h
#define i2c_h

#ifdef HAVE_I2C

class I2C
{
public:
	I2C(volatile i2c_t* lane, uint8_t prescale, uint8_t clkdiv);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Nonblocking host/device API (polling or interrupt based)

	void NonblockingStart(uint8_t len, uint8_t addr, bool read);

	///@brief Checks if the start/address sequence has completed
	bool IsStartDone()
	{ return (m_lane->CR2 & I2C_START) != I2C_START; }

	///@brief Sends a byte of write data, returning immediately and not waiting for it to finish
	void NonblockingWrite(uint8_t data)
	{ m_lane->TXDR = data; }

	///@brief Checks if the write is finished
	bool IsWriteDone()
	{ return (m_lane->ISR & I2C_TX_EMPTY) == I2C_TX_EMPTY; }

	///@brief Checks if read data is available
	bool IsReadReady()
	{ return (m_lane->ISR & I2C_RX_READY) == I2C_RX_READY; }

	///@brief Gets the read data
	uint8_t GetReadData()
	{ return m_lane->RXDR;	}

	///@brief Flush the transmit buffer to cancel a written, but not sent, byte
	void FlushTxBuffer()
	{ m_lane->ISR |= I2C_TX_EMPTY; }

	///@brief Full reset of the I2C peripheral
	void Reset()
	{
		m_lane->CR1 &= ~1;
		for(int i=0; i<50; i++)
			asm("nop");
		m_lane->CR1 |= 1;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Nonblocking device API (polling or interrupt based)

	///@brief Checks if the most recent incoming request was a read
	bool IsDeviceRequestRead()
	{ return (m_lane->ISR & I2C_DIR_READ) == I2C_DIR_READ; }

	bool PollAddressMatch();
	bool PollStop();
	bool PollNack();

	///@brief Returns true if the bus is ready for us to send a reply
	bool IsReadyForReply()
	{ return (m_lane->ISR & I2C_TX_READY); }

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Blocking API (slow and simple)

	void Start();

	bool BlockingRead(uint8_t addr, uint8_t* data, uint8_t len);
	void BlockingDeviceRead(uint8_t* data, uint8_t len);

	bool BlockingWrite(uint8_t addr, const uint8_t* data, uint8_t len);
	void BlockingDeviceWrite(const uint8_t* data, uint8_t len);

	bool BlockingWrite8(uint8_t addr, uint8_t data)
	{ return BlockingWrite(addr, &data, 1); }

	void BlockingDeviceWrite8(uint8_t data)
	{ BlockingDeviceWrite(&data, 1); }

	/**
		@brief Sends a 16-bit value to a device in host mode
	 */
	bool BlockingWrite16(uint8_t addr, uint16_t data)
	{
		uint8_t buf[2] =
		{
			static_cast<uint8_t>(data >> 8),
			static_cast<uint8_t>(data & 0xff)
		};
		return BlockingWrite(addr, buf, 2);
	}

	/**
		@brief Sends a 16-bit value in response to a request from a host
	 */
	void BlockingDeviceWrite16(uint16_t data)
	{
		uint8_t buf[2] =
		{
			static_cast<uint8_t>(data >> 8),
			static_cast<uint8_t>(data & 0xff)
		};
		BlockingDeviceWrite(buf, 2);
	}

	bool BlockingRead16(uint8_t addr, uint16_t& result)
	{
		uint8_t buf[2];
		if(!BlockingRead(addr, buf, sizeof(buf)))
			return false;
		result = (buf[0] << 8) | buf[1];
		return true;
	}

	/**
		@brief Read an 8-bit value in device mode (typically a register address someone is requesting from us)
	 */
	uint8_t BlockingDeviceRead8()
	{
		uint8_t tmp;
		BlockingDeviceRead(&tmp, 1);
		return tmp;
	}
	void SetThisNodeAddress(uint8_t addr);

protected:
	volatile i2c_t*	m_lane;
};

#endif

#endif
