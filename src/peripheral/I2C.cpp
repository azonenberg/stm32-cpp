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
#include <peripheral/I2C.h>

#ifdef HAVE_I2C

/**
	@brief Initialize an I2C lane

	@param lane			The peripheral to use
	@param prescale		Prescale divisor from APB clock
	@param clkdiv		Clock high/low divider from prescaled divider
 */
I2C::I2C(volatile i2c_t* lane, uint8_t prescale, uint8_t clkdiv)
	: m_lane(lane)
{
	RCCHelper::Enable(lane);

	lane->CR2 = 0;

	lane->TIMINGR =
		( (prescale - 1) << 28 ) |	//prescaler
		1 << 20 |					//Data setup time: 2 prescaled ticks
		1 << 16 |					//Data hold time: 2 prescaled ticks
		(clkdiv-1) << 8 |			//SCL high time
		(clkdiv-1);					//SCL low time

	//Enable the peripheral
	lane->CR1 = 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nonblocking API

/**
	@brief Sends a start (or restart) command, returning immediately and not waiting for it to finish

	@param len		Number of bytes we intend to send
	@param addr		Device address
	@param read		True to initiate a read request, false to initiate a write
 */
void I2C::NonblockingStart(uint8_t len, uint8_t addr, bool read)
{
	//Auto end, specified length
	m_lane->CR2 = I2C_AUTO_END | (len << 16) | addr;
	if(read)
		m_lane->CR2 |= I2C_READ;
	m_lane->CR2 |= I2C_START;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Blocking API

/**
	@brief Sends a start (or restart) command
 */
void I2C::Start()
{
	m_lane->CR2 |= I2C_START;
	while(m_lane->CR2 & I2C_START)
	{}
}

/**
	@brief Sends a write command in host mode

	@param addr		8-bit device address (LSB ignored)
	@param data		Data to send
	@param len		Number of bytes to send
 */
bool I2C::BlockingWrite(uint8_t addr, const uint8_t* data, uint8_t len)
{
	//Auto end, specified length
	m_lane->CR2 = I2C_AUTO_END | (len << 16) | addr;
	Start();

	//Send the data
	for(uint8_t i=0; i<len; i++)
	{
		//Wait for last byte to finish
		while(!(m_lane->ISR & I2C_TX_EMPTY))
		{
			//If we got a NACK, abort
			if(m_lane->ISR & I2C_NACK)
			{
				m_lane->ICR = I2C_NACK;
				return false;
			}
		}

		//Send it
		m_lane->TXDR = data[i];
	}

	//Wait for send to finish
	while((m_lane->ISR & I2C_TX_EMPTY) == 0)
	{
		//If we got a NACK, abort
		if(m_lane->ISR & I2C_NACK)
		{
			m_lane->ICR = I2C_NACK;
			return false;
		}
	}

	return true;
}

/**
	@brief Sends write data in device mode

	@param data		Data to send
	@param len		Number of bytes to send
 */
void I2C::BlockingDeviceWrite(const uint8_t* data, uint8_t len)
{
	//Send the data
	for(uint8_t i=0; i<len; i++)
	{
		//Wait for last byte to finish
		while(!(m_lane->ISR & I2C_TX_EMPTY))
		{}

		//Send it
		m_lane->TXDR = data[i];
	}

	//Wait for send to finish
	while((m_lane->ISR & I2C_TX_EMPTY) == 0)
	{}
}

/**
	@brief Sends a read command in host mode
 */
bool I2C::BlockingRead(uint8_t addr, uint8_t* data, uint8_t len)
{
	//Auto end, specified length
	m_lane->CR2 = I2C_AUTO_END | (len << 16) | addr | I2C_READ;
	Start();

	//Read the data
	for(uint8_t i=0; i<len; i++)
	{
		//Wait for data to be ready
		while(!(m_lane->ISR & I2C_RX_READY))
		{
			//If we got a NACK, abort
			if(m_lane->ISR & I2C_NACK)
			{
				m_lane->ICR = I2C_NACK;
				return false;
			}
		}

		//Read it
		data[i] = m_lane->RXDR;
	}

	return true;
}

/**
	@brief Process a read in device mode
 */
void I2C::BlockingDeviceRead(uint8_t* data, uint8_t len)
{
	//Read the data
	for(uint8_t i=0; i<len; i++)
	{
		//Wait for data to be ready
		while(!(m_lane->ISR & I2C_RX_READY))
		{}

		//Read it
		data[i] = m_lane->RXDR;
	}
}

/**
	@brief Sets the device address of this node

	For now, only 7-bit addresses are supported.
 */
void I2C::SetThisNodeAddress(uint8_t addr)
{
	//Clear enable flag
	m_lane->OAR1 &= ~0x8000;

	//Set address
	m_lane->OAR1 = (addr & 0xfe);

	//Set enable flag
	m_lane->OAR1 |= 0x8000;
}

/**
	@brief Checks if a packet addressed to this node has been received.

	Self clears.
 */
bool I2C::PollAddressMatch()
{
	if(m_lane->ISR & I2C_ADDR)
	{
		m_lane->ICR = I2C_ADDR;
		return true;
	}
	return false;
}

#endif
