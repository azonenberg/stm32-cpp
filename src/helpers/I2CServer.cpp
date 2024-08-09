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
#include "I2CServer.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main polling path

#include <embedded-utils/Logger.h>
extern Logger g_log;

/**
	@brief Polls for new I2C data and processes it
 */
void I2CServer::Poll()
{
	if(m_i2c.PollAddressMatch())
		OnAddressMatch();

	if(m_i2c.PollStop())
	{
		m_readActive = false;

		if(!m_txbuf.IsEmpty())
			m_txbuf.Reset();
	}

	PollIncomingData();

	//Send reply data if we have any
	if(m_readActive && m_i2c.IsReadyForReply())
	{
		if( !m_txbuf.IsEmpty() )
			m_i2c.NonblockingDeviceWrite8(m_txbuf.Pop());
	}
}

/**
	@brief Checks for incoming data
 */
void I2CServer::PollIncomingData()
{
	if(m_i2c.IsReadDataAvailable())
	{
		if(m_i2c.PollAddressMatch())
			OnAddressMatch();
		OnWriteData(m_i2c.NonblockingDeviceRead8());
	}
}

/**
	@brief Handles reception of our address
 */
void I2CServer::OnAddressMatch()
{
	bool isRead = m_i2c.IsDeviceRequestRead();
	m_writeIndex = 0;

	//Discard any pending stop request
	if(m_i2c.PollStop())
	{}

	//Clear transmit buffer when a new sequence starts
	m_txbuf.Reset();

	//Notify derived class to do any init it requires
	OnRequestStart();

	//Handle read or write
	if(isRead)
	{
		//need to account for any last minute regid writes
		PollIncomingData();

		m_readActive = true;
		OnRequestRead();
	}

	//If it's a write, no action needed
}

/**
	@brief Called when write data arrives

	The default implementation expects a big endian register ID of up to 32 bits
 */
void I2CServer::OnWriteData(uint8_t data)
{
	//Clear on the first byte
	if(m_writeIndex == 0)
		m_regid = 0;

	m_regid = (m_regid << 8) | data;
	m_writeIndex ++;
}
