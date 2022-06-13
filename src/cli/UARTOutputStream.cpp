/***********************************************************************************************************************
*                                                                                                                      *
* stm32-cpp v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2021-2022 Andrew D. Zonenberg and contributors                                                         *
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

/**
	@file
	@brief Declaration of UARTOutputStream
 */
#include "UARTOutputStream.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

UARTOutputStream::UARTOutputStream()
	: m_uart(nullptr)
{
}

UARTOutputStream::~UARTOutputStream()
{
}

void UARTOutputStream::Initialize(UART* uart)
{
	m_uart = uart;
	m_fifo.Reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Output processing

void UARTOutputStream::PutCharacter(char ch)
{
	//convert \n to \r\n
	if(ch == '\n')
		PutCharacter('\r');

	if(m_fifo.IsFull())
		Flush();

	m_fifo.Push(ch);
}

void UARTOutputStream::PutString(const char* str)
{
	while(*str != '\0')
	{
		PutCharacter(*str);
		str ++;
	}
}

void UARTOutputStream::Flush()
{
	while(!m_fifo.IsEmpty())
		m_uart->PrintBinary(m_fifo.Pop());
	m_fifo.Reset();
}
