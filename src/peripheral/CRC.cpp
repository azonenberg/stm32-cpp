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
#include "CRC.h"
#include "RCC.h"

#ifdef HAVE_CRC

uint32_t CRC::Checksum(const uint8_t* idata, uint32_t len, uint32_t poly, uint32_t init)
{
	//Initialize the CRC peripheral
	_CRC.INIT = init;
	_CRC.POL = poly;
	_CRC.CR = 0xe1;	//32 bit polynomial, bit swap for endianness

	//wait for reset
	while(_CRC.CR & 1)
	{}

	//Process data in 32-bit blocks as much as possible
	auto p = reinterpret_cast<const uint32_t*>(idata);
	uint32_t iend = len & ~0x3;
	uint32_t wend = iend / 4;
	for(uint32_t i=0; i<wend; i++)
		_CRC.DR = p[i];

	//Process any leftovers
	//(switch to byte-level input and don't reset)
	_CRC.CR = 0xa0;
	for(uint32_t i=iend; i<len; i++)
		*reinterpret_cast<volatile uint8_t*>(&_CRC.DR) = idata[i];

	return ~_CRC.DR;
}

void CRC::ChecksumInit(uint32_t poly, uint32_t init)
{
	//Initialize the CRC peripheral
	_CRC.INIT = init;
	_CRC.POL = poly;
	_CRC.CR = 0x01;

	//wait for reset
	while(_CRC.CR & 1)
	{}
}

void CRC::ChecksumUpdate(const uint8_t* idata, uint32_t len)
{
	//Process data in 32-bit blocks as much as possible
	_CRC.CR = 0xe0;
	auto p = reinterpret_cast<const uint32_t*>(idata);
	uint32_t iend = len & ~0x3;
	uint32_t wend = iend / 4;
	for(uint32_t i=0; i<wend; i++)
		_CRC.DR = p[i];

	//Process any leftovers
	//(switch to byte-level input and don't reset)
	_CRC.CR = 0xa0;
	for(uint32_t i=iend; i<len; i++)
		*reinterpret_cast<volatile uint8_t*>(&_CRC.DR) = idata[i];
}

#endif
