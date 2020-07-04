/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020 Andrew D. Zonenberg                                                                               *
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

#include <stm32fxxx.h>
#include <ctype.h>
#include <string.h>
#include <peripheral/RCC.h>
#include <peripheral/Timer.h>

/**
	@brief Initialize a timer

	@param chan			The peripheral to use
	@param features		Capabilities of the requested timer
 */
Timer::Timer(volatile tim_t* chan, Features features)
	: m_chan(chan)
	, m_features(features)
{
	RCCHelper::Enable(chan);
	/*
	//8-bit word size
	//TODO: make this configurable
	lane->CR2 = 7 << 8;

	//Turn on the peripheral in master mode.
	//To prevent problems, we need to have the internal CS# pulled high.
	//TODO: support slave mode
	lane->CR1 = SPI_MASTER | SPI_SOFT_CS | SPI_INTERNAL_CS;
	lane->CR1 |= SPI_ENABLE;

	//Calculate correct init value for baud rate divisor
	switch(baudDiv)
	{
		case 2:
			break;
		case 4:
			lane->CR1 |= 0x8;
			break;
		case 8:
			lane->CR1 |= 0x10;
			break;
		case 16:
			lane->CR1 |= 0x18;
			break;
		case 32:
			lane->CR1 |= 0x20;
			break;
		case 64:
			lane->CR1 |= 0x28;
			break;
		case 128:
			lane->CR1 |= 0x30;
			break;

		//use max value for invalid divisor
		case 256:
		default:
			lane->CR1 |= 0x38;
			break;
	}

	//Enable bidirectional mode if requested
	if(!fullDuplex)
		lane->CR1 |= SPI_BIDI_MODE;
	*/
}
