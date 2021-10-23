/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2021 Andrew D. Zonenberg                                                                          *
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

#ifndef RCC_h
#define RCC_h

#include <stm32fxxx.h>

/**
	@brief Reset and Clock Control

	Helper class for enabling various devices.

	All functions are static because there's only one RCC in the device.
 */
class RCCHelper
{
public:
	static void Enable(volatile gpio_t* gpio);

	#ifdef HAVE_I2C
	static void Enable(volatile i2c_t* i2c);
	#endif

	#ifdef HAVE_SPI
	static void Enable(volatile spi_t* spi);
	#endif

	#ifdef HAVE_TIM
	static void Enable(volatile tim_t* tim);
	#endif

	#ifdef HAVE_EMAC
	static void Enable(volatile emac_t* mac);
	#endif

	#ifdef HAVE_RNG
	static void Enable(volatile rng_t* rng);
	#endif

	static void Enable(volatile usart_t* uart);

	#ifdef STM32F0
	static void InitializePLLFromInternalOscillator(
		uint8_t prediv,
		uint8_t mult,
		uint16_t ahbdiv,
		uint8_t apbdiv
		);
	#endif

	#ifdef STM32F7
	static void InitializePLLFromInternalOscillator(
		uint8_t prediv,
		uint16_t mult,
		uint8_t pdiv,
		uint8_t qdiv,
		uint8_t rdiv,
		uint16_t ahbdiv,
		uint16_t apb1div,
		uint16_t apb2div
		);
	#endif
};

#endif
