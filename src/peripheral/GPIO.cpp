/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2022 Andrew D. Zonenberg                                                                          *
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
#include "GPIO.h"
#include "RCC.h"

/**
	@brief Configure pin mode
 */
void GPIOPin::SetMode(gpiomode_t mode, uint8_t altmode, bool open_drain)
{
	m_gpio->MODER &= ~(0x3 << 2*m_pin);
	m_gpio->MODER |= (mode << 2*m_pin);

	if(mode == MODE_PERIPHERAL)
	{
		if(m_pin < 8)
		{
			m_gpio->AFRL &= ~(0xf << 4*m_pin);
			m_gpio->AFRL |= (altmode << 4*m_pin);
		}
		else
		{
			m_gpio->AFRH &= ~(0xf << 4*(m_pin-8));
			m_gpio->AFRH |= (altmode << 4*(m_pin-8));
		}
	}

	if(open_drain)
		m_gpio->OTYPER |= (1 << m_pin);
	else
		m_gpio->OTYPER &= ~(1 << m_pin);
}

void GPIOPin::SetPullMode(PullMode mode)
{
	uint32_t mask = 3 << (2*m_pin);
	m_gpio->PUPDR = (m_gpio->PUPDR & ~mask) | (mode << 2*m_pin);
}
