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

#ifndef gpio_h
#define gpio_h

#include <peripheral/RCC.h>

/**
	@brief A GPIO pin
 */
class GPIOPin
{
public:

	/**
		@brief GPIO mode constants (must be same as STM32 MODER register)
	 */
	enum gpiomode_t
	{
		MODE_INPUT		= 0,
		MODE_OUTPUT		= 1,
		MODE_PERIPHERAL	= 2,
		MODE_ANALOG		= 3
	};

#if defined(STM32F7) || defined(STM32H7) || defined(STM32L0)
	/**
		GPIO slew constants (must be same as STM32 OSPEEDR register)
	 */
	enum gpioslew_t
	{
		SLEW_SLOW 		= 0,
		SLEW_MEDIUM 	= 1,
		SLEW_FAST 		= 2,
		SLEW_VERYFAST	= 3
	};
#endif

	/**
		@brief Initializes the pin
	 */
	GPIOPin(
		volatile gpio_t* gpio,
		uint8_t pin,
		gpiomode_t mode,
		uint8_t altmode = 0,
		bool open_drain = false)
	: m_gpio(gpio)
	, m_pin(pin)
	, m_setmask(1 << pin)
	, m_clearmask(~m_setmask)
	{
		//Make sure the bank we're part of is turned on
		RCCHelper::Enable(gpio);

		//Configure the pin
		SetMode(mode, altmode, open_drain);
	}

#if defined(STM32F7) || defined(STM32H7) || defined(STM32L0)
	/**
		@brief Initializes the pin
	 */
	GPIOPin(
		volatile gpio_t* gpio,
		uint8_t pin,
		gpiomode_t mode,
		gpioslew_t slew = SLEW_SLOW,
		uint8_t altmode = 0,
		bool open_drain = false)
	: m_gpio(gpio)
	, m_pin(pin)
	, m_setmask(1 << pin)
	, m_clearmask(~m_setmask)
	{
		//Make sure the bank we're part of is turned on
		RCCHelper::Enable(gpio);

		//Configure the pin
		SetMode(mode, altmode, open_drain);
		SetOutputSlew(slew);
	}
#endif

	void SetMode(gpiomode_t mode, uint8_t altmode = 1, bool open_drain = false);

	#if defined(STM32F7) || defined(STM32H7) || defined(STM32L0)
	void SetOutputSlew(gpioslew_t slew)
	{ m_gpio->OSPEEDR = (m_gpio->OSPEEDR & ~(0x3 << 2*m_pin)) | (slew << (2*m_pin)); }
	#endif

	/**
		@brief Drives a value out the pin
	 */
	void Set(bool b)
	{
		if(b)
			m_gpio->ODR |= m_setmask;
		else
			m_gpio->ODR &= m_clearmask;
	}

	//Convenience helper for assigning GPIOs
	void operator=(bool b)
	{ Set(b); }

	//Convenience helper for reading GPIOs
	operator bool() const
	{ return Get(); }

	/**
		@brief Reads the current value of the pin
	 */
	bool Get() const
	{
		if(m_gpio->IDR & m_setmask)
			return true;
		else
			return false;
	}

	enum PullMode
	{
		PULL_NONE,
		PULL_UP,
		PULL_DOWN
	};

	/**
		@brief Set the pullup/down mode
	 */
	void SetPullMode(PullMode mode);

protected:
	volatile gpio_t* 	m_gpio;
	uint8_t				m_pin;
	uint32_t			m_setmask;
	uint32_t			m_clearmask;
};

#endif
