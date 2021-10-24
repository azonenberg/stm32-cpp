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

#include <stm32fxxx.h>
#include <peripheral/RCC.h>

/**
	@brief Enable a GPIO bank
 */
void RCCHelper::Enable(volatile gpio_t* gpio)
{
	#if defined(STM32F0)

		if(gpio == &GPIOA)
			RCC.AHBENR |= RCC_AHB_GPIOA;
		else if(gpio == &GPIOB)
			RCC.AHBENR |= RCC_AHB_GPIOB;

	#elif defined(STM32F7)

		if(gpio == &GPIOA)
			RCC.AHB1ENR |= RCC_AHB1_GPIOA;
		else if(gpio == &GPIOB)
			RCC.AHB1ENR |= RCC_AHB1_GPIOB;
		else if(gpio == &GPIOC)
			RCC.AHB1ENR |= RCC_AHB1_GPIOC;
		else if(gpio == &GPIOD)
			RCC.AHB1ENR |= RCC_AHB1_GPIOD;
		else if(gpio == &GPIOE)
			RCC.AHB1ENR |= RCC_AHB1_GPIOE;
		else if(gpio == &GPIOF)
			RCC.AHB1ENR |= RCC_AHB1_GPIOF;
		else if(gpio == &GPIOG)
			RCC.AHB1ENR |= RCC_AHB1_GPIOG;
		else if(gpio == &GPIOH)
			RCC.AHB1ENR |= RCC_AHB1_GPIOH;
		else if(gpio == &GPIOI)
			RCC.AHB1ENR |= RCC_AHB1_GPIOI;
		else if(gpio == &GPIOJ)
			RCC.AHB1ENR |= RCC_AHB1_GPIOJ;
		else if(gpio == &GPIOK)
			RCC.AHB1ENR |= RCC_AHB1_GPIOK;

	#else
		#error Unknown STM32 family
	#endif
}

/**
	@brief Enable a UART
 */
void RCCHelper::Enable(volatile usart_t* uart)
{
	#if defined(STM32F0)

		if(uart == &USART1)
			RCC.APB2ENR |= RCC_APB2_USART1;

	#elif defined(STM32F7)

		if(uart == &USART1)
			RCC.APB2ENR |= RCC_APB2_USART1;
		else if(uart == &USART2)
			RCC.APB1ENR |= RCC_APB1_USART2;
		else if(uart == &USART3)
			RCC.APB1ENR |= RCC_APB1_USART3;
		else if(uart == &UART4)
			RCC.APB1ENR |= RCC_APB1_UART4;
		else if(uart == &UART5)
			RCC.APB1ENR |= RCC_APB1_UART5;
		else if(uart == &USART6)
			RCC.APB2ENR |= RCC_APB2_USART6;
		else if(uart == &UART7)
			RCC.APB1ENR |= RCC_APB1_UART7;
		else if(uart == &UART8)
			RCC.APB1ENR |= RCC_APB1_UART8;


	#else
		#error Unknown STM32 family
	#endif
}

/**
	@brief Enable a SPI bus
 */
#ifdef HAVE_SPI
void RCCHelper::Enable(volatile spi_t* spi)
{
	#if defined(STM32F031)

		if(spi == &SPI1)
			RCC.APB2ENR |= RCC_APB2_SPI1;

	#elif defined(STM32F777)

		if(spi == &SPI1)
			RCC.APB2ENR |= RCC_APB2_SPI1;
		else if(spi == &SPI2)
			RCC.APB1ENR |= RCC_APB1_SPI2;
		else if(spi == &SPI3)
			RCC.APB1ENR |= RCC_APB1_SPI3;
		else if(spi == &SPI4)
			RCC.APB2ENR |= RCC_APB2_SPI4;
		else if(spi == &SPI5)
			RCC.APB2ENR |= RCC_APB2_SPI5;
		else if(spi == &SPI6)
			RCC.APB2ENR |= RCC_APB2_SPI6;

	#else
		#error Unknown STM32 family
	#endif
}
#endif

#ifdef HAVE_EMAC
void RCCHelper::Enable(volatile emac_t* /*ignored*/)
{
	//TODO: take argument for MII or RMII mode

	//Select RMII mode
	//Disable all Ethernet clocks (except 1588 which we don't need, leave it off to save power), reset MAC
	RCC.AHB1ENR &= ~(RCC_AHB1_EMAC | RCC_AHB1_EMAC_TX | RCC_AHB1_EMAC_RX);
	RCC.AHB1RSTR |= RCC_AHB1_EMAC;

	//Enable RMII
	SYSCFG.PMC |= ETH_MODE_RMII;

	//Enable Ethernet clocks (except 1588 since we don't use that)
	RCC.AHB1ENR |= RCC_AHB1_EMAC | RCC_AHB1_EMAC_TX | RCC_AHB1_EMAC_RX | RCC_AHB1_PTP;

	//Clear resets
	RCC.AHB1RSTR &= ~RCC_AHB1_EMAC;
}
#endif

/**
	@brief Enable the RNG
 */
#ifdef HAVE_RNG
void RCCHelper::Enable(volatile rng_t* /*ignored*/)
{
	RCC.AHB2ENR |= RCC_AHB2_RNG;
}
#endif

/**
	@brief Enable the hash engine
 */
#ifdef HAVE_HASH
void RCCHelper::Enable(volatile hash_t* /*ignored*/)
{
	RCC.AHB2ENR |= RCC_AHB2_HASH;
}
#endif

/**
	@brief Enable an I2C bus
 */
#ifdef HAVE_I2C
void RCCHelper::Enable(volatile i2c_t* i2c)
{
	if(i2c == &I2C1)
		RCC.APB1ENR |= RCC_APB1_I2C1;
}
#endif

/**
	@brief Enable a timer
 */
#ifdef HAVE_TIM
void RCCHelper::Enable(volatile tim_t* tim)
{
	#if defined(STM32F031)
		if(tim == &TIM1)
			RCC.APB2ENR |= RCC_APB2_TIM1;
		else if(tim == &TIM2)
			RCC.APB1ENR |= RCC_APB1_TIM2;
		else if(tim == &TIM3)
			RCC.APB1ENR |= RCC_APB1_TIM3;
		else if(tim == &TIM14)
			RCC.APB1ENR |= RCC_APB1_TIM14;
		else if(tim == &TIM16)
			RCC.APB2ENR |= RCC_APB2_TIM16;
		else if(tim == &TIM17)
			RCC.APB2ENR |= RCC_APB2_TIM17;
	#elif defined(STM32F777)
		if(tim == &TIM1)
			RCC.APB2ENR |= RCC_APB2_TIM1;
		else if(tim == &TIM2)
			RCC.APB1ENR |= RCC_APB1_TIM2;
		else if(tim == &TIM3)
			RCC.APB1ENR |= RCC_APB1_TIM3;
		else if(tim == &TIM4)
			RCC.APB1ENR |= RCC_APB1_TIM4;
		else if(tim == &TIM5)
			RCC.APB1ENR |= RCC_APB1_TIM5;
		else if(tim == &TIM6)
			RCC.APB1ENR |= RCC_APB1_TIM6;
		else if(tim == &TIM7)
			RCC.APB1ENR |= RCC_APB1_TIM7;
		else if(tim == &TIM8)
			RCC.APB2ENR |= RCC_APB2_TIM8;
		else if(tim == &TIM9)
			RCC.APB2ENR |= RCC_APB2_TIM9;
		else if(tim == &TIM10)
			RCC.APB2ENR |= RCC_APB2_TIM10;
		else if(tim == &TIM11)
			RCC.APB2ENR |= RCC_APB2_TIM11;
		else if(tim == &TIM12)
			RCC.APB1ENR |= RCC_APB1_TIM12;
		else if(tim == &TIM13)
			RCC.APB1ENR |= RCC_APB1_TIM13;
		else if(tim == &TIM14)
			RCC.APB1ENR |= RCC_APB1_TIM14;
	#else
		#error Unknown timer configuration (unsupported part)
	#endif
}
#endif

#ifdef STM32F0

/**
	@brief Configures the PLL to use the internal 8 MHz oscillator, then selects it

	@param prediv	Pre-divider from 8 MHz input
	@param mult		Multiplier from pre-divided clock to CPU clock

	@param ahbdiv	Divider from CPU clock to AHB clock
	@param apbdiv	Divider from AHB clock to APB clock
 */
void RCCHelper::InitializePLLFromInternalOscillator(uint8_t prediv, uint8_t mult, uint16_t ahbdiv, uint8_t apbdiv)
{
	//Pre-divider is offset by 1
	prediv = (prediv - 1) & 0xf;
	RCC.CFGR2 = prediv;

	//Multiplier is offset by 2
	mult = (mult - 2) & 0xf;

	//Properly encode the APB clock
	uint8_t apbsel;
	switch(apbdiv)
	{
		case 1:
			apbsel = 0;
			break;
		case 2:
			apbsel = 4;
			break;
		case 4:
			apbsel = 5;
			break;
		case 8:
			apbsel = 6;
			break;

		//invalid selector is treated as max possible divide
		default:
		case 16:
			apbsel = 7;
			break;
	}

	//Properly encode the AHB clock
	uint8_t ahbsel;
	switch(ahbdiv)
	{
		case 1:
			ahbsel = 0;
			break;
		case 2:
			ahbsel = 8;
			break;
		case 4:
			ahbsel = 9;
			break;
		case 8:
			ahbsel = 10;
			break;
		case 16:
			ahbsel = 11;
			break;
		case 64:
			ahbsel = 12;
			break;
		case 128:
			ahbsel = 13;
			break;
		case 256:
			ahbsel = 14;
			break;

		//invalid selector is treated as max possible divide
		default:
		case 512:
			ahbsel = 15;
			break;
	}

	//Configure PLL multipliers/dividers
	RCC.CFGR = (mult << 18) |
				(prediv & 1) << 17 |
				0x8000 |
				(apbsel << 8) |
				(ahbsel << 4);

	//Start the PLL
	RCC.CR |= RCC_PLL_ON;

	//Wait for PLL to be stable, then start using it
	while( (RCC.CR & RCC_PLL_READY) == 0)
	{}
	RCC.CFGR = (RCC.CFGR & 0xfffffffc) | 2;
	while( (RCC.CFGR & 0xc) != 0x8)
	{}
}

#endif

#ifdef STM32F7

/**
	@brief Configures the PLL to use the internal 16 MHz oscillator, then selects it

	@param prediv	pre-divider for input clock to PLL
					Must be between 2 and 63
					PLL input must be between 1 and 2 MHz; best jitter performance with 2 MHz

	@param mult		Multiplier for PLL VCO
					Must be between 50 and 432.
					VCO must be between 100 and 432 MHz.

	@param pdiv		Divider "P" from VCO to main system clock
					Must be 2, 4, 6, or 8.
					Cannot exceed 216 MHz.

	@param qdiv		Divider "Q" from VCO to USB OTG, SD/MMC, and RNG
					Must be between 2 and 15.
					Cannot exceed 48 MHz, must be exactly 48 MHz if using USB

	@param rdiv		Divider "R" from VCO to DSI
					Must be between 2 and 7

	@param ahbdiv	Divider from VCO to AHB bus
					Must be 1, 2, 4, 8, 16, 64, 128, 256, or 512.
					Must be at least 25 MHz if using Ethernet.

	@param apb1div	Divider from AHB to APB1 (low speed) bus
					Must be 1, 2, 4, 8, or 16.
					Cannot exceed 45 MHz.

	@param apb2div	Divider from AHB to APB2 (high speed) bus
					Must be 1, 2, 4, 8, or 16.
					Cannot exceed 90 MHz.
 */
void RCCHelper::InitializePLLFromInternalOscillator(
	uint8_t prediv,
	uint16_t mult,
	uint8_t pdiv,
	uint8_t qdiv,
	uint8_t rdiv,
	uint16_t ahbdiv,
	uint16_t apb1div,
	uint16_t apb2div
	)
{
	//Shove all the PLL config bits into the right fields
	uint32_t pllconfig = 0;
	pllconfig |= (rdiv & 0x7) << 28;
	pllconfig |= (qdiv & 0xf) << 24;
	switch(pdiv)
	{
		case 2:
			pllconfig |= 0x0 << 16;
			break;

		case 4:
			pllconfig |= 0x1 << 16;
			break;

		case 6:
			pllconfig |= 0x2 << 16;
			break;

		case 8:
		default:
			pllconfig |= 0x3 << 16;
			break;
	}
	pllconfig |= (mult & 0x1ff) << 6;
	pllconfig |= (prediv & 0x3f);

	//Configure the PLL
	RCC.PLLCFGR = (RCC.PLLCFGR & RCC_PLLCFGR_RESERVED_MASK) | pllconfig;

	//Start the PLL and wait for it to lock
	RCC.CR |= RCC_PLL_ENABLE;
	while(0 == (RCC.CR & RCC_PLL_READY))
	{}

	//Configure main system clock dividers and PLL source
	uint32_t cfg = 0;
	switch(apb2div)
	{
		case 1:
			cfg |= RCC_APB2_DIV1;
			break;

		case 2:
			cfg |= RCC_APB2_DIV2;
			break;

		case 4:
			cfg |= RCC_APB2_DIV4;
			break;

		case 8:
			cfg |= RCC_APB2_DIV8;
			break;

		case 16:
		default:
			cfg |= RCC_APB2_DIV16;
			break;
	}

	switch(apb1div)
	{
		case 1:
			cfg |= RCC_APB1_DIV1;
			break;

		case 2:
			cfg |= RCC_APB1_DIV2;
			break;

		case 4:
			cfg |= RCC_APB1_DIV4;
			break;

		case 8:
			cfg |= RCC_APB1_DIV8;
			break;

		case 16:
		default:
			cfg |= RCC_APB1_DIV16;
			break;
	}

	switch(ahbdiv)
	{
		case 1:
			cfg |= RCC_AHB_DIV1;
			break;

		case 2:
			cfg |= RCC_AHB_DIV2;
			break;

		case 4:
			cfg |= RCC_AHB_DIV4;
			break;

		case 8:
			cfg |= RCC_AHB_DIV8;
			break;

		case 16:
			cfg |= RCC_AHB_DIV16;
			break;

		case 64:
			cfg |= RCC_AHB_DIV64;
			break;

		case 128:
			cfg |= RCC_AHB_DIV128;
			break;

		case 256:
			cfg |= RCC_AHB_DIV256;
			break;

		case 512:
		default:
			cfg |= RCC_AHB_DIV512;
			break;
	}

	//Enable the PLL
	RCC.CFGR = cfg | RCC_SYSCLK_PLL;
}

#endif
