/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2023 Andrew D. Zonenberg                                                                          *
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

	#elif defined(STM32L0)

		if(gpio == &GPIOA)
			RCC.IOPENR |= RCC_IO_GPIOA;
		else if(gpio == &GPIOB)
			RCC.IOPENR |= RCC_IO_GPIOB;
		else if(gpio == &GPIOC)
			RCC.IOPENR |= RCC_IO_GPIOC;

		/*
		else if(gpio == &GPIOD)
			RCC.IOPENR |= RCC_IO_GPIOD;
		else if(gpio == &GPIOE)
			RCC.IOPENR |= RCC_IO_GPIOE;
		else if(gpio == &GPIOH)
			RCC.IOPENR |= RCC_IO_GPIOH;
		*/

	#elif defined(STM32H7)

		if(gpio == &GPIOA)
			RCC.AHB4ENR |= RCC_AHB4_GPIOA;
		else if(gpio == &GPIOB)
			RCC.AHB4ENR |= RCC_AHB4_GPIOB;
		else if(gpio == &GPIOC)
			RCC.AHB4ENR |= RCC_AHB4_GPIOC;
		else if(gpio == &GPIOD)
			RCC.AHB4ENR |= RCC_AHB4_GPIOD;
		else if(gpio == &GPIOE)
			RCC.AHB4ENR |= RCC_AHB4_GPIOE;
		else if(gpio == &GPIOF)
			RCC.AHB4ENR |= RCC_AHB4_GPIOF;
		else if(gpio == &GPIOG)
			RCC.AHB4ENR |= RCC_AHB4_GPIOG;
		else if(gpio == &GPIOH)
			RCC.AHB4ENR |= RCC_AHB4_GPIOH;
		else if(gpio == &GPIOJ)
			RCC.AHB4ENR |= RCC_AHB4_GPIOJ;
		else if(gpio == &GPIOK)
			RCC.AHB4ENR |= RCC_AHB4_GPIOK;

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

#ifdef HAVE_UART

/**
	@brief Enable a UART
 */
void RCCHelper::Enable(volatile usart_t* uart)
{
	#if defined(STM32F031)

		if(uart == &USART1)
			RCC.APB2ENR |= RCC_APB2_USART1;

	#elif defined(STM32L031)

		if(uart == &USART2)
			RCC.APB1ENR |= RCC_APB1_USART2;
		else if(uart == &USART4)
			RCC.APB1ENR |= RCC_APB1_USART4;
		else if(uart == &USART5)
			RCC.APB1ENR |= RCC_APB1_USART5;

	#elif defined(STM32H735)

		if(uart == &USART2)
			RCC.APB1LENR |= RCC_APB1L_USART2;
		else if(uart == &USART3)
			RCC.APB1LENR |= RCC_APB1L_USART3;
		else if(uart == &UART4)
			RCC.APB1LENR |= RCC_APB1L_UART4;
		else if(uart == &UART5)
			RCC.APB1LENR |= RCC_APB1L_UART5;

	#elif defined(STM32F777)

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

#endif

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
	@brief Enable the digital temperature sensor
 */
#ifdef HAVE_DTS
void RCCHelper::Enable(volatile dts_t* /*ignored*/)
{
	#if defined(STM32H735)
		RCC.APB4ENR |= RCC_APB4_DTS;
	#else
	#error Unknown DTS configuration (unsupported part)
	#endif
}
#endif

/**
	@brief Enable the RNG
 */
#ifdef HAVE_RNG
void RCCHelper::Enable(volatile rng_t* /*ignored*/)
{
	#if defined(STM32F777)
		RCC.AHB2ENR |= RCC_AHB2_RNG;
	#elif defined(STM32H735)
		RCC.AHB2ENR |= RCC_AHB2_RNG;
	#else
	#error Unknown RNG configuration (unsupported part)
	#endif
}
#endif

/**
	@brief Enable the hash engine
 */
#ifdef HAVE_HASH
void RCCHelper::Enable(volatile hash_t* /*ignored*/)
{
	#if defined(STM32F777)
		RCC.AHB2ENR |= RCC_AHB2_HASH;
	#elif defined(STM32H735)
		RCC.AHB2ENR |= RCC_AHB2_HASH;
	#else
	#error Unknown hash configuration (unsupported part)
	#endif
}
#endif

/**
	@brief Enable the crypto engine
 */
#ifdef HAVE_CRYP
void RCCHelper::Enable(volatile cryp_t* /*ignored*/)
{
	#if defined(STM32F777)
		RCC.AHB2ENR |= RCC_AHB2_CRYP;
	#elif defined(STM32H735)
		RCC.AHB2ENR |= RCC_AHB2_CRYP;
	#else
	#error Unknown crypto configuration (unsupported part)
	#endif
}
#endif

/**
	@brief Enable an I2C bus
 */
#ifdef HAVE_I2C
void RCCHelper::Enable(volatile i2c_t* i2c)
{
	#if defined(STM32F031)

		if(i2c == &I2C1)
			RCC.APB1ENR |= RCC_APB1_I2C1;

	#elif defined(STM32H735)

		if(i2c == &I2C1)
			RCC.APB1LENR |= RCC_APB1L_I2C1;
		else if(i2c == &I2C2)
			RCC.APB1LENR |= RCC_APB1L_I2C2;
		else if(i2c == &I2C3)
			RCC.APB1LENR |= RCC_APB1L_I2C3;
		else if(i2c == &I2C4)
			RCC.APB4ENR |= RCC_APB4_I2C4;
		else if(i2c == &I2C5)
			RCC.APB1LENR |= RCC_APB1L_I2C5;

	#else
		#error Unknown I2C configuration (unsupported part)
	#endif
}
#endif

#ifdef HAVE_OCTOSPI

void RCCHelper::Enable(volatile octospim_t* /*octospim*/)
{
	RCC.AHB3ENR |= RCC_AHB3_OCTOSPIM;
}

void RCCHelper::Enable(volatile octospi_t* octospi)
{
	if(octospi == &OCTOSPI1)
		RCC.AHB3ENR |= RCC_AHB3_OCTOSPI1;
	else if(octospi == &OCTOSPI2)
		RCC.AHB3ENR |= RCC_AHB3_OCTOSPI2;
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

	#elif defined(STM32L031)

		if(tim == &TIMER2)
			RCC.APB1ENR |= RCC_APB1_TIM2;
		else if(tim == &TIMER3)
			RCC.APB1ENR |= RCC_APB1_TIM3;
		else if(tim == &TIMER6)
			RCC.APB1ENR |= RCC_APB1_TIM6;
		else if(tim == &TIMER7)
			RCC.APB1ENR |= RCC_APB1_TIM7;
		else if(tim == &TIMER21)
			RCC.APB2ENR |= RCC_APB2_TIM21;
		else if(tim == &TIMER22)
			RCC.APB2ENR |= RCC_APB2_TIM22;

	#elif defined(STM32H735)

		if(tim == &TIM2)
			RCC.APB1LENR |= RCC_APB1L_TIM2;
		else if(tim == &TIM3)
			RCC.APB1LENR |= RCC_APB1L_TIM3;
		else if(tim == &TIM4)
			RCC.APB1LENR |= RCC_APB1L_TIM4;
		else if(tim == &TIM5)
			RCC.APB1LENR |= RCC_APB1L_TIM5;
		else if(tim == &TIM6)
			RCC.APB1LENR |= RCC_APB1L_TIM6;
		else if(tim == &TIM7)
			RCC.APB1LENR |= RCC_APB1L_TIM7;
		else if(tim == &TIM12)
			RCC.APB1LENR |= RCC_APB1L_TIM12;
		else if(tim == &TIM13)
			RCC.APB1LENR |= RCC_APB1L_TIM13;
		else if(tim == &TIM14)
			RCC.APB1LENR |= RCC_APB1L_TIM14;

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

#ifdef STM32L0
/**
	@brief Configures the PLL to use the internal 16 MHz oscillator, then selects it

	@param mult		VCO multiplier from HSI16 (3 4, 6, 8, 12, 16, 24, 32, 48)
	@param hclkdiv	Divider on PLL output to HCLK (2, 3, or 4)
	@param ahbdiv	Divider from HCLK to AHB (0, 2, 4, 8, 16, 64, 128, 256, 512)
	@param apb2div	Divider from HCLK to APB2 (1, 2, 4, 8, or 16)
	@param apb1div	Divider from HCLK to APB1 (1, 2, 4, 8, or 16)
 */
void RCCHelper::InitializePLLFromHSI16(uint8_t mult, uint8_t hclkdiv, uint16_t ahbdiv, uint8_t apb2div, uint8_t apb1div)
{
	//Enable the HSI16 oscillator
	RCC.CR |= RCC_HSI_ON;

	//Wait until it's ready
	while(0 == (RCC.CR & RCC_HSI_READY))
	{}

	//Remove all PLL configuration
	RCC.CFGR &= ~0xFD3FF0;

	//Select VCO divider
	switch(hclkdiv)
	{
		case 2:
			RCC.CFGR |= 0x400000;
			break;

		case 3:
			RCC.CFGR |= 0x800000;
			break;

		case 4:
			RCC.CFGR |= 0xc00000;
			break;

		//any other input is illegal, fail
		default:
			while(1)
			{}
	}

	//Select PLL multiplier
	switch(mult)
	{
		case 3:
			RCC.CFGR |= 0x00000;
			break;
		case 4:
			RCC.CFGR |= 0x40000;
			break;
		case 6:
			RCC.CFGR |= 0x80000;
			break;
		case 8:
			RCC.CFGR |= 0xc0000;
			break;
		case 12:
			RCC.CFGR |= 0x100000;
			break;
		case 16:
			RCC.CFGR |= 0x140000;
			break;
		case 24:
			RCC.CFGR |= 0x180000;
			break;
		case 32:
			RCC.CFGR |= 0x1c0000;
			break;
		case 48:
			RCC.CFGR |= 0x200000;
			break;

		//any other input is illegal, fail
		default:
			while(1)
			{}
	}

	//HSI16 selected as PLL source (bit 16 = 0, cleared above)

	//Select APB2 divider
	switch(apb2div)
	{
		case 1:
			RCC.CFGR |= 0x00000;
			break;
		case 2:
			RCC.CFGR |= 0x2000;
			break;
		case 4:
			RCC.CFGR |= 0x2800;
			break;
		case 8:
			RCC.CFGR |= 0x3000;
			break;
		case 16:
			RCC.CFGR |= 0x3800;
			break;

		//any other input is illegal, fail
		default:
			while(1)
			{}
	}

	//Select APB1 divider
	switch(apb1div)
	{
		case 1:
			RCC.CFGR |= 0x00000;
			break;
		case 2:
			RCC.CFGR |= 0x400;
			break;
		case 4:
			RCC.CFGR |= 0x500;
			break;
		case 8:
			RCC.CFGR |= 0x600;
			break;
		case 16:
			RCC.CFGR |= 0x700;
			break;

		//any other input is illegal, fail
		default:
			while(1)
			{}
	}

	//Select AHB divider
	switch(ahbdiv)
	{
		case 1:
			RCC.CFGR |= 0x00000;
			break;
		case 2:
			RCC.CFGR |= 0x80;
			break;
		case 4:
			RCC.CFGR |= 0x90;
			break;
		case 8:
			RCC.CFGR |= 0xa0;
			break;
		case 16:
			RCC.CFGR |= 0xb0;
			break;
		case 64:
			RCC.CFGR |= 0xc0;
			break;
		case 128:
			RCC.CFGR |= 0xd0;
			break;
		case 256:
			RCC.CFGR |= 0xe0;
			break;
		case 512:
			RCC.CFGR |= 0xf0;
			break;

		//any other input is illegal, fail
		default:
			while(1)
			{}
	}

	//Enable PLL
	RCC.CR |= RCC_PLL_ON;
	while(0 == (RCC.CR & RCC_PLL_READY))
	{}

	//Switch clock source to PLL
	RCC.CFGR |= 0x3;

	//Wait until PLL switch is complete
	while(0xc != (RCC.CFGR & 0xc))
	{}
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

#ifdef STM32H7
/**
	@brief Enable the high speed external oscillator in bypass mode (external clock)
 */
void RCCHelper::EnableHighSpeedExternalClock()
{
	RCC.CR = RCC.CR | RCC_CR_HSEON | RCC_CR_HSEBYP;

	while( (RCC.CR & RCC_CR_HSERDY) == 0)
	{}
}

/**
	@brief Enable the high speed internal oscillator

	@param mhz Operating frequency for the HSI oscillator. Must be 8, 16, 32, or 64
 */
void RCCHelper::EnableHighSpeedInternalClock(int mhz)
{
	//Find clock divider
	int div = 0;
	switch(mhz)
	{
		case 64:
			div = 0;
			break;
		case 32:
			div = 1;
			break;
		case 16:
			div = 2;
			break;
		case 8:
			div = 3;
			break;
		default:
			return;
	}

	//Enable the HSI oscillator
	RCC.CR = (RCC.CR & ~RCC_CR_HSIDIVMASK) | RCC_CR_HSION | (div << 3);

	//Wait until it's ready
	while( (RCC.CR & RCC_CR_HSIRDY) == 0)
	{}
}

/**
	@brief Initialize and starts a PLL

	For now, the input must come from the HSE or HSI oscillator

	Input frequency at PFD must be 1 to 16 MHz
	VCO frequency must be 150 to 836 MHz
	VCO frequencies >420 MHz only available if PFD frequency >= 2 MHz
	VCO frequencies <150 MHz only available if PFD frequency < 2 MHz

	@param npll				PLL to configure (1-3)
	@param in_mhz			Input frequency, in MHz
	@param prediv			Pre-divider (1 to 63)
	@param mult				VCO multiplier (4 - 512)
	@param divP				Divider for output P (1 - 128)
	@param divQ				Divider for output Q (1 - 128)
	@param divR				Divider for output R (1 - 128)
	@param source			Input frequency
 */
void RCCHelper::InitializePLL(
	uint8_t npll,
	float in_mhz,
	uint8_t prediv,
	uint16_t mult,
	uint8_t divP,
	uint8_t divQ,
	uint8_t divR,
	ClockSource source
	)
{
	//TODO: fractional N support: multiplier = DIVN + (FRACN / 2^13)

	//Select PLL source
	if(source == CLOCK_SOURCE_HSE)
		RCC.PLLCKSELR = (RCC.PLLCKSELR & RCC_PLLCKSELR_SRC_MASK) | RCC_PLLCKSELR_SRC_HSE;
	else //if(source == CLOCK_SOURCE_HSI)
		RCC.PLLCKSELR = (RCC.PLLCKSELR & RCC_PLLCKSELR_SRC_MASK) | RCC_PLLCKSELR_SRC_HSI;

	//Prescaler is not shifted
	//div 0 = disabled
	uint32_t divm = prediv;

	//R/P/Q/N dividers are shifted by one
	uint8_t r = divR - 1;
	uint8_t q = divQ - 1;
	uint8_t p = divP - 1;
	uint16_t n = mult - 1;

	//Validate input frequency
	float pll_fin_mhz = in_mhz / prediv;
	if( (pll_fin_mhz < 1) || (pll_fin_mhz > 16) )
	{
		//ERROR if we get here: PLL input after predivider must be 1 to 16 MHz
		while(1)
		{}
	}

	//Figure out frequency ranges
	//Use low range VCO when 1-2 MHz
	//Use wide range when 2 to 16 MHz
	bool low_range_vco = (pll_fin_mhz < 2);
	uint32_t in_range;
	if(pll_fin_mhz < 2)
		in_range = 0;
	else if(pll_fin_mhz < 4)
		in_range = 1;
	else if(pll_fin_mhz < 8)
		in_range = 2;
	else //if(pll_fin_mhz < 16)
		in_range = 3;

	//Validate VCO frequency
	float fvco = pll_fin_mhz * mult;
	if(low_range_vco)
	{
		if( (fvco < 150) || (fvco > 420) )
		{
			//ERROR if we get here: VCO frequency out of range
			while(1)
			{}
		}
	}
	else
	{
		if( (fvco < 192) || (fvco > 836) )
		{
			//ERROR if we get here: VCO frequency out of range
			while(1)
			{}
		}
	}


	//Final config
	switch(npll)
	{
		case 1:
			RCC.PLLCKSELR = (RCC.PLLCKSELR & 0xfffffc0f) | (divm << 4);
			RCC.PLLCFGR |= (RCC_PLLCFGR_DIV1PEN | RCC_PLLCFGR_DIV1QEN | RCC_PLLCFGR_DIV1REN );
			RCC.PLLCFGR &= ~RCC_PLLCFGR_PLL1RGE_MASK;
			RCC.PLLCFGR |= (in_range << 2) | (low_range_vco << 1);
			RCC.PLL1DIVR = (r << 24) | (q << 16) | (p << 9) | n;
			//fractional mode not supported for now

			RCC.CR |= RCC_CR_PLL1ON;
			while( (RCC.CR & RCC_CR_PLL1RDY) == 0)
			{}
			break;

		case 2:
			RCC.PLLCKSELR = (RCC.PLLCKSELR & 0xfffc0fff) | (divm << 12);
			RCC.PLLCFGR |= (RCC_PLLCFGR_DIV2PEN | RCC_PLLCFGR_DIV2QEN | RCC_PLLCFGR_DIV2REN );
			RCC.PLLCFGR &= ~RCC_PLLCFGR_PLL2RGE_MASK;
			RCC.PLLCFGR |= (in_range << 6) | (low_range_vco << 5);
			RCC.PLL2DIVR = (r << 24) | (q << 16) | (p << 9) | n;
			//fractional mode not supported for now

			RCC.CR |= RCC_CR_PLL2ON;
			while( (RCC.CR & RCC_CR_PLL2RDY) == 0)
			{}
			break;

		case 3:
			RCC.PLLCKSELR = (RCC.PLLCKSELR & 0xfc0fffff) | (divm << 20);
			RCC.PLLCFGR |= (RCC_PLLCFGR_DIV3PEN | RCC_PLLCFGR_DIV3QEN | RCC_PLLCFGR_DIV3REN );
			RCC.PLLCFGR &= ~RCC_PLLCFGR_PLL3RGE_MASK;
			RCC.PLLCFGR |= (in_range << 10) | (low_range_vco << 9);
			RCC.PLL3DIVR = (r << 24) | (q << 16) | (p << 9) | n;
			//fractional mode not supported for now

			RCC.CR |= RCC_CR_PLL3ON;
			while( (RCC.CR & RCC_CR_PLL3RDY) == 0)
			{}
			break;

		//ignore invalid
		default:
			break;
	}
}

/**
	@brief Selects PLL1 as the system clock source
 */
void RCCHelper::SelectSystemClockFromPLL1()
{
	RCC.CFGR = (RCC.CFGR & RCC_CFGR_SW_MASK) | RCC_CFGR_SW_PLL1;

	while( ((RCC.CFGR >> 3) & 0x7) != RCC_CFGR_SW_PLL1)
	{}
}

/**
	@brief Initializes the system clock dividers

	All dividers must be powers of two.

	@param sysckdiv		D1CPRE (root divider for everything). Range [1, 512] except 32
						Output must be <= 550 MHz

	@param ahbdiv		HPRE (divider from CPU clock to AHB clock). Range [1, 512] except 32
						Output must be <= 275 MHz

	@param apb1div		D2PPRE1 (divider from AHB clock to APB1 clock). Range [1, 16]
	@param apb2div		D2PPRE2 (divider from AHB clock to APB2 clock). Range [1, 16]
	@param apb3div		D1PPRE (divider from AHB clock to APB3 clock). Range [1, 16]
	@param apb4div		D3PPRE (divider from AHB clock to APB4 clock). Range [1, 16]

	Input clock tree (see RM0468 figure 49):
		/ D1CPRE = sys_d1cpre_clk (550 MHz max core clock)
			= rcc_c_ck (CPU)
			= rcc_fclk_c
			= sys_d1cpre_ck
			/8
				= systick
			/HPRE (275 MHz max)
				= rcc_aclk (AXI peripherals)
				= rcc_hclk3 (AHB3 peripherals)
				/ D1PPRE
					= rcc_pclk3 (APB3 peripherals)
				= rcc_hclk2 (AHB2 peripherals)
				= rcc_hclk1 (AHB1 peripherals)
				/ D2PPRE1
					= rcc_pclk1 (APB1 peripherals)
					= rcc_timx_ker_ck (timer precaler)
				/ D2PPRE2
					= rcc_pclk2 (APB2 peripherals)
					= rcc_timy_ker_ck (timer prescale)
				= rcc_hclk4 (AHB4 peripherals)
				= rcc_fclk_d3
				/ D3PPRE
					= rcc_pclk4 (APB4 peripherals)
 */
void RCCHelper::InitializeSystemClocks(
	uint16_t sysckdiv,
	uint16_t ahbdiv,
	uint8_t apb1div,
	uint8_t apb2div,
	uint8_t apb3div,
	uint8_t apb4div)
{
	//Power domain D1 divider config
	RCC.D1CFGR = (GetDivider512Code(sysckdiv) << 8) | (GetDivider16Code(apb3div) << 4) | GetDivider512Code(ahbdiv);

	//Power domain D2 divider config
	RCC.D2CFGR = (GetDivider16Code(apb2div) << 8) | (GetDivider16Code(apb1div) << 4);

	//Power domain D3 divider config
	RCC.D3CFGR = GetDivider16Code(apb4div) << 4;
}

/**
	@brief Get divider config value for a 1-to-512 divider
 */
uint8_t RCCHelper::GetDivider512Code(uint16_t div)
{
	switch(div)
	{
		case 1:
			return 0x0;
		case 2:
			return 0x8;
		case 4:
			return 0x9;
		case 8:
			return 0xa;
		case 16:
			return 0xb;
		//div 32 is not legal
		case 64:
			return 0xc;
		case 128:
			return 0xd;
		case 256:
			return 0xe;
		case 512:
		default:
			return 0xf;
	}
}

/**
	@brief Get divider config value for a 1-to-16 divider
 */
uint8_t RCCHelper::GetDivider16Code(uint8_t div)
{
	switch(div)
	{
		case 1:
			return 0x0;
		case 2:
			return 0x4;
		case 4:
			return 0x5;
		case 8:
			return 0x6;
		case 16:
		default:
			return 0x7;
	}
}
#endif

#ifdef STM32H7
void RCCHelper::EnableSyscfg()
{
	RCC.APB4ENR |= RCC_APB4_SYSCFG;
}

void RCCHelper::EnableSram2()
{
	RCC.AHB2ENR |= RCC_AHB2_SRAM2;
}

void RCCHelper::EnableSram1()
{
	RCC.AHB2ENR |= RCC_AHB2_SRAM1;
}

void RCCHelper::EnableBackupSram()
{
	RCC.AHB4ENR |= RCC_AHB4_BKPRAM;
}
#endif
