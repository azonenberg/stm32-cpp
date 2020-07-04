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
#include <peripheral/RCC.h>

/**
	@brief Enable a GPIO bank
 */
void RCCHelper::Enable(volatile gpio_t* gpio)
{
	if(gpio == &GPIOA)
		RCC.AHBENR |= RCC_AHB_GPIOA;
	else if(gpio == &GPIOB)
		RCC.AHBENR |= RCC_AHB_GPIOB;
}

/**
	@brief Enable a UART
 */
void RCCHelper::Enable(volatile usart_t* uart)
{
	if(uart == &USART1)
		RCC.APB2ENR |= RCC_APB2_USART1;
}

/**
	@brief Enable a SPI bus
 */
void RCCHelper::Enable(volatile spi_t* spi)
{
	if(spi == &SPI1)
		RCC.APB2ENR |= RCC_APB2_SPI1;
}

/**
	@brief Enable a timer
 */
void RCCHelper::Enable(volatile tim_t* tim)
{
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
}

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
