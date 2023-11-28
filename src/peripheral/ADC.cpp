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

#include <stm32.h>
#include <ctype.h>
#include <string.h>
#include <peripheral/RCC.h>
#include <peripheral/ADC.h>

#ifdef HAVE_ADC

/**
	@brief Initialize an ADC lane clocked by the APB clock

	@param lane				The peripheral to use
	@param prescale			APB clock divider
	@param tsample			ADC sampling time to use (measured in half cycles of ADC clock)
							For STM32L031 at full bit depth:
								Total conversion time is sample delay plus fixed 12.5 cycle conversion
 */
ADC::ADC(volatile adc_t* lane, int16_t prescale, int16_t tsample)
	: m_lane(lane)
{
	RCCHelper::Enable(lane);

	#ifdef STM32L031

		//Run zero calibration with ADC disabled
		//Set ADCAL, wait for it to be cleared
		m_lane->CR |= ADC_CR_ADCAL;
		while(m_lane->CR & ADC_CR_ADCAL)
		{}

		//ADC voltage regulator is now enabled, we don't have to do anything special there

		//ADC is off, need to start it
		m_lane->ISR &= ~ADC_ISR_ADRDY;
		m_lane->CR |= ADC_CR_ADEN;

		//Turn on temperature sensor and voltage reference
		m_lane->CCR |= ADC_CCR_TSEN | ADC_CCR_VREFEN;

		//Wait for ADC to start
		while(0 == (m_lane->ISR & ADC_ISR_ADRDY))
		{}

		//Configure ADC clock at PCLK/prescale
		int ckmode = 0;
		switch(prescale)
		{
			case 1:
				ckmode = 3;
				break;

			case 2:
				ckmode = 1;
				break;

			case 4:
				ckmode = 2;
				break;

			//illegal input
			default:
				while(1)
				{}
		}
		m_lane->CFGR2 = (ckmode << 30);

		switch(tsample)
		{
			case 3:
				m_lane->SMPR = 0;
				break;

			case 7:
				m_lane->SMPR = 1;
				break;

			case 15:
				m_lane->SMPR = 2;
				break;

			case 25:
				m_lane->SMPR = 3;
				break;

			case 39:
				m_lane->SMPR = 4;
				break;

			case 79:
				m_lane->SMPR = 5;
				break;

			case 159:
				m_lane->SMPR = 6;
				break;

			case 321:
				m_lane->SMPR = 7;
				break;
		}

		//TODO: support low freq mode and running directly off the HSI16 clock
		//TODO: support modes other than single shot (CONT=0)
		//TODO: support <12 bit conversion resolution for faster acquisition
		//TODO: support oversampling

	#else
		#error unimplemented for this device family
	#endif
}

/**
	@brief Reads an ADC channel
 */
uint16_t ADC::ReadChannel(uint8_t channel)
{
	//Select the channel
	m_lane->CHSELR = (1 << channel);

	//Do the conversion
	m_lane->CR |= ADC_CR_ADSTART;
	while(m_lane->CR & ADC_CR_ADSTART)
	{}

	//Done
	return m_lane->DR;
}

/**
	@brief Gets the die temperature (for now only tested on STM32L031) in 8.8 fixed point format
 */
uint16_t ADC::GetTemperature()
{
	const uint32_t tref1 = 30;
	const uint32_t tref2 = 130;
	const uint32_t dtemp = tref2 - tref1;
	uint32_t dcal = TSENSE_CAL2 - TSENSE_CAL1;

	uint32_t tempnum = (ReadChannel(18) - TSENSE_CAL1) * 256 * dtemp;

	return (tempnum / dcal) + 256*tref1;
}

/**
	@brief Gets the supply voltage, in mV
 */
uint16_t ADC::GetSupplyVoltage()
{
	return 3000 * VREFINT_CAL / ReadChannel(17);
}

#endif
