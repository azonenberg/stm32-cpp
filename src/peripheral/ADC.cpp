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
#include <ctype.h>
#include <string.h>
#include <peripheral/RCC.h>
#include <peripheral/ADC.h>
#include <algorithm>
#include <array>
#include <math.h>

#ifdef HAVE_ADC

/**
	@brief Initialize an ADC lane clocked by the APB clock

	@param lane				The peripheral to use
	@param chan				Channel within a multi-channel ADC (if present)
	@param prescale			APB clock divider
 */
#if ADC_T_VERSION == 2
	ADC::ADC(volatile adc_t* lane, volatile adcchan_t* chan, int16_t prescale)
#else
	ADC::ADC(volatile adc_t* lane, int16_t prescale)
#endif
	: m_lane(lane)

	#if ADC_T_VERSION == 2
	, m_chan(chan)
	#endif
{
	RCCHelper::Enable(lane);

	//STM32L031
	#if (ADC_T_VERSION == 1)

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

		//TODO: support low freq mode and running directly off the HSI16 clock
		//TODO: support modes other than single shot (CONT=0)
		//TODO: support <12 bit conversion resolution for faster acquisition
		//TODO: support oversampling

	//STM32L431
	#elif (ADC_T_VERSION == 2)

		//Exit deep power-down mode
		m_chan->CR &= ~ADC_CR_DEEPPWD;

		//Turn on power
		m_chan->CR |= ADC_CR_ADVREGEN;

		//Need to wait 20us for ADC to power up, but we don't know what clock rate we're at
		//or what timers are enabled! For now, assume worst case 80 MHz CPU clock so we need 1600 clocks of delay
		//This loop will probably overestimate, but is conservative
		for(int i=0; i<800; i++)
			asm("nop");

		//Run zero calibration with ADC disabled
		//Set ADCAL, wait for it to be cleared
		//(for now, assume single ended calibration only)
		m_chan->CR |= ADC_CR_ADCAL;
		while(m_chan->CR & ADC_CR_ADCAL)
		{}

		//Configure ADC clock at PCLK/prescale
		//(this must happen before enabling the ADC)
		int presc = 0;
		switch(prescale)
		{
			case 1:
				presc = 0;
				break;

			case 2:
				presc = 1;
				break;

			case 4:
				presc = 2;
				break;

			case 6:
				presc = 3;
				break;

			case 8:
				presc = 4;
				break;

			case 10:
				presc = 5;
				break;

			case 12:
				presc = 6;
				break;

			case 16:
				presc = 7;
				break;

			case 32:
				presc = 8;
				break;

			case 64:
				presc = 9;
				break;

			case 128:
				presc = 10;
				break;

			case 256:
				presc = 11;
				break;

			//illegal input
			default:
				while(1)
				{}
		}
		m_lane->CCR = (m_lane->CCR & 0xFFC3FFFF) | (presc << 18);

		//Turn on temperature sensor, VBAT sensor, and voltage reference
		m_lane->CCR |= ADC_CCR_CH18SEL | ADC_CCR_CH17SEL | ADC_CCR_VREFEN;

		//ADC is off, need to start it
		m_chan->ISR &= ~ADC_ISR_ADRDY;
		m_chan->CR |= ADC_CR_ADEN;

		//Wait for ADC to start
		while(0 == (m_chan->ISR & ADC_ISR_ADRDY))
		{}

		//TODO: support all kinds of other modes besides default

	#else
		#error unimplemented for this device family
	#endif
}

//STM32L031
#if (ADC_T_VERSION == 1)

/**
	@brief Sets the sampling time, in half-cycles of the ADC clock

	@param tsample			ADC sampling time to use (measured in half cycles of ADC clock)
							For STM32L031 at full bit depth:
							Total acquisition time is sample delay plus fixed 12.5 cycle conversion
 */
void ADC::SetSampleTime(int16_t tsample)
{
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
}

//STM32L431
#elif (ADC_T_VERSION == 2)

/**
	@brief Sets the sampling time, in half-cycles of the ADC clock

	@param tsample			ADC sampling time to use (measured in half cycles of ADC clock)
 */
void ADC::SetSampleTime(int16_t tsample, uint8_t channel)
{
	uint32_t smpr = 0;

	switch(tsample)
	{
		case 5:
			smpr = 0;
			break;

		case 13:
			smpr = 1;
			break;

		case 25:
			smpr = 2;
			break;

		case 49:
			smpr = 3;
			break;

		case 95:
			smpr = 4;
			break;

		case 185:
			smpr = 5;
			break;

		case 495:
			smpr = 6;
			break;

		case 1281:
			smpr = 7;
			break;

		default:
			break;
	}

	//Low half, use SMPR1
	if(channel < 10)
	{
		uint8_t nshift = 3*channel;
		m_chan->SMPR1 &= ~(7 << nshift);
		m_chan->SMPR1 |= (smpr << nshift);
	}

	//High half, use SMPR2
	else
	{
		uint8_t nshift = 3*(channel - 10);
		m_chan->SMPR2 &= ~(7 << nshift);
		m_chan->SMPR2 |= (smpr << nshift);
	}
}

#endif

/**
	@brief Reads an ADC channel
 */
uint16_t ADC::ReadChannel(uint8_t channel)
{
	//STM32L031
	#if (ADC_T_VERSION == 1)

		//Select the channel
		m_lane->CHSELR = (1 << channel);

		//Do the conversion
		m_lane->CR |= ADC_CR_ADSTART;
		while(m_lane->CR & ADC_CR_ADSTART)
		{}

		//Done
		return m_lane->DR;

	//STM32L431
	#elif (ADC_T_VERSION == 2)

		//Create a single-channel sequence
		m_chan->SQR1 = (static_cast<uint32_t>(channel) << 6) | 0;

		//Do the conversion
		m_chan->CR |= ADC_CR_ADSTART;
		while( (m_chan->ISR & ADC_ISR_EOC) == 0)
		{}

		//Done
		return m_chan->DR;

	#else
		#error unimplemented for this device family
	#endif
}

/**
	@brief Gets the die temperature (for now only tested on STM32L031) in 8.8 fixed point format
 */
uint16_t ADC::GetTemperature()
{
	//STM32L031
	#if (ADC_T_VERSION == 1)

		//TS_CAL1 and TS_CAL2 were taken with VDDA=3.0V
		//so we need to rescale given our higher Vdd
		//TODO: rewrite this as fixed point so we don't have to pull in ~4 kB of softfloat lib
		const float vdd = GetSupplyVoltage();
		const float vddscale = vdd / 3000;
		const float cal2 = TSENSE_CAL2 / vddscale;
		const float cal1 = TSENSE_CAL1 / vddscale;

		//Then we can calculate the calibration equation from RM0394 16.4.32
		const uint32_t tref1 = 30;
		const uint32_t tref2 = 130;
		const float dtemp = tref2 - tref1;
		const float dcal = cal2 - cal1;
		const float calscale = dtemp / dcal;

		//For reasons unknown, the first measurement always delivers garbage
		//Until we find a root cause, just throw it away
		ReadChannel(18);

		//Now do the actual measurement
		auto adcval = ReadChannel(18);

		//Convert ADC codes to degrees C
		float temp = (calscale * (adcval - cal1)) + 30;

		//Convert back to 8.8 fixed point
		return temp * 256;

	//STM32L431
	#elif (ADC_T_VERSION == 2)

		#ifdef HAVE_FPU

			//Read and discard extra readings to work around STM32L431 errata 2.5.2
			//(TODO: not needed if we've done another ADC reading in the last millisecond)
			#ifdef STM32L431
				ReadChannel(17);
			#endif

			//TS_CAL1 and TS_CAL2 were taken with VDDA=3.0V
			//so we need to rescale given our higher Vdd
			const float vdd = GetSupplyVoltage();
			const float vddscale = vdd / 3000;
			const float cal2 = TSENSE_CAL2 / vddscale;
			const float cal1 = TSENSE_CAL1 / vddscale;

			//Then we can calculate the calibration equation from RM0394 16.4.32
			const uint32_t tref1 = 30;
			const uint32_t tref2 = 130;
			const float dtemp = tref2 - tref1;
			const float dcal = cal2 - cal1;
			const float calscale = dtemp / dcal;

			//Now do the actual measurement
			#ifdef STM32L431

				//Workaround for STM32L431 errata 2.5.3: gather 64 samples, sort, discard outliers

				//Collect the samples
				const int nsamples = 64;
				float samples[nsamples];
				for(int i=0; i < nsamples; i++)
					samples[i] = ReadChannel(17);

				//Sort them
				std::sort(std::begin(samples), std::end(samples));

				//Average the middle half
				const int nstart = nsamples / 4;
				const int nend = nsamples * 3 / 4;
				const int navg = nend - nstart;
				float adcval = 0;
				for(int i=nstart; i < nend; i++)
					adcval += samples[i];
				adcval /= navg;

			#else
				float adcval = ReadChannel(17);
			#endif

			//Convert ADC codes to degrees C
			float temp = (calscale * (adcval - cal1)) + 30;

			//Convert back to 8.8 fixed point
			return temp * 256;

		#else
			#error Dont have fixed point conversion for ADC_T_VERSION = 2
		#endif

	#else
		#error unimplemented for this device family
	#endif
}

#ifdef STM32L431
/**
	@brief Gets the temperature, working around errata 2.5.3, without doing all acquisitions in one go

	This function uses internal static variables and is not thread safe.

	TODO: consider refactoring out of the ADC class??
 */
bool ADC::GetTemperatureNonblocking(uint16_t& temp)
{
	static int state = 0;
	static int nsample = 0;

	const int nsamples = 64;
	static float samples[nsamples];

	switch(state)
	{
		//Read and discard extra readings to work around STM32L431 errata 2.5.2
		//(TODO: not needed if we've done another ADC reading in the last millisecond)
		case 0:
			ReadChannel(17);
			state = 1;
			nsample = 0;
			return false;

		//Collect the samples
		case 1:
			samples[nsample] = ReadChannel(17);
			nsample ++;
			if(nsample >= nsamples)
				state = 2;
			return false;

		//Postprocess
		case 2:
		default:
			{
				//TS_CAL1 and TS_CAL2 were taken with VDDA=3.0V
				//so we need to rescale given our higher Vdd
				const float vdd = 3000 * VREFINT_CAL / ReadChannel(0);
				const float vddscale = vdd / 3000;
				const float cal2 = TSENSE_CAL2 / vddscale;
				const float cal1 = TSENSE_CAL1 / vddscale;

				//Then we can calculate the calibration equation from RM0394 16.4.32
				const uint32_t tref1 = 30;
				const uint32_t tref2 = 130;
				const float dtemp = tref2 - tref1;
				const float dcal = cal2 - cal1;
				const float calscale = dtemp / dcal;

				//Sort the samples
				std::sort(std::begin(samples), std::end(samples));

				//Average the middle half
				const int nstart = nsamples / 4;
				const int nend = nsamples * 3 / 4;
				const int navg = nend - nstart;
				float adcval = 0;
				for(int i=nstart; i < nend; i++)
					adcval += samples[i];
				adcval /= navg;

				//Convert ADC codes to degrees C
				float ftemp = (calscale * (adcval - cal1)) + 30;

				//Convert back to 8.8 fixed point
				temp = round(ftemp * 256);

				//Done, ready to go again
				state = 0;
			}
			return true;
	}
}
#endif

/**
	@brief Gets the supply voltage, in mV
 */
uint16_t ADC::GetSupplyVoltage()
{
	//STM32L031
	#if (ADC_T_VERSION == 1)

		return 3000 * VREFINT_CAL / ReadChannel(17);

	//STM32L431
	#elif (ADC_T_VERSION == 2)

		#ifdef STM32L431
			//Read and discard extra readings to work around STM32L431 errata 2.5.2
			//(TODO: not needed if we've done another ADC reading in the last millisecond)
			ReadChannel(0);
		#endif

		return 3000 * VREFINT_CAL / ReadChannel(0);

	#else
		#error unimplemented for this device family
	#endif
}

#ifdef HAVE_FPU

/**
	@brief Reads a channel several times, average the result, and return a reading in millivolts

	@param channel	Channel index
	@param navg		Number of averages to take
	@param vdd		Supply voltage in mV
 */
float ADC::ReadChannelScaledAveraged(uint8_t channel, uint32_t navg, float vdd)
{
	//read and throw out a value to wake up the ADC
	ReadChannel(channel);

	//Integrate a few samples to denoise
	float vtemp = 0;
	for(uint32_t i=0; i<navg; i++)
		vtemp += ReadChannel(channel);

	//Convert sum of raw adc codes to average millivolts
	//TODO: this assumes a 12-bit ADC, do any stm32s have more/less?
	return (vtemp * vdd) / (navg * 4096);
}

/**
	@brief Reads a channel several times, take the median of the result, and return a reading in millivolts

	@param channel	Channel index
	@param navg		Number of averages to take
	@param vdd		Supply voltage in mV
 */
float ADC::ReadChannelScaledMedian(uint8_t channel, uint32_t navg, float vdd)
{
	//read and throw out a value to wake up the ADC
	ReadChannel(channel);

	//allow up to this many samples to be integrated
	const uint32_t nmax = 64;
	if(navg > nmax)
		navg = nmax;
	float samples[nmax];

	//Integrate a few samples to denoise
	for(uint32_t i=0; i<navg; i++)
		samples[i] = ReadChannel(channel);

	//Sort them
	std::sort(std::begin(samples), std::begin(samples) + navg);

	//Convert raw adc codes to millivolts
	//TODO: this assumes a 12-bit ADC, do any stm32s have more/less?
	return (samples[navg/2] * vdd) / 4096;
}
#endif //HAVE_FPU

#endif	//HAVE_ADC
