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

#ifndef adc_h
#define adc_h

#ifdef HAVE_ADC

class ADC
{
public:

	#if (ADC_T_VERSION == 2)
		ADC(volatile adc_t* lane, volatile adcchan_t* chan, int16_t prescale);
	#else
		ADC(volatile adc_t* lane, int16_t prescale);
	#endif

	uint16_t ReadChannel(uint8_t channel);

	uint16_t GetTemperature();
	uint16_t GetSupplyVoltage();

	#if (ADC_T_VERSION == 1)
		void SetSampleTime(int16_t tsample);
	#elif (ADC_T_VERSION == 2)
		void SetSampleTime(int16_t tsample, uint8_t channel);
	#endif

	#ifdef HAVE_FPU

		/**
			@brief Reads a channel, returning a value in millivolts corrected for Vref
		 */
		float ReadChannelScaled(uint8_t channel)
		{
			float code = ReadChannel(channel);
			return (code / 4095) * GetSupplyVoltage();
		}

		float ReadChannelScaledAveraged(uint8_t channel, uint32_t navg);
	#endif

protected:
	volatile adc_t*	m_lane;

	#if (ADC_T_VERSION == 2)
		volatile adcchan_t* m_chan;
	#endif
};

#endif

#endif
