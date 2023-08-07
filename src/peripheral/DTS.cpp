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
#include <peripheral/DTS.h>
#include <peripheral/RCC.h>

#ifdef HAVE_DTS

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DigitalTempSensor driver class

/**
	@brief Initializes the temperature sensor

	@param dts			Sensor to initialize
	@param clkdiv		Clock divisor from APB clock (1-127)
	@param sampleTime	Number of sampling clock periods to use (0-15)
	@param fpclk		Frequency of PCLK, in Hz
 */
DigitalTempSensor::DigitalTempSensor(volatile dts_t* dts, uint8_t clkdiv, uint8_t sampleTime, uint32_t fpclk)
	: m_dts(dts)
	, m_samplingTime(sampleTime)
	, m_fpclk(fpclk)
{
	//Turn on the sensor
	RCCHelper::Enable(dts);

	//Enable the sensor, software trigger only
	dts->CFGR1 = (clkdiv << 24) | (sampleTime << 16);
	dts->CFGR1 |= 0x1;
}

/**
	@brief Gets the temperature reading in 8.8 fixed point format
 */
uint16_t DigitalTempSensor::GetTemperature()
{
	//Wait for sensor to be ready
	while( (m_dts->SR & 0x8000) == 0)
	{}

	//Request a measurement once sensor is ready
	m_dts->CFGR1 |= 0x10;

	//Block until measurement starts
	while( (m_dts->SR & 0x8000) != 0)
	{}

	//Clear start flag
	m_dts->CFGR1 &= ~0x10;

	//Block until measurement completes
	while( (m_dts->SR & 0x8000) == 0)
	{}

	//Frequency of the sensor measured at the calibration temperature T0, in 100 Hz steps
	uint8_t t0ref = (m_dts->T0VALR1 >> 16) & 3;
	uint8_t t0 = (t0ref == 1) ? 130 : 30;
	uint32_t t0freq = (m_dts->T0VALR1 & 0xffff) * 100;

	//Ramp rate in Hz/degC
	uint16_t ramp = m_dts->RAMPVALR & 0xffff;

	//Convert measured frequency from timer ticks to Hz
	uint16_t mfreq = m_dts->DR & 0xffff;
	float measuredHz = (m_fpclk * m_samplingTime) / mfreq;

	//Rescale to get final temperature
	float dfreq = measuredHz - t0freq;
	float caltemp = t0 + (dfreq / ramp);

	return caltemp * 256;
}

#endif
