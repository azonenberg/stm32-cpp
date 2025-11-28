/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2025 Andrew D. Zonenberg                                                                          *
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
#include <peripheral/Timer.h>

#ifdef HAVE_TIM
#ifndef SIMULATION

/**
	@brief Initialize a timer

	@param chan			The peripheral to use
	@param features		Capabilities of the requested timer
	@param prescale		Pre-scaler value (one based, i.e. 1 = divide by 1)
 */
Timer::Timer(volatile tim_t* chan, Features features, uint16_t prescale)
	: m_chan(chan)
	, m_features(features)
{
	RCCHelper::Enable(chan);
	Initialize(prescale);
}

/**
	@brief Reinitialize a timer

	@param prescale	Pre-scaler value (one based, i.e. 1 = divide by 1)
 */
void Timer::Initialize(uint16_t prescale)
{
	//Configure the counter
	m_chan->CNT = 0x0;
	m_chan->PSC = (prescale - 1);

	//Turn off most features
	m_chan->CR2 = 0x0;
	m_chan->SMCR = 0x0;
	m_chan->DIER = 0x0;
	m_chan->CCMR1 = 0x0;
	m_chan->CCMR2 = 0x0;
	m_chan->CCR1 = 0x0;
	m_chan->CCR2 = 0x0;
	m_chan->CCR3 = 0x0;
	m_chan->CCR4 = 0x0;
	m_chan->CCER = 0x0;
#if !defined(STM32H7) && !defined(STM32L0)
	m_chan->BDTR = 0x0;
#endif
	m_chan->DCR = 0x0;
	m_chan->DMAR = 0x0;

	//Reset the counter
	m_chan->CR1 = 0x0;
	m_chan->EGR = 0x1;

	//Enable the counter
	m_chan->CR1 = 0x1;
}

/**
	@brief Blocking wait until the timer has advanced by the specified number of ticks

	@param ticks		Number of ticks to wait
	@param reset		If true, restart the counter from zero before starting the delay interval
 */
void Timer::Sleep(uint32_t ticks, bool reset)
{
	if(reset)
	{
		Restart();
		m_chan->CNT = 0;
		asm("dmb st");
	}

	if(m_features == FEATURE_GENERAL_PURPOSE_16BIT)
	{
		uint16_t target = m_chan->CNT + ticks;
		while(m_chan->CNT != target)
		{}
	}
	else
	{
		uint32_t target = m_chan->CNT + ticks;
		while(m_chan->CNT != target)
		{}
	}
}

#endif
#endif
