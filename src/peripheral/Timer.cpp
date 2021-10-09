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
#include <ctype.h>
#include <string.h>
#include <peripheral/RCC.h>
#include <peripheral/Timer.h>

#ifdef HAVE_TIM

/**
	@brief Initialize a timer

	@param chan			The peripheral to use
	@param features		Capabilities of the requested timer
 */
Timer::Timer(volatile tim_t* chan, Features features, uint16_t prescale)
	: m_chan(chan)
	, m_features(features)
{
	RCCHelper::Enable(chan);

	//Configure the counter
	chan->CNT = 0x0;
	chan->PSC = (prescale - 1);

	//Turn off most features
	chan->CR2 = 0x0;
	chan->SMCR = 0x0;
	chan->DIER = 0x0;
	chan->CCMR1 = 0x0;
	chan->CCMR2 = 0x0;
	chan->CCR1 = 0x0;
	chan->CCR2 = 0x0;
	chan->CCR3 = 0x0;
	chan->CCR4 = 0x0;
	chan->CCER = 0x0;
	chan->BDTR = 0x0;
	chan->DCR = 0x0;
	chan->DMAR = 0x0;

	//Reset the counter
	chan->CR1 = 0x0;
	chan->EGR = 0x1;

	//Enable the counter
	chan->CR1 = 0x1;
}

/**
	@brief Blocking wait until the timer has advanced by the specified number of ticks

	@param ticks		Number of ticks to wait
	@param reset		If true, restart the counter from zero before starting the delay interval
 */
void Timer::Sleep(uint16_t ticks, bool reset)
{
	if(reset)
	{
		Restart();
		m_chan->CNT = 0;
	}

	unsigned int target = m_chan->CNT + ticks;
	while(m_chan->CNT != target)
	{}
}

#endif
