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

#ifndef timer_h
#define timer_h

#ifdef HAVE_TIM

#ifdef SIMULATION

#include <time.h>
#include <unistd.h>

/**
	@brief Dummy timer class for simulation

	For now, always runs at 10 kHz
 */
class Timer
{
public:
	Timer()
	{
		m_starttime = GetCountInternal();
	}

	uint32_t GetCount()
	{ return GetCountInternal() - m_starttime; }

	void Sleep(uint32_t ticks, bool /*reset*/ = false)
	{ usleep(100 * ticks); }

protected:
	uint32_t GetCountInternal()
	{
		timespec t;
		clock_gettime(CLOCK_REALTIME,&t);

		return (t.tv_sec % 86400) + (t.tv_nsec / 100000);
	}

protected:
	uint32_t m_starttime;
};

#else

class Timer
{
public:

	enum Features
	{
		FEATURE_GENERAL_PURPOSE,
		FEATURE_ADVANCED
	};

	Timer(volatile tim_t* chan, Features features, uint16_t prescale);

	/**
		@brief Gets the current counter value
	 */
	uint32_t GetCount()
	{ return m_chan->CNT; }

	/**
		@brief Restarts the counter from the default value (0 if up, auto reload value if down)
	 */
	void Restart()
	{ m_chan->EGR = 0x1; }

	void Sleep(uint32_t ticks, bool reset = false);

protected:
	volatile tim_t*	m_chan;
	Features m_features;
};

#endif	//ifdef SIMULATION

#endif	//ifdef HAVE_TIM

#endif
