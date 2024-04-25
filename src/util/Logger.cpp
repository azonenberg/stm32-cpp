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
#include <stdint.h>
#include "Logger.h"

#ifdef LOGGER_USE_RTC_TIMESTAMP
#include <peripheral/RTC.h>
#endif

#ifdef HAVE_TIM

void Logger::Timestamp(LogType type)
{
	//Print the prefix for the log message
	switch(type)
	{
		case NORMAL:
			m_target->Printf("[\033[32m");
			break;

		case WARNING:
			m_target->Printf("[\033[33;1m");
			break;

		case ERROR:
			m_target->Printf("[\033[31;1m");
			break;
	}

	#ifdef LOGGER_USE_RTC_TIMESTAMP

		//this assumes RTC sub-second counter is 10 kHz ticks for now, should we find a way to change this later?
		tm rtctime;
		uint16_t rtcsubsec;
		RTC::GetTime(rtctime, rtcsubsec);

		m_target->Printf("%04d-%02d-%02dT%02d:%02d:%02d.%04d\033[0m] ",
			rtctime.tm_year + 1900,
			rtctime.tm_mon+1,
			rtctime.tm_mday,
			rtctime.tm_hour,
			rtctime.tm_min,
			rtctime.tm_sec,
			rtcsubsec);

	#else

		//TODO: support printf %ld and use int64 here
		uint32_t uptime = (m_timeOffset + m_timer->GetCount()) / 10;
		m_target->Printf("%8d.%03d\033[0m] ", uptime / 1000, uptime % 1000);

	#endif
}

void Logger::PrintIndent()
{
	for(uint32_t i=0; i<m_indentLevel; i++)
		m_target->PrintString("    ");
}

/**
	@brief Updates the internal offset if the timer is getting close to wrapping, then reset the timer
 */
void Logger::UpdateOffset()
{
	m_timeOffset += m_timer->GetCount();
	m_timer->Restart();
}

/**
	@brief Updates the internal offset if the timer is getting close to wrapping, then reset the timer
 */
bool Logger::UpdateOffset(uint32_t threshold)
{
	uint32_t timestamp = m_timer->GetCount();
	if(timestamp >= threshold)
	{
		m_timeOffset += timestamp;
		m_timer->Restart();
		return true;
	}
	return false;
}

#endif
