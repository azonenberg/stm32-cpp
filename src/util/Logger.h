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

#ifndef logger_h
#define logger_h

#include "CharacterDevice.h"
#include "../peripheral/Timer.h"

/**
	@brief Simple logging framework with uptime timestamps
 */
class Logger
{
public:

	Logger()
	: m_target(NULL)
	, m_timer(NULL)
	{}

	/**
		@brief Initializes a logger

		@param target	The UART or other destination for log messages
		@param timer	Timer with 1ms ticks since reset
	 */
	void Initialize(CharacterDevice* target, Timer* timer)
	{
		m_target = target;
		m_timer = timer;
	}

	/**
		@brief Prints a log message
	 */
	void operator()(const char* format, ...)
	{
		if(!m_target)
			return;

		Timestamp();
		PrintIndent();

		__builtin_va_list list;
		__builtin_va_start(list, format);
		m_target->Printf(format, list);
		__builtin_va_end(list);
	}

	/**
		@brief Increments the log level
	 */
	void Indent()
	{ m_indentLevel ++; }

	/**
		@brief Decrements the log level
	 */
	void Unindent()
	{
		if(m_indentLevel > 0)
			m_indentLevel --;
	}

protected:
	void Timestamp()
	{
		//TODO: handle wrap at 49.7 days
		//We're not Windows 95 though, it's just a status message. So not a priority to fix for now...

		auto uptime = m_timer->GetCount();
		m_target->Printf("[%8d.%03d] ", uptime / 1000, uptime % 1000);
	}

	void PrintIndent()
	{
		for(int i=0; i<m_indentLevel; i++)
			m_target->PrintBinary(' ');
	}

protected:
	CharacterDevice* m_target;
	Timer* m_timer;
	uint32_t m_indentLevel;
};

/**
	@brief Helper for auto indenting stuff in the log
 */
class LogIndenter
{
public:
	LogIndenter(Logger& log)
	: m_logger(log)
	{ log.Indent(); }

	~LogIndenter()
	{ m_logger.Unindent(); }

protected:
	Logger& m_logger;
};


#endif
