/***********************************************************************************************************************
*                                                                                                                      *
* stm32-cpp v0.1                                                                                                       *
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

#ifndef fifo_h
#define fifo_h

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	Declaration of Fifo class
 */

/**
	@brief A circular buffer based FIFO interlocked for safe use across interrupt domains
 */
template<class objtype, uint32_t depth>
class FIFO
{
public:

	/**
		@brief Creates a new FIFO
	 */
	FIFO()
		: m_wptr(0)
		, m_rptr(0)
		, m_empty(true)
	{}

	/**
		@brief Adds a new item to the FIFO.

		Pushing when the FIFO is full is a legal no-op.
	 */
	void Push(objtype item)
	{
		#ifndef SIMULATION
			uint32_t sr = EnterCriticalSection();
		#endif

		if(!InternalIsFull())
		{
			m_storage[m_wptr] = item;
			m_wptr ++;
			m_empty = false;
		}

		if(m_wptr == depth)
			m_wptr = 0;

		#ifndef SIMULATION
			LeaveCriticalSection(sr);
		#endif
	}

	/**
		@brief Removes an item from the FIFO and returns it.

		Popping an empty FIFO is a legal no-op with an undefined return value.
	 */
	objtype Pop()
	{
		#ifndef SIMULATION
			uint32_t sr = EnterCriticalSection();
		#endif

		objtype ret = *const_cast<objtype*>(&m_storage[m_rptr]);

		if(!IsEmpty())
		{
			m_rptr ++;

			if(m_rptr == depth)
				m_rptr = 0;

			if(m_rptr == m_wptr)
				m_empty = true;
		}

		#ifndef SIMULATION
			LeaveCriticalSection(sr);
		#endif

		return ret;
	}

	/**
		@brief Returns the number of elements in the FIFO
	 */
	uint32_t size()
	{
		#ifndef SIMULATION
			uint32_t sr = EnterCriticalSection();
		#endif

		uint32_t size = 0;
		if(IsEmpty())
			size = 0;
		else if(IsFull())
			size = depth;
		else
			size = (m_wptr - m_rptr) % depth;

		#ifndef SIMULATION
			LeaveCriticalSection(sr);
		#endif

		return size;
	}

	/**
		@brief Checks if the FIFO is empty
	 */
	bool IsEmpty()
	{ return m_empty; }

	/**
		@brief Checks if the FIFO is full
	 */
	bool IsFull()
	{
		#ifndef SIMULATION
			uint32_t sr = EnterCriticalSection();
		#endif

		bool full = InternalIsFull();

		#ifndef SIMULATION
			LeaveCriticalSection(sr);
		#endif

		return full;
	}

	/**
		@brief Resets the FIFO to an empty state
	 */
	void Reset()
	{
		#ifndef SIMULATION
			uint32_t sr = EnterCriticalSection();
		#endif

		m_wptr = 0;
		m_rptr = 0;
		m_empty = true;

		#ifndef SIMULATION
			LeaveCriticalSection(sr);
		#endif
	}

protected:

	/**
		@brief Checks if the FIFO is full without any interlocks.
	 */
	bool InternalIsFull()
	{
		if(m_empty)
			return false;
		return (m_wptr == m_rptr);
	}

	volatile objtype m_storage[depth];
	volatile uint32_t m_wptr;
	volatile uint32_t m_rptr;
	volatile bool m_empty;
};

#endif
