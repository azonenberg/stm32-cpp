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

#ifndef exti_h
#define exti_h

#ifdef HAVE_EXTI

/**
	@brief The external interrupt controller

	For now only tested on STM32L431
 */
class EXTI
{
public:

	enum ExtiPort
	{
		PORT_A = 0,
		PORT_B = 1,
		PORT_C = 2,
		PORT_D = 3,
		PORT_E = 4,
		PORT_F = 5,
		PORT_G = 6,
		PORT_H = 7
	};

	static void SetExtInterruptMux(int channel, ExtiPort sel);

	///@brief Clear a pending interrupt
	static void ClearPending(int channel)
	{
		#if EXTI_T_VERSION == 1
			if(channel < 32)
				_EXTI.PR1 = (1 << channel);
			else
				_EXTI.PR2 = (1 << (channel - 32));
		#elif EXTI_T_VERSION == 2
			_EXTI.PR = (1 << channel);
		#else
			#error Unrecognized EXTI_T_VERSION
		#endif
	}

	///@brief Enables an interrupt lane
	static void EnableChannel(int channel)
	{
		#if EXTI_T_VERSION == 1
			if(channel < 32)
				_EXTI.IMR1 = (1 << channel);
			else
				_EXTI.IMR2 = (1 << (channel - 32));
		#elif EXTI_T_VERSION == 2
			_EXTI.IMR = (1 << channel);
		#else
			#error Unrecognized EXTI_T_VERSION
		#endif
	}

	///@brief Enable rising edge trigger
	static void EnableRisingEdgeTrigger(int channel)
	{
		#if EXTI_T_VERSION == 1
			if(channel < 32)
				_EXTI.RTSR1 = (1 << channel);
			else
				_EXTI.RTSR2 = (1 << (channel - 32));
		#elif EXTI_T_VERSION == 2
			_EXTI.RTSR = (1 << channel);
		#else
			#error Unrecognized EXTI_T_VERSION
		#endif
	}

	///@brief Enable falling edge trigger
	static void EnableFallingEdgeTrigger(int channel)
	{
		#if EXTI_T_VERSION == 1
			if(channel < 32)
				_EXTI.FTSR1 = (1 << channel);
			else
				_EXTI.FTSR2 = (1 << (channel - 32));
		#elif EXTI_T_VERSION == 2
			_EXTI.FTSR = (1 << channel);
		#else
			#error Unrecognized EXTI_T_VERSION
		#endif
	}
};

#endif

#endif
