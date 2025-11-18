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

#ifndef octospimanager_h
#define octospimanager_h

#ifdef HAVE_OCTOSPI

/**
	@brief OCTOSPI manager
 */
class OctoSPIManager
{
public:

	//STM32H735
	#if OCTOSPI_T_VERSION == 1

		/**
			@brief A group of four DQ lines from the internal controllers. May or may not be mapped to output pins.
		 */
		enum halfport_t
		{
			C1_LOW		= 0,
			C1_HIGH		= 1,
			C2_LOW		= 2,
			C2_HIGH		= 3,

			//mux mode reuses C1 selector
			MUX_OUT_LOW		= C1_LOW,
			MUX_OUT_HIGH	= C1_HIGH
		};

		/**
			@brief Selector for a line that can come from port 1 or 2
		 */
		enum channel_t
		{
			PORT_1 = 0,
			PORT_2 = 1
		};

		static void ConfigureMux(
			bool			muxEnable,
			int				busTurnaround = 1
			);

		static void ConfigurePort(
			int				port,
			bool			dq74Enabled,
			halfport_t		dq74Source,
			bool			dq30Enabled,
			halfport_t		dq30Source,
			bool			csEnabled,
			channel_t		csSource,
			bool			dqsEnabled,
			channel_t		dqsSource,
			bool			clkEnabled,
			channel_t		clkSource);

	#endif
};

#endif
#endif
