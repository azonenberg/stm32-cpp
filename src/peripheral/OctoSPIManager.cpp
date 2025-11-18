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
#include <peripheral/OctoSPIManager.h>

#ifdef HAVE_OCTOSPI

//STM32H735
#if OCTOSPI_T_VERSION == 1

/**
	@brief Enables or disables mux mode

	@param muxEnable		True to enable mux mode, false to disable muix
	@param busTurnaround	Bus turnaround time, in clock cycles. ignored if muxEnable is false.
 */
void OctoSPIManager::ConfigureMux(
	bool			muxEnable,
	int				busTurnaround
	)
{
	uint8_t turn = busTurnaround - 1;
	OCTOSPIM.CR = (turn << 16);
	if(muxEnable)
		OCTOSPIM.CR |= 1;
}

/**
	@brief Configures a single port's I/O pins

	@param port				The port to configure
	@param dq74Enabled		True if DQ pins 7:4 are used
	@param dq74Source		Data source for DQ pins 7:4. Ignored if dq74Enabled is false.
	@param dq30Enabled		True if DQ pins 3:0 are used
	@param dq30Source		Data source for DQ pins 3:0. Ignored if dq30Enabled is false.
	@param csEnabled		True if CS# pin is used
	@param csSource			Selector for CS# pin
	@param dqsEnabled		True if DQS pin is used
	@param dqsSource		Selector for DQS pin
	@param clkEnabled		True if CLK pin is used
	@param clkSource		Selector for CLK pin
 */
void OctoSPIManager::ConfigurePort(
	int				port,
	bool			dq74Enabled,
	halfport_t		dq74Source,
	bool			dq30Enabled,
	halfport_t		dq30Source,
	bool			csEnabled,
	channel_t		csSource,
	bool			/*dqsEnabled*/,
	channel_t		/*dqsSource*/,
	bool			clkEnabled,
	channel_t		clkSource)
{
	uint32_t val = 0;

	if(dq74Enabled)
		val |= 0x01000000 | (dq74Source << 25);
	if(dq30Enabled)
		val |= 0x00010000 | (dq30Source << 17);
	if(csEnabled)
		val |= 0x00000100 | (csSource << 9);
	if(clkEnabled)
		val |= 0x00000001 | (clkSource << 1);

	if(port == 1)
		OCTOSPIM.P1CR = val;
	else
		OCTOSPIM.P2CR = val;
}

#endif
#endif
