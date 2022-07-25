/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2022 Andrew D. Zonenberg                                                                          *
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

#ifndef octospi_h
#define octospi_h

#ifdef HAVE_OCTOSPI

class OctoSPI
{
public:

	enum mode_t
	{
		MODE_NONE	= 0,
		MODE_SINGLE = 1,
		MODE_DUAL	= 2,
		MODE_QUAD	= 3,
		MODE_OCTAL	= 4
	};

	OctoSPI(volatile octospi_t* lane, uint32_t sizeBytes, uint8_t prescale);

	void SetDoubleRateMode(bool ddr);
	void SetInstructionMode(mode_t mode = MODE_SINGLE, uint8_t nbytes = 0);
	void SetAddressMode(mode_t mode = MODE_SINGLE, uint8_t nbytes = 0);
	void SetAltBytesMode(mode_t mode = MODE_SINGLE, uint8_t nbytes = 0);
	void SetDataMode(mode_t mode = MODE_SINGLE);
	void SetDummyCycleCount(uint8_t ncycles);
	void SetDeselectTime(uint8_t ncycles);

	void SetDQSEnable(bool enable);

	void SetFifoThreshold(uint8_t threshold);

	void BlockingWrite(uint32_t insn, uint32_t addr, uint8_t* data, uint32_t len);

protected:
	volatile octospi_t*	m_lane;
};

#endif

#endif
