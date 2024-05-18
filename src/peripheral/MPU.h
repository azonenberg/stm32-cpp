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

#ifndef MPU_h
#define MPU_h

class MPU
{
public:
	enum defaultMapMode
	{
		KEEP_DEFAULT_MAP = 4,
		NO_DEFAULT_MAP = 0
	};

	enum faultMode
	{
		ENABLE_IN_FAULT_HANDLERS = 2,
		DISABLE_IN_FAULT_HANDLERS = 0
	};

	static void Configure(defaultMapMode mapMode, faultMode fmode)
	{ _MPU.ctrl = 1 | mapMode | fmode; }

	enum sizeCode
	{
		SIZE_128K	= 0x10,
		SIZE_256K	= 0x11,
		SIZE_512K	= 0x12,
		SIZE_1M		= 0x13,
		SIZE_2M		= 0x14,
		SIZE_4M		= 0x15,
		SIZE_8M		= 0x16,
		SIZE_16M	= 0x17,
		SIZE_32M	= 0x18
	};

	enum texscb
	{
		STRONGLY_ORDERED	= 0x000
	};

	enum aperm
	{
		NO_ACCESS		= 0x00,
		FULL_ACCESS		= 0x03
	};

	enum nxmode
	{
		EXECUTE_ALLOWED	= 0x0,
		EXECUTE_NEVER	= 0x1
	};

	static void ConfigureRegion(uint8_t region, uint32_t baseAddress, texscb t, aperm ap, nxmode nx, sizeCode sizecode)
	{
		_MPU.rnr = region;
		_MPU.rbar = (baseAddress & 0xFFFFFFE0);
		_MPU.rasr = (nx << 28) | (ap << 24) | (t << 16) | (sizecode << 1) | 1;
	}
};

#endif
