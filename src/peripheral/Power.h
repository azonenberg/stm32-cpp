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

#ifndef Power_h
#define Power_h

#ifdef HAVE_PWR

/**
	@brief Power control

	All methods are static as there's only one power subsystem in the device.
 */
class Power
{
public:

	#if ( defined(STM32L031) || defined(STM32L431) )
	static void ConfigureLDO(VoltageRange vcore);
	#endif

	#if defined(STM32L431)
	static void EnableBackupSramWrites()
	{ PWR.CR1 |= PWR_CR1_DBP; }
	#endif

	#ifdef STM32H735

	enum SmpsVoltage
	{
		VOLTAGE_1V8,
		VOLTAGE_2V5
	};

	static void ConfigureSMPSToLDOCascade(SmpsVoltage vsmps, VoltageRange vcore);
	#endif

	#ifdef STM32H750
	static void ConfigureLDO(VoltageRange vcore);
	#endif

	#ifdef STM32MP257
		static void EnableBackupSramWrites()
		{ PWR.BDCR1 |= PWR_BDCR1_DBD3P; }

		static void EnableVDDIO1Monitor()
		{ PWR.CR8 |= PWR_CR8_VDDIO1VMEN; }

		static void EnableVDDIO2Monitor()
		{ PWR.CR7 |= PWR_CR7_VDDIO2VMEN; }

		static void EnableVDDIO3Monitor()
		{ PWR.CR1 |= PWR_CR1_VDDIO3VMEN; }

		static void EnableVDDIO4Monitor()
		{ PWR.CR1 |= PWR_CR1_VDDIO4VMEN; }

		static bool IsVDDIO1Ready()
		{ return (PWR.CR8 & PWR_CR8_VDDIO1RDY) == PWR_CR8_VDDIO1RDY; }

		static bool IsVDDIO2Ready()
		{ return (PWR.CR7 & PWR_CR7_VDDIO2RDY) == PWR_CR7_VDDIO2RDY; }

		static bool IsVDDIO3Ready()
		{ return (PWR.CR1 & PWR_CR1_VDDIO3RDY) == PWR_CR1_VDDIO3RDY; }

		static bool IsVDDIO4Ready()
		{ return (PWR.CR1 & PWR_CR1_VDDIO4RDY) == PWR_CR1_VDDIO4RDY; }

		static void SetVDDIO1Valid(bool valid = true)
		{
			if(valid)
				PWR.CR8 |= PWR_CR8_VDDIO1SV;
			else
				PWR.CR8 &= ~PWR_CR8_VDDIO1SV;
		}

		static void SetVDDIO2Valid(bool valid = true)
		{
			if(valid)
				PWR.CR7 |= PWR_CR7_VDDIO2SV;
			else
				PWR.CR7 &= ~PWR_CR7_VDDIO2SV;
		}

		static void SetVDDIO3Valid(bool valid = true)
		{
			if(valid)
				PWR.CR1 |= PWR_CR1_VDDIO3SV;
			else
				PWR.CR1 &= ~PWR_CR1_VDDIO3SV;
		}

		static void SetVDDIO4Valid(bool valid = true)
		{
			if(valid)
				PWR.CR1 |= PWR_CR1_VDDIO4SV;
			else
				PWR.CR1 &= ~PWR_CR1_VDDIO4SV;
		}

	#endif
};

#endif

#endif
