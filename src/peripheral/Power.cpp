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

#include <stm32.h>
#include "Power.h"

#ifdef HAVE_PWR

#ifdef STM32H735

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Supply configuration

/**
	@brief Configures the supply in Mode 3 (SMPS supplies LDO)
 */
__attribute__((optimize("O2")))
void Power::ConfigureSMPSToLDOCascade(SmpsVoltage vsmps, VoltageRange vcore)
{
	//SMPS enabled in normal mode, LDO enabled, not bypassed
	if(vsmps == VOLTAGE_1V8)
		PWR.CR3 = 0x56;
	else //if(vsmps == VOLTAGE_2V5)
		PWR.CR3 = 0x66;

	//Configure D3 voltage scaling
	PWR.D3CR = (PWR.D3CR & ~PWR_D3CR_VOSMASK) | (vcore << 14);

	//Wait for power rails to stabilize
	while((PWR.D3CR & PWR_D3CR_VOSRDY) == 0)
	{}
	while((PWR.CSR1 & PWR_CSR1_ACTVOSRDY) == 0)
	{}
}

#endif

#endif
