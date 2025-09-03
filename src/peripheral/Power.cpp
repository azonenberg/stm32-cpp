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
#include "Power.h"
#include "RCC.h"

#ifdef HAVE_PWR

#if ( defined(STM32L031) || defined(STM32L431) )

/**
	@brief Configures the internal LDO and blocks until it's stabilized
 */
void Power::ConfigureLDO(VoltageRange vcore)
{
	#ifdef STM32L031

		//Set the voltage mode
		PWR.CR = (PWR.CR & ~PWR_CR_VOS) | (vcore << 11);

		//Wait until stable
		while( (PWR.CSR & PWR_CSR_VOSF) != 0)
		{}

	#elif defined(STM32L431)

		//Set the voltage mode
		PWR.CR1 = (PWR.CR1 & ~PWR_CR1_VOS) | (vcore << 9);

		//Wait until stable
		while( (PWR.SR2 & PWR_SR2_VOSF) != 0)
		{}

	#else

		#error unimplemented

	#endif
}
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Supply configuration

#ifdef STM32H750
/**
	@brief Configures the supply in Mode FIXME (LDO only)
 */
void Power::ConfigureLDO(VoltageRange vcore)
{
	//Can't jump to VOS0 direct from powerup, we need to jump to VOS1 first
	VoltageRange target = vcore;
	if(vcore == RANGE_VOS0)
		target = RANGE_VOS1;

	//LDO enabled, not bypassed
	PWR.CR3 = 0x6;

	//Configure D3 voltage scaling
	PWR.D3CR = (PWR.D3CR & ~PWR_D3CR_VOSMASK) | (target << 14);

	//Wait for power rails to stabilize
	while((PWR.D3CR & PWR_D3CR_VOSRDY) == 0)
	{}
	while((PWR.CSR1 & PWR_CSR1_ACTVOSRDY) == 0)
	{}

	//If we requested VOS0, special stuff needed after we get to VOS1
	//section: vos0 activate/deactivate sequence
	//set SYSCFG_PWRCR in VOS0
	if(vcore == RANGE_VOS0)
	{
		//See "VOS0 activation/deactivation sequence" in RM0433 page 279

		//Enable SYSCFG
		RCCHelper::EnableSyscfg();

		//Enable overdrive mode (don't set VOS to 0 like on the H735)
		SYSCFG.PWRCR |= 1;

		//Wait for VOSRDY
		while((PWR.D3CR & PWR_D3CR_VOSRDY) == 0)
		{}
	}
}
#endif

#ifdef STM32H735
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
