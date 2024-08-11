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

#include <stm32.h>
#include "EXTI.h"

#ifdef HAVE_EXTI

/**
	@brief Sets the external interrupt mux selector for a given interrupt channel
 */
void EXTI::SetExtInterruptMux(int channel, ExtiPort sel)
{
	//STM32L431 specific possibly?
	if(channel < 4)
	{
		int shift = channel * 4;
		SYSCFG.EXTICR1 = (SYSCFG.EXTICR1 & ~(0x7 << shift)) | (sel << shift);
	}
	else if(channel < 8)
	{
		int shift = (channel - 4) * 4;
		SYSCFG.EXTICR2 = (SYSCFG.EXTICR2 & ~(0x7 << shift)) | (sel << shift);
	}
	else if(channel < 12)
	{
		int shift = (channel - 8) * 4;
		SYSCFG.EXTICR3 = (SYSCFG.EXTICR3 & ~(0x7 << shift)) | (sel << shift);
	}
	else
	{
		int shift = (channel - 12) * 4;
		SYSCFG.EXTICR4 = (SYSCFG.EXTICR4 & ~(0x7 << shift)) | (sel << shift);
	}
}

#endif
