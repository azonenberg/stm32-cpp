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

#ifndef stm32_itm_h
#define stm32_itm_h

enum demcr_t
{
	DEMCR_TRCENA	= 0x0100'0000
};

enum itm_tcr_t
{
	ITM_TCR_TXENA	= 0x0000'0008,
	ITM_TCR_ITMENA	= 0x0000'0001
};

enum itm_lar_t
{
	ITM_LAR_UNLOCK	= 0xC5ACCE55
};

struct itm_t
{
	uint32_t STIM[256];
	uint32_t field_400[640];
	uint32_t TER[8];
	uint32_t field_e20[8];
	uint32_t TPR;
	uint32_t field_e44[15];
	uint32_t TCR;

	//Cortex-M7 specific, TODO figure this out for other chips?
	uint32_t field_e84[27];
	uint32_t ITATRDY;
	uint32_t field_ef4;
	uint32_t ITATVAL;
	uint32_t field_efc;
	uint32_t TCTRL;
	uint32_t field_f04[43];
	uint32_t LAR;
	uint32_t LSR;
};

#endif	//include guard
