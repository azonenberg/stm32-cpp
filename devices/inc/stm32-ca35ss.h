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

#ifndef stm32_ca35ss_h
#define stm32_ca35ss_h

#define HAVE_CA35SS

//STM32MP257
#if CA35SS_T_VERSION == 1

struct ca35ss_syscfg_t
{
	uint32_t ETR_LPI_CR;
	uint32_t ETR_LPI_SR;
	uint32_t STM_NSGUAREN_CR;
	uint32_t TACE_CLK_DIV_CR;
	uint32_t DBGPWR_CR;
	uint32_t DBGL1RSTDISABLE_CR;
	uint32_t DBGPWR_SR;
	uint32_t EDBGACK_SR;
	uint32_t GIC_CFGR;
	uint32_t LP_SR;
	uint32_t RSTACK_SR;
	uint32_t field_2c[21];
	uint32_t AARCH_MODE_CR;
	uint32_t VBAR_CR;
	uint32_t M33_ACCESS_CR;
	uint32_t field_8c[5];
	uint32_t M33_TZEN_CR;
	uint32_t M33_INITSVTOR_CR;
	uint32_t M33_INITNSVTOR_CR;
};

struct ca35ss_bitband_t
{
	uint32_t reg;
	uint32_t set;
	uint32_t clear;
	uint32_t toggle;
};

struct ca35ss_t
{
	ca35ss_bitband_t CHGCLKREQ;
	ca35ss_bitband_t BRM;
	uint32_t field_20[24];
	ca35ss_bitband_t PLL_FREQ1;
	ca35ss_bitband_t PLL_FREQ2;
	ca35ss_bitband_t PLL_EN;
	uint32_t field_b0[8];
	ca35ss_bitband_t LPI_TSGEN_NTS_CR;
	uint32_t field_e0[8];
	uint32_t C0_SMP;
	uint32_t field_104[3];
	uint32_t C1_SMP;
	uint32_t field_114[11];
	ca35ss_bitband_t LPI_STGEN_NTS_CR;
	uint32_t field_150[156];
	ca35ss_bitband_t NS_ENABLE_0_RW;
	uint32_t SSC_NS_ENABLE_1;
	uint32_t field_3d4[1803];
	ca35ss_syscfg_t syscfg;
};

#else

#error Undefined or unspecified CA35SS_T_VERSION

#endif	//version check

#endif	//include guard
