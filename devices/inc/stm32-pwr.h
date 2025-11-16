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

#ifndef stm32_pwr_h
#define stm32_pwr_h

#define HAVE_PWR

//STM32H735, H750
#if PWR_T_VERSION == 1

typedef struct
{
	uint32_t CR1;
	uint32_t CSR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t CPUCR;
	uint32_t field_14;
	uint32_t D3CR;
	uint32_t WKUPCR;
	uint32_t WKUPFR;
	uint32_t WKUPEPR;
} pwr_t;
extern volatile pwr_t PWR;

#ifdef STM32H735
	enum VoltageRange
	{
		RANGE_VOS3 = 1,	//0.95 - 1.05V
		RANGE_VOS2 = 2,	//1.05 - 1.15V
		RANGE_VOS1 = 3,	//1.15 - 1.26V
		RANGE_VOS0 = 0	//1.26 - 1.4V
	};
#elif defined(STM32H750)
	enum VoltageRange
	{
		RANGE_VOS3 = 1,	//0.95 - 1.26V
		RANGE_VOS2 = 2,	//1.05 - 1.26V
		RANGE_VOS1 = 3,	//1.15 - 1.26V
		RANGE_VOS0 = 0	//1.26 - 1.4V (PWR_D3CR suggests reserved and unavailable?? but seems rev V parts have it)
	};
#else
	#error Unknown STM32H7 PWR version
#endif

enum pwr_csr1
{
	PWR_CSR1_ACTVOSRDY = 0x2000
};

enum pwr_d3cr
{
	PWR_D3CR_VOSRDY = 0x2000,

	PWR_D3CR_VOSMASK = 0xc000
};

//STM32MP257
#elif PWR_T_VERSION == 2

enum pwr_cr1
{
	PWR_CR1_VDDIO4RDY	= 0x0002'0000,
	PWR_CR1_VDDIO3RDY	= 0x0001'0000,

	PWR_CR1_VDDIO4SV 	= 0x0000'0200,
	PWR_CR1_VDDIO3SV 	= 0x0000'0100,

	PWR_CR1_VDDIO4VMEN	= 0x0000'0002,
	PWR_CR1_VDDIO3VMEN	= 0x0000'0001
};

enum pwr_cr7
{
	PWR_CR7_VDDIO2RDY	= 0x0001'0000,
	PWR_CR7_VDDIO2SV	= 0x0000'0100,
	PWR_CR7_VDDIO2VMEN	= 0x0000'0001
};

enum pwr_cr8
{
	PWR_CR8_VDDIO1RDY	= 0x0001'0000,
	PWR_CR8_VDDIO1SV	= 0x0000'0100,
	PWR_CR8_VDDIO1VMEN	= 0x0000'0001
};

enum pwr_bdcr1
{
	PWR_BDCR1_DBD3P		= 0x0000'0001
};

struct pwr_t
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t field_0c;
	uint32_t CR5;
	uint32_t CR6;
	uint32_t CR7;
	uint32_t CR8;
	uint32_t CR9;
	uint32_t CR10;
	uint32_t CR11;
	uint32_t CR12;
	uint32_t field_30[2];
	uint32_t BDCR1;
	uint32_t BDCR2;
	uint32_t CPU1CR;
	uint32_t CPU2CR;
	uint32_t CPU3CR;
	uint32_t D1CR;
	uint32_t D2CR;
	uint32_t D3CR;
	uint32_t field_58;
	uint32_t field_5c;
	uint32_t WKUPCR1;
	uint32_t WKUPCR2;
	uint32_t WKUPCR3;
	uint32_t WKUPCR4;
	uint32_t WKUPCR5;
	uint32_t WKUPCR6;
	uint32_t field_78[8];
	uint32_t D3WKUPEN;
	uint32_t field_9c[25];
	uint32_t RSECCFGR;
	uint32_t RPRIVCFGR;
	uint32_t R0CIDCFGR;
	uint32_t R1CIDCFGR;
	uint32_t R2CIDCFGR;
	uint32_t R3CIDCFGR;
	uint32_t R4CIDCFGR;
	uint32_t R5CIDCFGR;
	uint32_t R6CIDCFGR;
	uint32_t field_124[23];
	uint32_t WIOSECCFGR;
	uint32_t WIOPRIVCFGR;
	uint32_t WIO1CIDCFGR;
	uint32_t WIO2CIDCFGR;
	uint32_t WIO3CIDCFGR;
	uint32_t WIO4CIDCFGR;
	uint32_t WIO5CIDCFGR;
	uint32_t WIO6CIDCFGR;
	uint32_t WIO1SEMCR;
	uint32_t WIO2SEMCR;
	uint32_t WIO3SEMCR;
	uint32_t WIO4SEMCR;
	uint32_t WIO5SEMCR;
	uint32_t WIO6SEMCR;
	uint32_t field_1b8[18];
	uint32_t CPU1D1SR;
	uint32_t CPU2D2SR;
	uint32_t CPU3D3SR;
	uint32_t field_20c[122];
	uint32_t VERR;
	uint32_t IPIDR;
	uint32_t SIDR;
};

#else

#error Undefined or unspecified PWR_T_VERSION

#endif	//version check

#endif	//include guard
