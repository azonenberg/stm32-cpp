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

#ifndef stm32_syscfg_h
#define stm32_syscfg_h

#define HAVE_SYSCFG

//STM32H735, STM32H750
#if SYSCFG_T_VERSION == 1

#define HAVE_PKG

typedef struct
{
	uint32_t field_0;
	uint32_t PMCR;
	uint32_t EXTICR1;
	uint32_t EXTICR2;
	uint32_t EXTICR3;
	uint32_t EXTICR4;
	uint32_t CFGR;
	uint32_t field_1c;
	uint32_t CCCSR;
	uint32_t CCVR;
	uint32_t CCCR;
	uint32_t PWRCR;				//reserved / not present in H735
	uint32_t ADC2ALT;			//reserved / not present in H750
	uint32_t field_34[60];
	uint32_t PKGR;
	uint32_t field_128[118];
	uint32_t UR0;
	uint32_t field_304;
	uint32_t UR2;
	uint32_t UR3;
	uint32_t UR4;
	uint32_t UR5;
	uint32_t UR6;
	uint32_t UR7;
	uint32_t UR8;				//reserved / not present in H735
	uint32_t UR9;				//reserved / not present in H735
	uint32_t UR10;				//reserved / not present in H735
	uint32_t UR11;
	uint32_t UR12;
	uint32_t UR13;
	uint32_t UR14;
	uint32_t UR15;
	uint32_t UR16;
	uint32_t UR17;
} syscfg_t;

enum syscfg_pmcr
{
	ETH_MODE_RMII	= 0x00800000
};

//STM32MP257
#elif SYSCFG_T_VERSION == 2

enum syscfg_bootmode_t
{
	BOOT_MODE_DEV1		= 3,
	BOOT_MODE_M33TD_SPI = 11,
	BOOT_MODE_DEV2		= 12
};

struct syscfg_t
{
	uint32_t	BOOTSR;
	uint32_t	BOOTCR;
	uint32_t	field_08[254];
	uint32_t	DLYBSD1CR;
	uint32_t	DLYBSD1SR;
	uint32_t	SDMMC1CR;
	uint32_t	field_40c[253];
	uint32_t	DLYBSD2CR;
	uint32_t	DLYBSD2SR;
	uint32_t	SDMMC2CR;
	uint32_t	field_80c[253];
	uint32_t	DLYBSD3CR;
	uint32_t	DLYBSD3SR;
	uint32_t	field_c08[254];
	uint32_t	DLYBOS1CR;
	uint32_t	DLYBOS1SR;
	uint32_t	field_1008[254];
	uint32_t	DLYBOS2CR;
	uint32_t	DLYBOS2SR;
	uint32_t	field_1408[254];
	uint32_t	VDERAMCR;
	uint32_t	POTTAMPRSTCR;
	uint32_t	field_1808[254];
	uint32_t	M33SSCR;
	uint32_t	field_1c04[255];
	uint32_t	ICNQPCR1;
	uint32_t	ICNQPCR2;
	uint32_t	ICNEWRCR;
	uint32_t	ICNCGCR;
	uint32_t	ICNGPUBWLCR;
	uint32_t	ICNE2EBWRCR;
	uint32_t	SAFERSTCR;
	uint32_t	ICNPCIBWLCR;
	uint32_t	ICNETHBWLCR;
	uint32_t	ICNUSB3BWLCR;
	uint32_t	ICNCPU1BWLCR;
	uint32_t	ICNLTDCBWLCR;
	uint32_t	ICNDCMIPPBWLCR;
	uint32_t	ICNVDEBWLCR;
	uint32_t	field_2038[242];
	uint32_t	USB2PHY1CR;
	uint32_t	USB2PHY1BCCR;
	uint32_t	USB2PHY1BCSR;
	uint32_t	USB2PHY1TRIM1CR;
	uint32_t	USB2PHY1TRIM2CR;
	uint32_t	field_2414[3];
	uint32_t	USBHCR;
	uint32_t	field_2424[247];
	uint32_t	USB2PHY2CR;
	uint32_t	field_2804;
	uint32_t	USB2PHY2TRIM1CR;
	uint32_t	USB2PHY2TRIM2CR;
	uint32_t	field_2810[252];
	uint32_t	OCTOSPIAMCR;
	uint32_t	field_2c04[255];
	uint32_t	ETH1CR;
	uint32_t	field_3004[3];
	uint32_t	ETH1SR;
	uint32_t	field_3014[251];
	uint32_t	ETH2CR;
	uint32_t	field_3404[3];
	uint32_t	ETH2SR;
	uint32_t	field_3414[251];
	uint32_t	ETHSWCR;
	uint32_t	field_3804[511];
	uint32_t	VDDIO3CCCR;
	uint32_t	VDDIO3CCSR;
	uint32_t	VDDIO4CCCR;
	uint32_t	VDDIO4CCSR;
	uint32_t	SYSCFG_VDDCCCR;
	uint32_t	SYSCFG_VDDCCSR;
	uint32_t	VDDIO2CCCR;
	uint32_t	VDDIO2CCSR;
	uint32_t	VDDIO1CCCR;
	uint32_t	VDDIO1CCSR;
	uint32_t	field_4028[246];
	uint32_t	CBR;
	uint32_t	field_4404[255];
	uint32_t	USB3DRCR;
	uint32_t	USB3DRSR;
	uint32_t	field_4808[254];
	uint32_t	COMBOPHYCR1;
	uint32_t	COMBOPHYCR2;
	uint32_t	COMBOPHYCR3;
	uint32_t	COMBOPHYCR4;
	uint32_t	COMBOPHYCR5;
	uint32_t	COMBOPHYSR;
	uint32_t	field_4c18[250];
	uint32_t	DISPLAYCLKCR;
	uint32_t	field_5004[1023];
	uint32_t	PCIECR;
	uint32_t	PCIEPMEMSICR;
	uint32_t	PCIEAERRCMSICR;
	uint32_t	PCIESYSRCCR;
	uint32_t	PCIEEPTMIRQCR;
	uint32_t	field_6014[27];
	uint32_t	PCIEPRGCR;
	uint32_t	field_6084[31];
	uint32_t	PCIESR1;
	uint32_t	PCIESR2;
	uint32_t	field_6108[190];
	uint32_t	IDC;
	uint32_t	field_6404[1788];
	uint32_t	VERR;
	uint32_t	IPIDR;
	uint32_t	SIDR;
};

#else

#error Undefined or unspecified SYSCFG_T_VERSION

#endif	//version check

#endif	//include guard
