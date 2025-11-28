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

#ifndef stm32_rcc_h
#define stm32_rcc_h

//STM32H735, H750
#if RCC_T_VERSION == 1

struct rcc_t
{
	uint32_t CR;
	uint32_t HSICFGR;
	uint32_t CRCCR;
	uint32_t CSICFGR;
	uint32_t CFGR;
	uint32_t field_14;
	uint32_t D1CFGR;
	uint32_t D2CFGR;
	uint32_t D3CFGR;
	uint32_t field_24;
	uint32_t PLLCKSELR;
	uint32_t PLLCFGR;
	uint32_t PLL1DIVR;
	uint32_t PLL1FRACR;
	uint32_t PLL2DIVR;
	uint32_t PLL2FRACR;
	uint32_t PLL3DIVR;
	uint32_t PLL3FRACR;
	uint32_t field_48;
	uint32_t D1CCIPR;
	uint32_t D2CCIP1R;
	uint32_t D2CCIP2R;
	uint32_t D3CCIPR;
	uint32_t field_5c;
	uint32_t CIER;
	uint32_t CIFR;
	uint32_t CICR;
	uint32_t field_6c;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t field_78;
	uint32_t AHB3RSTR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB4RSTR;
	uint32_t APB3RSTR;
	uint32_t APB1LRSTR;
	uint32_t APB1HRSTR;
	uint32_t APB2RSTR;
	uint32_t APB4RSTR;
	uint32_t GCR;
	uint32_t field_a4;
	uint32_t D3AMR;
	uint32_t field_ac[9];
	uint32_t RSR;
	uint32_t AHB3ENR;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB4ENR;
	uint32_t APB3ENR;
	uint32_t APB1LENR;
	uint32_t APB1HENR;
	uint32_t APB2ENR;
	uint32_t APB4ENR;
	uint32_t field_f8;
	uint32_t AHB3LPENR;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB4LPENR;
	uint32_t APB3LPENR;
	uint32_t APB1LLPENR;
	uint32_t APB1HLPENR;
	uint32_t APB2LPENR;
	uint32_t APB4LPENR;
	uint32_t field_120[4];
	uint32_t C1_RSR;
	uint32_t C1_AHB3ENR;
	uint32_t C1_AHB1ENR;
	uint32_t C1_AHB2ENR;
	uint32_t C1_AHB4ENR;
	uint32_t C1_APB3ENR;
	uint32_t C1_APB1LENR;
	uint32_t C1_APB1HENR;
	uint32_t C1_APB2ENR;
	uint32_t C1_APB4ENR;
	uint32_t field_158;
	uint32_t C1_AHB3LPENR;
	uint32_t C1_AHB1LPENR;
	uint32_t C1_AHB2LPENR;
	uint32_t C1_AHB4LPENR;
	uint32_t C1_APB3LPENR;
	uint32_t C1_APB1LLPENR;
	uint32_t C1_APB1HLPENR;
	uint32_t C1_APB2LPENR;
	uint32_t C1_APB4LPENR;
};

//STM32MP257
#elif RCC_T_VERSION == 2

struct cidsem_t
{
	uint32_t CIDCFGR;
	uint32_t SEMCR;
};

struct pllcfg_t
{
	uint32_t CFGR1;
	uint32_t CFGR2;
	uint32_t CFGR3;
	uint32_t CFGR4;
	uint32_t CFGR5;
	uint32_t field_18;
	uint32_t CFGR6;
	uint32_t CFGR7;
	uint32_t field_24;
	uint32_t field_28;
};

enum rcc_genericcfgr
{
	RCC_GENERIC_CFGR_EN = 2
};

enum rcc_ocen
{
	RCC_OCEN_HSEON		= 0x0000'0100,
	RCC_OCEN_HSION		= 0x0000'0001
};

enum rcc_ocrdy
{
	RCC_OCRDY_HSERDY	= 0x0000'0100,
	RCC_OCRDY_HSIRDY	= 0x0000'0001
};

//Bit positions in MUXSELCFGR
enum rcc_muxsel_lane
{
	RCC_MUXSEL_PLL3		= 28,
	RCC_MUXSEL_PLL2		= 24,
	RCC_MUXSEL_PLL1		= 20,
	RCC_MUXSEL_PLL8		= 16,
	RCC_MUXSEL_PLL7		= 12,
	RCC_MUXSEL_PLL6		= 8,
	RCC_MUXSEL_PLL5		= 4,
	RCC_MUXSEL_PLL4		= 0
};

//Selectors for MUXSELCFGR
enum rcc_muxsel_val
{
	RCC_MUXSEL_HSI		= 0,
	RCC_MUXSEL_HSE		= 1,
	RCC_MUXSEL_MSI		= 2
};

enum rcc_pll_cfgr1
{
	RCC_PLLCFGR1_CKREFST	= 0x1000'0000,
	RCC_PLLCFGR1_PLLRDY		= 0x0100'0000,
	RCC_PLLCFGR1_PLLEN		= 0x0000'0100,
};

enum rcc_pll_cfgr3
{
	RCC_PLL3CFGR_SSCGDIS	= 0x0400'0000
};

enum rcc_pll_cfgr4
{
	RCC_PLL4CFGR_FOUTPOSTDIVEN	= 0x0000'0200
};

enum rcc_prediv
{
	RCC_PREDIV_1			= 0,
	RCC_PREDIV_2			= 1,
	RCC_PREDIV_4			= 3,
	RCC_PREDIV_1024			= 0x3ff
};

enum rcc_findiv
{
	RCC_FINDIV_EN			= 0x40
};

enum rcc_xbarcfg
{
	RCC_XBAR_STS			= 0x80,
	RCC_XBAR_EN				= 0x40
};

enum rcc_xbarmux
{
	RCC_XBAR_PLL4			= 0,
	RCC_XBAR_PLL5			= 1,
	RCC_XBAR_PLL6			= 2,
	RCC_XBAR_PLL7			= 3,
	RCC_XBAR_PLL8			= 4,
	RCC_XBAR_HSI			= 5,
	RCC_XBAR_HSE			= 6,
	RCC_XBAR_MSI			= 7,
	RCC_XBAR_HSI_KER		= 8,
	RCC_XBAR_HSE_KER		= 9,
	RCC_XBAR_MSI_KER		= 0xa,
	RCC_XBAR_SPDIF_SYMB		= 0xb,
	RCC_XBAR_I2C_CK			= 0xc,
	RCC_XBAR_LSI			= 0xd,
	RCC_XBAR_LSE			= 0xe
};

//see table 13 and 129
enum rcc_xbar_channel
{
	RCC_ck_icn_hs_mcu		= 0,	//max 400 MHz
	RCC_ck_icn_sdmmc		= 1,	//max 200 MHz
	RCC_ck_icn_ddr			= 2,	//max 600 MHz
	RCC_ck_icn_display		= 3,	//max 400 MHz
	RCC_ck_icn_hsl			= 4,	//max 300 MHz
	RCC_ck_icn_nic			= 5,	//max 400 MHz
	RCC_ck_ker_usart6		= 20,	//max 100 MHz
	RCC_ck_ker_ospi1		= 48,	//max 133 MHz
	RCC_ck_mco1				= 61,	//max 160 MHz
	RCC_ck_mco2				= 62	//max 160 MHz
};

enum rcc_apb_div
{
	RCC_APB_DIV_1		= 0,
	RCC_APB_DIV_2		= 1,
	RCC_APB_DIV_4		= 2,
	RCC_APB_DIV_8		= 3,
	RCC_APB_DIV_16		= 4
};

struct rcc_t
{
	uint32_t SECCFGR0;
	uint32_t SECCFGR1;
	uint32_t SECCFGR2;
	uint32_t SECCFGR3;
	uint32_t PRIVCFGR0;
	uint32_t PRIVCFGR1;
	uint32_t PRIVCFGR2;
	uint32_t PRIVCFGR3;
	uint32_t RCFGLOCKR0;
	uint32_t RCFGLOCKR1;
	uint32_t RCFGLOCKR2;
	uint32_t RCFGLOCKR3;
	cidsem_t cidsem[114];
	uint32_t field_3c0[16];
	uint32_t GRSTCSETR;
	uint32_t C1RSTCSETR;
	uint32_t C1P1RSTCSETR;
	uint32_t C2RSTCSETR;
	uint32_t HWRSTCLRR;
	uint32_t C1HWRSTSCLRR;
	uint32_t C2HWRSTSCLRR;
	uint32_t C1BOOTRSTSSETR;
	uint32_t C1BOOTRSTSCLRR;
	uint32_t C2BOOTRSTSSETR;
	uint32_t C2BOOTRSTSCLRR;
	uint32_t C1SREQSETR;
	uint32_t C1SREQCLRR;
	uint32_t CPUBOOTCR;
	uint32_t STBYBOOTCR;
	uint32_t LEGBOOTCR;
	uint32_t BDCR;
	uint32_t D3CR;
	uint32_t D3DSR;
	uint32_t RDCR;
	uint32_t C1MSRDCR;
	uint32_t PWRLPDLYCR;
	uint32_t C1CIESETR;
	uint32_t C1CIFCLRR;
	uint32_t C2CIESETR;
	uint32_t C2CIFCLRR;
	uint32_t IWDGC1FZSETR;
	uint32_t IWDGC1FZCLRR;
	uint32_t IWDGC1CFGSETR;
	uint32_t IWDGC1CFGCLRR;
	uint32_t IWDGC2FZSETR;
	uint32_t IWDGC2FZCLRR;
	uint32_t IWDGC2CFGSETR;
	uint32_t IWDGC2CFGCLRR;
	uint32_t IWDGC3CFGSETR;
	uint32_t IWDGC3CFGCLRR;
	uint32_t C3CFGR;
	uint32_t MCO1CFGR;
	uint32_t MCO2CFGR;
	uint32_t OCENSETR;
	uint32_t OCENCLRR;
	uint32_t OCRDYR;
	uint32_t HSICFGR;
	uint32_t MSICFGR;
	uint32_t RTCDIVR;
	uint32_t APB1DIVR;
	uint32_t APB2DIVR;
	uint32_t APB3DIVR;
	uint32_t APB4DIVR;
	uint32_t APBDBGDIVR;
	uint32_t TIMG1PRER;
	uint32_t TIMG2PRER;
	uint32_t LSMCUDIVR;
	uint32_t DDRCPCFGR;
	uint32_t DDRCAPBCFGR;
	uint32_t DDRPHYCAPBCFGR;
	uint32_t DDRPHYCCFGR;
	uint32_t DDRCFGR;
	uint32_t DDRITFCFGR;
	uint32_t field_4ec;
	uint32_t SYSRAMCFGR;
	uint32_t VDERAMCFGR;
	uint32_t SRAM1CFGR;
	uint32_t SRAM2CFGR;
	uint32_t RETRAMCFGR;
	uint32_t BKPSRAMCFGR;
	uint32_t LPSRAM1CFGR;
	uint32_t LPSRAM2CFGR;
	uint32_t LPSRAM3CFGR;
	uint32_t OSPI1CFGR;
	uint32_t OSPI2CFGR;
	uint32_t FMCCFGR;
	uint32_t DBGCFGR;
	uint32_t STMCFGR;
	uint32_t ETRCFGR;
	uint32_t GPIOACFGR;
	uint32_t GPIOBCFGR;
	uint32_t GPIOCCFGR;
	uint32_t GPIODCFGR;
	uint32_t GPIOECFGR;
	uint32_t GPIOFCFGR;
	uint32_t GPIOGCFGR;
	uint32_t GPIOHCFGR;
	uint32_t GPIOICFGR;
	uint32_t GPIOJCFGR;
	uint32_t GPIOKCFGR;
	uint32_t GPIOZCFGR;
	uint32_t HPDMA1CFGR;
	uint32_t HPDMA2CFGR;
	uint32_t HPDMA3CFGR;
	uint32_t LPDMACFGR;
	uint32_t HSEMCFGR;
	uint32_t IPCC1CFGR;
	uint32_t IPCC2CFGR;
	uint32_t RTCCFGR;
	uint32_t field_57c;
	uint32_t SYSCPU1CFGR;
	uint32_t BSECCFGR;
	uint32_t field_588;
	uint32_t field_58c;
	pllcfg_t PLL2;
	pllcfg_t PLL3;
	uint32_t HSIFMONCR;
	uint32_t HSIFVALR;
	uint32_t field_5e8[70];
	uint32_t TIM1CFGR;
	uint32_t TIM2CFGR;
	uint32_t TIM3CFGR;
	uint32_t TIM4CFGR;
	uint32_t TIM5CFGR;
	uint32_t TIM6CFGR;
	uint32_t TIM7CFGR;
	uint32_t TIM8CFGR;
	uint32_t TIM10CFGR;
	uint32_t TIM11CFGR;
	uint32_t TIM12CFGR;
	uint32_t TIM13CFGR;
	uint32_t TIM14CFGR;
	uint32_t TIM15CFGR;
	uint32_t TIM16CFGR;
	uint32_t TIM17CFGR;
	uint32_t TIM20CFGR;
	uint32_t LPTIM1CFGR;
	uint32_t LPTIM2CFGR;
	uint32_t LPTIM3CFGR;
	uint32_t LPTIM4CFGR;
	uint32_t LPTIM5CFGR;
	uint32_t SPI1CFGR;
	uint32_t SPI2FGR;
	uint32_t SPI3CFGR;
	uint32_t SPI4CFGR;
	uint32_t SPI5CFGR;
	uint32_t SPI6CFGR;
	uint32_t SPI7CFGR;
	uint32_t SPI8CFGR;
	uint32_t SPDIFRXCFGR;
	uint32_t USART1CFGR;
	uint32_t USART2CFGR;
	uint32_t USART3CFGR;
	uint32_t UART4CFGR;
	uint32_t UART5CFGR;
	uint32_t USART6CFGR;
	uint32_t UART7CFGR;
	uint32_t UART8CFGR;
	uint32_t UART9CFGR;
	uint32_t LPUART1CFGR;
	uint32_t I2C1CFGR;
	uint32_t I2C2CFGR;
	uint32_t I2C3CFGR;
	uint32_t I2C4CFGR;
	uint32_t I2C5CFGR;
	uint32_t I2C6CFGR;
	uint32_t I2C7CFGR;
	uint32_t I2C8CFGR;
	uint32_t SAI1CFGR;
	uint32_t SAI2CFGR;
	uint32_t SAI3CFGR;
	uint32_t SAI4CFGR;
	uint32_t field_7d4;
	uint32_t MDF1CFGR;
	uint32_t ADF1CFGR;
	uint32_t FDCANCFGR;
	uint32_t HDPCFGR;
	uint32_t ADC12CFGR;
	uint32_t ADC3CFGR;
	uint32_t ETH1CFGR;
	uint32_t ETH2CFGR;
	uint32_t field_7f8;
	uint32_t USBHCFGR;
	uint32_t USB2PHY1CFGR;
	uint32_t USB2PHY2CFGR;
	uint32_t USB3DRCFGR;
	uint32_t USB3PCIEPHYCFGR;
	uint32_t PCIECFGR;
	uint32_t UCPDCFGR;
	uint32_t ETHSWCFGR;
	uint32_t ETHSWACMCFGR;
	uint32_t ETHSWACMMSGCFGR;
	uint32_t STGENCFGR;
	uint32_t field_828;
	uint32_t field_82c;
	uint32_t SDMMC1CFGR;
	uint32_t SDMMC2CFGR;
	uint32_t SDMMC3CFGR;
	uint32_t GPUCFGR;
	uint32_t LTDCCFGR;
	uint32_t DSICFGR;
	uint32_t field_848;
	uint32_t field_84c;
	uint32_t LVDSCFGR;
	uint32_t field_854;
	uint32_t CSICFGR;
	uint32_t DCMIPPCFGR;
	uint32_t CCICFGR;
	uint32_t VDECCFGR;
	uint32_t VENCCFGR;
	uint32_t field_86c;
	uint32_t RNGCFGR;
	uint32_t PKACFGR;
	uint32_t SAESCFGR;
	uint32_t HASHCFGR;
	uint32_t CRYP1CFGR;
	uint32_t CRYP2CFGR;
	uint32_t IWDG1CFGR;
	uint32_t IWDG2CFGR;
	uint32_t IWDG3CFGR;
	uint32_t IWDG4CFGR;
	uint32_t IWDG5CFGR;
	uint32_t WWDG1CFGR;
	uint32_t WWDG2CFGR;
	uint32_t field_8a4;
	uint32_t VREFCFGR;
	uint32_t DTSCFGR;
	uint32_t field_8b0;
	uint32_t CRCCFGR;
	uint32_t SERCCFGR;
	uint32_t OSPIIOMCFGR;
	uint32_t GICV2MCFGR;
	uint32_t field_8c4;
	uint32_t I3C1CFGR;
	uint32_t I3C2CFGR;
	uint32_t I3C3CFGR;
	uint32_t I3C4CFGR;
	uint32_t field_8d8[458];
	uint32_t MUXSELCFGR;
	uint32_t field_1004[5];
	uint32_t XBARCFGR[64];
	uint32_t PREDIVCFGR[64];
	uint32_t PREDIVSR1;
	uint32_t PREDIVSR2;
	uint32_t field_1220;
	uint32_t FINDIVCFGR[64];
	uint32_t FINDIVSR1;
	uint32_t FINDIVSR2;
	uint32_t field_132c[5];
	uint32_t FCALOBS0CFGR;
	uint32_t FCALOBS1CFGR;
	uint32_t FCALREFCFGR;
	uint32_t FCALCCR1;
	uint32_t field_1350;
	uint32_t FCALCCR2;
	uint32_t FCALSR;
	uint32_t field_135c;
	pllcfg_t PLL4;
	pllcfg_t PLL5;
	pllcfg_t PLL6;
	pllcfg_t PLL7;
	pllcfg_t PLL8;
	uint32_t field_1428[15091];
	uint32_t VERR;
	uint32_t IDR;
	uint32_t SIDR;
};

#else

#error Undefined or unspecified RCC_T_VERSION

#endif	//version check

#endif	//include guard
