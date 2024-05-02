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

#ifndef stm32h735_h
#define stm32h735_h

#include <stdint.h>

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

enum VoltageRange
{
	RANGE_VOS3 = 1,	//0.95 - 1.05V
	RANGE_VOS2 = 2,	//1.05 - 1.15V
	RANGE_VOS1 = 3,	//1.15 - 1.26V
	RANGE_VOS0 = 0	//1.26 - 1.4V
};

enum pwr_csr1
{
	PWR_CSR1_ACTVOSRDY = 0x2000
};

enum pwr_d3cr
{
	PWR_D3CR_VOSRDY = 0x2000,

	PWR_D3CR_VOSMASK = 0xc000
};

#define FLASH_T_VERSION 1
#include "stm32-flash.h"

extern volatile flash_t FLASH;

#define GPIO_T_VERSION 1
#include "stm32-gpio.h"

extern volatile gpio_t GPIOA;
extern volatile gpio_t GPIOB;
extern volatile gpio_t GPIOC;
extern volatile gpio_t GPIOD;
extern volatile gpio_t GPIOE;
extern volatile gpio_t GPIOF;
extern volatile gpio_t GPIOG;
extern volatile gpio_t GPIOH;
extern volatile gpio_t GPIOJ;
extern volatile gpio_t GPIOK;

enum rcc_ahb4
{
	RCC_AHB4_GPIOA		= 0x00000001,
	RCC_AHB4_GPIOB		= 0x00000002,
	RCC_AHB4_GPIOC		= 0x00000004,
	RCC_AHB4_GPIOD		= 0x00000008,
	RCC_AHB4_GPIOE		= 0x00000010,
	RCC_AHB4_GPIOF		= 0x00000020,
	RCC_AHB4_GPIOG		= 0x00000040,
	RCC_AHB4_GPIOH		= 0x00000080,
	//0x100 reserved
	RCC_AHB4_GPIOJ		= 0x00000200,
	RCC_AHB4_GPIOK		= 0x00000400,
	//0x0000_0800 to 0x0004_0000 reserved
	RCC_AHB4_CRC		= 0x00080000,
	//0x0010_0000 reserved
	RCC_AHB4_BDMA		= 0x00200000,
	//0x0040_0000 to 0x0080_0000 reserved
	RCC_AHB4_ADC3		= 0x01000000,
	RCC_AHB4_HSEM		= 0x02000000,
	//0x0400_0000 to 0x0800_0000 reserved
	RCC_AHB4_BKPRAM		= 0x10000000
	//0x2000_0000 to 0x8000_0000 reserved
};

enum rcc_ahb3
{
	RCC_AHB3_OCTOSPIM	= 0x00200000,
	RCC_AHB3_OCTOSPI2	= 0x00080000,
	RCC_AHB3_OCTOSPI1	= 0x00004000
};

enum rcc_ahb2
{
	RCC_AHB2_SRAM2		= 0x40000000,
	RCC_AHB2_SRAM1		= 0x20000000,
	RCC_AHB2_RNG		= 0x00000040,
	RCC_AHB2_HASH		= 0x00000020,
	RCC_AHB2_CRYP		= 0x00000010
};

enum rcc_apb4
{
	RCC_APB4_SYSCFG		= 0x00000002,
	RCC_APB4_SPI6		= 0x00000020,
	RCC_APB4_I2C4		= 0x00000080,
	RCC_APB4_RTC		= 0x00010000,
	RCC_APB4_DTS		= 0x04000000
};

enum rcc_apb2
{
	RCC_APB2_SPI1		= 0x00001000,
	RCC_APB2_SPI4		= 0x00002000,
	RCC_APB2_SPI5		= 0x00100000
};

enum rcc_apb1l
{
	RCC_APB1L_TIM2		= 0x00000001,
	RCC_APB1L_TIM3		= 0x00000002,
	RCC_APB1L_TIM4		= 0x00000004,
	RCC_APB1L_TIM5		= 0x00000008,
	RCC_APB1L_TIM6		= 0x00000010,
	RCC_APB1L_TIM7		= 0x00000020,
	RCC_APB1L_TIM12		= 0x00000040,
	RCC_APB1L_TIM13		= 0x00000080,
	RCC_APB1L_TIM14		= 0x00000100,
	RCC_APB1L_SPI2		= 0x00004000,
	RCC_APB1L_SPI3		= 0x00008000,
	RCC_APB1L_USART2	= 0x00020000,
	RCC_APB1L_USART3	= 0x00040000,
	RCC_APB1L_UART4		= 0x00080000,
	RCC_APB1L_UART5		= 0x00100000,
	RCC_APB1L_I2C1		= 0x00200000,
	RCC_APB1L_I2C2		= 0x00400000,
	RCC_APB1L_I2C3		= 0x00800000,
	//I2C 4 is on APB4, not here
	//(corresponding bit is reserved)
	RCC_APB1L_I2C5		= 0x02000000
};

enum rcc_cr
{
	RCC_CR_HSION		= 0x00000001,
	RCC_CR_HSIRDY		= 0x00000004,
	RCC_CR_HSIDIVMASK	= 0x0000000c,

	RCC_CR_HSEON 		= 0x00010000,
	RCC_CR_HSERDY		= 0x00020000,
	RCC_CR_HSEBYP		= 0x00040000,

	RCC_CR_PLL3RDY		= 0x20000000,
	RCC_CR_PLL3ON		= 0x10000000,
	RCC_CR_PLL2RDY		= 0x08000000,
	RCC_CR_PLL2ON		= 0x04000000,
	RCC_CR_PLL1RDY		= 0x02000000,
	RCC_CR_PLL1ON		= 0x01000000
};

enum rcc_cfgr
{
	RCC_CFGR_SW_MASK	= 0xfffffff8,
	RCC_CFGR_SW_PLL1	= 0x3
};

enum rcc_pllckselr
{
	RCC_PLLCKSELR_SRC_MASK	= 0xfffffffc,
	RCC_PLLCKSELR_SRC_HSE	= 0x00000002,
	RCC_PLLCKSELR_SRC_HSI	= 0x00000000,
};

enum rcc_pllcfgr
{
	RCC_PLLCFGR_DIV3REN			= 0x01000000,
	RCC_PLLCFGR_DIV3QEN			= 0x00800000,
	RCC_PLLCFGR_DIV3PEN			= 0x00400000,
	RCC_PLLCFGR_DIV2REN			= 0x00200000,
	RCC_PLLCFGR_DIV2QEN			= 0x00100000,
	RCC_PLLCFGR_DIV2PEN			= 0x00080000,
	RCC_PLLCFGR_DIV1REN			= 0x00040000,
	RCC_PLLCFGR_DIV1QEN			= 0x00020000,
	RCC_PLLCFGR_DIV1PEN			= 0x00010000,

	RCC_PLLCFGR_PLL3RGE_MASK	= 0x00000c00,
	RCC_PLLCFGR_PLL2RGE_MASK	= 0x000000c0,
	RCC_PLLCFGR_PLL1RGE_MASK	= 0x0000000c
};

typedef struct
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
} rcc_t;

extern volatile rcc_t RCC;

#define USART_T_VERSION 2
#include "stm32-usart.h"

//extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;
extern volatile usart_t UART4;
extern volatile usart_t UART5;
/*
extern volatile usart_t USART6;
extern volatile usart_t UART7;
extern volatile usart_t UART8;
*/

enum i2c_cr2_bits
{
	I2C_AUTO_END	= 0x02000000,
	I2C_STOP		= 0x00004000,
	I2C_START		= 0x00002000,
	I2C_READ		= 0x00000400
};

enum i2c_isr_bits
{
	I2C_DIR_READ			= 0x10000,
	I2C_BUSY				= 0x08000,
	I2C_TRANSFER_COMPLETE	= 0x00040,
	I2C_NACK 				= 0x00010,
	I2C_ADDR				= 0x00008,
	I2C_RX_READY			= 0x00004,
	I2C_TX_EMPTY			= 0x00001
};


typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t OAR1;
	uint32_t OAR2;
	uint32_t TIMINGR;
	uint32_t TIMEOUTR;
	uint32_t ISR;
	uint32_t ICR;
	uint32_t PECR;
	uint32_t RXDR;
	uint32_t TXDR;
} i2c_t;

extern volatile i2c_t I2C5;
extern volatile i2c_t I2C4;
extern volatile i2c_t I2C3;
extern volatile i2c_t I2C2;
extern volatile i2c_t I2C1;

#define SPI_T_VERSION 2
#include "stm32-spi.h"

extern volatile spi_t SPI1;
extern volatile spi_t SPI2;
extern volatile spi_t SPI3;
extern volatile spi_t SPI4;
extern volatile spi_t SPI5;
extern volatile spi_t SPI6;

#define RTC_T_VERSION 1
#include "stm32-rtc.h"

extern volatile rtc_t _RTC;

/*
typedef struct
{
	uint32_t reserved_0;
	uint32_t ictr;
	uint32_t reserved_8[62];
} nvic_t;

extern volatile nvic_t NVIC;

typedef struct
{
	uint32_t	MACCR;
	uint32_t	MACFFR;
	uint32_t	MACHTHR;
	uint32_t	MACHTLR;
	uint32_t	MACMIIAR;
	uint32_t	MACMIIDR;
	uint32_t	MACFCR;
	uint32_t	MACVLANTR;
	uint32_t	field_20;
	uint32_t	field_24;
	uint32_t	MACRWUFFR;
	uint32_t	MACPMTCSR;
	uint32_t	field_30;
	uint32_t	MACDBGR;
	uint32_t	MACSR;
	uint32_t	MACIMR;
	uint32_t	MACA0HR;
	uint32_t	MACA0LR;
	uint32_t	MACA1HR;
	uint32_t	MACA1LR;
	uint32_t	MACA2HR;
	uint32_t	MACA2LR;
	uint32_t	MACA3HR;
	uint32_t	MACA3LR;
	uint32_t	padding_60[40];
	uint32_t	MMCCR;				//offset 0x100
	uint32_t	MMCRIR;
	uint32_t	MMCTIR;
	uint32_t	MMCRIMR;
	uint32_t	MMCTIMR;
	uint32_t	padding_114[15];
	uint32_t	MMCTGFSCCR;
	uint32_t	MMCTGFMSCCR;
	uint32_t	padding_154[4];
	uint32_t	MMCTGFCR;
	uint32_t	padding_16c[10];
	uint32_t	MMCRFCECR;
	uint32_t	MMCRFAECR;
	uint32_t	padding_1
enum i2c_cr2_bits
{
	I2C_AUTO_END	= 0x02000000,
	I2C_STOP		= 0x00004000,
	I2C_START		= 0x00002000,
	I2C_READ		= 0x00000400
};

enum i2c_isr_bits
{
	I2C_BUSY				= 0x8000,
	I2C_TRANSFER_COMPLETE	= 0x0040,
	I2C_RX_READY			= 0x0004,
	I2C_TX_EMPTY			= 0x0001
};
9c[10];
	uint32_t	MMCRGUFCR;
} emac_t;

typedef struct
{
	uint32_t	PTPTSCR;
	uint32_t	PTPSSIR;
	uint32_t	PTPTSHR;
	uint32_t	PTPTSLR;
	uint32_t	PTPTSHUR;
	uint32_t	PTPTSLUR;
	uint32_t	PTPTSAR;
	uint32_t	PTPTTHR;
	uint32_t	PTPTTLR;
	uint32_t	field_724;
	uint32_t	PTPTSSR;
	uint32_t	PTPPPSCR;
} ptp_t;

typedef struct
{
	uint32_t RDES0;
	uint32_t RDES1;
	uint8_t* RDES2;
	uint8_t* RDES3;
} edma_rx_descriptor_t;

typedef struct
{
	uint32_t TDES0;
	uint32_t TDES1;
	uint8_t* TDES2;
	uint8_t* TDES3;
} edma_tx_descriptor_t;

typedef struct
{
	uint32_t						DMABMR;
	uint32_t						DMATPDR;
	uint32_t						DMARPDR;
	volatile edma_rx_descriptor_t*	DMARDLAR;
	volatile edma_tx_descriptor_t*	DMATDLAR;
	uint32_t						DMASR;
	uint32_t						DMAOMR;
	uint32_t						DMAIER;
	uint32_t						DMAMFBOCR;
	uint32_t						DMARSWTR;
	uint32_t						padding_1028[8];
	volatile edma_tx_descriptor_t*	DMACHTDR;
	volatile edma_rx_descriptor_t*	DMACHRDR;
	uint32_t						DMACHTBAR;
	uint32_t						DMACHRBAR;
} edma_t;

typedef enum
{
	DMA_FATAL_BUS_ERROR				= 0x00002000,
	DMA_ERROR_TXDMA					= 0x00800000,
	DMA_ERROR_RXDMA					= 0x01000000,
	DMA_ERROR_DESCRIPTOR			= 0x02000000
} dmasr_t;

extern volatile emac_t EMAC;
extern volatile ptp_t PTP;
extern volatile edma_t EDMA;
*/
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
	uint32_t field_2c;
	uint32_t ADC2ALT;
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
	uint32_t field_320;
	uint32_t field_324;
	uint32_t field_328;
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

extern volatile syscfg_t SYSCFG;

typedef struct
{
	uint32_t	IDCODE;
	uint32_t	CR;
	uint32_t	APB1_FZ;
	uint32_t	APB2_FZ;
} dbgmcu_t;

extern volatile dbgmcu_t DBGMCU;

typedef struct
{
	uint32_t	CR1;
	uint32_t	CR2;
	uint32_t	SMCR;
	uint32_t	DIER;
	uint32_t	SR;
	uint32_t	EGR;
	uint32_t	CCMR1;
	uint32_t	CCMR2;
	uint32_t	CCER;
	uint32_t	CNT;
	uint32_t	PSC;
	uint32_t	ARR;
	uint32_t	field_30;
	uint32_t	CCR1;
	uint32_t	CCR2;
	uint32_t	CCR3;
	uint32_t	CCR4;
	uint32_t	field_44;
	uint32_t	DCR;
	uint32_t	DMAR;
	uint32_t	field_50;
	uint32_t	field_54;
	uint32_t	field_58;
	uint32_t	field_5c;
	uint32_t	AF1;
	uint32_t	field_64;
	uint32_t	TISEL;
} tim_t;

//extern volatile tim_t TIM1;
extern volatile tim_t TIM2;
extern volatile tim_t TIM3;
extern volatile tim_t TIM4;
extern volatile tim_t TIM5;
extern volatile tim_t TIM6;
extern volatile tim_t TIM7;
//extern volatile tim_t TIM8;
//extern volatile tim_t TIM9;
//extern volatile tim_t TIM10;
//extern volatile tim_t TIM11;
extern volatile tim_t TIM12;
extern volatile tim_t TIM13;
extern volatile tim_t TIM14;

extern volatile uint32_t U_ID[3];
extern volatile uint16_t F_ID;
extern volatile uint32_t L_ID;
extern volatile uint16_t PKG_ID;

typedef struct
{
	uint32_t	CR;
	uint32_t	SR;
	uint32_t	DIN;
	uint32_t	DOUT;
	uint32_t	DMACR;
	uint32_t	IMSCR;
	uint32_t	RISR;
	uint32_t	MISR;
	uint32_t	K0LR;
	uint32_t	K0RR;
	uint32_t	K1LR;
	uint32_t	K1RR;
	uint32_t	K2LR;
	uint32_t	K2RR;
	uint32_t	K3LR;
	uint32_t	K3RR;
	uint32_t	IV0LR;
	uint32_t	IV0RR;
	uint32_t	IV1LR;
	uint32_t	IV1RR;
	uint32_t	CSGCMCCM0R;
	uint32_t	CSGCMCCM1R;
	uint32_t	CSGCMCCM2R;
	uint32_t	CSGCMCCM3R;
	uint32_t	CSGCMCCM4R;
	uint32_t	CSGCMCCM5R;
	uint32_t	CSGCMCCM6R;
	uint32_t	CSGCMCCM7R;
	uint32_t	CSGCM0R;
	uint32_t	CSGCM1R;
	uint32_t	CSGCM2R;
	uint32_t	CSGCM3R;
	uint32_t	CSGCM4R;
	uint32_t	CSGCM5R;
	uint32_t	CSGCM6R;
	uint32_t	CSGCM7R;
} cryp_t;

enum cryp_cr
{
	CRYP_ALG_AES_GCM	= 0x80000,
	CRYP_EN				= 0x8000,
	CRYP_GCM_PHASE_INIT	= 0x0,
	CRYP_GCM_PHASE_AAD	= 0x10000,
	CRYP_GCM_PHASE_DATA	= 0x20000,
	CRYP_GCM_PHASE_TAG	= 0x30000,
	CRYP_GCM_PHASE_MASK	= 0x30000,
	CRYP_BSWAP_BYTE		= 0x80,
	CRYP_DECRYPT		= 0x0004,
	CRYP_KEY_128		= 0x0,

	CRYP_FFLUSH			= 0x4000
};

enum cryp_sr
{
	CRYP_BUSY = 0x10,
	CRYP_OFNE = 0x4
};

extern volatile cryp_t CRYP;

typedef struct
{
	uint32_t	CR;
	uint32_t	SR;
	uint32_t	DR;
	uint32_t	HTCR;
} rng_t;

enum rng_cr
{
	RNG_CONDRST	= 0x40000000,

	RNG_CED		= 0x20,
	RNG_IE		= 0x08,
	RNG_EN		= 0x04
};

enum rng_sr
{
	RNG_SECS	= 0x4,
	RNG_CECS	= 0x2,
	RNG_DRDY	= 0x1
};

extern volatile rng_t RNG;

typedef struct
{
	uint32_t	CR;
	uint32_t	DIN;
	uint32_t	STR;
	uint32_t	HR_low[5];	//duplicates of HR, for backward compat with older IP versions probably?
	uint32_t	IMR;
	uint32_t	SR;
	uint32_t	padding_28[52];
	uint32_t	CSR[54];
	uint32_t	padding_1d0[80];
	uint32_t	HR[8];
} hash_t;

enum hash_sr
{
	HASH_BUSY	= 0x8
};

extern volatile hash_t HASH;

#include "stm32-scb.h"
extern volatile scb_t SCB;

typedef struct
{
	uint32_t	CLIDR;
	uint32_t	CTR;
	uint32_t	CCSIDR;
	uint32_t	CCSELR;
} cpuid_t;

extern volatile cpuid_t CPUID;

typedef struct
{
	uint32_t		CR;
	uint32_t		field_4;
	uint32_t		DCR1;
	uint32_t		DCR2;
	uint32_t		DCR3;
	uint32_t		DCR4;
	uint32_t		field_18[2];
	uint32_t		SR;
	uint32_t		FCR;
	uint32_t		field_28[6];
	uint32_t		DLR;
	uint32_t		field_44;
	uint32_t		AR;
	uint32_t		field_4c;
	uint32_t		DR;
	uint32_t		field_54[11];
	uint32_t		PSMKR;
	uint32_t		field_84;
	uint32_t		PSMAR;
	uint32_t		field_8c;
	uint32_t		PIR;
	uint32_t		field_94[27];
	uint32_t		CCR;
	uint32_t		field_104;
	uint32_t		TCR;
	uint32_t		field_10c;
	uint32_t		IR;
	uint32_t		field_114[3];
	uint32_t		ABR;
	uint32_t		field_124[3];
	uint32_t		LPTR;
	uint32_t		field_134[3];
	uint32_t		WPCCR;
	uint32_t		field_144;
	uint32_t		WPTCR;
	uint32_t		field_14c;
	uint32_t		WPIR;
	uint32_t		field_154[3];
	uint32_t		WPABR;
	uint32_t		field_164[3];
	uint32_t		WCCR;
	uint32_t		field_184;
	uint32_t		WTCR;
	uint32_t		field_18c;
	uint32_t		WIR;
	uint32_t		field_194[3];
	uint32_t		WABR;
	uint32_t		field_1a4[23];
	uint32_t		HLCR;
} octospi_t;

extern volatile octospi_t OCTOSPI1;
extern volatile octospi_t OCTOSPI2;

enum octospi_cr
{
	OCTOSPI_FMODE_MASK				= 0x30000000,
	OCTOSPI_FMODE_INDIRECT_WRITE	= 0x00000000,
	OCTOSPI_FMODE_INDIRECT_READ		= 0x10000000,
	OCTOSPI_FMODE_AUTO_POLL			= 0x20000000,
	OCTOSPI_FMODE_MMAP				= 0x30000000,

	OCTOSPI_FTHRES_MASK				= 0x00001f00,

	OCTOSPI_TCEN					= 0x00000008,
	OCTOSPI_DMAEN					= 0x00000004,
	OCTOSPI_ABORT					= 0x00000002,
	OCTOSPI_EN						= 0x00000001
};

enum octospi_dcr1
{
	OCTOSPI_MEM_TYPE_STANDARD		= 0x02000000,

	OCTOSPI_CSHT_MASK				= 0x00003f00
};

enum octospi_sr
{
	OCTOSPI_BUSY					= 0x00000020,
	OCTOSPI_TOF						= 0x00000010,
	OCTOSPI_SMF						= 0x00000008,
	OCTOSPI_FTF						= 0x00000004,
	OCTOSPI_TCF						= 0x00000002,
	OCTOSPI_TEF						= 0x00000001
};

enum octospi_ccr
{
	OCTOSPI_SIOO					= 0x80000000,
	OCTOSPI_DQSE					= 0x20000000,
	OCTOSPI_DDTR					= 0x08000000,
	OCTOSPI_DMODE_MASK				= 0x07000000,
	OCTOSPI_ABSIZE_MASK				= 0x00300000,
	OCTOSPI_ABDTR					= 0x00080000,
	OCTOSPI_ABMODE_MASK				= 0x00070000,
	OCTOSPI_ADSIZE_MASK				= 0x00003000,
	OCTOSPI_ADDTR					= 0x00000800,
	OCTOSPI_ADMODE_MASK				= 0x00000700,
	OCTOSPI_ISIZE_MASK				= 0x00000030,
	OCTOSPI_IDTR					= 0x00000008,
	OCTOSPI_IMODE_MASK				= 0x00000007
};

enum octospi_tcr
{
	OCTOSPI_SSHIFT					= 0x40000000,
	OCTOSPI_DCYC_MASK				= 0x0000001f
};

typedef struct
{
	uint32_t		CR;
	uint32_t		P1CR;
	uint32_t		P2CR;
} octospim_t;

extern volatile octospim_t OCTOSPIM;

typedef struct
{
	uint32_t CFGR1;
	uint32_t field_4;
	uint32_t T0VALR1;
	uint32_t field_c;
	uint32_t RAMPVALR;
	uint32_t ITR1;
	uint32_t field_18;
	uint32_t DR;
	uint32_t SR;
	uint32_t ITENR;
	uint32_t ICIFR;
	uint32_t OR;
} dts_t;

extern volatile dts_t DTS;

//Defines for what peripherals are present / implemented
#define HAVE_I2C
#define HAVE_DTS
#define HAVE_TIM
//#define HAVE_EMAC
#define HAVE_CRYP
#define HAVE_RNG
#define HAVE_HASH
#define HAVE_PWR
#define HAVE_OCTOSPI

#endif
