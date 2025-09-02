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

#ifndef stm32h750_h
#define stm32h750_h

#include <stdint.h>

/*
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
*/
enum VoltageRange
{
	RANGE_VOS3 = 1,	//0.95 - 1.26V
	RANGE_VOS2 = 2,	//1.05 - 1.26V
	RANGE_VOS1 = 3,	//1.15 - 1.26V
	RANGE_VOS0 = 0	//1.26 - 1.4V (PWR_D3CR suggests reserved and unavailable?? but seems rev V parts have it)
};
/*
enum pwr_csr1
{
	PWR_CSR1_ACTVOSRDY = 0x2000
};

enum pwr_d3cr
{
	PWR_D3CR_VOSRDY = 0x2000,

	PWR_D3CR_VOSMASK = 0xc000
};
*/
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
	RCC_AHB4_GPIOI		= 0x00000100,
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
	RCC_AHB3_FMC		= 0x00001000,
	RCC_AHB3_MDMA		= 0x00000001
};

enum rcc_ahb2
{
	RCC_AHB2_SRAM3		= 0x80000000,
	RCC_AHB2_SRAM2		= 0x40000000,
	RCC_AHB2_SRAM1		= 0x20000000,
	RCC_AHB2_RNG		= 0x00000040,
	RCC_AHB2_HASH		= 0x00000020,
	RCC_AHB2_CRYP		= 0x00000010
};

enum rcc_ahb1
{
	RCC_AHB1_DMA2		= 0x00000002,
	RCC_AHB1_DMA1		= 0x00000001
};

enum rcc_apb4
{
	RCC_APB4_RTC		= 0x00010000,
	RCC_APB4_I2C4		= 0x00000080,
	RCC_APB4_SPI6		= 0x00000020,
	RCC_APB4_SYSCFG		= 0x00000002,
};

enum rcc_apb2
{
	RCC_APB2_TIM1		= 0x00000001,
	RCC_APB2_TIM8		= 0x00000002,
	RCC_APB2_USART1		= 0x00000010,
	RCC_APB2_USART6		= 0x00000020,
	RCC_APB2_SPI1		= 0x00001000,
	RCC_APB2_SPI4		= 0x00002000,
	RCC_APB2_TIM15		= 0x00010000,
	RCC_APB2_TIM16		= 0x00020000,
	RCC_APB2_TIM17		= 0x00040000,
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
	RCC_APB1L_LPTIM1	= 0x00000200,
	RCC_APB1L_SPI2		= 0x00004000,
	RCC_APB1L_SPI3		= 0x00008000,
	RCC_APB1L_SPDIFRX	= 0x00010000,
	RCC_APB1L_USART2	= 0x00020000,
	RCC_APB1L_USART3	= 0x00040000,
	RCC_APB1L_UART4		= 0x00080000,
	RCC_APB1L_UART5		= 0x00100000,
	RCC_APB1L_I2C1		= 0x00200000,
	RCC_APB1L_I2C2		= 0x00400000,
	RCC_APB1L_I2C3		= 0x00800000,
	RCC_APB1L_UART7		= 0x40000000,
	RCC_APB1L_UART8		= 0x80000000
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
	RCC_CFGR_SW_PLL1	= 0x3,
	RCC_CFGR_SW_HSI		= 0x0
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

#define RCC_T_VERSION 1
#include "stm32-rcc.h"

extern volatile rcc_t RCC;

/*
typedef struct
{
	uint32_t reserved_0;
	uint32_t ictr;
	uint32_t reserved_8[62];
} nvic_t;

extern volatile nvic_t NVIC;
*/

/*
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
	uint32_t	padding_19c[10];
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

#define SYSCFG_T_VERSION 1
#include "stm32-syscfg.h"

extern volatile syscfg_t SYSCFG;

typedef struct
{
	uint32_t	IDCODE;
	uint32_t	CR;
} dbgmcu_t;

extern volatile dbgmcu_t DBGMCU;

#define TIM_T_VERSION 1
#include "stm32-tim.h"

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

#define CRYPT_T_VERSION 1
#include "stm32-crypt.h"
extern volatile cryp_t CRYP;

#define HASH_T_VERSION 1
#include "stm32-hash.h"
extern volatile hash_t HASH;

#define USART_T_VERSION 2
#include "stm32-usart.h"
extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;
extern volatile usart_t UART4;
extern volatile usart_t UART5;
extern volatile usart_t USART6;
//extern volatile usart_t UART7;
//extern volatile usart_t UART8;

#define CRC_T_VERSION 1
#include "stm32-crc.h"
extern volatile crc_t _CRC;

#define MDMA_T_VERSION 1
#include "stm32-mdma.h"
extern volatile mdma_t _MDMA;

#define FLASH_T_VERSION 4
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
extern volatile gpio_t GPIOI;
extern volatile gpio_t GPIOJ;
extern volatile gpio_t GPIOK;

#define I2C_T_VERSION 1
#define STM32H7
#include "stm32-i2c.h"
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

#define RTC_T_VERSION 2
#include "stm32-rtc.h"
extern volatile rtc_t _RTC;
/*
#define FMC_T_VERSION 1
#include "stm32-fmc.h"
extern volatile fmc_t _FMC;
*/
#define SCB_T_VERSION 2
#include "stm32-scb.h"
extern volatile scb_t SCB;

#include "stm32-itm.h"
extern volatile itm_t _ITM;

#include "stm32-dwt.h"
extern volatile dwt_t _DWT;

#include "stm32-tpiu.h"
extern volatile tpiu_t _TPIU;

#define RNG_T_VERSION 1
#include "stm32-rng.h"
extern volatile rng_t RNG;
/*
#define DMA_T_VERSION 1
#include "stm32-dma.h"
extern volatile dma_t DMA1;
extern volatile dma_t DMA2;
extern volatile dmamux_t DMAMUX1;

//Defines for what peripherals are present / implemented
//#define HAVE_EMAC
#define HAVE_HASH
#define HAVE_PWR
*/
#define HAVE_ITCM
#define HAVE_FPU
#define HAVE_L1

#endif
