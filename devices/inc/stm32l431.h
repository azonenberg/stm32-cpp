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

#ifndef stm32l431_h
#define stm32l431_h

#include <stdint.h>

enum VoltageRange
{
	RANGE_VOS1 = 1,	//Vcore = 1.2V
	RANGE_VOS2 = 2	//Vcore = 1.0V
};

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t CR4;
	uint32_t SR1;
	uint32_t SR2;
	uint32_t SCR;
	uint32_t PUCRA;
	uint32_t PDCRA;
	uint32_t PUCRB;
	uint32_t PDCRB;
	uint32_t PUCRC;
	uint32_t PDCRC;
	uint32_t PUCRD;
	uint32_t PDCRD;
	uint32_t PUCRE;
	uint32_t PDCRE;
	uint32_t field_48;
	uint32_t field_4c;
	uint32_t field_50;
	uint32_t field_54;
	uint32_t PUCRH;
	uint32_t PDCRH;
} pwr_t;

enum pwr_cr1
{
	PWR_CR1_VOS		= 0x600,
	PWR_CR1_DBP		= 0x100
};

enum pwr_sr2
{
	PWR_SR2_VOSF	= 0x400
};

extern volatile pwr_t PWR;

enum rcc_ahb1
{
	RCC_AHB1_CRC	= 0x1000
};

enum rcc_ahb2
{
	RCC_AHB2_GPIOA	= 0x01,
	RCC_AHB2_GPIOB	= 0x02,
	RCC_AHB2_GPIOC	= 0x04,
	RCC_AHB2_GPIOD	= 0x08,
	RCC_AHB2_GPIOE	= 0x10,
	RCC_AHB2_GPIOH	= 0x80,
	RCC_AHB2_ADC	= 0x00002000,
	RCC_AHB2_RNG	= 0x00040000
};

enum rcc_ahb3
{
	RCC_AHB3_QSPI	= 0x100
};

enum rcc_apb2
{
	RCC_APB2_TIM16	= 0x00020000,
	RCC_APB2_TIM15	= 0x00010000,
	RCC_APB2_USART1	= 0x00004000,
	RCC_APB2_SPI1	= 0x00001000,
	RCC_APB2_TIM1	= 0x00000800,
	RCC_APB2_SYSCFG	= 0x00000001
};

enum rcc_apb1_en1
{
	RCC_APB1_1_PWR			= 0x10000000,
	RCC_APB1_1_I2C3			= 0x00800000,
	RCC_APB1_1_I2C2			= 0x00400000,
	RCC_APB1_1_I2C1			= 0x00200000,
	RCC_APB1_1_UART4		= 0x00080000,
	RCC_APB1_1_USART3		= 0x00040000,
	RCC_APB1_1_USART2		= 0x00020000,
	RCC_APB1_1_SPI3			= 0x00008000,
	RCC_APB1_1_SPI2			= 0x00004000,
	RCC_APB1_1_RTC			= 0x00000400,
	RCC_APB1_1_TIM7			= 0x00000020,
	RCC_APB1_1_TIM6			= 0x00000010,
	RCC_APB1_1_TIM3			= 0x00000002,
	RCC_APB1_1_TIM2			= 0x00000001
};

typedef struct
{
	uint32_t CR;
	uint32_t ICSCR;
	uint32_t CFGR;
	uint32_t PLLCFGR;
	uint32_t PLLSAI1CFGR;
	uint32_t field_14;
	uint32_t CIER;
	uint32_t CIFR;
	uint32_t CICR;
	uint32_t field_24;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
	uint32_t field_34;
	uint32_t APB1RSTR1;
	uint32_t APB1RSTR2;
	uint32_t APB2RSTR;
	uint32_t field_44;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
	uint32_t field_54;
	uint32_t APB1ENR1;
	uint32_t APB1ENR2;
	uint32_t APB2ENR;
	uint32_t field_64;
	uint32_t AHB1SMENR;
	uint32_t AHB2SMENR;
	uint32_t AHB3SMENR;
	uint32_t field_74;
	uint32_t APB1SMENR1;
	uint32_t APB1SMENR2;
	uint32_t APB2SMENR;
	uint32_t field_84;
	uint32_t CCIPR;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t CRRCR;
	uint32_t CCIPR2;
} rcc_t;

enum rcc_cr_bits
{
	RCC_PLL_READY	= 0x02000000,
	RCC_PLL_ON		= 0x01000000,
	RCC_HSI_READY	= 0x00000400,
	RCC_HSI_KERON	= 0x00000200,
	RCC_HSI_ON		= 0x00000100
};
extern volatile rcc_t RCC;

typedef struct
{
	uint32_t MEMRMP;
	uint32_t CFGR1;
	uint32_t EXTICR1;
	uint32_t EXTICR2;
	uint32_t EXTICR3;
	uint32_t EXTICR4;
	uint32_t SCSR;
	uint32_t CFGR2;
	uint32_t SWPR;
	uint32_t SKR;
} syscfg_t;

extern volatile syscfg_t SYSCFG;

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
	uint32_t	RCR;
	uint32_t	CCR1;
	uint32_t	CCR2;
	uint32_t	CCR3;
	uint32_t	CCR4;
	uint32_t	BDTR;
	uint32_t	DCR;
	uint32_t	DMAR;
	uint32_t	OR1;
	uint32_t	CCMR3;
	uint32_t	CCMR5;
	uint32_t	CCMR6;
	uint32_t	OR2;
	uint32_t	OR3;
} tim_t;

extern volatile tim_t TIM1;
extern volatile tim_t TIM2;
extern volatile tim_t TIM3;
extern volatile tim_t TIM6;
extern volatile tim_t TIM7;
extern volatile tim_t TIM15;
extern volatile tim_t TIM16;

typedef struct
{
	uint32_t	IDCODE;
	uint32_t	CR;
	uint32_t	APB1FZR1;
	uint32_t	APB1FZR2;
	uint32_t	APB2FZR;
} dbgmcu_t;

enum dbgmcu_cr_t
{
	DBGMCU_CR_TRACE_IOEN	= 0x20
};

extern volatile dbgmcu_t DBGMCU;

#define CRC_T_VERSION 1
#include "stm32-crc.h"

extern volatile crc_t _CRC;

#define GPIO_T_VERSION 2
#include "stm32-gpio.h"

extern volatile gpio_t GPIOA;
extern volatile gpio_t GPIOB;
extern volatile gpio_t GPIOC;
extern volatile gpio_t GPIOD;
extern volatile gpio_t GPIOE;
extern volatile gpio_t GPIOH;

#define USART_T_VERSION 1
#include "stm32-usart.h"

extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;
extern volatile usart_t UART4;

//doesn't have a full RTC, but backup/tamper registers are in the RTC subsystem
#define RTC_T_VERSION 2
#include "stm32-rtc.h"

extern volatile rtc_t _RTC;

#define SPI_T_VERSION 1
#include "stm32-spi.h"

extern volatile spi_t SPI1;
extern volatile spi_t SPI2;
extern volatile spi_t SPI3;

#define I2C_T_VERSION 1
#include "stm32-i2c.h"

extern volatile i2c_t I2C1;
extern volatile i2c_t I2C2;
extern volatile i2c_t I2C3;
//TODO: I2C4

#define FLASH_T_VERSION 2
#include "stm32-flash.h"

extern volatile flash_t FLASH;

#define EXTI_T_VERSION 1
#include "stm32-exti.h"
extern volatile exti_t _EXTI;

#define SCB_T_VERSION 2
#include "stm32-scb.h"
extern volatile scb_t SCB;

#include "stm32-itm.h"
extern volatile itm_t _ITM;

#include "stm32-dwt.h"
extern volatile dwt_t _DWT;

extern volatile uint32_t DEMCR;

#define QUADSPI_T_VERSION 1
#include "stm32-quadspi.h"
extern volatile quadspi_t QUADSPI;

#define RNG_T_VERSION 2
#include "stm32-rng.h"
extern volatile rng_t RNG;

#define ADC_T_VERSION 2
#include "stm32-adc.h"
extern volatile adc_t _ADC;
extern volatile uint16_t VREFINT_CAL;
extern volatile uint16_t TSENSE_CAL1;
extern volatile uint16_t TSENSE_CAL2;

extern volatile uint32_t U_ID[3];
extern volatile uint16_t FLASH_SIZE;
extern volatile uint16_t PKG;

//Defines for what peripherals are present / implemented
#define HAVE_TIM
#define HAVE_PWR
#define HAVE_FPU

#endif
