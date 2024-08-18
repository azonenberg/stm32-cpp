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

#ifndef stm32l031_h
#define stm32l031_h

#include <stdint.h>

enum VoltageRange
{
	RANGE_VOS1 = 1,	//Vcore = 1.8V
	RANGE_VOS2 = 2,	//Vcore = 1.5V
	RANGE_VOS3 = 3	//Vcore = 1.2V
};

typedef struct
{
	uint32_t CR;
	uint32_t CSR;
} pwr_t;

enum pwr_cr
{
	PWR_CR_VOS		= 0x1800
};

enum pwr_csr
{
	PWR_CSR_VOSF	= 0x10
};

extern volatile pwr_t PWR;

#define GPIO_T_VERSION 1
#include "stm32-gpio.h"

extern volatile gpio_t GPIOA;
extern volatile gpio_t GPIOB;
extern volatile gpio_t GPIOC;
extern volatile gpio_t GPIOD;
extern volatile gpio_t GPIOE;
extern volatile gpio_t GPIOH;

enum rcc_io
{
	RCC_IO_GPIOA	= 0x01,
	RCC_IO_GPIOB	= 0x02,
	RCC_IO_GPIOC	= 0x04,
	RCC_IO_GPIOD	= 0x08,
	RCC_IO_GPIOE	= 0x10,
	RCC_IO_GPIOH	= 0x80,
};

enum rcc_ahb
{
	RCC_AHB_CRC		= 0x00001000
};

enum rcc_apb2
{
	RCC_APB2_SPI1	= 0x00001000,
	RCC_APB2_ADC	= 0x00000200,
	RCC_APB2_TIM22	= 0x00000020,
	RCC_APB2_TIM21	= 0x00000004,
	RCC_APB2_SYSCFG = 0x00000001
};

enum rcc_apb1
{
	RCC_APB1_PWR		= 0x10000000,
	RCC_APB1_I2C1		= 0x00200000,
	RCC_APB1_USART5		= 0x00100000,
	RCC_APB1_USART4		= 0x00080000,
	RCC_APB1_LPUART1	= 0x00040000,
	RCC_APB1_USART2		= 0x00020000,
	RCC_APB1_TIM7		= 0x00000020,
	RCC_APB1_TIM6		= 0x00000010,
	RCC_APB1_TIM3		= 0x00000002,
	RCC_APB1_TIM2		= 0x00000001
};

typedef struct
{
	uint32_t CR;
	uint32_t ICSCR;
	uint32_t field_8;
	uint32_t CFGR;
	uint32_t CIER;
	uint32_t CIFR;
	uint32_t CICR;
	uint32_t IOPRSTR;
	uint32_t AHBRSTR;
	uint32_t APB2RSTR;
	uint32_t APB1RSTR;
	uint32_t IOPENR;
	uint32_t AHBENR;
	uint32_t APB2ENR;
	uint32_t APB1ENR;
	uint32_t IOPSMEN;
	uint32_t AHBSMENR;
	uint32_t APB2SMENR;
	uint32_t APB1SMENR;
	uint32_t CCIPR;
	uint32_t CSR;
} rcc_t;

enum rcc_cr_bits
{
	RCC_PLL_READY	= 0x02000000,
	RCC_PLL_ON		= 0x01000000,

	RCC_HSI_READY	= 0x00000004,
	RCC_HSI_KERON	= 0x00000002,
	RCC_HSI_ON		= 0x00000001
};

extern volatile rcc_t RCC;

#define USART_T_VERSION 1
#include "stm32-usart.h"

extern volatile usart_t USART2;
extern volatile usart_t USART4;
extern volatile usart_t USART5;

typedef struct
{
	uint32_t CFGR1;
	uint32_t CFGR2;
	uint32_t EXTICR1;
	uint32_t EXTICR2;
	uint32_t EXTICR3;
	uint32_t EXTICR4;
	uint32_t COMP1_CTRL;
	uint32_t COMP2_CTRL;
	uint32_t CFGR3;
} syscfg_t;

extern volatile syscfg_t SYSCFG;

#define SPI_T_VERSION 3
#include "stm32-spi.h"

extern volatile spi_t SPI1;

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
	uint32_t	OR;
} tim_t;

extern volatile tim_t TIMER2;
extern volatile tim_t TIMER3;
extern volatile tim_t TIMER6;
extern volatile tim_t TIMER7;
extern volatile tim_t TIMER21;
extern volatile tim_t TIMER22;

#define I2C_T_VERSION 1
#include "stm32-i2c.h"
extern volatile i2c_t I2C1;

#define FLASH_T_VERSION 3
#include "stm32-flash.h"
extern volatile flash_t FLASH;

#define ADC_T_VERSION 1
#include "stm32-adc.h"
extern volatile adc_t ADC1;

#define CRC_T_VERSION 1
#include "stm32-crc.h"
extern volatile crc_t _CRC;

#define SCB_T_VERSION 1
#include "stm32-scb.h"
extern volatile scb_t SCB;

#define EXTI_T_VERSION 2
#include "stm32-exti.h"
extern volatile exti_t _EXTI;

extern volatile uint16_t VREFINT_CAL;
extern volatile uint16_t TSENSE_CAL1;
extern volatile uint16_t TSENSE_CAL2;

typedef struct
{
	uint32_t	IDCODE;
	uint32_t	CR;
	uint32_t	APB1_FZ;
	uint32_t	APB2_FZ;
} dbgmcu_t;

extern volatile dbgmcu_t DBGMCU;

extern volatile uint32_t U_ID[3];
extern volatile uint16_t F_ID;

//Defines for what peripherals are present / implemented
#define HAVE_TIM
#define HAVE_PWR

#endif
