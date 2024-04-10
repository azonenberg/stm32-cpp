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
	PWR_CR1_VOS		= 0x600
};

enum pwr_sr2
{
	PWR_SR2_VOSF	= 0x400
};

extern volatile pwr_t PWR;

#define GPIO_T_VERSION 2
#include "stm32-gpio.h"

extern volatile gpio_t GPIOA;
extern volatile gpio_t GPIOB;
extern volatile gpio_t GPIOC;
extern volatile gpio_t GPIOD;
extern volatile gpio_t GPIOE;
extern volatile gpio_t GPIOH;

enum rcc_ahb2
{
	RCC_AHB2_GPIOA	= 0x01,
	RCC_AHB2_GPIOB	= 0x02,
	RCC_AHB2_GPIOC	= 0x04,
	RCC_AHB2_GPIOD	= 0x08,
	RCC_AHB2_GPIOE	= 0x10,
	RCC_AHB2_GPIOH	= 0x80
};

enum rcc_apb2
{
	RCC_APB2_TIM16	= 0x00020000,
	RCC_APB2_TIM15	= 0x00010000,
	RCC_APB2_USART1	= 0x00004000,
	RCC_APB2_SPI1	= 0x00001000,
	RCC_APB2_TIM1	= 0x00000800
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

#define USART_T_VERSION 1
#include "stm32-usart.h"

extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;
extern volatile usart_t UART4;
/*
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
*/
typedef struct
{
	uint32_t	CR1;
	uint32_t	CR2;
	uint32_t	SR;
	uint8_t		DR;			//STM32f0x1 datasheet page 807, 28.9.4 says the register is 16-bits wide, and that
							//"Unused bits are ignored when writing to the register". This is untrue.
							//If you access DR as a 16-bit write, you get *two* bytes of data sent.
	uint8_t		padding1;
	uint16_t	padding2;
	uint32_t	CRCPR;
	uint32_t	RXCRCR;
	uint32_t	TXCRCR;
} spi_t;

enum spi_cr1_bits
{
	SPI_BIDI_MODE	= 0x8000,
	SPI_BIDI_OE		= 0x4000,
	SPI_RX_ONLY		= 0x0400,
	SPI_SOFT_CS		= 0x0200,
	SPI_INTERNAL_CS	= 0x0100,
	SPI_LSB_FIRST	= 0x0080,
	SPI_ENABLE		= 0x0040,
	SPI_MASTER		= 0x0004,
	SPI_CPOL		= 0x0002
};

enum spi_cr2_bits
{
	SPI_FRXTH		= 0x1000
};

enum spi_sr_bits
{
	SPI_TX_FIFO_MASK	= 0x1800,
	SPI_RX_FIFO_MASK	= 0x0600,
	SPI_BUSY			= 0x0080,
	SPI_TX_EMPTY		= 0x0002,
	SPI_RX_NOT_EMPTY	= 0x0001,
};

extern volatile spi_t SPI1;
extern volatile spi_t SPI2;
extern volatile spi_t SPI3;

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
	uint32_t	CR1;
	uint32_t	CR2;
	uint32_t	OAR1;
	uint32_t	OAR2;
	uint32_t	TIMINGR;
	uint32_t	TIMEOUTR;
	uint32_t	ISR;
	uint32_t	ICR;
	uint32_t	PECR;
	uint32_t	RXDR;
	uint32_t	TXDR;
} i2c_t;

extern volatile i2c_t I2C1;
extern volatile i2c_t I2C2;
extern volatile i2c_t I2C3;
//TODO: I2C4

typedef struct
{
	uint32_t ACR;
	uint32_t PDKEYR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t ECR;
	uint32_t OPTR;
	uint32_t PCROP1SR;
	uint32_t PCROP1ER;
	uint32_t WRP1AR;
	uint32_t WRP1BR;
} flash_t;

extern volatile flash_t FLASH;

enum flash_acr
{
	FLASH_ACR_DCEN			= 0x400,
	FLASH_ACR_ICEN 			= 0x200,
	FLASH_ACR_PREFETCHEN	= 0x100,
};

enum flash_cr
{
	FLASH_CR_LOCK			= 0x80000000,
	FLASH_CR_STRT			= 0x10000,

	FLASH_CR_PSIZE_MASK		= 0x300,
	FLASH_CR_PSIZE_X8		= 0x000,
	FLASH_CR_PSIZE_X16		= 0x100,
	FLASH_CR_PSIZE_X32		= 0x200,
	FLASH_CR_PSIZE_X64		= 0x300,

	FLASH_CR_SECTOR_MASK	= 0xf8,

	FLASH_CR_PER			= 0x2,
	FLASH_CR_PG				= 0x1
};

enum flash_sr
{
	FLASH_SR_BUSY			= 0x10000,

	FLASH_SR_ERR_MASK		= 0xf2
};
/*
typedef struct
{
	uint32_t	ISR;
	uint32_t	IER;
	uint32_t	CR;
	uint32_t	CFGR1;
	uint32_t	CFGR2;
	uint32_t	SMPR;
	uint32_t	field_18;
	uint32_t	field_1c;
	uint32_t	TR;
	uint32_t	field_24;
	uint32_t	CHSELR;
	uint32_t	field_2c;
	uint32_t	field_30;
	uint32_t	field_34;
	uint32_t	field_38;
	uint32_t	field_3c;
	uint32_t	DR;
	uint32_t	field_44[28];
	uint32_t	CALFACT;
	uint32_t	field_b8[148];
	uint32_t	CCR;
} adc_t;

enum adc_cr
{
	ADC_CR_ADCAL	= 0x80000000,
	ADC_CR_ADSTART	= 0x00000004,
	ADC_CR_ADEN		= 0x00000001
};

enum adc_ccr
{
	ADC_CCR_TSEN	= 0x00800000,
	ADC_CCR_VREFEN	= 0x00400000
};

enum adc_isr
{
	ADC_ISR_ADRDY	= 0x00000001
};

extern volatile adc_t ADC1;

extern volatile uint16_t VREFINT_CAL;
extern volatile uint16_t TSENSE_CAL1;
extern volatile uint16_t TSENSE_CAL2;
*/
typedef struct
{
	uint32_t	IDCODE;
	uint32_t	CR;
	uint32_t	APB1FZR1;
	uint32_t	APB1FZR2;
	uint32_t	APB2FZR;
} dbgmcu_t;

extern volatile dbgmcu_t DBGMCU;

extern volatile uint32_t U_ID[3];
extern volatile uint16_t FLASH_SIZE;
extern volatile uint16_t PKG;

//Defines for what peripherals are present / implemented
#define HAVE_I2C
#define HAVE_TIM
#define HAVE_SPI
#define HAVE_UART
//#define HAVE_ADC
#define HAVE_PWR

#endif
