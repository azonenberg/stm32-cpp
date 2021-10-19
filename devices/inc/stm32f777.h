/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2021 Andrew D. Zonenberg                                                                          *
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

#ifndef stm32f777_h
#define stm32f777_h

#include <stdint.h>

typedef struct
{
	uint32_t ACR;
	uint32_t KEYR;
	uint32_t OPTKEYR;
	uint32_t SR;
	uint32_t CR;
	uint32_t OPTCR;
	uint32_t OPTCR1;
} flash_t;

extern volatile flash_t FLASH;

enum flash_acr
{
	FLASH_ACR_ARTEN = 0x20,
	FLASH_ACR_PREFETCHEN = 0x10,
};

typedef struct
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t LCKR;
	uint32_t AFRL;
	uint32_t AFRH;
} gpio_t;

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

enum rcc_ahb1
{
	RCC_AHB1_GPIOA		= 0x00000001,
	RCC_AHB1_GPIOB		= 0x00000002,
	RCC_AHB1_GPIOC		= 0x00000004,
	RCC_AHB1_GPIOD		= 0x00000008,
	RCC_AHB1_GPIOE		= 0x00000010,
	RCC_AHB1_GPIOF		= 0x00000020,
	RCC_AHB1_GPIOG		= 0x00000040,
	RCC_AHB1_GPIOH		= 0x00000080,
	RCC_AHB1_GPIOI		= 0x00000100,
	RCC_AHB1_GPIOJ		= 0x00000200,
	RCC_AHB1_GPIOK		= 0x00000400,
	//0x800 reserved
	RCC_AHB1_CRC		= 0x00001000,
	//0x00002000 to 0x00020000 reserved
	RCC_AHB1_BBRAM		= 0x00040000,
	//0x00080000 reserved
	RCC_AHB1_DTCM		= 0x00100000,
	RCC_AHB1_DMA1		= 0x00200000,
	RCC_AHB1_DMA2		= 0x00400000,
	RCC_AHB1_DMA2D		= 0x00800000,
	//0x01000000 reserved
	RCC_AHB1_EMAC		= 0x02000000,
	RCC_AHB1_EMAC_TX	= 0x04000000,
	RCC_AHB1_EMAC_RX	= 0x08000000,
	RCC_AHB1_PTP		= 0x10000000,
	RCC_AHB1_USB		= 0x20000000,
	RCC_AHB1_USB_ULPI	= 0x40000000,
	//0x80000000 reserved
};

enum rcc_apb1
{
	RCC_APB1_UART5 = 0x100000,
	RCC_APB1_UART4 = 0x080000,
	RCC_APB1_USART3 = 0x040000,
	RCC_APB1_USART2 = 0x020000
};

enum rcc_apb2
{
	RCC_APB2_SPI1	= 0x1000,
	RCC_APB2_SPI4	= 0x2000,
	RCC_APB2_SYSCFG	= 0x4000,
	RCC_APB2_SPI5	= 0x100000,
	RCC_APB2_SPI6 	= 0x200000
};

typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
	uint32_t field_1c;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t field_28;
	uint32_t field_2c;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
	uint32_t field_3c;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t field_48;
	uint32_t field_4c;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB3LPENR;
	uint32_t field_5c;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t field_68;
	uint32_t field_6c;
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t field_78;
	uint32_t field_7c;
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t PLLSAICFGR;
	uint32_t DCKCFGR1;
	uint32_t DCKCFGR2;
} rcc_t;

enum rcc_pll_bits
{
	RCC_PLL_READY	= 0x2000000,
	RCC_PLL_ENABLE 	= 0x1000000,

	RCC_APB2_DIV1 	= 0x0000,
	RCC_APB2_DIV2 	= 0x8000,
	RCC_APB2_DIV4 	= 0xA000,
	RCC_APB2_DIV8 	= 0xC000,
	RCC_APB2_DIV16 	= 0xE000,

	RCC_APB1_DIV1	= 0x0000,
	RCC_APB1_DIV2	= 0x1000,
	RCC_APB1_DIV4	= 0x1400,
	RCC_APB1_DIV8	= 0x1800,
	RCC_APB1_DIV16	= 0x1c00,

	RCC_AHB_DIV1	= 0x0000,
	RCC_AHB_DIV2	= 0x0080,
	RCC_AHB_DIV4	= 0x0090,
	RCC_AHB_DIV8	= 0x00a0,
	RCC_AHB_DIV16	= 0x00b0,
	//No divider setting for divide-by-32
	RCC_AHB_DIV64	= 0x00c0,
	RCC_AHB_DIV128	= 0x00d0,
	RCC_AHB_DIV256	= 0x00e0,
	RCC_AHB_DIV512	= 0x00f0,

	RCC_SYSCLK_PLL	= 0x2,

	RCC_PLLCFGR_RESERVED_MASK = 0x80BC8000
};

extern volatile rcc_t RCC;

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t BRR;
	uint32_t GTPR;
	uint32_t RTOR;
	uint32_t RQR;
	uint32_t ISR;
	uint32_t ICR;
	uint32_t RDR;
	uint32_t TDR;
} usart_t;

enum usart_bits
{
	USART_ISR_TXE = 0x80,
	USART_ISR_RXNE = 0x20
};

extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;
extern volatile usart_t UART4;
extern volatile usart_t UART5;
extern volatile usart_t USART6;

typedef struct
{
	uint16_t CR1;
	uint16_t field_02;
	uint16_t CR2;
	uint16_t field_06;
	uint16_t SR;
	uint16_t field_0a;
	uint8_t  DR;
	uint8_t  field_0d;
	uint16_t field_0e;
	uint16_t CRCPR;
	uint16_t field_12;
	uint16_t RXCRCR;
	uint16_t field_16;
	uint16_t TXCRCR;
	uint16_t field_1a;
	uint16_t I2SCFGR;
	uint16_t field_1e;
	uint16_t I2SPR;
	uint16_t field_22;
} spi_t;

extern volatile spi_t SPI1;
extern volatile spi_t SPI4;
extern volatile spi_t SPI5;
extern volatile spi_t SPI6;

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
	uint32_t RDES2;
	uint32_t RDES3;
} edma_rx_descriptor_t;

typedef struct
{
	uint32_t	DMABMR;
	uint32_t	DMATPDR;
	uint32_t	DMARPDR;
	volatile edma_rx_descriptor_t*		DMARDLAR;
	volatile void*		DMATDLAR;
	uint32_t	DMASR;
	uint32_t	DMAOMR;
	uint32_t	DMAIER;
	uint32_t	DMAMFBOCR;
	uint32_t	DMARSWTR;
	uint32_t	padding_1028[8];
	uint32_t	DMACHTDR;
	uint32_t	DMACHRDR;
	uint32_t	DMACHTBAR;
	uint32_t	DMACHRBAR;
} edma_t;

extern volatile emac_t EMAC;
extern volatile ptp_t PTP;
extern volatile edma_t EDMA;

typedef struct
{
	uint32_t	MEMRMP;
	uint32_t	PMC;
	uint32_t	EXTICR1;
	uint32_t	EXTICR2;
	uint32_t	EXTICR3;
	uint32_t	EXTICR4;
	uint32_t	reserved_18;
	uint32_t	CBR;
	uint32_t	CMPCR;
} syscfg_t;

enum syscfg_pmc
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

//Defines for what peripherals are present / implemented
//#define HAVE_I2C
//#define HAVE_TIM
//#define HAVE_SPI
#define HAVE_EMAC

#endif
