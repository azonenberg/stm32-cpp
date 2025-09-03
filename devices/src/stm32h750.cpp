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

#include <stm32h750.h>

volatile gpio_t GPIOA __attribute__((section(".gpioa")));
volatile gpio_t GPIOB __attribute__((section(".gpiob")));
volatile gpio_t GPIOC __attribute__((section(".gpioc")));
volatile gpio_t GPIOD __attribute__((section(".gpiod")));
volatile gpio_t GPIOE __attribute__((section(".gpioe")));
volatile gpio_t GPIOF __attribute__((section(".gpiof")));
volatile gpio_t GPIOG __attribute__((section(".gpiog")));
volatile gpio_t GPIOH __attribute__((section(".gpioh")));
volatile gpio_t GPIOI __attribute__((section(".gpioi")));
volatile gpio_t GPIOJ __attribute__((section(".gpioj")));
volatile gpio_t GPIOK __attribute__((section(".gpiok")));

volatile rcc_t RCC __attribute__((section(".rcc")));

volatile rtc_t _RTC __attribute__((section(".rtc")));
volatile crc_t _CRC __attribute__((section(".crc")));

volatile flash_t FLASH __attribute__((section(".flash")));

volatile mdma_t _MDMA __attribute__((section(".mdma")));

volatile pwr_t PWR __attribute__((section(".pwr")));

volatile i2c_t I2C1 __attribute__((section(".i2c1")));
volatile i2c_t I2C2 __attribute__((section(".i2c2")));
volatile i2c_t I2C3 __attribute__((section(".i2c3")));
volatile i2c_t I2C4 __attribute__((section(".i2c4")));
volatile i2c_t I2C5 __attribute__((section(".i2c5")));

volatile spi_t SPI1 __attribute__((section(".spi1")));
volatile spi_t SPI2 __attribute__((section(".spi2")));
volatile spi_t SPI3 __attribute__((section(".spi3")));
volatile spi_t SPI4 __attribute__((section(".spi4")));
volatile spi_t SPI5 __attribute__((section(".spi5")));
volatile spi_t SPI6 __attribute__((section(".spi6")));

volatile usart_t USART1 __attribute__((section(".usart1")));
volatile usart_t USART2 __attribute__((section(".usart2")));
volatile usart_t USART3 __attribute__((section(".usart3")));
volatile usart_t UART4 __attribute__((section(".uart4")));
volatile usart_t UART5 __attribute__((section(".uart5")));
volatile usart_t USART6 __attribute__((section(".usart6")));
/*
volatile usart_t UART7 __attribute__((section(".uart7")));
volatile usart_t UART8 __attribute__((section(".uart8")));

volatile emac_t EMAC __attribute__((section(".emac")));
volatile ptp_t PTP __attribute__((section(".ptp")));
volatile edma_t EDMA __attribute__((section(".edma")));
*/

volatile syscfg_t SYSCFG __attribute__((section(".syscfg")));

volatile dbgmcu_t DBGMCU __attribute__((section(".dbgmcu")));

volatile scb_t SCB __attribute__((section(".scb")));
volatile itm_t _ITM __attribute__((section(".itm")));
volatile dwt_t _DWT __attribute__((section(".dwt")));
volatile tpiu_t _TPIU __attribute__((section(".tpiu")));

//volatile tim_t TIM1 __attribute__((section(".tim1")));
volatile tim_t TIM2 __attribute__((section(".tim2")));
volatile tim_t TIM3 __attribute__((section(".tim3")));
volatile tim_t TIM4 __attribute__((section(".tim4")));
volatile tim_t TIM5 __attribute__((section(".tim5")));
volatile tim_t TIM6 __attribute__((section(".tim6")));
volatile tim_t TIM7 __attribute__((section(".tim7")));
//volatile tim_t TIM8 __attribute__((section(".tim8")));
//volatile tim_t TIM9 __attribute__((section(".tim9")));
//volatile tim_t TIM10 __attribute__((section(".tim10")));
//volatile tim_t TIM11 __attribute__((section(".tim11")));
volatile tim_t TIM12 __attribute__((section(".tim12")));
volatile tim_t TIM13 __attribute__((section(".tim13")));
volatile tim_t TIM14 __attribute__((section(".tim14")));
/*
volatile dma_t DMA1 __attribute__((section(".dma1")));
volatile dma_t DMA2 __attribute__((section(".dma2")));
volatile dmamux_t DMAMUX1 __attribute__((section(".dmamux1")));
*/
volatile uint32_t U_ID[3] __attribute__((section(".uid")));
volatile uint16_t F_ID __attribute__((section(".fid")));
volatile uint16_t PKG_ID __attribute__((section(".pkg")));

volatile cryp_t CRYP __attribute__((section(".cryp")));
volatile rng_t RNG __attribute__((section(".rng")));
volatile hash_t HASH __attribute__((section(".chash")));

//volatile fmc_t _FMC __attribute__((section(".fmc")));
