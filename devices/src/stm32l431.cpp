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

#include <stm32l431.h>

volatile gpio_t GPIOA __attribute__((section(".gpioa")));
volatile gpio_t GPIOB __attribute__((section(".gpiob")));
volatile gpio_t GPIOC __attribute__((section(".gpioc")));
volatile gpio_t GPIOD __attribute__((section(".gpiod")));
volatile gpio_t GPIOE __attribute__((section(".gpioe")));
volatile gpio_t GPIOH __attribute__((section(".gpioh")));

volatile rcc_t RCC __attribute__((section(".rcc")));

volatile rtc_t _RTC __attribute__((section(".rtc")));

volatile pwr_t PWR __attribute__((section(".pwr")));

volatile flash_t FLASH __attribute__((section(".flash")));

volatile crc_t _CRC __attribute__((section(".crc")));

volatile i2c_t I2C1 __attribute__((section(".i2c1")));
volatile i2c_t I2C2 __attribute__((section(".i2c2")));
volatile i2c_t I2C3 __attribute__((section(".i2c3")));

volatile spi_t SPI1 __attribute__((section(".spi1")));
volatile spi_t SPI2 __attribute__((section(".spi2")));
volatile spi_t SPI3 __attribute__((section(".spi3")));

//volatile adc_t ADC1 __attribute__((section(".adc1")));

volatile syscfg_t SYSCFG __attribute__((section(".syscfg")));

volatile usart_t USART1 __attribute__((section(".usart1")));
volatile usart_t USART2 __attribute__((section(".usart2")));
volatile usart_t USART3 __attribute__((section(".usart3")));
volatile usart_t UART4 __attribute__((section(".uart4")));

volatile tim_t TIM1 __attribute__((section(".tim1")));
volatile tim_t TIM2 __attribute__((section(".tim2")));
volatile tim_t TIM3 __attribute__((section(".tim3")));
volatile tim_t TIM6 __attribute__((section(".tim6")));
volatile tim_t TIM7 __attribute__((section(".tim7")));
volatile tim_t TIM15 __attribute__((section(".tim15")));
volatile tim_t TIM16 __attribute__((section(".tim16")));

volatile exti_t EXTI __attribute__((section(".exti")));

volatile dbgmcu_t DBGMCU __attribute__((section(".dbgmcu")));
volatile scb_t SCB __attribute__((section(".scb")));

volatile uint32_t U_ID[3] __attribute__((section(".uid")));
volatile uint16_t FLASH_SIZE __attribute__((section(".fid")));
volatile uint16_t PKG __attribute__((section(".pkg")));
/*
volatile uint16_t VREFINT_CAL __attribute__((section(".vrefint")));
volatile uint16_t TSENSE_CAL1 __attribute__((section(".tcal1")));
volatile uint16_t TSENSE_CAL2 __attribute__((section(".tcal2")));
*/
