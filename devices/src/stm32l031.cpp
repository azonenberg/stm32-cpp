/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP v0.1                                                                                                       *
*                                                                                                                      *
* Copyright (c) 2020-2023 Andrew D. Zonenberg                                                                          *
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

#include <stm32l031.h>

volatile gpio_t GPIOA __attribute__((section(".gpioa")));
volatile gpio_t GPIOB __attribute__((section(".gpiob")));
volatile gpio_t GPIOC __attribute__((section(".gpioc")));
volatile gpio_t GPIOD __attribute__((section(".gpiod")));
volatile gpio_t GPIOE __attribute__((section(".gpioe")));
volatile gpio_t GPIOH __attribute__((section(".gpioh")));

volatile rcc_t RCC __attribute__((section(".rcc")));

volatile flash_t FLASH __attribute__((section(".flash")));
/*
volatile i2c_t I2C1 __attribute__((section(".i2c1")));

volatile spi_t SPI1 __attribute__((section(".spi1")));
*/
volatile syscfg_t SYSCFG __attribute__((section(".syscfg")));

volatile usart_t USART2 __attribute__((section(".usart2")));
volatile usart_t USART4 __attribute__((section(".usart4")));
volatile usart_t USART5 __attribute__((section(".usart5")));

volatile tim_t TIMER2 __attribute__((section(".tim2")));
volatile tim_t TIMER3 __attribute__((section(".tim3")));
volatile tim_t TIMER6 __attribute__((section(".tim6")));
volatile tim_t TIMER7 __attribute__((section(".tim7")));
volatile tim_t TIMER21 __attribute__((section(".tim21")));
volatile tim_t TIMER22 __attribute__((section(".tim22")));

volatile dbgmcu_t DBGMCU __attribute__((section(".dbgmcu")));

volatile uint32_t U_ID[3] __attribute__((section(".uid")));
volatile uint16_t F_ID __attribute__((section(".fid")));
