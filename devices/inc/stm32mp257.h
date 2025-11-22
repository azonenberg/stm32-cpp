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

#ifndef stm32mp257_h
#define stm32mp257_h

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// RCC

#define RCC_T_VERSION 2
#include "stm32-rcc.h"
extern volatile rcc_t RCC;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Generic peripherals

#define GPIO_T_VERSION 3
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
extern volatile gpio_t GPIOZ;

#define TIM_T_VERSION 2
#include "stm32-tim.h"
extern volatile tim_t TIM1;
extern volatile tim_t TIM2;
extern volatile tim_t TIM3;
extern volatile tim_t TIM4;
extern volatile tim_t TIM5;
extern volatile tim_t TIM6;
extern volatile tim_t TIM7;
extern volatile tim_t TIM8;
//no TIM9??
extern volatile tim_t TIM10;
extern volatile tim_t TIM11;
extern volatile tim_t TIM12;
extern volatile tim_t TIM13;
extern volatile tim_t TIM14;
extern volatile tim_t TIM15;
extern volatile tim_t TIM16;
extern volatile tim_t TIM17;
//no TIM18
//no TIM19
extern volatile tim_t TIM20;

#define CRC_T_VERSION 2
#include "stm32-crc.h"
extern volatile crc_t _CRC;

#define I2C_T_VERSION 2
#include "stm32-i2c.h"
extern volatile i2c_t I2C1;
extern volatile i2c_t I2C2;
extern volatile i2c_t I2C3;
extern volatile i2c_t I2C4;
extern volatile i2c_t I2C5;
extern volatile i2c_t I2C6;
extern volatile i2c_t I2C7;
extern volatile i2c_t I2C8;

#define USART_T_VERSION 3
#include "stm32-usart.h"
extern volatile usart_t USART1;
extern volatile usart_t USART2;
extern volatile usart_t USART3;
extern volatile usart_t UART4;
extern volatile usart_t UART5;
extern volatile usart_t USART6;
extern volatile usart_t UART7;
extern volatile usart_t UART8;

#define OCTOSPI_T_VERSION 2
#include "stm32-octospi.h"
extern volatile octospi_t OCTOSPI1;
extern volatile octospi_t OCTOSPI2;
extern volatile octospim_t OCTOSPIM;

#define PWR_T_VERSION 2
#include "stm32-pwr.h"
extern volatile pwr_t PWR;

#define BSEC_T_VERSION 1
#include "stm32-bsec.h"
extern volatile bsec_t _BSEC;

#define SYSCFG_T_VERSION 2
#include "stm32-syscfg.h"
extern volatile syscfg_t SYSCFG;

#define EXTI_T_VERSION 3
#include "stm32-exti.h"
extern volatile exti_t EXTI1;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripherals only for CPU2 (Cortex-M33)

#ifdef STM32MP2_CPU2

#define SCB_T_VERSION 3
#include "stm32-scb.h"
extern volatile scb_t SCB;

//TODO: The M33 does in fact have an L1
//#define HAVE_L1

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Feature flags common to the entire platform

//don't try to use internal flash
#define NO_INTERNAL_FLASH
#define HAVE_PKG

#endif
