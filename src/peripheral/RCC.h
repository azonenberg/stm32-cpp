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

#ifndef RCC_h
#define RCC_h

#include <stm32.h>

/**
	@brief Reset and Clock Control

	Helper class for enabling various devices.

	All functions are static because there's only one RCC in the device.
 */
class RCCHelper
{
public:
	static void Enable(volatile gpio_t* gpio);

	#ifdef HAVE_BSEC
	static void Enable(volatile bsec_t* bsec);
	#endif

	#ifdef HAVE_DMA
	static void Enable(volatile dma_t* dma);
	#endif

	#ifdef HAVE_MDMA
	static void Enable(volatile mdma_t* mdma);
	#endif

	#ifdef HAVE_FMC
	static void Enable(volatile fmc_t* fmc);
	#endif

	#ifdef HAVE_QUADSPI
	static void Enable(volatile quadspi_t* quadspi);
	#endif

	#ifdef HAVE_CRC
	static void Enable(volatile crc_t* crc);
	#endif

	#ifdef HAVE_RTC
	static void Enable(volatile rtc_t* rtc);
	#endif

	#ifdef HAVE_DTS
	static void Enable(volatile dts_t* dts);
	#endif

	#if defined(STM32L4) || defined(STM32L0)
	static void Enable(volatile pwr_t* pwr);
	#endif

	#ifdef HAVE_ADC
	static void Enable(volatile adc_t* adc);
	#endif

	#ifdef HAVE_I2C
	static void Enable(volatile i2c_t* i2c);
	#endif

	#ifdef HAVE_SPI
	static void Enable(volatile spi_t* spi);
	#endif

	#ifdef HAVE_TIM
	static void Enable(volatile tim_t* tim);
	#endif

	#ifdef HAVE_EMAC
	static void Enable(volatile emac_t* mac);
	#endif

	#ifdef HAVE_RNG
	static void Enable(volatile rng_t* rng);
	#endif

	#ifdef HAVE_HASH
	static void Enable(volatile hash_t* hash);
	#endif

	#ifdef HAVE_CRYP
	static void Enable(volatile cryp_t* cryp);
	#endif

	#ifdef HAVE_UART
	static void Enable(volatile usart_t* uart);
	#endif

	#ifdef HAVE_OCTOSPI
	static void Enable(volatile octospim_t* octospim);
	static void Enable(volatile octospi_t* octospi);
	#endif

	#ifdef STM32F0
	static void InitializePLLFromInternalOscillator(
		uint8_t prediv,
		uint8_t mult,
		uint16_t ahbdiv,
		uint8_t apbdiv
		);
	#endif

	#ifdef STM32L0
	static void InitializePLLFromHSI16(uint8_t mult, uint8_t hclkdiv, uint16_t ahbdiv, uint8_t apb2div, uint8_t apb1div);
	#endif

	#ifdef STM32L4
	static void InitializePLLFromHSI16(
		uint8_t prediv,
		uint8_t mult,
		uint8_t qdiv,
		uint8_t rdiv,
		uint16_t ahbdiv,
		uint8_t apb1div,
		uint8_t apb2div);
	#endif

	#ifdef STM32F7
	static void InitializePLLFromInternalOscillator(
		uint8_t prediv,
		uint16_t mult,
		uint8_t pdiv,
		uint8_t qdiv,
		uint8_t rdiv,
		uint16_t ahbdiv,
		uint16_t apb1div,
		uint16_t apb2div
		);
	#endif

	#if defined(STM32H735) || defined(STM32H750)
		enum FMCClockSource
		{
			FMC_CLOCK_HCLK3		= 0,
			FMC_CLOCK_PLL1_Q	= 1,
			FMC_CLOCK_PLL2_R	= 2,
			FMC_CLOCK_CKPER		= 3,
			FMC_CLOCK_MASK		= 3
		};

		/**
			@brief Selects the kernel clock for the FMC
		 */
		static void SetFMCKernelClock(FMCClockSource src)
		{ RCC.D1CCIPR = (RCC.D1CCIPR & ~FMC_CLOCK_MASK) | src; }
	#endif

	#ifdef STM32H7
		enum ClockSource
		{
			CLOCK_SOURCE_HSE,
			CLOCK_SOURCE_HSI
		};

		static void EnableHighSpeedExternalClock();
		static void EnableHighSpeedInternalClock(int mhz);
		static void InitializePLL(
			uint8_t npll,
			float in_mhz,
			uint8_t prediv,
			uint16_t mult,
			uint8_t divP,
			uint8_t divQ,
			uint8_t divR,
			ClockSource source
			);
		static void SelectSystemClockFromHSI();
		static void SelectSystemClockFromPLL1();
		static void InitializeSystemClocks(
			uint16_t sysckdiv,
			uint16_t ahbdiv,
			uint8_t apb1div,
			uint8_t apb2div,
			uint8_t apb3div,
			uint8_t apb4div
			);
		static uint8_t GetDivider512Code(uint16_t div);
		static uint8_t GetDivider16Code(uint8_t div);

		#ifdef STM32H750
			static void EnableSram3();
		#endif

		static void EnableSram2();
		static void EnableSram1();
		static void EnableBackupSram();
	#endif

	#ifdef STM32MP2

		///@brief Set the clock divider for ck_icn_ls_mcu (max 200 MHz) from ck_icn_hs_mcu. Must be /1 (powerup default) or /2
		static void SetLowSpeedMCUClockDivider(bool divideBy2)
		{ RCC.LSMCUDIVR = divideBy2; }

		///@brief Set the divider from ck_icn_ls_mcu to ck_icn_apb1 (max 200 MHz)
		static void SetAPB1ClockDivider(rcc_apb_div div)
		{ RCC.APB1DIVR = div; }

		///@brief Set the divider from ck_icn_ls_mcu to ck_icn_apb2 (max 200 MHz)
		static void SetAPB2ClockDivider(rcc_apb_div div)
		{ RCC.APB2DIVR = div; }

		///@brief Set the divider from ck_icn_ls_mcu to ck_icn_apb3 (max 200 MHz)
		static void SetAPB3ClockDivider(rcc_apb_div div)
		{ RCC.APB3DIVR = div; }

		///@brief Set the divider from ck_icn_ls_mcu to ck_icn_apb4 (max 200 MHz)
		static void SetAPB4ClockDivider(rcc_apb_div div)
		{ RCC.APB4DIVR = div; }

		///@brief Set the divider from ck_icn_ls_mcu to ck_icn_apbdbg (max 200 MHz)
		static void SetAPBDebugClockDivider(rcc_apb_div div)
		{ RCC.APBDBGDIVR = div; }

		///@brief Configure mux path from clock source to PLL input
		static void SetPLLInputMux(rcc_muxsel_lane lane, rcc_muxsel_val val)
		{ RCC.MUXSELCFGR = (RCC.MUXSELCFGR & ~(3 << lane)) | (val << lane); }

		static void WaitForPreDividerIdle(rcc_xbar_channel lane);
		static void WaitForPostDividerIdle(rcc_xbar_channel lane);

		static void WaitForCrossbarIdle(rcc_xbar_channel lane)
		{
			while( (RCC.XBARCFGR[lane] & RCC_XBAR_STS) == RCC_XBAR_STS)
			{}
		}

		static void SetCrossbarDivider(rcc_xbar_channel lane, rcc_prediv prediv, uint32_t findiv);

		static void SetCrossbarMux(rcc_xbar_channel lane, rcc_xbarmux mux);

		static void ConfigureGeneralPLL(
			uint32_t idx,
			uint32_t prediv,
			uint32_t vcomult,
			uint32_t postdiv1,
			uint32_t postdiv2
			);

		static void EnableHighSpeedExternalClock();
	#endif

	#if defined(STM32L0) || defined(STM32L4) || defined(STM32H7)
	static void EnableSyscfg();
	#endif
};

#endif
