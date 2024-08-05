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

#ifndef stm32_adc_h
#define stm32_adc_h

#define HAVE_ADC

//STM32031
#if ADC_T_VERSION == 1

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

//STM32L431
#elif ADC_T_VERSION == 2

typedef struct
{
	uint32_t	ISR;
	uint32_t	IER;
	uint32_t	CR;
	uint32_t	CFGR;
	uint32_t	CFGR2;
	uint32_t	SMPR1;
	uint32_t	SMPR2;
	uint32_t	field_1c;
	uint32_t	TR1;
	uint32_t	TR2;
	uint32_t	TR3;
	uint32_t	field_2c;
	uint32_t	SQR1;
	uint32_t	SQR2;
	uint32_t	SQR3;
	uint32_t	SQR4;
	uint32_t	DR;
	uint32_t	field_44;
	uint32_t	field_48;
	uint32_t	JSQR;
	uint32_t	field_50;
	uint32_t	field_54;
	uint32_t	field_58;
	uint32_t	field_5c;
	uint32_t	OFR1;
	uint32_t	OFR2;
	uint32_t	OFR3;
	uint32_t	OFR4;
	uint32_t	field_70;
	uint32_t	field_74;
	uint32_t	field_78;
	uint32_t	field_7c;
	uint32_t	JDR1;
	uint32_t	JDR2;
	uint32_t	JDR3;
	uint32_t	JDR4;
	uint32_t	field_90;
	uint32_t	field_94;
	uint32_t	field_98;
	uint32_t	field_9c;
	uint32_t	AWD2CR;
	uint32_t	AWD3CR;
	uint32_t	field_a8;
	uint32_t	field_ac;
	uint32_t	DIFSEL;
	uint32_t	CALFACT;

	//padding to total struct size 0x100
	uint32_t	field_b8[18];
} adcchan_t;

typedef struct
{
	//ADC2 only present on some SKUs, but still takes up space in the register map
	adcchan_t	chans[2];

	//space in the register map for a third ADC but it's not used
	uint32_t	padding[64];

	//Common registers
	uint32_t	CSR;
	uint32_t	field_304;
	uint32_t	CCR;
	uint32_t	CDR;
} adc_t;

enum adcchan_cr_t
{
	ADC_CR_ADCAL	= 0x8000'0000,
	ADC_CR_DEEPPWD	= 0x2000'0000,
	ADC_CR_ADVREGEN	= 0x1000'0000,
	ADC_CR_ADSTART	= 0x0000'0004,
	ADC_CR_ADEN		= 0x0000'0001,
};

enum adcchan_isr_t
{
	ADC_ISR_ADRDY	= 0x0000'0001,
	ADC_ISR_EOC		= 0x0000'0004
};

enum adc_ccr_t
{
	ADC_CCR_CH18SEL	= 0x0100'0000,
	ADC_CCR_CH17SEL	= 0x0080'0000,
	ADC_CCR_VREFEN	= 0x0040'0000
};

#else

#error Undefined or unspecified ADC_T_VERSION

#endif	//version check

#endif	//include guard
