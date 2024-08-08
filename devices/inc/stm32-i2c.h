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

#ifndef stm32_i2c_h
#define stm32_i2c_h

#define HAVE_I2C

//STM32L431
#if I2C_T_VERSION == 1

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

#else

#error Undefined or unspecified I2C_T_VERSION

#endif	//version check

#endif	//include guard
