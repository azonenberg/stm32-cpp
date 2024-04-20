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

#ifndef stm32_spi_h
#define stm32_spi_h

#define HAVE_SPI

//STM32L431
#if SPI_T_VERSION == 1

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
	SPI_FRXTH		= 0x1000,
	SPI_RXNEIE		= 0x0040
};

enum spi_sr_bits
{
	SPI_TX_FIFO_MASK	= 0x1800,
	SPI_RX_FIFO_MASK	= 0x0600,
	SPI_BUSY			= 0x0080,
	SPI_TX_EMPTY		= 0x0002,
	SPI_RX_NOT_EMPTY	= 0x0001,
};

//STM32H735
#elif SPI_T_VERSION == 2

enum spi_cr1_bits
{
	SPI_ENABLE		= 0x00000001,
	SPI_AF_LOCK		= 0x00010000
};

enum spi_cfg2_bits
{
	SPI_CPOL		= 0x02000000,
	SPI_LSB_FIRST	= 0x00800000,
	SPI_SSOE		= 0x20000000,
	SPI_MASTER		= 0x00400000
};

enum spi_sr_bits
{
	SPI_TX_FIFO_MASK	= 0x0002,
	//SPI_RX_FIFO_MASK	= 0x0600,
	//SPI_BUSY			= 0x0080,
	SPI_TX_EMPTY		= 0x1000,
	SPI_RX_NOT_EMPTY	= 0x0001,
	SPI_START			= 0x0200
};

enum spi_ier_bits
{
	SPI_IER_RXPIE		= 0x0001
};

typedef struct
{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CFG1;
	uint32_t CFG2;
	uint32_t IER;
	uint32_t SR;
	uint32_t IFCR;
	uint32_t field_1c;
	uint8_t	TXDR;			//TODO: find a cleaner way of doing this
	uint8_t field_21[3];
	uint32_t field_24;
	uint32_t field_28;
	uint32_t field_2c;
	uint32_t RXDR;
	uint32_t field_34;
	uint32_t field_38;
	uint32_t field_3c;
	uint32_t CRCPOLY;
	uint32_t TXCRC;
	uint32_t RXCRCR;
	uint32_t UDRDR;
	uint32_t I2SCFGR;
} spi_t;

#else

#error Undefined or unspecified USART_T_VERSION

#endif	//version check

#endif	//include guard
