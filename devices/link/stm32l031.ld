MEMORY
{
	FLASH(RX):	ORIGIN = 0x08000000, LENGTH = 32K
	SRAM(RW):	ORIGIN = 0x20000000, LENGTH = 8K

	/* SFRs */
	IOPORT(RW):	ORIGIN = 0x50000000, LENGTH = 8K
	AHB(RW):	ORIGIN = 0x40020000, LENGTH = 32K
	APB2(RW):	ORIGIN = 0x40010000, LENGTH = 32K
	APB1(RW):	ORIGIN = 0x40000000, LENGTH = 32K

	SIG(R):		ORIGIN = 0x1ff80000, LENGTH = 64K
	MCU(RW):	ORIGIN = 0xe0000000, LENGTH = 64K
}

SECTIONS
{
	/* Code comes right after the vector tables */
	.text :
	{
		KEEP (*(.vector))
		. = ALIGN(32);
		KEEP (*(.note.gnu.build-id))
		. = ALIGN(32);
		KEEP (*(.fwid))
		. = ALIGN(32);
		*(.text.*)
		*(.rodata)
		*(.rodata.*)
		*(.ctors)

		. = ALIGN(4);
		KEEP(*(.init))

		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP(*(.preinit_array))
		__preinit_array_end = .;

		. = ALIGN(4);
		__init_array_start = .;
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		__init_array_end = .;

		__dtor_start = .;
		*(.fini_array)
		__dtor_end = .;
	} > FLASH

	/* Initialized data needs special handling because it lives in two places */
	.data :
	{
		__data_romstart = LOADADDR(.data);
		__data_start = .;
		*(.data)
		__data_end = .;
	} > SRAM AT> FLASH

	/* BSS is pretty straightforward. Use all remaining space for the stack */
	.bss :
	{
		__bss_start__ = .;
		*(.bss)
		__bss_end__ = .;
		__end = .;
		. = ORIGIN(SRAM) + LENGTH(SRAM);
		__stack = .;

	} > SRAM

	/* SFRs */
	.sfr_ahb :
	{
		. += 1024;		/* DMA1 not implemented */
		. += 3072;		/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.rcc))
		. = ALIGN(1024);
		. += 3072;		/* Reserved */
		KEEP(*(.flash))
		. = ALIGN(1024);
		. += 3072;		/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.crc))
		. = ALIGN(1024);
		. += 3072;		/* Reserved */
		. += 3072;		/* Reserved */
		. += 3072;		/* Reserved */
		. = ALIGN(1024);
		. += 1024;		/* AES not present in L031 */
	} > AHB

	.sfr_io :
	{
		. = ALIGN(1024);
		KEEP(*(.gpioa))
		. = ALIGN(1024);
		KEEP(*(.gpiob))
		. = ALIGN(1024);
		KEEP(*(.gpioc))
		. = ALIGN(1024);
		KEEP(*(.gpiod))
		. = ALIGN(1024);
		KEEP(*(.gpioe))
		. = ALIGN(1024);
		. += 1024;	/* no GPIOF present */
		. += 1024;	/* no GPIOG present */
		. = ALIGN(1024);
		*(.gpioh)
	} > IOPORT

	.sfr_apb1 :
	{
		. = ALIGN(1024);
		KEEP(*(.tim2))
		. = ALIGN(1024);
		KEEP(*(.tim3))
		. = ALIGN(1024);
		. += 2048;			/* Reserved (Errata in datasheet page 55, says 1K when should be 2K, see case #00183734) */
		. = ALIGN(1024);
		KEEP(*(.tim6))
		. = ALIGN(1024);
		KEEP(*(.tim7))
		. = ALIGN(1024);
		. += 4096;			/* Reserved (Errata in datasheet page 55, says 3K when should be 4K, see case #00183734)  */
		. = ALIGN(1024);
		. += 1024;			/* RTC / BKUP_REG not yet implemented */
		. = ALIGN(1024);
		. += 1024;			/* WWDG not yet implemented */
		. = ALIGN(1024);
		. += 1024;			/* IWDG not yet implemented */
		. = ALIGN(1024);
		. += 1024;			/* Reserved */
		. = ALIGN(1024);
		. += 1024;			/* SPI2 not yet implemented */
		. = ALIGN(1024);
		. += 2048;			/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.usart2))
		. = ALIGN(1024);
		. += 1024;			/* LPUART1 not yet implemented */
		. = ALIGN(1024);
		KEEP(*(.usart4))
		. = ALIGN(1024);
		KEEP(*(.usart5))
		. = ALIGN(1024);
		KEEP(*(.i2c1))
		. += 1024;			/* I2C2 not yet implemented */
		. += 5120;			/* 0x4000_5c00 reserved (datasheet errata, page 54 says 1k) */
		. = ALIGN(1024);
		*(.pwr)
	} > APB1

	.sfr_apb2 :
	{
		. = ALIGN(1024);
		KEEP(*(.syscfg))
		. = ALIGN(1024);
		KEEP(*(.exti))
		. = ALIGN(1024);
		KEEP(*(.tim21))
		. = ALIGN(1024);
		. += 2048;			/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.tim22))
		. = ALIGN(1024);
		. += 1024;			/* Reserved */
		. = ALIGN(1024);
		. += 1024;			/* Firewall not implemented */
		. = ALIGN(1024);
		. += 1024;			/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.adc1))
		. = ALIGN(1024);
		. += 2048;			/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.spi1))
		. = ALIGN(1024);
		. += 1024;			/* Reserved */
		. = ALIGN(1024);
		. += 1024;			/* USART1 not implemented */
		. = ALIGN(1024);
		. += 7168;			/* Reserved */
		. = ALIGN(1024);
		KEEP(*(.dbgmcu))
	} > APB2

	.sfr_sig :
	{
		. += 0x50;
		KEEP(*(.uid))
		. += 0x1c;
		KEEP(*(.vrefint))
		KEEP(*(.tcal1))
		KEEP(*(.fid))
		KEEP(*(.tcal2))
	} > SIG

	.sfr_mcu  (NOLOAD):
	{
		. += 0xed00;
		KEEP (*(.scb))
		. = ALIGN(1024);
	} > MCU
}
