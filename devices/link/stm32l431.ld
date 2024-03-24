MEMORY
{
	FLASH(RX):	ORIGIN = 0x08000000, LENGTH = 32K
	SRAM(RW):	ORIGIN = 0x20000000, LENGTH = 8K

	/* SFRs */
	IOPORT(RW):	ORIGIN = 0x48000000, LENGTH = 8K
	AHB1(RW):	ORIGIN = 0x40020000, LENGTH = 32K
	APB1(RW):	ORIGIN = 0x40000000, LENGTH = 32K
	MCU2(RW):	ORIGIN = 0xe0042000, LENGTH = 16K

	SIG(R):		ORIGIN = 0x1fff7500, LENGTH = 16K
}

SECTIONS
{
	/* Code comes right after the vector tables */
	.text :
	{
		*(.vector)
		*(.text.*)
		*(.rodata)
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
		. = ORIGIN(SRAM) + LENGTH(SRAM) - 4;
		__stack = .;

	} > SRAM

	/* SFRs */
	.sfr_ahb1 :
	{
		. = ALIGN(1024);
		. += 1024;		/* DMA1 not implemented */
		. += 1024;		/* DMA2 not implemented */
		. += 2048;		/* 0x4002_0800 reserved */
		. = ALIGN(1024);
		*(.rcc)
		. += 3072;		/* 0x4002_2400 reserved */
		. = ALIGN(1024);
		*(.flash)
	} > AHB1

	.sfr_apb1 :
	{
		. = ALIGN(1024);
		*(.tim2)
		. = ALIGN(1024);
		*(.tim3)
		. += 2048;		/* reserved 4000_0800 */
		. = ALIGN(1024);
		*(.tim6)
		. = ALIGN(1024);
		*(.tim7)
		. += 3072;		/* reserved 4000_1800 */
		. += 1024;		/* LCD not implemented */
		. += 1024;		/* RTC not implemented */
		. += 1024;		/* WWDG not implemented */
		. += 1024;		/* IWDG not implemented */
		. += 1024;		/* reserved 4000_3400 */
		. += 1024;		/* SPI2 not implemented */
		. += 1024;		/* SPI3 not implemented */
		. += 1024;		/* reserved 4000_4000 */
		. = ALIGN(1024);
		*(.usart2)
		. = ALIGN(1024);
		*(.usart3)
		. = ALIGN(1024);
		*(.uart4)
		. += 1024;		/* reserved 4000_5000 */
		. += 1024;		/* I2C1 not implemented */
		. += 1024;		/* I2C2 not implemented */
		. += 1024;		/* I2C3 not implemented */
		. += 1024;		/* CRS not implemented */
		. += 1024;		/* CAN1 not implemented */
		. += 1024;		/* USBFS not implemented */
		. += 1024;		/* USBSRAM not implemented */
		. = ALIGN(1024);
		*(.pwr)
	} > APB1

	.sfr_io :
	{
		. = ALIGN(1024);
		*(.gpioa)
		. = ALIGN(1024);
		*(.gpiob)
		. = ALIGN(1024);
		*(.gpioc)
		. = ALIGN(1024);
		*(.gpiod)
		. = ALIGN(1024);
		*(.gpioe)
		. = ALIGN(1024);
		. += 1024;	/* no GPIOF present */
		. += 1024;	/* no GPIOG present */
		. = ALIGN(1024);
		*(.gpioh)
	} > IOPORT

	.sfr_mcu :
	{
		*(.dbgmcu)
	} > MCU2

	.sfr_sig :
	{
		*(.pkg)
		. += 0x8c;
		*(.uid)
		. += 0x44;
		*(.fid)
	} > SIG
}
