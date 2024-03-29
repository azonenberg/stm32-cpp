MEMORY
{
	FLASH(RX):	ORIGIN = 0x08000000, LENGTH = 32K
	SRAM(RW):	ORIGIN = 0x20000000, LENGTH = 4K

	/* SFRs */
	AHB2(RW):	ORIGIN = 0x48000000, LENGTH = 384K
	AHB1(RW):	ORIGIN = 0x40020000, LENGTH = 384K
	APB2(RW):	ORIGIN = 0x40010000, LENGTH = 32K
	APB1(RW):	ORIGIN = 0x40000000, LENGTH = 32K,

	SIG(R):		ORIGIN = 0x1ffff700, LENGTH = 256
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
		. += 1024;		/* DMA not implemented */
		. += 3072;		/* Reserved */
		. = ALIGN(1024);
		*(.rcc)
		. = ALIGN(1024);
		. += 3072;		/* Reserved */
		*(.flash)
	} > AHB1

	.sfr_ahb2 :
	{
		. = ALIGN(1024);
		*(.gpioa)
		. = ALIGN(1024);
		*(.gpiob)
		. = ALIGN(1024);
		*(.gpioc)
	} > AHB2

	.sfr_apb1 :
	{
		. = ALIGN(1024);
		*(.tim2)
		. = ALIGN(1024);
		*(.tim3)
		. = ALIGN(1024);
		. += 6144;		/* Reserved */
		. = ALIGN(1024);
		*(.tim14)
		. = ALIGN(1024);
		. += 1024;		/* Reserved */
		. += 1024;		/* RTC */
		. += 1024;		/* WWDG */
		. += 1024;		/* IWDG */
		. += 8192;		/* Reserved */
		. = ALIGN(1024);
		*(.i2c1)
	} > APB1

	.sfr_apb2 :
	{
		. = ALIGN(1024);
		*(.syscfg)
		. = ALIGN(1024);
		. += 1024;		/* EXTI not yet implemented */
		. += 7168;		/* Reserved */
		. += 1024;		/* ADC not yet implemented */
		. += 1024;		/* Reserved */
		. = ALIGN(1024);
		*(.tim1)
		. = ALIGN(1024);
		*(.spi1)
		. += 1024;		/* Reserved */
		. = ALIGN(1024);
		*(.usart1)
		. = ALIGN(1024);
		. += 2048;		/* Reserved */
		. = ALIGN(1024);
		*(.tim16)
		. = ALIGN(1024);
		*(.tim17)
		. = ALIGN(1024);
		. += 3072;		/* Reserved */
		*(.dbgmcu)
	} > APB2

	.sfr_sig :
	{
		. += 0xac;
		*(.uid)
		. += 0x14;
		*(.fid)
	} > SIG
}
