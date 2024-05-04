SECTIONS
{
	/* Code comes right after the vector tables */
	.text :
	{
		*(.vector)
		. = ALIGN(32);
		*(.fwver)
		. = ALIGN(32);
		*(.fwid)
		. = ALIGN(32);
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
	} > SRAM1 AT> FLASH

	/* BSS is pretty straightforward, put that in SRAM1 */
	.bss (NOLOAD):
	{
		__bss_start__ = .;
		*(.bss)
		__bss_end__ = .;
		__end = .;
		. = ORIGIN(SRAM1) + LENGTH(SRAM1) - 4;
		__stack = .;
	} > SRAM1

	/* SRAM2 currently unused */

	/* SFRs */
	.sfr_ahb1 (NOLOAD):
	{
		. = ALIGN(1024);
		. += 1024;		/* DMA1 not implemented */
		. += 1024;		/* DMA2 not implemented */
		. += 2048;		/* 0x4002_0800 reserved */
		. = ALIGN(1024);
		*(.rcc)
		. += 3072;		/* 0x4002_1400 reserved */
		. = ALIGN(1024);
		*(.flash)
		. = ALIGN(1024);
		. += 3072;		/* 0x4002_2400 reserved */
		. = ALIGN(1024);
		*(.crc)
	} > AHB1

	.sfr_apb1 (NOLOAD):
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
		. = ALIGN(1024);
		*(.rtc)
		. += 1024;		/* WWDG not implemented */
		. += 1024;		/* IWDG not implemented */
		. += 1024;		/* reserved 4000_3400 */
		. = ALIGN(1024);
		*(.spi2)
		. = ALIGN(1024);
		*(.spi3)
		. += 1024;		/* reserved 4000_4000 */
		. = ALIGN(1024);
		*(.usart2)
		. = ALIGN(1024);
		*(.usart3)
		. = ALIGN(1024);
		*(.uart4)
		. += 1024;		/* reserved 4000_5000 */
		. = ALIGN(1024);
		*(.i2c1)
		. = ALIGN(1024);
		*(.i2c2)
		. = ALIGN(1024);
		*(.i2c3)
		. += 1024;		/* CRS not implemented */
		. += 1024;		/* CAN1 not implemented */
		. += 1024;		/* USBFS not implemented */
		. += 1024;		/* USBSRAM not implemented */
		. = ALIGN(1024);
		*(.pwr)
	} > APB1

	.sfr_apb2 (NOLOAD):
	{
		. = ALIGN(1024);
		*(.syscfg)
		. = ALIGN(1024);
		*(.exti)
		. += 5120;		/* 0x4001_0800 reserved */
		. += 1024;		/* FIREWALL not implemented */
		. += 2048;		/* 0x4001_2000 reserved */
		. += 1024;		/* SDMMC not implemented */
		. = ALIGN(1024);
		*(.tim1)
		. = ALIGN(1024);
		*(.spi1)
		. += 1024;		/* 0x4001_3400 reserved */
		. = ALIGN(1024);
		*(.usart1)
		. += 1024;		/* 0x4001_3c00 reserved */
		. = ALIGN(1024);
		*(.tim15)
		. = ALIGN(1024);
		*(.tim16)
	} > APB2

	.sfr_io (NOLOAD):
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

	.sfr_mcu  (NOLOAD):
	{
		. += 0xed00;
		*(.scb)
		. = ALIGN(1024);
	} > MCU

	.sfr_mcu2 (NOLOAD):
	{
		*(.dbgmcu)
	} > MCU2

	.sfr_sig (NOLOAD):
	{
		*(.pkg)
		. += 0x8c;
		*(.uid)
		. += 0x44;
		*(.fid)
	} > SIG
}
