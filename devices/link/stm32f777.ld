ENTRY(_start)

MEMORY
{
	FLASH(RX):	ORIGIN = 0x08000000, LENGTH = 1M
	SRAM1(RW):	ORIGIN = 0x20020000, LENGTH = 368K
	DTCM(RW):	ORIGIN = 0x20000000, LENGTH = 128K
	SRAM2(RW):	ORIGIN = 0x2007c000, LENGTH = 16K

	/* SFRs */
	AHB1(RW):	ORIGIN = 0x40020000, LENGTH = 384K
	APB2(RW):	ORIGIN = 0x40010000, LENGTH = 32K
	APB1(RW):	ORIGIN = 0x40000000, LENGTH = 32K
	MCU(RW):	ORIGIN = 0xe0000000, LENGTH = 512K
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
	} > SRAM1 AT> FLASH

	/* Stack goes in TCM to keep it fast. We probably won't be DMAing etc to/from it. */
	.stack :
	{
		__end = .;
		. = ORIGIN(DTCM) + LENGTH(DTCM) - 4;
		__stack = .;
	} > DTCM

	/* BSS is pretty straightforward */
	.bss :
	{
		__bss_start__ = .;
		*(.bss)
		__bss_end__ = .;
	} > SRAM1

	/* Fill all remaining SRAM1 space with the heap */
	.heap :
	{
		. = ALIGN(4);
		__heap_start = .;
		. = ORIGIN(SRAM1) + LENGTH(SRAM1) - 4;
		__heap_end = .;
		PROVIDE(end = .);
	} > SRAM1

	/* SFRs */
	.sfr_ahb1 :
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
		*(.gpiof)
		. = ALIGN(1024);
		*(.gpiog)
		. = ALIGN(1024);
		*(.gpioh)
		. = ALIGN(1024);
		*(.gpioi)
		. = ALIGN(1024);
		*(.gpioj)
		. = ALIGN(1024);
		*(.gpiok)

		. += 1024;		/* reserved 4002_2c00 */
		. += 1024;		/* CRC not yet implemented */
		. += 1024;		/* reserved 4002_3400 */

		. = ALIGN(1024);
		*(.rcc)
		. = ALIGN(1024);
		*(.flash)
		. = ALIGN(1024);
		. += 4096;		/* Backup SRAM not yet implemented */
		. += 4096;		/* reserved 4002_5000 */
		. += 1024;		/* DMA1 not yet implemented */
		. += 1024;		/* DMA2 not yet implemented */
		. += 6144;		/* reserved 4002_6800 */
		*(.emac)
		. = ALIGN(1024);
		. += 768;
		*(.ptp)
		. = ALIGN(4096);
		*(.edma)
	} > AHB1

	.sfr_apb1 :
	{
		*(.tim2)
		. = ALIGN(1024);
		*(.tim3)
		. = ALIGN(1024);
		*(.tim4)
		. = ALIGN(1024);
		*(.tim5)
		. = ALIGN(1024);
		*(.tim6)
		. = ALIGN(1024);
		*(.tim7)
		. = ALIGN(1024);
		*(.tim12)
		. = ALIGN(1024);
		*(.tim13)
		. = ALIGN(1024);
		*(.tim14)
		. = ALIGN(1024);
		. += 1024;		/* LPTIM1 not yet implemented */
		. += 1024;		/* RTC not yet implemented */
		. += 1024;		/* WWDG not yet implemented */
		. += 1024;		/* IWDG not yet implemented */
		. += 1024;		/* CAN3 not yet implemented */
		*(.spi2)
		. = ALIGN(1024);
		*(.spi3)
		. = ALIGN(1024);
		. += 1024;		/* SPDIFRX not yet implemented */
		. = ALIGN(1024);
		*(.usart2)
		. = ALIGN(1024);
		*(.usart3)
		. = ALIGN(1024);
		*(.uart4)
		. = ALIGN(1024);
		*(.uart5)
		. = ALIGN(1024);
		. += 1024;		/* I2C1 not yet implemented */
		. += 1024;		/* I2C2 not yet implemented */
		. += 1024;		/* I2C3 not yet implemented */
		. += 1024;		/* I2C4 not yet implemented */
		. += 1024;		/* CAM1 not yet implemented */
		. += 1024;		/* CAN2 not yet implemented */
		. += 1024;		/* HDMI-CEC not yet implemented */
		. += 1024;		/* PWR not yet implemented */
		. += 1024;		/* DAC not yet implemented */
		*(.uart7)
		. = ALIGN(1024);
		*(.uart8)
		. = ALIGN(1024);
	} > APB1

	.sfr_apb2 :
	{
		*(.tim1)
		. = ALIGN(1024);
		*(.tim8)
		. = ALIGN(1024);
		. += 2048;		/* reserved unimplemented */
		. = ALIGN(1024);
		*(.usart1)
		. = ALIGN(1024);
		*(.usart6)
		. += 1024;		/* reserved unimplemented */
		. += 1024;		/* SDMMC2 not yet implemented */
		. += 1024;		/* ADC not yet implemented */
		. += 2048;		/* reserved unimplemented */
		. += 1024;		/* SDMMC1 not yet implemented */
		. = ALIGN(1024);
		*(.spi1)
		. = ALIGN(1024);
		*(.spi4)
		. = ALIGN(1024);
		*(.syscfg)
		. = ALIGN(1024);
		. += 1024;		/* EXTI not yet implemented */
		*(.tim9)
		. = ALIGN(1024);
		*(.tim10)
		. = ALIGN(1024);
		*(.tim11)
		. = ALIGN(1024);
		. += 1024;		/* reserved unimplemented */
		. = ALIGN(1024);
		*(.spi5)
		. = ALIGN(1024);
		*(.spi6)
		. += 1024;		/* SAI1 not yet implemented */
		. += 1024;		/* SAI2 not yet implemented */
		. += 2048;		/* reserved unimplemented */
		. += 1024;		/* LCD-TFT not yet implemented */
		. += 1024;		/* DSI-host not yet implemented */
		. += 1024;		/* DFSDM1 not yet implemented */
		. += 1024;		/* MDIOS not yet implemented */
	} > APB2

	.sfr_mcu :
	{
		. += 0x42000;	/* skip early address space */
		*(.dbgmcu)
	} > MCU
}
