ENTRY(_start)

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
	} > AXISRAM AT> FLASH

	/* Stack goes in TCM to keep it fast. We probably won't be DMAing etc to/from it. */
	.stack  (NOLOAD):
	{
		__end = .;
		. = ORIGIN(DTCM) + LENGTH(DTCM) - 4;
		__stack = .;
	} > DTCM

	/* BSS is pretty straightforward */
	.bss  (NOLOAD):
	{
		__bss_start__ = .;
		*(.bss)
		__bss_end__ = .;
	} > AXISRAM

	/* Fill all remaining AXI SRAM space with the heap (which we don't actually use) */
	.heap  (NOLOAD):
	{
		. = ALIGN(4);
		__heap_start = .;
		. = ORIGIN(AXISRAM) + LENGTH(AXISRAM) - 4;
		__heap_end = .;
		PROVIDE(end = .);
	} > AXISRAM

	/* SFRs */
	.sfr_apb1  (NOLOAD):
	{
		. = ALIGN(1024);
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
		. += 1024;			/* LPTIM1 not implemented */
		. += 4096;			/* 4000_2800 - 4000_37ff reserved */
		. = ALIGN(1024);
		*(.spi2)
		. = ALIGN(1024);
		*(.spi3)
		. += 1024;			/* SPDIFRX1 not implemented */
		. = ALIGN(1024);
		*(.usart2)
		. = ALIGN(1024);
		*(.usart3)
		. = ALIGN(1024);
		*(.uart4)
		. = ALIGN(1024);
		*(.uart5)
		. = ALIGN(1024);
		*(.i2c1)
		. = ALIGN(1024);
		*(.i2c2)
		. = ALIGN(1024);
		*(.i2c3)
		/* I2C4 is elsewhere */
		. = ALIGN(1024);
		*(.i2c5)
	} > APB1

	.sfr_apb2  (NOLOAD):
	{
		. += 1024;			/* TIM1 not implemented */
		. += 1024;			/* TIM8 not implemented */
		. += 2048;			/* 40010800 - 40010fff reserved */
		. += 1024;			/* USART1 not implemented */
		. += 1024;			/* USART6 not implemented */
		. += 1024;			/* UART9 not implemented */
		. += 1024;			/* USART10 not implemented */
		. += 4096;			/* 40012000 - 40012fff reserved */
		. = ALIGN(1024);
		*(.spi1)
		. = ALIGN(1024);
		*(.spi4)
		. += 2048;			/* 40013800 - 40013fff reserved */
		. += 1024;			/* TIM15 not implemented */
		. += 1024;			/* TIM16 not implemented */
		. += 1024;			/* TIM17 not implemented */
		. += 1024;			/* 40014c00 - 40014fff reserved */
		. = ALIGN(1024);
		*(.spi5)
	} > APB2

	.sfr_ahb2  (NOLOAD):
	{
		. += 1024;			/* DCMI not implemented */
		. += 1024;			/* PSSI not implemented */
		. += 2048;			/* reserved */
		. = ALIGN(1024);
		*(.cryp)
		. = ALIGN(1024);
		*(.chash)
		. = ALIGN(1024);
		*(.rng)
	} > AHB2

	.sfr_ahb3  (NOLOAD):
	{
		. += 1048576;		/* GPV not implemented */
		. += 15728640;		/* 5110_0000 - 51ff_ffff reserved */
		. += 4096;			/* MDMA not implemented */
		. += 4096;			/* DMA2D not implemented */
		. = ALIGN(1024);
		*(.flash)
		. = ALIGN(1024);
		. += 11264;
		*(.octospi1);
		. = ALIGN(4096);
		. += 4096;			/* DLYB not implemented */
		. += 4096;			/* SDMMC1 not implemented */
		. += 4096;			/* DLYB not implemented */
		. += 1024;			/* RAMECC not implemented */
		. += 3072;			/* 5200_9400 - 5200_9fff reserved */
		*(.octospi2);
		. = ALIGN(4096);
		. += 1024;			/* DLYB not implemented */
		*(.octospim);
		/* TODO: OTFDEC */
	} > AHB3

	.sfr_ahb4  (NOLOAD):
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
		. += 1024;		/* no GPIOI */
		. = ALIGN(1024);
		*(.gpioj)
		. = ALIGN(1024);
		*(.gpiok)

		. = ALIGN(1024);
		. += 6144;		/* reserved 5802_8000 */

		. = ALIGN(1024);
		*(.rcc)
		. = ALIGN(1024);
		*(.pwr)
		. = ALIGN(1024);
		*(.crc)

		/*
			TODO: BDMA, DMAMUX2, ADC3, HSEM
		 */
	} > AHB4

	.sfr_apb4  (NOLOAD):
	{
		. += 1024;	/* EXTI not yet implemented */
		. = ALIGN(1024);
		*(.syscfg)
		. = ALIGN(1024);
		. += 1024;	/* reserved 5800_0800 */
		. += 1024;	/* LPUART1 not implemented */
		. += 1024;	/* reserved 5800_1000 */
		. = ALIGN(1024);
		*(.spi6)
		. += 1024;	/* reserved 5800_1800 */
		. = ALIGN(1024);
		*(.i2c4);
		. = ALIGN(1024);
		. += 1024;	/* reserved 5800_2000 */
		. = ALIGN(1024);
		. += 1024;	/* LPTIM2 not yet implemented */
		. = ALIGN(1024);
		. += 1024;	/* LPTIM3 not yet implemented */
		. = ALIGN(1024);
		. += 1024;	/* LPTIM4 not yet implemented */
		. = ALIGN(1024);
		. += 1024;	/* LPTIM5 not yet implemented */
		. = ALIGN(1024);
		. += 1024;	/* reserved 5800_3400 */
		. = ALIGN(1024);
		. += 1024;	/* COMP1 not yet implemented */
		. = ALIGN(1024);
		. += 1024;	/* VREF not yet implemented */
		. = ALIGN(1024);
		*(.rtc);
		. = ALIGN(1024);
		. += 1024;	/* reserved 5800_4400 */
		. = ALIGN(1024);
		. += 1024;	/* IWDG not yet implemented */
		. = ALIGN(1024);
		. += 2048;	/* reserved 5800_4c00 */
		. = ALIGN(1024);
		. += 1024;	/* SAI4 not yet implemented */
		. = ALIGN(1024);
		. += 4096;	/* reserved 5800_5800 */
		. = ALIGN(1024);
		*(.dts);
		. = ALIGN(1024);

		/*
			TODO: LPTIM2, LPTIM3, LPTIM4, LPTIM5, COMP1/2, VREF, RTC/BKP, IWDG, reserved, SAI4, DTS
		 */
	} > APB4

	.sfr_mcu  (NOLOAD):
	{
		. += 0xed00;
		*(.scb)
		. += 0x4;
		*(.cpuid)
		. = ALIGN(1024);
	} > MCU

	.sfr_mcu2  (NOLOAD):
	{
		*(.dbgmcu)
	} > MCU2

	.sfr_sig  (NOLOAD):
	{
		. += 0x1e800;
		*(.uid)
		. += 0x74;
		*(.fid)
		. += 0x3c;
		*(.lid)
	} > SIG
}
