ENTRY(_start)

MEMORY
{
	SYSFLASH(RX):			ORIGIN = 0x1ff00000, LENGTH = 128K

	SRAM1(RW):				ORIGIN = 0x30000000, LENGTH = 16K
	SRAM2(RW):				ORIGIN = 0x30004000, LENGTH = 16K
	/* no SRAM3 */
	SRAM4(RW):				ORIGIN = 0x38000000, LENGTH = 16K
	BACKUPSRAM(RW):			ORIGIN = 0x38800000, LENGTH = 4K

	/* SFRs */
	AHB4(RW):				ORIGIN = 0x58020000, LENGTH = 26K
	APB4(RW): 				ORIGIN = 0x58000000, LENGTH = 27K
	AHB3(RW):				ORIGIN = 0x51000000, LENGTH = 16432K
	APB3(RW):				ORIGIN = 0x50000000, LENGTH = 16K
	AHB2(RW):				ORIGIN = 0x48020000, LENGTH = 18K
	AHB1(RW):				ORIGIN = 0x40020000, LENGTH = 384K
	APB2(RW):				ORIGIN = 0x40010000, LENGTH = 32K
	APB1(RW):				ORIGIN = 0x40000000, LENGTH = 58K
	MCU(RW):				ORIGIN = 0xe0000000, LENGTH = 1024K
	MCU2(RW):				ORIGIN = 0x5c001000, LENGTH = 1024K
	SIG(RW):				ORIGIN = 0x1ff00000, LENGTH = 1024K

	/* External memory regions */
	FMC_BANK1(RW):			ORIGIN = 0x60000000, LENGTH = 256M
	FMC_NAND(RW):			ORIGIN = 0x80000000, LENGTH = 256M
	FMC_BANK1_DEVICE(RW):	ORIGIN = 0xc0000000, LENGTH = 64M
	OCTOSPI1(RW):			ORIGIN = 0x90000000, LENGTH = 256M
	OCTOSPI2(RW):			ORIGIN = 0x70000000, LENGTH = 256M
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

		/* TODO: consider not emitting global destructors since they're never called */
		__dtor_start = .;
		*(.fini_array)
		__dtor_end = .;
	} > FLASH

	/*
		Hot functions designated by user go in ITCM

		Also needs to be aligned on a flash write block boundary to avoid problems when flashing targets
	 */
	.tcmtext : ALIGN(32)
	{
		__itcm_romstart = LOADADDR(.tcmtext);
		__itcm_start = .;

		/* DEBUG: block off first 256 bytes of ITCM since it's mapped at 0 and we want to catch null derefs */
		. += 256;

		*(.tcmtext)
		__itcm_end = .;
	} > ITCM AT> FLASH

	/*
		Initialized data needs special handling because it lives in two places

		Also needs to be aligned on a flash write block boundary to avoid problems when flashing targets
	 */
	.data : ALIGN(32)
	{
		__data_romstart = LOADADDR(.data);
		__data_start = .;
		*(.data)
		__data_end = .;
	} > AXISRAM AT> FLASH

	/* Stack goes in TCM to keep it fast. We probably won't be DMAing etc to/from it. */
	.tcmbss (NOLOAD):
	{
		__tcmstart = .;
		*(.tcmbss)
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
		KEEP (*(.tim2))
		. = ALIGN(1024);
		KEEP (*(.tim3))
		. = ALIGN(1024);
		KEEP (*(.tim4))
		. = ALIGN(1024);
		KEEP (*(.tim5))
		. = ALIGN(1024);
		KEEP (*(.tim6))
		. = ALIGN(1024);
		KEEP (*(.tim7))
		. = ALIGN(1024);
		KEEP (*(.tim12))
		. = ALIGN(1024);
		KEEP (*(.tim13))
		. = ALIGN(1024);
		KEEP (*(.tim14))
		. = ALIGN(1024);
		. += 1024;			/* LPTIM1 not implemented */
		. += 4096;			/* 4000_2800 - 4000_37ff reserved */
		. = ALIGN(1024);
		KEEP (*(.spi2))
		. = ALIGN(1024);
		KEEP (*(.spi3))
		. += 1024;			/* SPDIFRX1 not implemented */
		. = ALIGN(1024);
		KEEP (*(.usart2))
		. = ALIGN(1024);
		KEEP (*(.usart3))
		. = ALIGN(1024);
		KEEP (*(.uart4))
		. = ALIGN(1024);
		KEEP (*(.uart5))
		. = ALIGN(1024);
		KEEP (*(.i2c1))
		. = ALIGN(1024);
		KEEP (*(.i2c2))
		. = ALIGN(1024);
		KEEP (*(.i2c3))
		/* I2C4 is elsewhere */
		. = ALIGN(1024);
		KEEP (*(.i2c5))
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
		KEEP (*(.spi1))
		. = ALIGN(1024);
		KEEP (*(.spi4))
		. += 2048;			/* 40013800 - 40013fff reserved */
		. += 1024;			/* TIM15 not implemented */
		. += 1024;			/* TIM16 not implemented */
		. += 1024;			/* TIM17 not implemented */
		. += 1024;			/* 40014c00 - 40014fff reserved */
		. = ALIGN(1024);
		KEEP (*(.spi5))
	} > APB2

	.sfr_ahb2  (NOLOAD):
	{
		. += 1024;			/* DCMI not implemented */
		. += 1024;			/* PSSI not implemented */
		. += 2048;			/* reserved */
		. = ALIGN(1024);
		KEEP (*(.cryp))
		. = ALIGN(1024);
		KEEP (*(.chash))
		. = ALIGN(1024);
		KEEP (*(.rng))
	} > AHB2

	.sfr_ahb3  (NOLOAD):
	{
		. += 1048576;		/* GPV not implemented */
		. += 15728640;		/* 5110_0000 - 51ff_ffff reserved */
		. = ALIGN(4096);
		KEEP (*(.mdma))
		. = ALIGN(4096);
		. += 4096;			/* DMA2D not implemented */
		. = ALIGN(1024);
		KEEP (*(.flash))
		. += 4096;			/* 5200_3000 to 5200_3fff reserved */
		. = ALIGN(4096);
		KEEP (*(.fmc))
		. = ALIGN(4096);
		KEEP (*(.octospi1));
		. = ALIGN(4096);
		. += 4096;			/* DLYB not implemented */
		. += 4096;			/* SDMMC1 not implemented */
		. += 4096;			/* DLYB not implemented */
		. += 1024;			/* RAMECC not implemented */
		. += 3072;			/* 5200_9400 - 5200_9fff reserved */
		KEEP (*(.octospi2));
		. = ALIGN(4096);
		. += 1024;			/* DLYB not implemented */
		KEEP (*(.octospim));
		/* TODO: OTFDEC */
	} > AHB3

	.sfr_ahb4  (NOLOAD):
	{
		. = ALIGN(1024);
		KEEP (*(.gpioa))
		. = ALIGN(1024);
		KEEP (*(.gpiob))
		. = ALIGN(1024);
		KEEP (*(.gpioc))
		. = ALIGN(1024);
		KEEP (*(.gpiod))
		. = ALIGN(1024);
		KEEP (*(.gpioe))
		. = ALIGN(1024);
		KEEP (*(.gpiof))
		. = ALIGN(1024);
		KEEP (*(.gpiog))
		. = ALIGN(1024);
		KEEP (*(.gpioh))
		. = ALIGN(1024);
		. += 1024;		/* no GPIOI */
		. = ALIGN(1024);
		KEEP (*(.gpioj))
		. = ALIGN(1024);
		KEEP (*(.gpiok))

		. = ALIGN(1024);
		. += 6144;		/* reserved 5802_8000 */

		. = ALIGN(1024);
		KEEP (*(.rcc))
		. = ALIGN(1024);
		KEEP (*(.pwr))
		. = ALIGN(1024);
		KEEP (*(.crc))

		/*
			TODO: BDMA, DMAMUX2, ADC3, HSEM
		 */
	} > AHB4

	.sfr_apb4  (NOLOAD):
	{
		. += 1024;	/* EXTI not yet implemented */
		. = ALIGN(1024);
		KEEP (*(.syscfg))
		. = ALIGN(1024);
		. += 1024;	/* reserved 5800_0800 */
		. += 1024;	/* LPUART1 not implemented */
		. += 1024;	/* reserved 5800_1000 */
		. = ALIGN(1024);
		KEEP (*(.spi6))
		. += 1024;	/* reserved 5800_1800 */
		. = ALIGN(1024);
		KEEP (*(.i2c4));
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
		KEEP (*(.rtc));
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
		KEEP (*(.dts));
		. = ALIGN(1024);

		/*
			TODO: LPTIM2, LPTIM3, LPTIM4, LPTIM5, COMP1/2, VREF, RTC/BKP, IWDG, reserved, SAI4, DTS
		 */
	} > APB4

	.sfr_mcu  (NOLOAD):
	{
		KEEP (*(.itm))
		. = ALIGN(4096);
		KEEP (*(.dwt))
		. = ALIGN(4096);
		. += 0xcd00;
		KEEP (*(.scb))
		. += 0x4;
		KEEP (*(.cpuid))
		. += 0x18;
		KEEP (*(.mpu))
		. += 48;
		KEEP (*(.demcr))
		. = ALIGN(4096);
	} > MCU

	.sfr_mcu2  (NOLOAD):
	{
		KEEP (*(.dbgmcu))
		. = ALIGN(4096);
		. += 0x13000;
		KEEP (*(.tpiu))
	} > MCU2

	.sfr_sig  (NOLOAD):
	{
		. += 0x1e800;
		KEEP (*(.uid))
		. += 0x74;
		KEEP (*(.fid))
		. += 0x3c;
		KEEP (*(.lid))
	} > SIG
}
