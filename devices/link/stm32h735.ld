ENTRY(_start)

MEMORY
{
	SYSFLASH(RX):		ORIGIN = 0x1ff00000, LENGTH = 128K
	FLASH(RX):			ORIGIN = 0x08000000, LENGTH = 1M
	ITCM(RWX):			ORIGIN = 0x00000000, LENGTH = 64K

	/*
		Same physical memory as AXISRAM_EXT, split controlled by TCM_AXI_SHARED option byte
		Factory default: all memory allocated to AXISRAM_EXT, no ITCM_EXT usable
	 */
	ITCM_EXT(RWX):		ORIGIN = 0x00010000, LENGTH = 192K

	DTCM(RW):			ORIGIN = 0x20000000, LENGTH = 128K

	AXISRAM(RW):		ORIGIN = 0x24000000, LENGTH = 128K

	/*
		same physical memory as ITCM_EXT, split controlled by TCM_AXI_SHARED option byte
		Factory default: all memory allocated to AXISRAM_EXT, no ITCM_EXT usable
	 */
	AXISRAM_EXT(RW):	ORIGIN = 0x24020000, LENGTH = 192K

	SRAM1(RW):			ORIGIN = 0x30000000, LENGTH = 16K
	SRAM2(RW):			ORIGIN = 0x30004000, LENGTH = 16K
	/* no SRAM3 */
	SRAM4(RW):			ORIGIN = 0x38000000, LENGTH = 16K
	BACKUPSRAM(RW):		ORIGIN = 0x38800000, LENGTH = 4K

	/* SFRs */
	AHB4(RW):	ORIGIN = 0x58020000, LENGTH = 26K
	APB4(RW): 	ORIGIN = 0x58000000, LENGTH = 26K
	AHB3(RW):	ORIGIN = 0x51000000, LENGTH = 16432K
	APB3(RW):	ORIGIN = 0x50000000, LENGTH = 16K
	AHB2(RW):	ORIGIN = 0x48020000, LENGTH = 18K
	AHB1(RW):	ORIGIN = 0x40020000, LENGTH = 384K
	APB2(RW):	ORIGIN = 0x40010000, LENGTH = 32K
	APB1(RW):	ORIGIN = 0x40000000, LENGTH = 58K
	MCU(RW):	ORIGIN = 0xe0000000, LENGTH = 1024K
	MCU2(RW):	ORIGIN = 0x5c001000, LENGTH = 4K
	SIG(RW):	ORIGIN = 0x1ff00000, LENGTH = 1024K
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
	} > AXISRAM AT> FLASH

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
	} > AXISRAM

	/* Fill all remaining AXI SRAM space with the heap */
	.heap :
	{
		. = ALIGN(4);
		__heap_start = .;
		. = ORIGIN(AXISRAM) + LENGTH(AXISRAM) - 4;
		__heap_end = .;
		PROVIDE(end = .);
	} > AXISRAM

	/* SFRs */
	.sfr_apb1 :
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
		. += 1024;			/* SPI2 not implemented */
		. += 1024;			/* SPI3 not implemented */
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

	.sfr_ahb3 :
	{
		. += 1048576;		/* GPV not implemented */
		. += 15728640;		/* 5110_0000 - 51ffffff reserved */
		. += 4096;			/* MDMA not implemented */
		. += 4096;			/* DMA2D not implemented */
		. = ALIGN(1024);
		*(.flash)
	} > AHB3

	.sfr_ahb4 :
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
		. += 1024;		/* CRC not yet implemented */

		/*
			TODO: BDMA, DMAMUX2, ADC3, HSEM
		 */
	} > AHB4

	.sfr_apb4 :
	{
		. += 1024;	/* EXTI not yet implemented */
		. = ALIGN(1024);
		*(.syscfg)
		. = ALIGN(1024);
		. += 1024;	/* reserved 5800_0800 */
		. += 1024;	/* LPUART1 not implemented */
		. += 1024;	/* reserved 5800_1000 */
		. += 1024;	/* SPI/I2S6 not implemented */
		. += 1024;	/* reserved 5800_1800 */
		. = ALIGN(1024);
		*(.i2c4);

		/*
			TODO: LPTIM2, LPTIM3, LPTIM4, LPTIM5, COMP1/2, VREF, RTC/BKP, IWDG, reserved, SAI4, DTS
		 */
	} > APB4

	.sfr_mcu :
	{
		. += 0xed00;
		*(.scb)
		. += 0x4;
		*(.cpuid)
		. = ALIGN(1024);
	} > MCU

	.sfr_mcu2 :
	{
		*(.dbgmcu)
	} > MCU2

	.sfr_sig :
	{
		. += 0x1e800;
		*(.uid)
		. += 0x74;
		*(.fid)
		. += 0x3c;
		*(.lid)
	} > SIG
}
