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
	MCU(RW):	ORIGIN = 0xe0000000, LENGTH = 512K
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
		. += 1024;		/* PWR not yet implemented */
		. = ALIGN(1024);
		. += 1024;		/* CRC not yet implemented */

		/*
			TODO: BDMA, DMAMUX2, ADC3, HSEM
		 */
	} > AHB4

	.sfr_apb4 :
	{
		/*
			TODO: EXTI, SYSCFG, LPUART1, SPI/I2S6, I2C4, LPTIM2, LPTIM3, LPTIM4, LPTIM5, COMP1/2, VREF, RTC/BKP, IWDG,
			reserved, SAI4, DTS
		 */
	} > APB4

	.sfr_mcu :
	{
		/*
		. += 0xed00;
		*(.scb)
		. += 0x4;
		*(.cpuid)
		. = ALIGN(1024);
		. += 0x33000;
		*(.dbgmcu)
		*/
	} > MCU

	.sfr_sig :
	{
		/*
		. += 0xf420;
		*(.uid)
		. += 0x16;
		*(.fid)
		. += 0xe87ac;
		*(.pkg)
		*/
	} > SIG
}
