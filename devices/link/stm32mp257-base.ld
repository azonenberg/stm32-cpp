ENTRY(_start)

MEMORY
{
	/* Actual RAM */
	RETRAM(RWX):			ORIGIN = 0x20080000, LENGTH = 128K

	/* AXI buses */
	/* TODO: AXIM */

	/* APB buses */
	APB1(RW):				ORIGIN = 0x40000000, LENGTH = 2048K
	APB2(RW):				ORIGIN = 0x40200000, LENGTH = 1536K
	/* TODO: APB3 */
	/* TODO: APB4 */
	/* TODO: SmartRun APB */
	/* TODO: A35 debug APB */

	/* AHB buses */
	/* TODO: AHB2 */
	/* TODO: AHB3 */
	AHB4(RW):				ORIGIN = 0x44200000, LENGTH = 960K
	/* TODO: AHB5 */
	/* TODO: AHB6 */
	/* TODO: SmartRun AHB */
}

SECTIONS
{
	/* SFRs */
	.sfr_ahb4  (NOLOAD):
	{
		. = ALIGN(65536);
		KEEP (*(.rcc))
		. = ALIGN(65536);
		KEEP (*(.pwr))
		. = ALIGN(65536);
		. += 1024;			/* EXTI1 not implemented */
		. = ALIGN(65536);
		. += 1024;			/* SYSCFG not implemented */
		. = ALIGN(65536);
		KEEP (*(.gpioa))
		. = ALIGN(65536);
		KEEP (*(.gpiob))
		. = ALIGN(65536);
		KEEP (*(.gpioc))
		. = ALIGN(65536);
		KEEP (*(.gpiod))
		. = ALIGN(65536);
		KEEP (*(.gpioe))
		. = ALIGN(65536);
		KEEP (*(.gpiof))
		. = ALIGN(65536);
		KEEP (*(.gpiog))
		. = ALIGN(65536);
		KEEP (*(.gpioh))
		. = ALIGN(65536);
		KEEP (*(.gpioi))
		. = ALIGN(65536);
		KEEP (*(.gpioj))
		. = ALIGN(65536);
		KEEP (*(.gpiok))
	} > AHB4

	.sfr_apb1  (NOLOAD):
	{
		. = ALIGN(65536);
		KEEP (*(.tim2))
		. = ALIGN(65536);
		KEEP (*(.tim3))
		. = ALIGN(65536);
		KEEP (*(.tim4))
		. = ALIGN(65536);
		KEEP (*(.tim5))
		. = ALIGN(65536);
		KEEP (*(.tim6))
		. = ALIGN(65536);
		KEEP (*(.tim7))
		. = ALIGN(65536);
		KEEP (*(.tim12))
		. = ALIGN(65536);
		KEEP (*(.tim13))
		. = ALIGN(65536);
		KEEP (*(.tim14))
		. = ALIGN(65536);
		. += 1024; /* LPTIM1 not implemented */
		. = ALIGN(65536);
		. += 1024; /* LPTIM2 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SPI2 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SPI3 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SPDIFRX not implemented */
		. = ALIGN(65536);
		KEEP (*(.usart2))
		. = ALIGN(65536);
		KEEP (*(.usart3))
		. = ALIGN(65536);
		KEEP (*(.uart4))
		. = ALIGN(65536);
		KEEP (*(.uart5))
		. = ALIGN(65536);
		KEEP (*(.i2c1))
		. = ALIGN(65536);
		KEEP (*(.i2c2))
		. = ALIGN(65536);
		KEEP (*(.i2c3))
		. = ALIGN(65536);
		KEEP (*(.i2c4))
		. = ALIGN(65536);
		KEEP (*(.i2c5))
		. = ALIGN(65536);
		KEEP (*(.i2c6))
		. = ALIGN(65536);
		KEEP (*(.i2c7))
		. = ALIGN(65536);
		. += 1024; /* I3C1 not implemented */
		. = ALIGN(65536);
		. += 1024; /* I3C2 not implemented */
		. = ALIGN(65536);
		. += 1024; /* I3C3 not implemented */
		. = ALIGN(65536);
		KEEP (*(.tim10))
		. = ALIGN(65536);
		KEEP (*(.tim11))
	} > APB1

	.sfr_apb2  (NOLOAD):
	{
		. = ALIGN(65536);
		KEEP (*(.tim1))
		. = ALIGN(65536);
		KEEP (*(.tim8))
		. = ALIGN(65536);
		KEEP (*(.usart6))
		. = ALIGN(65536);
		. += 1024; /* SPI1 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SPI4 not implemented */
		. = ALIGN(65536);
		KEEP (*(.tim15))
		. = ALIGN(65536);
		KEEP (*(.tim16))
		. = ALIGN(65536);
		KEEP (*(.tim17))
		. = ALIGN(65536);
		. += 1024; /* SPI5 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SAI1 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SAI2 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SAI3 not implemented */
		. = ALIGN(65536);
		KEEP (*(.uart9))
		. = ALIGN(65536);
		. += 1024; /* FDCAN1 not implemented */
		. = ALIGN(65536);
		. += 1024; /* FDCAN2 not implemented */
		. = ALIGN(65536);
		. += 1024; /* FDCAN3 not implemented */
		. = ALIGN(65536);
		. += 1024; /* CCU not implemented */
		. = ALIGN(65536);
		. += 1024; /* FDCANRAM not implemented */
		. = ALIGN(65536);
		KEEP (*(.tim20))
		. = ALIGN(65536);
		KEEP (*(.usart1))
		. = ALIGN(65536);
		. += 1024; /* SAI4 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SPI6 not implemented */
		. = ALIGN(65536);
		. += 1024; /* SPI7 not implemented */
		. = ALIGN(65536);
		KEEP (*(.uart7))
		. = ALIGN(65536);
		KEEP (*(.uart8))
	} > APB2
}
