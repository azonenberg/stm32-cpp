ENTRY(_start)

MEMORY
{
	RETRAM(RWX):			ORIGIN = 0x20080000, LENGTH = 128K

	/* SFRs */
	AHB4(RW):				ORIGIN = 0x54200000, LENGTH = 960K
	APB2(RW):				ORIGIN = 0x50200000, LENGTH = 1536K
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
