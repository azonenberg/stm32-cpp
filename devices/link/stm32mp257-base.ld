ENTRY(_start)

MEMORY
{
	/* Actual RAM (boot/secure aliases)*/
	SYSRAM(RWX):			ORIGIN = 0x0e000000, LENGTH = 256K
	SRAM1(RWX):				ORIGIN = 0x0e040000, LENGTH = 128K
	SRAM2(RWX):				ORIGIN = 0x0e060000, LENGTH = 128K
	RETRAM(RWX):			ORIGIN = 0x0e080000, LENGTH = 128K
	VDERAM(RWX):			ORIGIN = 0x0e0a0000, LENGTH = 128K

	/* Secure D-side bus aliases */
	SEC_DSIDE_RETRAM(RW):	ORIGIN = 0x30080000, LENGTH = 128K
	SEC_DSIDE_SRAM2(RWX):	ORIGIN = 0x30060000, LENGTH = 128K
	SEC_DSIDE_SYSRAM(RW):	ORIGIN = 0x30000000, LENGTH = 256K

	/*
		SYSRAM with the first 0x2600 bytes reserved for the boot ROM (where is this documented??)
	 */
	FSBL_SYSRAM(RWX):		ORIGIN = 0x0e002600, LENGTH = 252416

	/* AXI buses */
	/* TODO: AXIM */

	/* APB buses */
	APB1(RW):				ORIGIN = 0x50000000, LENGTH = 2048K
	APB2(RW):				ORIGIN = 0x50200000, LENGTH = 1536K
	APB3(RW):				ORIGIN = 0x54000000, LENGTH = 2048K
	/* TODO: APB4 */
	/* TODO: SmartRun APB */
	/* TODO: A35 debug APB */

	/* AHB buses */
	AHB2(RW):				ORIGIN = 0x50400000, LENGTH = 1025K
	AHB3(RW):				ORIGIN = 0x52000000, LENGTH = 1285K
	AHB4(RW):				ORIGIN = 0x54200000, LENGTH = 960K
	AHB5(RW):				ORIGIN = 0x58200000, LENGTH = 14M
	/* TODO: AHB6 */
	/* TODO: SmartRun AHB */
}

SECTIONS
{
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

	.sfr_apb3  (NOLOAD):
	{
		. = ALIGN(65536);
		KEEP (*(.bsec))
	} > APB3

	.sfr_ahb2  (NOLOAD):
	{
		. = ALIGN(65536);
		. += 1024;	/* HPDMA1 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* HPDMA2 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* HPDMA3 not implemented */
		. = ALIGN(65536);
		KEEP (*(.octospi1))
		. = ALIGN(65536);
		KEEP (*(.octospi2))
		. = ALIGN(65536);
		. += 1024;	/* OTFDEC1 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* OTFDEC2 not implemented */
		. = ALIGN(65536);
		KEEP (*(.icache))
		. = ALIGN(65536);
		KEEP (*(.dcache))
		. = ALIGN(65536);
		KEEP (*(.ipcc1))
		. = ALIGN(65536);
		. += 1024;	/* DCMI not implemented */
		. = ALIGN(65536);
		. += 1024;	/* PSSI not implemented */
		. = ALIGN(65536);
		. += 1024;	/* CRC not implemented */
		. = ALIGN(65536);
		. += 1024;	/* MDF1 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* ADC1/2 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* ADC3 not implemented */
		. = ALIGN(65536);
		KEEP (*(.octospim))
	} > AHB2

	.sfr_ahb3  (NOLOAD):
	{
		. = ALIGN(65536);
		. += 1024;	/* BKPSRAM not implemented */
		. = ALIGN(65536);
		. += 1024;	/* HASH not implemented */
		. = ALIGN(65536);
		. += 1024;	/* RNG not implemented */
		. = ALIGN(65536);
		. += 1024;	/* CRYP1 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* CRYP2 not implemented */
		. = ALIGN(65536);
		. += 1024;	/* SAES not implemented */
		. = ALIGN(65536);
		. += 1024;	/* PKA not implemented */
		. = ALIGN(65536);
		KEEP (*(.ramcfg))
		. = ALIGN(65536);
		. += 1024;	/* RIFSC not implemented */
		. = ALIGN(65536);
		. += 1024;	/* IAC not implemented */
		. = ALIGN(65536);
		KEEP (*(.risaf1))
		. = ALIGN(65536);
		KEEP (*(.risaf2))
		. = ALIGN(65536);
		. += 1024;	/* void space where RISAF3 would be, but there's nothing there */
		. = ALIGN(65536);
		KEEP (*(.risaf4))
		. = ALIGN(65536);
		KEEP (*(.risaf5))
	} > AHB3

	.sfr_ahb4  (NOLOAD):
	{
		. = ALIGN(65536);
		KEEP (*(.rcc))
		. = ALIGN(65536);
		KEEP (*(.pwr))
		. = ALIGN(65536);
		KEEP (*(.exti1))
		. = ALIGN(65536);
		KEEP (*(.syscfg))
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

	.sfr_ahb5  (NOLOAD):
	{
		. = ALIGN(64K);
		. += 1024;	/* FMC not implemented */
		. = ALIGN(128K);
		. += 1024;	/* SDMMC1 not implemented */
		. = ALIGN(64K);
		. += 1024;	/* SDMMC2 not implemented */
		. = ALIGN(64K);
		. += 1024;	/* SDMMC3 not implemented */
		. = ALIGN(256K);
		. += 1024;	/* GPU not implemented */
		. = ALIGN(256K);
		. += 1024;	/* ETH1 not implemented */
		. = ALIGN(64K);
		. += 1024;	/* ETH2 not implemented */
		. = ALIGN(64K);
		. += 1024;	/* USBH_OHCI not implemented */
		. = ALIGN(64K);
		. += 1024;	/* USBH_EHCI not implemented */
		. = ALIGN(1024K);
		. += 1024;	/* USB3DR not implemented */
		. = ALIGN(4096K);
		. += 1024;	/* PCIE not implemented */
		. = ALIGN(4096K);
		KEEP (*(.ca35ss))
	} > AHB5
}
