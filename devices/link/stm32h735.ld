MEMORY
{
	FLASH(RX):			ORIGIN = 0x08000000, LENGTH = 1M

	/* Factory default: all memory allocated to AXISRAM_EXT, no ITCM_EXT usable */
	ITCM(RWX):			ORIGIN = 0x00000000, LENGTH = 64K

	DTCM(RW):			ORIGIN = 0x20000000, LENGTH = 128K

	/* with TCM_AXI_SHARED=2'b00 AXI SRAM is 320K */
	AXISRAM(RW):		ORIGIN = 0x24000000, LENGTH = 320K
}

INCLUDE "stm32h735-base.ld"
