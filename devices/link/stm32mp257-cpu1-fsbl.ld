INCLUDE "stm32mp257-base.ld"

SECTIONS
{
	.resetptr : SUBALIGN(512)
	{
		/* Add jump to reset vector */
		KEEP (*(.resetptr))

		/* Padding to align .text to a 2K boundary */
		. = ALIGN(2048);

	} > FSBL_SYSRAM

	/* Code comes right after the vector tables and is aligned */
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
		KEEP(*(.init.*))
		KEEP(*(.ARM.exidx))

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
	} > FSBL_SYSRAM

	/*
		Dummy section since the linker doesn't understand SYSRAM and SEC_DSIDE_SYSRAM are the same memory
		so we need to block out the space in SEC_DSIDE_SYSRAM occupied by .text
	 */
	.padding (NOLOAD):
	{
		. += 0x2600;
		. += SIZEOF(.resetptr);
		. += SIZEOF(.text);
	} > SEC_DSIDE_SYSRAM

	/* Initialized data for the RAM load configuration just goes in SYSRAM like everything else */
	.data : ALIGN(32)
	{
		__data_romstart = LOADADDR(.data);
		__data_start = .;
		*(.data)
		*(.got)
		*(.got.*)
		__data_end = .;
	} > SEC_DSIDE_SYSRAM AT> FSBL_SYSRAM

	/* BSS is pretty straightforward */
	.bss  (NOLOAD):
	{
		__bss_start__ = .;
		*(.bss)
		*(.tcmbss)
		__bss_end__ = .;
		*(.bssnoinit)
	} > SEC_DSIDE_SYSRAM

	/* Stack goes in SYSRAM too, surprising nobody. We have two cores, so allocate two stacks */
	.stack (NOLOAD):
	{
		__end = .;

		/* Blank space between end of globals and stack */

		/* Core 0 gets 16 kB stack for now, starting 16 kB from the end of SYSRAM */
		. = ORIGIN(SEC_DSIDE_SYSRAM) + LENGTH(SEC_DSIDE_SYSRAM) - 32768;
		__core0stackend = .;
		. = ORIGIN(SEC_DSIDE_SYSRAM) + LENGTH(SEC_DSIDE_SYSRAM) - 16384;
		__core0stackstart = .;

		/* Core 1 gets the rest */
		. = ORIGIN(SEC_DSIDE_SYSRAM) + LENGTH(SEC_DSIDE_SYSRAM) - 16384;
		__core1stackend = .;
		. = ORIGIN(SEC_DSIDE_SYSRAM) + LENGTH(SEC_DSIDE_SYSRAM);
		__core1stackstart = .;
	} > SEC_DSIDE_SYSRAM

	/* IPC buffers live in SRAM2 */
	.ipcbuf (NOLOAD):
	{
		/* At the very start we put the IPC channel descriptors */
		*(.ipcdescriptors)

		/* General purpose IPC buffers come after the descriptors*/
		*(.ipcbuf)
	} > SEC_DSIDE_SRAM2
}
