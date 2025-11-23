INCLUDE "stm32mp257-base.ld"

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

	/* Initialized data for the RAM load configuration just goes in RETRAM like everything else */
	.data : ALIGN(32)
	{
		__data_romstart = LOADADDR(.data);
		__data_start = .;
		*(.data)
		__data_end = .;
	} > FSBL_SYSRAM

	/* BSS is pretty straightforward */
	.bss  (NOLOAD):
	{
		__bss_start__ = .;
		*(.bss)
		*(.tcmbss)
		__bss_end__ = .;
		*(.bssnoinit)
	} > FSBL_SYSRAM

	/* Stack goes in SYSRAM too, surprising nobody. We have two cores, so allocate two stacks */
	.stack (NOLOAD):
	{
		__end = .;

		/* Blank space between end of globals and stack */

		/* Core 0 gets 16 kB stack for now, starting 16 kB from the end of SYSRAM */
		. = ORIGIN(FSBL_SYSRAM) + LENGTH(FSBL_SYSRAM) - 32768;
		__core0stackend = .;
		. = ORIGIN(FSBL_SYSRAM) + LENGTH(FSBL_SYSRAM) - 16384 - 4;
		__core0stackstart = .;

		/* Core 1 gets the rest */
		. = ORIGIN(FSBL_SYSRAM) + LENGTH(FSBL_SYSRAM) - 16384;
		__core1stackend = .;
		. = ORIGIN(FSBL_SYSRAM) + LENGTH(FSBL_SYSRAM) - 4;
		__core1stackstart = .;
	} > FSBL_SYSRAM
}
