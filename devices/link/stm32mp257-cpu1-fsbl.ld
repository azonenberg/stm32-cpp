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

	/* Stack goes in SYSRAM too, surprising nobody */
	.stack (NOLOAD):
	{
		__end = .;
		. = ORIGIN(SYSRAM) + LENGTH(SYSRAM) - 4;
		__stack = .;
	} > FSBL_SYSRAM
}
