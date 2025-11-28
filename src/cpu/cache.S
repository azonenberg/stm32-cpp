////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AARCH64 path

#ifdef __aarch64__

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ARM-M path

#else

.text
.thumb
.syntax unified

.equ ICIALLU,	0xe000ef50
.equ CCR,		0xe000ed14
.equ CCR_IC,	0x20000
.equ CCR_DC,	0x10000
.equ CCSIDR,	0xe000ed80
.equ CCSELR,	0xe000ed84
.equ DCISW,		0xe000ef60

//reference: Cortex-M7 TRM section 4

.globl InvalidateInstructionCache
.type InvalidateInstructionCache, %function
InvalidateInstructionCache:
	mov		r0, #0
	ldr		r11, =ICIALLU
	str		r0, [r11]

	dsb
	isb

	bx		lr

.globl DisableInstructionCache
.type DisableInstructionCache, %function
DisableInstructionCache:
	ldr		r11, =CCR
	ldr		r0, [r11]
	ldr		r1, =CCR_IC
	mvns	r1, r1
	and		r0, r0, r1
	str		r0, [r11]

	dsb
	isb
	bx		lr

.globl EnableInstructionCache
.type EnableInstructionCache, %function
EnableInstructionCache:
	ldr		r11, =CCR
	ldr		r0, [r11]
	ldr		r1, =CCR_IC
	orr		r0, r0, r1
	str		r0, [r11]

	dsb
	isb
	bx		lr

.globl DisableDataCache
.type DisableDataCache, %function
DisableDataCache:
	ldr		r11, =CCR
	ldr		r0, [r11]
	ldr		r1, =CCR_DC
	mvns	r1, r1
	and		r0, r0, r1
	str		r0, [r11]

	dsb
	isb
	bx		lr

.globl EnableDataCache
.type EnableDataCache, %function
EnableDataCache:
	ldr		r11, =CCR
	ldr		r0, [r11]
	ldr		r1, =CCR_DC
	orr		r0, r0, r1
	str		r0, [r11]

	dsb
	isb
	bx		lr

//TODO can we reimplement this in C?
.globl InvalidateDataCache
.type InvalidateDataCache, %function
InvalidateDataCache:

	//select L1D$
	mov		r0, #0
	ldr		r11, =CCSELR
	str		r0, [r11]
	dsb

	ldr		r11, =CCSIDR
	ldr		r2, [r11]
	and		r1, r2, #7
	add		r7, r1, #4
	ubfx	r4, r2, #3, #10
	ubfx	r2, r2, #13, #15
	clz		r6, r4
	ldr		r11, =DCISW

	//for each set
inv_loop1:
	mov		r1, r4
	lsls	r8, r2, r7

	//for each way
inv_loop2:
	lsls	r3, r1, r6
	orrs	r3, r3, r8
	str		r3, [r11]
	subs	r1, r1, #1
	bge		inv_loop2

	subs	r2, r2, 1
	bge		inv_loop1

	dsb
	isb
	bx		lr

#endif
