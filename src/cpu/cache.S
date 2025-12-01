////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AARCH64 path

#ifdef __aarch64__

//x0 = addr
//x1 = size
//for now assumes 64 byte cache line
.globl CleanDataCache
.type CleanDataCache, %function
CleanDataCache:

	//Commit any pending writes to cache
	dmb		st

	//End address
	add		x3, x0, x1

	//Round start address to cache line
	mov		x4, #0x3f
	neg		x4, x4
	and		x0, x0, x4

cleanloop:
	dc		ivac, x0
	add		x0, x0, #64
	cmp		x0, x3
	blt		cleanloop

	br		x30

.globl EnableDataCache
.type EnableDataCache, %function
EnableDataCache:

	mrs		x0, SCTLR_EL3
	mov		x1, #4
	orr		x0, x0, x1
	msr		SCTLR_EL3, x0
	isb

	br		x30

.globl EnableInstructionCache
.type EnableInstructionCache, %function
EnableInstructionCache:

	mrs		x0, SCTLR_EL3
	mov		x1, #0x1000
	orr		x0, x0, x1
	msr		SCTLR_EL3, x0
	isb

	br		x30

.globl InvalidateInstructionCache
.type InvalidateInstructionCache, %function
InvalidateInstructionCache:
	ic		iallu
	br		x30

.globl InvalidateDataCache
.type InvalidateDataCache, %function
InvalidateDataCache:

	//TODO: handle arbitrarily large caches
	//For now this is hard coded for the STM32MP2 Cortex-A35

	//Don't bother using the stack to save our return address, just shove it in a register
	mov		x19, x30

	//Flush the L1
	mov		x0, #128	//128 sets
	mov		x1, #4		//4-way associative
	mov		x2, #0		//level 1
	mov		x3, 6		//log(linelen)
	mov		x4, 2		//log(nways)
	bl		InvalidateDataOrUnifiedCache

	//Flush the L2
	mov		x0, #1024	//1024 sets
	mov		x1, #8		//8-way associative
	mov		x2, #1		//level 2
	mov		x3, 6		//log(linelen)
	mov		x4, 3		//log(nways)
	bl		InvalidateDataOrUnifiedCache

	//Just jump straight to the saved return address, no reason to move it back to x30 first
	//(will this confuse branch predictors or something?)
	br		x19

//Clear the D-side L1 cache
//x0 = number of sets
//x1 = number of ways
//x2 = cache level minus 1
//x3 = log(linelen)
//x4 = log(nways)
InvalidateDataOrUnifiedCache:

	//Select the cache level we're flushing
	//Pre-shift it left by one bit since "dc isw" wants the level in bits 3:1
	lsl		x2, x2, #1

	//x4 = 32 - log(nways)
	mov		x6, #32
	sub		x4, x6, x4

	//Current cache set, x6. ranges from x0-1 to 0
	mov		x6, x0
setloop:
	sub		x6, x6, #1

	//Current cache way, x7. ranges from x1-1 to 0
	mov		x7, x1
wayloop:
	sub		x7, x7, #1

	//Given set in x6 and way in x7, format setway into 5
	//bits 31 to (32-A) is way number
	//bits B-1 to L is set
	lsl		x5, x7, x4		//nway << (32 - log(nways))
	lsl		x9, x6, x3		//nset << log(linelen)
	orr		x5, x5, x9		//way | set
	orr		x5, x5, x2		//way | set | level
	dc		isw, x5			//Cache line invalidate by set/way

	//End of inner loop
	cbnz	x7, wayloop

	//End of outer loop
	cbnz	x6, setloop

	//Done
	br		x30


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
