////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AARCH64 path

#ifdef __aarch64__

.globl GetCurrentCore
.type GetCurrentCore, %function
GetCurrentCore:
	mrs		x0, MPIDR_EL1
	and		x0, x0, #0xff
	br		x30

//Turn on the MMU
//void InitializeMMU(void* rootPageTable)
.globl InitializeMMU
.type InitializeMMU, %function
InitializeMMU:

	//Point TTBR0 to the top level page table
	msr		TTBR0_EL3, x0

	//Initialize MAIR_EL3 to have attributes matching MAIR_IDX
	//(0 = uncacheable, 1 = normal, 2 = device)
	mov		x1, #0x00ff44
	msr		MAIR_EL3, x1

	//Translation regime configuration
	mov		x2, #30				//5:0 = T0SZ, virtual address space is (64-30) = 34 bits wide
	orr		x2, x2, #(1 << 8)	//IRGN0: translation table walks are inner WB/WA
	orr		x2, x2, #(1 << 10)	//IRGN1: translation table walks are outer WB/WA
	orr		x2, x2, #(3 << 12)	//SH0: translation table walks are inner shareable
	//orr		x2, x2, #(0 << 14)	//TG0: 4 kB granules
								//Leave PS=0 so 32 bit physical address space (4GB)
	msr		TCR_EL3, x2

	//Invalidate the TLB
	tlbi	ALLE3
	dsb		sy
	isb

	//Turn on the MMU
	mov		x3, #1
	//TODO: enable caches see 2.8 of tutorial
	msr		SCTLR_EL3, x3
	isb

	//DEBUG
	nop
	nop
	nop
	nop
	nop
	nop

	br		x30

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ARM-M path

#else

//see https://stackoverflow.com/questions/78309423/bad-blx-instruction-generated-when-calling-asm-function-from-c-function-gcc-on
.globl DisableInterrupts
.type DisableInterrupts, %function
DisableInterrupts:
	cpsid	i
	bx		lr

.globl EnableInterrupts
.type EnableInterrupts, %function
EnableInterrupts:
	cpsie	i
	bx		lr

.globl EnterCriticalSection
.type EnterCriticalSection, %function
EnterCriticalSection:
	mrs		r0, primask
	cpsid	i
	bx		lr

.globl LeaveCriticalSection
.type LeaveCriticalSection, %function
LeaveCriticalSection:
	msr		primask, r0
	bx		lr

.globl DisableFaults
.type DisableFaults, %function
DisableFaults:
	mrs		r0, faultmask
	cpsid	f
	bx		lr

.globl EnableFaults
.type EnableFaults, %function
EnableFaults:
	msr		faultmask, r0
	bx		lr

#endif
