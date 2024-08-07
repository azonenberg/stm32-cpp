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
