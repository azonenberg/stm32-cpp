.globl DisableInterrupts
DisableInterrupts:
	cpsid	i
	bx		lr

.globl EnableInterrupts
EnableInterrupts:
	cpsie	i
	bx		lr

.globl EnterCriticalSection
EnterCriticalSection:
	mrs		r0, primask
	cpsid	i
	bx		lr

.globl LeaveCriticalSection
LeaveCriticalSection:
	msr		primask, r0
	bx		lr

.globl DisableFaults
DisableFaults:
	mrs		r0, faultmask
	cpsid	f
	bx		lr

.globl EnableFaults
EnableFaults:
	msr		faultmask, r0
	bx		lr
