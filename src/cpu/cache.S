.text
.thumb
.syntax unified

.equ ICIALLU,	0xe000ef50
.equ CCR,		0xe000ed14
.equ CCR_IC,	0x20000

.globl InvalidateInstructionCache
.type InvalidateInstructionCache, %function
InvalidateInstructionCache:
	mov		r0, #0
	ldr		r11, =ICIALLU
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
