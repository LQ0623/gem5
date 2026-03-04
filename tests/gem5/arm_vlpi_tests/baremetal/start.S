.section .text.boot
.global _start
.extern main
.extern el1_irq_handler

_start:
    ldr x0, =_stack_top
    mov sp, x0

    ldr x0, =vectors
    msr vbar_el1, x0
    isb

    bl main
1:  wfi
    b 1b

.align 11
vectors:
    // Current EL with SP0
    b .
    b .
    b .
    b .

    // Current EL with SPx
    b .
    b .
    b el1_irq_vector
    b .

    // Lower EL using AArch64
    b .
    b .
    b .
    b .

    // Lower EL using AArch32
    b .
    b .
    b .
    b .

el1_irq_vector:
    sub sp, sp, #16
    stp x0, x1, [sp]
    bl el1_irq_handler
    ldp x0, x1, [sp]
    add sp, sp, #16
    eret

.section .bss
.align 12
.global _stack
_stack:
    .space 16384
.global _stack_top
_stack_top:
