// NOTE: Adapted from riscv-rt/asm.S
#define REGBYTES (1 << 2)

.section .init, "ax"
	.global reset_vector
	.global irq_vector

// The reset vector, jumps to startup code
reset_vector:
	j _start

// Interrupt handler
.option push
.option norelax // To prevent an unsupported R_RISCV_ALIGN relocation from being generated
.balign 16
irq_vector:
    addi sp, sp, -16*REGBYTES

    sw ra, 0*REGBYTES(sp)
    sw t0, 1*REGBYTES(sp)
    sw t1, 2*REGBYTES(sp)
    sw t2, 3*REGBYTES(sp)
    sw t3, 4*REGBYTES(sp)
    sw t4, 5*REGBYTES(sp)
    sw t5, 6*REGBYTES(sp)
    sw t6, 7*REGBYTES(sp)
    sw a0, 8*REGBYTES(sp)
    sw a1, 9*REGBYTES(sp)
    sw a2, 10*REGBYTES(sp)
    sw a3, 11*REGBYTES(sp)
    sw a4, 12*REGBYTES(sp)
    sw a5, 13*REGBYTES(sp)
    sw a6, 14*REGBYTES(sp)
    sw a7, 15*REGBYTES(sp)

    add a0, sp, zero
    jal ra, _start_trap_rust

    lw ra, 0*REGBYTES(sp)
    lw t0, 1*REGBYTES(sp)
    lw t1, 2*REGBYTES(sp)
    lw t2, 3*REGBYTES(sp)
    lw t3, 4*REGBYTES(sp)
    lw t4, 5*REGBYTES(sp)
    lw t5, 6*REGBYTES(sp)
    lw t6, 7*REGBYTES(sp)
    lw a0, 8*REGBYTES(sp)
    lw a1, 9*REGBYTES(sp)
    lw a2, 10*REGBYTES(sp)
    lw a3, 11*REGBYTES(sp)
    lw a4, 12*REGBYTES(sp)
    lw a5, 13*REGBYTES(sp)
    lw a6, 14*REGBYTES(sp)
    lw a7, 15*REGBYTES(sp)

    addi sp, sp, 16*REGBYTES
    ret
.option pop

_start:
   .cfi_startproc
    .cfi_undefined ra

    li  x1, 0
    li  x2, 0
    li  x3, 0
    li  x4, 0
    li  x5, 0
    li  x6, 0
    li  x7, 0
    li  x8, 0
    li  x9, 0
    li  x10,0
    li  x11,0
    li  x12,0
    li  x13,0
    li  x14,0
    li  x15,0
    li  x16,0
    li  x17,0
    li  x18,0
    li  x19,0
    li  x20,0
    li  x21,0
    li  x22,0
    li  x23,0
    li  x24,0
    li  x25,0
    li  x26,0
    li  x27,0
    li  x28,0
    li  x29,0
    li  x30,0
    li  x31,0

    .option push
    .option norelax // To prevent an unsupported R_RISCV_ALIGN relocation from being generated
    la gp, __global_pointer$
    .option pop

    // Allocate stack
    la sp, _stack_top

    // Set frame pointer
    add s0, sp, zero

    jal zero, _start_rust

    .cfi_endproc

loop:
	j loop

// Make sure there is an abort when linking
.globl abort
abort:
    j abort
