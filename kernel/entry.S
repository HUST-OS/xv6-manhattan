    # boot entry for those boot harts
    .section .text.entry
    .globl _entry
_entry:
    add t0, a0, 1
    slli t0, t0, 14
    la sp, boot_stack
    add sp, sp, t0

    # jump into main
    call main

loop:
    j loop

    # boot entry for those non-boot harts
    .section .text
    .globl _entry_nonboot
_entry_nonboot:
    add t0, a0, 1
    slli t0, t0, 14
    la sp, boot_stack
    add sp, sp, t0

    # jump into hart_boot
    call hart_boot
    j loop

    .section .bss.stack
    .align 12
    .globl boot_stack
boot_stack:
    .space 4096 * 4 * 2
    .globl boot_stack_top
boot_stack_top: