# Copyright (c) 2022 zerosensei
# SPDX-License-Identifier: Apache-2.0

.extern freertos_risc_v_trap_handler

	.section	.init,"ax",@progbits
	.global	_start
	.align	1
_start:
	j	Reset_Handler

    .text
    .section .vectors, "ax"
    .option norvc;

 	jal x0, freertos_risc_v_trap_handler

	# .section .startup

/* Reset Handler */
Reset_Handler:
.option push
.option	norelax
	la gp, __global_pointer$
.option	pop
1:
	la sp, __StackTop

2:
    la a0, _highcode_copy_lma
    la a1, _highcode_copy_start
    la a2, _highcode_copy_end
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    sw t0, (a1)
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b

	jal SystemInit

2:
	/* 流水线控制位 & 动态预测控制位 */
	li t0, 0x1f
	csrw 0xbc0, t0

	csrwi 0x804, 0x0

    li t0, 0x1888
    csrs mstatus, t0

	jal main

    ret

