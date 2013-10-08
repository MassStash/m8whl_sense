/*
 * Idle processing for ARMv7-based Qualcomm SoCs.
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, 2011-2013 The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/linkage.h>
#include <linux/threads.h>
#include <asm/assembler.h>

#include "idle.h"
#include "idle-macros.S"

#ifdef CONFIG_MSM_SCM
#define SCM_SVC_BOOT 0x1
#define SCM_CMD_TERMINATE_PC 0x2
#endif

ENTRY(msm_arch_idle)
#ifdef CONFIG_ARCH_MSM_KRAIT
	mrc 	p15, 0, r0, c0, c0, 0
	bic	r1, r0, #0xff
	movw	r2, #0x0400
	movt	r2, #0x511F
	movw	r3, #0x0600
	movt	r3, #0x510F
	cmp	r2, r1
	cmpne	r3, r1
	bne	go_wfi

	mrs	r0, cpsr
	cpsid	if

	mrc	p15, 7, r1, c15, c0, 5
	bic	r2, r1, #0x20000
	mcr	p15, 7, r2, c15, c0, 5
	isb

go_wfi:
	wfi
	bne	wfi_done
	mcr	p15, 7, r1, c15, c0, 5
	isb
	msr	cpsr_c, r0

wfi_done:
	bx	lr
#else
	wfi
#ifdef CONFIG_ARCH_MSM8X60
	mrc	p14, 1, r1, c1, c5, 4 /* read ETM PDSR to clear sticky bit */
	mrc     p14, 0, r1, c1, c5, 4 /* read DBG PRSR to clear sticky bit */
	isb
#endif
	bx	lr
#endif

ENTRY(msm_pm_collapse)
#if defined(CONFIG_MSM_FIQ_SUPPORT)
	cpsid   f
#endif

	ldr     r0, =msm_saved_state	/* address of msm_saved_state ptr */
	ldr	r0, [r0]		/* load ptr */
#if (NR_CPUS >= 2)
	mrc	p15, 0, r1, c0, c0, 5	/* MPIDR */
	ands	r1, r1, #15		/* What CPU am I */
	mov	r2, #CPU_SAVED_STATE_SIZE
	mul	r1, r1, r2
	add	r0, r0, r1
#endif

	stmia   r0!, {r4-r14}
	mrc     p15, 0, r1, c1, c0, 0 /* MMU control */
	mrc     p15, 0, r2, c2, c0, 0 /* TTBR0 */
	mrc     p15, 0, r3, c3, c0, 0 /* dacr */
#ifdef CONFIG_ARCH_MSM_SCORPION
	/* This instruction is not valid for non scorpion processors */
	mrc     p15, 3, r4, c15, c0, 3 /* L2CR1 is the L2 cache control reg 1 */
#endif
	mrc     p15, 0, r5, c10, c2, 0 /* PRRR */
	mrc     p15, 0, r6, c10, c2, 1 /* NMRR */
	mrc     p15, 0, r7, c1, c0, 1 /* ACTLR */
	mrc     p15, 0, r8, c2, c0, 1 /* TTBR1 */
	mrc     p15, 0, r9, c13, c0, 3 /* TPIDRURO */
	mrc     p15, 0, ip, c13, c0, 1 /* context ID */
	stmia   r0!, {r1-r9, ip}

#if defined(CONFIG_MSM_JTAG) || defined(CONFIG_MSM_JTAG_MM)
	bl      msm_jtag_save_state
#endif

	ldr	r0, =msm_pm_flush_l2_flag
	ldr	r0, [r0]
	mov	r1, #0
	mcr	p15, 2, r1, c0, c0, 0 /*CCSELR*/
	isb
	mrc	p15, 1, r1, c0, c0, 0 /*CCSIDR*/
	mov	r2, #1
	and	r1, r2, r1, ASR #30 /* Check if the cache is write back */
	orr	r1, r0, r1
	and	r1, r1, #1
	cmp	r1, #1
	bne	skip
	bl	v7_flush_dcache_all
	ldr	r1, =msm_pm_flush_l2_fn
	ldr	r1, [r1]
	cmp	r1, #0
	blxne	r1

skip:
	ldr	r1, =msm_pm_disable_l2_fn
	ldr	r1, [r1]
	cmp	r1, #0
	blxne	r1
	dmb

	mrc	p15, 0, r0, c0, c0, 5	/* MPIDR */
	and	r0, r0, #15		/* what CPU am I */

	ldr	r1, =msm_pc_debug_counters /*load the IMEM debug location */
	ldr	r1, [r1]
	cmp	r1, #0
	beq	skip_pc_debug1
	add	r1, r1, r0, LSL #4	/* debug location for this CPU */
	ldr	r2, [r1]
	add	r2, #1
	str	r2, [r1]
skip_pc_debug1:

#ifdef CONFIG_MSM_SCM
	ldr	r0, =SCM_SVC_BOOT
	ldr	r1, =SCM_CMD_TERMINATE_PC
	ldr	r2, =msm_pm_flush_l2_flag
	ldr	r2, [r2]
	bl	scm_call_atomic1
#else
	mrc     p15, 0, r4, c1, c0, 0    /* read current CR    */
	bic     r0, r4, #(1 << 2)        /* clear dcache bit   */
	bic     r0, r0, #(1 << 12)       /* clear icache bit   */
	mcr     p15, 0, r0, c1, c0, 0    /* disable d/i cache  */
	isb

	SUSPEND_8x25_L2
	SET_SMP_COHERENCY OFF
	wfi
	DELAY_8x25 300

	mcr     p15, 0, r4, c1, c0, 0    /* restore d/i cache  */
	isb
	ENABLE_8x25_L2 /* enable only l2, no need to restore the reg back */
	SET_SMP_COHERENCY ON
#endif

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
/* r0 already has the cpu number */
.macro set_cpu_foot_print_virt, x
	stmfd	sp!, {r5-r7}
	mrc	p15, 0, r5, c0, c0, 5	/* MPIDR */
	ands	r5, r5, #15		/* What CPU am I */
	ldr	r6, =mnemosyne_base			/* Get the base of footprint */
	ldr	r6, [r6]

	cmp	r6, #0
	beq	skip_footprint\@				/* if base is NULL, just skip */

	ldr	r7, =mnemosyne_kernel_footprint_cpu	/* Get the offset of an element */
	add	r6, r6, r7				/* Get the address of an element */

	mov	r7, #\x
	strb	r7, [r6, r5, LSL #2] /* write footprint x, x must be 1 byte*/
	dsb						/* ensure data are written. */
skip_footprint\@:
	ldmfd	sp!, {r5-r7}
.endm
#endif

	.arm
ENTRY(msm_pm_boot_entry)
	mrc     p15, 0, r0, c0, c0, 5    /* MPIDR                          */
	and     r0, r0, #15              /* what CPU am I                  */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	ldr	r1, =mnemosyne_phys				/* Get the base of footprint */
	ldr	r2, =msm_pm_boot_entry
	adr	r3, msm_pm_boot_entry
	add	r1, r1, r3					/* translate virt to phys addr    */
	sub	r1, r1, r2
	ldr	r4, [r1]					/* Keep mnemosyne_phys in R4 */

	cmp	r4, #0
	beq	1f						/* if base is NULL, skip it. */

	ldr	r2, =mnemosyne_kernel_footprint_cpu		/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	mov	r2, #2              /* write footprint 2 */
	strb	r2, [r3, r0, LSL #2]
	dsb						/* ensure data are written. */
1:
#endif

	ldr	r1, =msm_pc_debug_counters_phys /*phys addr for IMEM reg */
	ldr	r2, =msm_pm_boot_entry
	adr	r3, msm_pm_boot_entry
	add	r1, r1, r3               /* translate virt to phys addr    */
	sub	r1, r1, r2
	ldr	r1,[r1]

	cmp	r1, #0
	beq	skip_pc_debug3
	add	r1, r1, r0, LSL #4	/* debug location for this CPU */
	add	r1, #4			/* warmboot entry counter*/
	ldr	r2, [r1]
	add	r2, #1
	str	r2, [r1]

skip_pc_debug3:
	ldr     r1, =msm_pm_boot_vector
	ldr     r2, =msm_pm_boot_entry
	adr     r3, msm_pm_boot_entry
	add     r1, r1, r3               /* translate virt to phys addr    */
	sub     r1, r1, r2

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	cmp	r4, #0
	beq	1f						/* if base is NULL, skip it. */

	ldr	r2, =mnemosyne_kernel_footprint_cpu		/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	mov	r2, #3              /* write footprint 3 */
	strb	r2, [r3, r0, LSL #2]
	dsb						/* ensure data are written. */
1:
#endif

	add     r1, r1, r0, LSL #2       /* locate boot vector for our cpu */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	cmp	r4, #0
	beq	1f						/* if base is NULL, skip it. */

	ldr	r2, =mnemosyne_cpu_reset_vector_address		/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	str	r1, [r3, r0, LSL#2]

	ldr	r2, =mnemosyne_cpu_reset_vector_address_value	/* Get the offset of an element */
	add	r3, r4, r2					/* Get the address of an element */
	ldr	r2, [r1]
	str	r2, [r3, r0, LSL#2]
	dsb						/* ensure data are written. */
1:
#endif

	ldr     pc, [r1]                 /* jump                           */

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
ENTRY(get_pm_boot_vector_symbol_address)
	ldr	r2, =msm_pm_boot_vector
	str	r2, [r0]
	bx	lr
#endif

3:	.long	.
	.data

	.globl msm_pm_pc_pgd
msm_pm_pc_pgd:
	.long	0x0

	.globl msm_saved_state
msm_saved_state:
	.long	0x0

	.globl msm_saved_state_phys
msm_saved_state_phys:
	.long	0x0

	.globl msm_pm_boot_vector
msm_pm_boot_vector:
	.space  4 * NR_CPUS

	.globl msm_pc_debug_counters_phys
msm_pc_debug_counters_phys:
	.long 0x0
