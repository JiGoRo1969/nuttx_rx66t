/*****************************************************************************
 * arch/renesas/src/rx66t/rx66t_vector.S
 *
 *   Copyright (C) 2008-2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>               /* NuttX configuration settings */
#include <arch/board/board.h>   /* Board-specific settings */
#include <arch/irq.h>                   /* IRQ definitons */
#include "up_internal.h"
#include "rx66t/iodefine.h"
#include "rx66t_definitions.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * External references
 *****************************************************************************/

        .section .text
        .globl  _up_doirq               /* C interrupt processing logic */

        .section .bss
        .globl  _up_num_irq
_up_num_irq:
        .long   0
        .size   _up_num_irq, . - _up_num_irq

/*****************************************************************************
 * Text
 *****************************************************************************/
        .section .text
        .text

/*****************************************************************************
 * Macros
 *****************************************************************************/

/*****************************************************************************
 * Macro: trampoline
 *
 * Description:
 *   Enter on exception with:
 *
 *   SP -> PC
 *         PSW
 *
 *   Branch to up_vector with:
 *
 *****************************************************************************/
    .macro  trampoline, irq, label
    .globl  \label
\label:
    PUSH.L  R15
    MOV.L   #_up_num_irq, R15
    MOV.W   #\irq, [R15]
    POP     R15
    BRA.A   _up_vector /* Jump to the common vector handling logic */
    NOP
    .endm

/*****************************************************************************
 * Macro: PUSHREGS
 *
 * Description:
 *   push registers to stacks
 *     registers : R15, R14, ..., R1, FPSW,
 *                 ACC1GU, ACC1HI, ACC1LO, ACC0GU, ACC0HI, ACC0LO, USP
 *****************************************************************************/
    .macro  PUSHREGS
    PUSHM   R1 - R15
    PUSHC   FPSW
    MVFACGU #0, A1, R15
    PUSH.L  R15
    MVFACHI #0, A1, R15
    PUSH.L  R15
    MVFACLO #0, A1, R15
    PUSH.L  R15
    MVFACGU #0, A0, R15
    PUSH.L  R15
    MVFACHI #0, A0, R15
    PUSH.L  R15
    MVFACLO #0, A0, R15
    PUSH.L  R15
    PUSHC   USP
    .endm

/*****************************************************************************
 * Macro: POPREGS
 *
 * Description:
 *   ppop registers from stacks
 *     registers : USP, ACC0LO, ACC0,HI, ACC0GU, ACC1LO, ACC1,HI, ACC1GU,
 *                 FPSW, R1, R2, ..., R15
 *****************************************************************************/
    .macro  POPREGS
    POPC    USP
    POP     R15
    MVTACLO R15, A0
    POP     R15
    MVTACHI R15, A0
    POP     R15
    MVTACGU R15, A0
    POP     R15
    MVTACLO R15, A1
    POP     R15
    MVTACHI R15, A1
    POP     R15
    MVTACGU R15, A1
    POPC    FPSW
    POPM    R1 - R15
    .endm

/*****************************************************************************
 * Macro: MULTI_SWITCHING
 *
 * Description:
 *   interrupt group branch for groupa and groupb
 *****************************************************************************/
    .macro  multi_switching, is_adrs, en_adrs, bit_number, irq_number
    PUSH.L  R1
    MOV.L   #\is_adrs, R1
    MOV.L   [R1], R1
    BTST    #\bit_number, R1
    POP     R1
    BZ      .L\@
    NOP
    PUSH.L  R1
    MOV.L   #\en_adrs, R1
    MOV.L   [R1], R1
    BTST    #\bit_number, R1
    POP     R1
    BZ      .L\@
    NOP
    PUSH.L  R1
    PUSH.L  R2
    MOV.L   #\irq_number, R1
    MOV.L   #_up_num_irq, R2
    MOV.W   R1, [R2]
    POP     R2
    POP     R1
    PUSHC   PSW         /* for RTE of the end of _up_vector */
    BSR.A   _up_vector  /* Jump to the common vector handling logic */
.L\@:
    NOP
    .endm

/*****************************************************************************
 * Name: _up_vector
 * R1 = ISP because PC jump from each interrupt handler to _up_vector
 ****************************************************************************/
    .global _up_vector
    .type   _up_vector, #function

_up_vector:
    SETPSW I               /*       Re-enable interrupts. */
    PUSHREGS
#ifdef  RX66T_INTERRUPT_IPL

/* Ensure the interrupt mask is set to the syscall priority while the kernel
 * structures are being accessed.
 */

   SETPSW  I                       /* Enable Inerrupt */
   MVTIPL  RX66T_INTERRUPT_IPL     /* configMAX_SYSCALL_INTERRUPT_PRIORITY */
#endif
    MOV.L   #_up_num_irq, R2
    MOV.W   [R2], R1
    MVFC    ISP, R2         /* stack pointer as 2nd args of _up_doirq */
    BSR.A   _up_doirq       /* remaining R1 register as 1st args of _up_doirq */
    NOP
    MVFC    ISP, R2
    CMP     R2, R1
    BEQ     .lint_contextswitch
    NOP
    MOV.L   R1, R2
    MVFC    ISP, R1
    MOV.L   #(XCPTCONTEXT_SIZE), R3 /* R3 = Number of bytes to transfer */
    SMOVF                           /* while(R3 > 0) {*R1++ = *R2++; R3--; } */
    NOP
.lint_contextswitch:
#ifdef  RX66T_INTERRUPT_IPL

/* Reset the interrupt mask as no more data structure access is required.
 */
    CLRPSW  I                       /* Disable Inerrupt */
    MVTIPL  #15                     /* configKERNEL_INTERRUPT_PRIORITY */
#endif
    POPREGS
    RTE
    NOP
    NOP
   .size   _up_vector, . - _up_vector

/**************************************************************************
 * Name: int up_saveusercontext(uint32_t *regs)
 *
 * Description:
 *   Save the context of the calling function at the point of the return from
 *   the
 *   call.  This basically a setjmp.
 *
 * Input Parameters:
 *   R1 = Address of register save array
 *
 * Returned Value:
 *   R1 = 0 on normal call; 1 on context switch
 **************************************************************************/

    .globl  _up_saveusercontext
    .type   _up_saveusercontext, #function

_up_saveusercontext:
    INT             #VECT_ICU_SWINT         /* #RX66T_SWINT_VNDX */
    NOP
    NOP
    RTS
    NOP
    NOP
    .size   _up_saveusercontext,  . - _up_saveusercontext

_uprx66t_swint_handler:
    PUSHREGS

/* Then copy the stack content to the register context array pointed to by r1
 */
    MVFC    ISP, R2                 /* stack pointer as source context info */
    MOV.L   #XCPTCONTEXT_SIZE, R3   /* R3 = Number of bytes to transfer */
    SMOVF                          /* while(R3 > 0) {*R1++ = *R2++; R3--; } */
    NOP
    SUB     #XCPTCONTEXT_SIZE, R1
    ADD     #(REG_R1 * 4), R1
    MOV.L   #1, R2
    MOV.L   R2, [R1]                                /* Return 1 on context switch */

/* Then restore R2, R3 and ISP
 */
    MVFC    ISP, R1
    MOV.L   (REG_R2 * 4)[R1], R2
    MOV.L   (REG_R3 * 4)[R1], R3
    ADD             #(XCPTCONTEXT_SIZE - 8), R1
    MVTC    R1, ISP

/* Then return zero to indicate a normal function call return
 */
    MOV             #0, R1                                  /* And return zero */
    RTE
    NOP
    NOP
   .size   _uprx66t_swint_handler,  . - _uprx66t_swint_handler

/*****************************************************************************
 * Name: void up_fullcontextrestore(uint32_t *regs)
 *
 * Description:
 *   Restore the context of the using the provided register save array.
 *
 * Input Parameters:
 *   R1 = Address of register save array
 *
 * Returned Value:
 *   None
 *****************************************************************************/
    .global _up_fullcontextrestore
    .type   _up_fullcontextrestore, #function

_up_fullcontextrestore:
    INT  #VECT_ICU_SWINT2  /* #RX66T_SWINT2_VNDX */
    NOP
    NOP
    RTS
    NOP
    NOP
   .size   _up_fullcontextrestore, . - _up_fullcontextrestore

_uprx66t_swint2_handler:
    MOV.L   R1, R2

/* Then copy the stack content from the register context array pointed
 */
    MVFC    ISP, R1                                         /* stack pointer as destination context info */
    ADD     # - (XCPTCONTEXT_SIZE - 8), R1      /* TODO : check to need -8 */
    MVTC    R1, ISP                                         /* ISP = address of base of new context info */
    MOV.L   #(XCPTCONTEXT_SIZE), R3 /* R3 = Number of bytes to transfer */
    SMOVF                                                           /* while(R3 > 0) {*R1++ = *R2++; R3--;} */
    POPREGS
    RTE
    NOP
    NOP
    .size   _uprx66t_swint2_handler, . - _uprx66t_swint2_handler

/*****************************************************************************
 * Name: _uprx66t_*_handler
 *
 * Description:
 *   Trampoline entry points for each, individual IRQ
 *
 *  R1 :  Points to a the register save structure
 *
 *****************************************************************************/
_uprx66t_start_handler:
#if 1
    trampoline NR_IRQS, _uprx66t_invalid_handler
#else
    trampoline RX66T_TRAP0_IRQ, _uprx66t_trap0_handler
    trampoline RX66T_TRAP1_IRQ, _uprx66t_trap1_handler
    trampoline RX66T_TRAP2_IRQ, _uprx66t_trap2_handler
    trampoline RX66T_TRAP3_IRQ, _uprx66t_trap3_handler
    trampoline RX66T_TRAP4_IRQ, _uprx66t_trap4_handler
    trampoline RX66T_TRAP5_IRQ, _uprx66t_trap5_handler
    trampoline RX66T_TRAP6_IRQ, _uprx66t_trap6_handler
    trampoline RX66T_TRAP7_IRQ, _uprx66t_trap7_handler
    trampoline RX66T_TRAP8_IRQ, _uprx66t_trap8_handler
    trampoline RX66T_TRAP9_IRQ, _uprx66t_trap9_handler
    trampoline RX66T_TRAP10_IRQ, _uprx66t_trap10_handler
    trampoline RX66T_TRAP11_IRQ, _uprx66t_trap11_handler
    trampoline RX66T_TRAP12_IRQ, _uprx66t_trap12_handler
    trampoline RX66T_TRAP13_IRQ, _uprx66t_trap13_handler
    trampoline RX66T_TRAP14_IRQ, _uprx66t_trap14_handler
    trampoline RX66T_TRAP15_IRQ, _uprx66t_trap15_handler
#endif
#ifdef CONFIG_BSC
    trampoline RX66T_BUSERR_IRQ, _uprx66t_buserr_handler
#endif
#ifdef CONFIG_RAM
    trampoline RX66T_RAMERR_IRQ, _uprx66t_ramerr_handler
#endif
#ifdef CONFIG_FCU
    trampoline RX66T_FIFERR_IRQ, _uprx66t_fiferr_handler
#endif
#ifdef CONFIG_FRDYI
    trampoline RX66T_FRDYI_IRQ, _uprx66t_frdyi_handler
#endif
    trampoline RX66T_CMI0_IRQ, _uprx66t_cmi0_handler
#ifdef CONFIG_RX66T_CMT1
    trampoline RX66T_CMI1_IRQ, _uprx66t_cmi1_handler
#endif
#ifdef CONFIG_RX66T_CMT2
    trampoline RX66T_CMI2_IRQ, _uprx66t_cmi2_handler
#endif
#ifdef CONFIG_RX66T_CMT3
    trampoline RX66T_CMI3_IRQ, _uprx66t_cmi3_handler
#endif
#ifdef CONFIG_RX66T_USB0
    trampoline RX66T_D0FIFO0_IRQ, _uprx66t_d0fifo0_handler
    trampoline RX66T_D1FIFO0_IRQ, _uprx66t_d1fifo0_handler
#endif
#ifdef CONFIG_RX66T_RSPI0
    trampoline RX66T_SPRI0_IRQ, _uprx66t_spri0_handler
    trampoline RX66T_SPTI0_IRQ, _uprx66t_spti0_handler
#endif
#ifdef CONFIG_RX66T_RIIC0
    trampoline RX66T_RIIC0_RXI0_IRQ, _uprx66t_rxii0_handler
    trampoline RX66T_RIIC0_TXI0_IRQ, _uprx66t_txii0_handler
#endif
#ifdef CONFIG_RX66T_SCI1
    trampoline  RX66T_RXI1_IRQ, _uprx66t_rxi1_handler
    trampoline  RX66T_TXI1_IRQ, _uprx66t_txi1_handler
#endif
#if defined(CONFIG_ICU) || defined(CONFIG_RX66T_ICU)
    trampoline RX66T_IRQ0_IRQ,  _uprx66t_irq0_handler
    trampoline RX66T_IRQ1_IRQ,  _uprx66t_irq1_handler
    trampoline RX66T_IRQ2_IRQ,  _uprx66t_irq2_handler
    trampoline RX66T_IRQ3_IRQ,  _uprx66t_irq3_handler
    trampoline RX66T_IRQ4_IRQ,  _uprx66t_irq4_handler
    trampoline RX66T_IRQ5_IRQ,  _uprx66t_irq5_handler
    trampoline RX66T_IRQ6_IRQ,  _uprx66t_irq6_handler
    trampoline RX66T_IRQ7_IRQ,  _uprx66t_irq7_handler
    trampoline RX66T_IRQ8_IRQ,  _uprx66t_irq8_handler
    trampoline RX66T_IRQ9_IRQ,  _uprx66t_irq9_handler
    trampoline RX66T_IRQ10_IRQ, _uprx66t_irq10_handler
    trampoline RX66T_IRQ11_IRQ, _uprx66t_irq11_handler
    trampoline RX66T_IRQ12_IRQ, _uprx66t_irq12_handler
    trampoline RX66T_IRQ13_IRQ, _uprx66t_irq13_handler
    trampoline RX66T_IRQ14_IRQ, _uprx66t_irq14_handler
    trampoline RX66T_IRQ15_IRQ, _uprx66t_irq15_handler
#endif
#ifdef CONFIG_RX66T_SCI5
    trampoline RX66T_RXI5_IRQ, _uprx66t_rxi5_handler
    trampoline RX66T_TXI5_IRQ, _uprx66t_txi5_handler
#endif
#ifdef CONFIG_RX66T_SCI6
    trampoline RX66T_RXI6_IRQ, _uprx66t_rxi6_handler
    trampoline RX66T_TXI6_IRQ, _uprx66t_txi6_handler
#endif
#ifdef CONFIG_RX66T_LVD1
    trampoline RX66T_LVD1_IRQ, _uprx66t_lvd1_handler
#endif
#ifdef CONFIG_RX66T_LVD2
    trampoline RX66T_LVD2_IRQ, _uprx66t_lvd2_handler
#endif
#ifdef CONFIG_RX66T_USB0_R0
    trampoline RX66T_USBR0_IRQ, _uprx66t_usbr0_handler
#endif
#ifdef CONFIG_RX66T_IWDT
    trampoline RX66T_IWUNI_IRQ, _uprx66t_iwuni_handler
#endif
#ifdef CONFIG_RX66T_WDT
    trampoline RX66T_WUNI_IRQ, _uprx66t_wuni_handler
#endif
#ifdef CONFIG_RX66T_SCI8
    trampoline RX66T_RXI8_IRQ, _uprx66t_rxi8_handler
    trampoline RX66T_TXI8_IRQ, _uprx66t_txi8_handler
#endif
#ifdef CONFIG_RX66T_SCI9
    trampoline RX66T_RXI9_IRQ, _uprx66t_rxi9_handler
    trampoline RX66T_TXI9_IRQ, _uprx66t_txi9_handler
#endif
#if defined(CONFIG_RX66T_IRQ_GROUP)
_uprx66t_groupbe0_handler:

/* TODO: Please write the others with definition RX66T_XXX_IRQ in irq.h as IRQ number */

    RTE
    NOP
_uprx66t_groupbl2_handler:

/* TODO: Please write the others with definition RX66T_XXX_IRQ in irq.h as IRQ number */

    RTE
    NOP
#endif
#ifdef CONFIG_RX66T_RSPI2
    trampoline RX66T_SPRI2_IRQ, _uprx66t_spri2_handler
    trampoline RX66T_SPTI2_IRQ, _uprx66t_spti2_handler
#endif
#if defined(CONFIG_RX66T_IRQ_GROUP)
_uprx66t_groupbl0_handler:
#ifdef CONFIG_RX66T_SCI0
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 0, RX66T_TEI0_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 1, RX66T_ERI0_IRQ
#endif
#ifdef CONFIG_RX66T_SCI1
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 2, RX66T_TEI1_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 3, RX66T_ERI1_IRQ
#endif
#ifdef CONFIG_RX66T_SCI2
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR,  4, RX66T_TEI2_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR,  5, RX66T_ERI2_IRQ
#endif
#ifdef CONFIG_RX66T_SCI3
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR,  6, RX66T_TEI3_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR,  7, RX66T_ERI3_IRQ
#endif
#ifdef CONFIG_RX66T_SCI4
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR,  8, RX66T_TEI4_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR,  9, RX66T_ERI4_IRQ
#endif
#ifdef CONFIG_RX66T_SCI5
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 10, RX66T_TEI5_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 11, RX66T_ERI5_IRQ
#endif
#ifdef CONFIG_RX66T_SCI6
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 12, RX66T_TEI6_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 13, RX66T_ERI6_IRQ
#endif
#ifdef CONFIG_RX66T_SCI7
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 14, RX66T_TEI7_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 15, RX66T_ERI7_IRQ
#endif
#ifdef CONFIG_RX66T_SCI12
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 16, RX66T_TEI12_IRQ
    multi_switching RX66T_GRPBL0_ADDR, RX66T_GENBL0_ADDR, 17, RX66T_ERI12_IRQ
#endif

/* TODO: Please write the others with definition RX66T_XXX_IRQ in irq.h as IRQ number */

    RTE
    NOP
_uprx66t_groupbl1_handler:
#ifdef CONFIG_RX66T_SCI8
    multi_switching RX66T_GRPBL1_ADDR, RX66T_GENBL1_ADDR, 24, RX66T_TEI8_IRQ
    multi_switching RX66T_GRPBL1_ADDR, RX66T_GENBL1_ADDR, 25, RX66T_ERI8_IRQ
#endif /* CONFIG_RX66T_SCI8 */

#ifdef CONFIG_RX66T_SCI9
    multi_switching RX66T_GRPBL1_ADDR, RX66T_GENBL1_ADDR, 26, RX66T_TEI9_IRQ
    multi_switching RX66T_GRPBL1_ADDR, RX66T_GENBL1_ADDR, 27, RX66T_ERI9_IRQ
#endif /* CONFIG_RX66T_SCI9 */

/* TODO: Please write the others with definition RX66T_XXX_IRQ in irq.h as IRQ number */

    RTE
    NOP
_uprx66t_groupal0_handler:
#ifdef CONFIG_RX66T_SCI10
    multi_switching RX66T_GRPAL0_ADDR, RX66T_GENAL0_ADDR,  8, RX66T_TEI10_IRQ
    multi_switching RX66T_GRPAL0_ADDR, RX66T_GENAL0_ADDR,  9, RX66T_ERI10_IRQ
#endif /* CONFIG_RX66T_SCI10 */

#ifdef CONFIG_RX66T_SCI11
    multi_switching RX66T_GRPAL0_ADDR, RX66T_GENAL0_ADDR, 12, RX66T_TEI11_IRQ
    multi_switching RX66T_GRPAL0_ADDR, RX66T_GENAL0_ADDR, 13, RX66T_ERI11_IRQ
#endif /* CONFIG_RX66T_SCI11 */

/* TODO: Please write the others with definition RX66T_XXX_IRQ in irq.h as IRQ number */

    RTE
    NOP
_uprx66t_groupal1_handler:

/* TODO: Please write the others with definition RX66T_XXX_IRQ in irq.h as IRQ number */

    RTE
    NOP
#endif  /* CONFIG_RX66T_IRQ_GROUP */

#ifdef CONFIG_RX66T_SCI11
    trampoline RX66T_RXI11_IRQ, _uprx66t_rxi11_handler
    trampoline RX66T_TXI11_IRQ, _uprx66t_txi11_handler
#endif
#ifdef CONFIG_RX66T_SCI12
    trampoline RX66T_RXI12_IRQ, _uprx66t_rxi12_handler
    trampoline RX66T_TXI12_IRQ, _uprx66t_txi12_handler
#endif
#ifdef CONFIG_RX66T_DMAC
    trampoline RX66T_DMAC0I_IRQ, _uprx66t_dmac0i_handler
    trampoline RX66T_DMAC1I_IRQ, _uprx66t_dmac1i_handler
    trampoline RX66T_DMAC2I_IRQ, _uprx66t_dmac2i_handler
    trampoline RX66T_DMAC3I_IRQ, _uprx66t_dmac3i_handler
    trampoline RX66T_DMAC74I_IRQ, _uprx66t_dmac74i_handler
#endif
#ifdef CONFIG_RX66T_OST
    trampoline RX66T_OSTD1_IRQ, _uprx66t_ostdi_handler
#endif
#ifdef CONFIG_RX66T_EXDMAC
    trampoline RX66T_EXDMAC0I_IRQ, _uprx66t_exdmac0i_handler
    trampoline RX66T_EXDMAC1I_IRQ, _uprx66t_exdmac1i_handler
#endif
#ifdef CONFIG_RX66T_PERIB
    trampoline RX66T_INTB128_IRQ, _uprx66t_intb128_handler
    trampoline RX66T_INTB129_IRQ, _uprx66t_intb129_handler
    trampoline RX66T_INTB130_IRQ, _uprx66t_intb130_handler
    trampoline RX66T_INTB131_IRQ, _uprx66t_intb131_handler
    trampoline RX66T_INTB132_IRQ, _uprx66t_intb132_handler
    trampoline RX66T_INTB133_IRQ, _uprx66t_intb133_handler
    trampoline RX66T_INTB134_IRQ, _uprx66t_intb134_handler
    trampoline RX66T_INTB135_IRQ, _uprx66t_intb135_handler
    trampoline RX66T_INTB136_IRQ, _uprx66t_intb136_handler
    trampoline RX66T_INTB137_IRQ, _uprx66t_intb137_handler
    trampoline RX66T_INTB138_IRQ, _uprx66t_intb138_handler
    trampoline RX66T_INTB139_IRQ, _uprx66t_intb139_handler
    trampoline RX66T_INTB140_IRQ, _uprx66t_intb140_handler
    trampoline RX66T_INTB141_IRQ, _uprx66t_intb141_handler
    trampoline RX66T_INTB142_IRQ, _uprx66t_intb142_handler
    trampoline RX66T_INTB143_IRQ, _uprx66t_intb143_handler
    trampoline RX66T_INTB144_IRQ, _uprx66t_intb144_handler
    trampoline RX66T_INTB145_IRQ, _uprx66t_intb145_handler
    trampoline RX66T_INTB146_IRQ, _uprx66t_intb146_handler
    trampoline RX66T_INTB147_IRQ, _uprx66t_intb147_handler
    trampoline RX66T_INTB148_IRQ, _uprx66t_intb148_handler
    trampoline RX66T_INTB149_IRQ, _uprx66t_intb149_handler
    trampoline RX66T_INTB150_IRQ, _uprx66t_intb150_handler
    trampoline RX66T_INTB151_IRQ, _uprx66t_intb151_handler
    trampoline RX66T_INTB152_IRQ, _uprx66t_intb152_handler
    trampoline RX66T_INTB153_IRQ, _uprx66t_intb153_handler
    trampoline RX66T_INTB154_IRQ, _uprx66t_intb154_handler
    trampoline RX66T_INTB155_IRQ, _uprx66t_intb155_handler
    trampoline RX66T_INTB156_IRQ, _uprx66t_intb156_handler
    trampoline RX66T_INTB157_IRQ, _uprx66t_intb157_handler
    trampoline RX66T_INTB158_IRQ, _uprx66t_intb158_handler
    trampoline RX66T_INTB159_IRQ, _uprx66t_intb159_handler
    trampoline RX66T_INTB160_IRQ, _uprx66t_intb160_handler
    trampoline RX66T_INTB161_IRQ, _uprx66t_intb161_handler
    trampoline RX66T_INTB162_IRQ, _uprx66t_intb162_handler
    trampoline RX66T_INTB163_IRQ, _uprx66t_intb163_handler
    trampoline RX66T_INTB164_IRQ, _uprx66t_intb164_handler
    trampoline RX66T_INTB165_IRQ, _uprx66t_intb165_handler
    trampoline RX66T_INTB166_IRQ, _uprx66t_intb166_handler
    trampoline RX66T_INTB167_IRQ, _uprx66t_intb167_handler
    trampoline RX66T_INTB168_IRQ, _uprx66t_intb168_handler
    trampoline RX66T_INTB169_IRQ, _uprx66t_intb169_handler
    trampoline RX66T_INTB170_IRQ, _uprx66t_intb170_handler
    trampoline RX66T_INTB171_IRQ, _uprx66t_intb171_handler
    trampoline RX66T_INTB172_IRQ, _uprx66t_intb172_handler
    trampoline RX66T_INTB173_IRQ, _uprx66t_intb173_handler
    trampoline RX66T_INTB174_IRQ, _uprx66t_intb174_handler
    trampoline RX66T_INTB175_IRQ, _uprx66t_intb175_handler
    trampoline RX66T_INTB176_IRQ, _uprx66t_intb176_handler
    trampoline RX66T_INTB177_IRQ, _uprx66t_intb177_handler
    trampoline RX66T_INTB178_IRQ, _uprx66t_intb178_handler
    trampoline RX66T_INTB179_IRQ, _uprx66t_intb179_handler
    trampoline RX66T_INTB180_IRQ, _uprx66t_intb180_handler
    trampoline RX66T_INTB181_IRQ, _uprx66t_intb181_handler
    trampoline RX66T_INTB182_IRQ, _uprx66t_intb182_handler
    trampoline RX66T_INTB183_IRQ, _uprx66t_intb183_handler
    trampoline RX66T_INTB184_IRQ, _uprx66t_intb184_handler
    trampoline RX66T_INTB185_IRQ, _uprx66t_intb185_handler
    trampoline RX66T_INTB186_IRQ, _uprx66t_intb186_handler
    trampoline RX66T_INTB187_IRQ, _uprx66t_intb187_handler
    trampoline RX66T_INTB188_IRQ, _uprx66t_intb188_handler
    trampoline RX66T_INTB189_IRQ, _uprx66t_intb189_handler
    trampoline RX66T_INTB190_IRQ, _uprx66t_intb190_handler
    trampoline RX66T_INTB191_IRQ, _uprx66t_intb191_handler
    trampoline RX66T_INTB192_IRQ, _uprx66t_intb192_handler
    trampoline RX66T_INTB193_IRQ, _uprx66t_intb193_handler
    trampoline RX66T_INTB194_IRQ, _uprx66t_intb194_handler
    trampoline RX66T_INTB195_IRQ, _uprx66t_intb195_handler
    trampoline RX66T_INTB196_IRQ, _uprx66t_intb196_handler
    trampoline RX66T_INTB197_IRQ, _uprx66t_intb197_handler
    trampoline RX66T_INTB198_IRQ, _uprx66t_intb198_handler
    trampoline RX66T_INTB199_IRQ, _uprx66t_intb199_handler
    trampoline RX66T_INTB200_IRQ, _uprx66t_intb200_handler
    trampoline RX66T_INTB201_IRQ, _uprx66t_intb201_handler
    trampoline RX66T_INTB202_IRQ, _uprx66t_intb202_handler
    trampoline RX66T_INTB203_IRQ, _uprx66t_intb203_handler
    trampoline RX66T_INTB204_IRQ, _uprx66t_intb204_handler
    trampoline RX66T_INTB205_IRQ, _uprx66t_intb205_handler
    trampoline RX66T_INTB206_IRQ, _uprx66t_intb206_handler
    trampoline RX66T_INTB207_IRQ, _uprx66t_intb207_handler
#endif
#ifdef CONFIG_RX66T_PERIA
    trampoline RX66T_INTA208_IRQ, _uprx66t_inta208_handler
    trampoline RX66T_INTA209_IRQ, _uprx66t_inta209_handler
    trampoline RX66T_INTA210_IRQ, _uprx66t_inta210_handler
    trampoline RX66T_INTA211_IRQ, _uprx66t_inta211_handler
    trampoline RX66T_INTA212_IRQ, _uprx66t_inta212_handler
    trampoline RX66T_INTA213_IRQ, _uprx66t_inta213_handler
    trampoline RX66T_INTA214_IRQ, _uprx66t_inta214_handler
    trampoline RX66T_INTA215_IRQ, _uprx66t_inta215_handler
    trampoline RX66T_INTA216_IRQ, _uprx66t_inta216_handler
    trampoline RX66T_INTA217_IRQ, _uprx66t_inta217_handler
    trampoline RX66T_INTA218_IRQ, _uprx66t_inta218_handler
    trampoline RX66T_INTA219_IRQ, _uprx66t_inta219_handler
    trampoline RX66T_INTA220_IRQ, _uprx66t_inta220_handler
    trampoline RX66T_INTA221_IRQ, _uprx66t_inta221_handler
    trampoline RX66T_INTA222_IRQ, _uprx66t_inta222_handler
    trampoline RX66T_INTA223_IRQ, _uprx66t_inta223_handler
    trampoline RX66T_INTA224_IRQ, _uprx66t_inta224_handler
    trampoline RX66T_INTA225_IRQ, _uprx66t_inta225_handler
    trampoline RX66T_INTA226_IRQ, _uprx66t_inta226_handler
    trampoline RX66T_INTA227_IRQ, _uprx66t_inta227_handler
    trampoline RX66T_INTA228_IRQ, _uprx66t_inta228_handler
    trampoline RX66T_INTA229_IRQ, _uprx66t_inta229_handler
    trampoline RX66T_INTA230_IRQ, _uprx66t_inta230_handler
    trampoline RX66T_INTA231_IRQ, _uprx66t_inta231_handler
    trampoline RX66T_INTA232_IRQ, _uprx66t_inta232_handler
    trampoline RX66T_INTA233_IRQ, _uprx66t_inta233_handler
    trampoline RX66T_INTA234_IRQ, _uprx66t_inta234_handler
    trampoline RX66T_INTA235_IRQ, _uprx66t_inta235_handler
    trampoline RX66T_INTA236_IRQ, _uprx66t_inta236_handler
    trampoline RX66T_INTA237_IRQ, _uprx66t_inta237_handler
    trampoline RX66T_INTA238_IRQ, _uprx66t_inta238_handler
    trampoline RX66T_INTA239_IRQ, _uprx66t_inta239_handler
    trampoline RX66T_INTA240_IRQ, _uprx66t_inta240_handler
    trampoline RX66T_INTA241_IRQ, _uprx66t_inta241_handler
    trampoline RX66T_INTA242_IRQ, _uprx66t_inta242_handler
    trampoline RX66T_INTA243_IRQ, _uprx66t_inta243_handler
    trampoline RX66T_INTA244_IRQ, _uprx66t_inta244_handler
    trampoline RX66T_INTA245_IRQ, _uprx66t_inta245_handler
    trampoline RX66T_INTA246_IRQ, _uprx66t_inta246_handler
    trampoline RX66T_INTA247_IRQ, _uprx66t_inta247_handler
    trampoline RX66T_INTA248_IRQ, _uprx66t_inta248_handler
    trampoline RX66T_INTA249_IRQ, _uprx66t_inta249_handler
    trampoline RX66T_INTA250_IRQ, _uprx66t_inta250_handler
    trampoline RX66T_INTA251_IRQ, _uprx66t_inta251_handler
    trampoline RX66T_INTA252_IRQ, _uprx66t_inta252_handler
    trampoline RX66T_INTA253_IRQ, _uprx66t_inta253_handler
    trampoline RX66T_INTA254_IRQ, _uprx66t_inta254_handler
    trampoline RX66T_INTA255_IRQ, _uprx66t_inta255_handler
#endif

/****************************************************************************
 * Data
 ***************************************************************************/

/***************************************************************************
 *  Name: g_intstackalloc/g_intstackbase
 *
 * Description:
 *   Should not happen
 *
 ***************************************************************************/
    .bss
    .align  2

#if CONFIG_ARCH_INTERRUPTSTACK > 3
    .bss
    .align  2
    .global  _g_intstackalloc
    .type   _g_intstackalloc, object
    .global  _g_intstackbase
    .type   _g_intstackbase, object
_g_intstackalloc:
    .skip   ((CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4)
_g_intstackbase:
    .skip   2
    .size   _g_intstackbase, 4
    .size   _g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#endif

/***************************************************************************
 * Interrupt Vectors
 **************************************************************************/
    .section        .rvectors

/*****************************************************************************
 * Name: __int_vector_table
 *
 * Description:
 *   Interrupt vector table
 *****************************************************************************/
    .global __int_vector_table
    .type   __int_vector_table, % object

__int_vector_table:
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_BSC
    .long   _uprx66t_buserr_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RAM
    .long   _uprx66t_ramerr_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_FCU
    .long   _uprx66t_fiferr_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_FRDYI
    .long   _uprx66t_frdyi_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_swint2_handler
    .long   _uprx66t_swint_handler
    .long   _uprx66t_cmi0_handler
#ifdef CONFIG_RX66T_CMT1
    .long   _uprx66t_cmi1_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_CMTW0
    .long   _uprx66t_cmwi0_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_CMTW1
    .long   _uprx66t_cmwi1_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_USB0
    .long   _uprx66t_d0fifo0_handler
    .long   _uprx66t_d1fifo0_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_RSPI0
    .long   _uprx66t_spri0_handler
    .long   _uprx66t_spti0_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_RSPI1
    .long   _uprx66t_spri1_handler
    .long   _uprx66t_spti1_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_QSPI
    .long   _uprx66t_spri_handler
    .long   _uprx66t_spti_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SDHI
    .long   _uprx66t_sbfai_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_MMCIF
    .long   _uprx66t_mbfai_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_RIIC1
    .long   _uprx66t_rxii1_handler
    .long   _uprx66t_txii1_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_RIIC0
    .long   _uprx66t_rxii0_handler
    .long   _uprx66t_txii0_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_RIIC2
    .long   _uprx66t_rxii2_handler
    .long   _uprx66t_txii2_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_SCI0
    .long   _uprx66t_rxi0_handler
    .long   _uprx66t_txi0_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI1
    .long   _uprx66t_rxi1_handler
    .long   _uprx66t_txi1_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI2
    .long   _uprx66t_rxi2_handler
    .long   _uprx66t_txi2_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#if defined(CONFIG_ICU) || defined(CONFIG_RX66T_ICU)
    .long   _uprx66t_irq0_handler
    .long   _uprx66t_irq1_handler
    .long   _uprx66t_irq2_handler
    .long   _uprx66t_irq3_handler
    .long   _uprx66t_irq4_handler
    .long   _uprx66t_irq5_handler
    .long   _uprx66t_irq6_handler
    .long   _uprx66t_irq7_handler
    .long   _uprx66t_irq8_handler
    .long   _uprx66t_irq9_handler
    .long   _uprx66t_irq10_handler
    .long   _uprx66t_irq11_handler
    .long   _uprx66t_irq12_handler
    .long   _uprx66t_irq13_handler
    .long   _uprx66t_irq14_handler
    .long   _uprx66t_irq15_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI3
    .long   _uprx66t_rxi3_handler
    .long   _uprx66t_txi3_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI4
    .long   _uprx66t_rxi4_handler
    .long   _uprx66t_txi4_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI5
    .long   _uprx66t_rxi5_handler
    .long   _uprx66t_txi5_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI6
    .long   _uprx66t_rxi6_handler
    .long   _uprx66t_txi6_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_LVD1
    .long   _uprx66t_lvd1_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_LVD2
    .long   _uprx66t_lvd2_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_USB0_R0
    .long   _uprx66t_usbr0_handler
#else
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_RTC
    .long   _uprx66t_alm_handler
    .long   _uprx66t_prd_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_IWDT
    .long   _uprx66t_iwuni_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_WDT
    .long   _uprx66t_wuni_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_PDC
    .long   _uprx66t_pcdfi_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI7
    .long   _uprx66t_rxi7_handler
    .long   _uprx66t_txi7_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI8
    .long   _uprx66t_rxi8_handler
    .long   _uprx66t_txi8_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI9
    .long   _uprx66t_rxi9_handler
    .long   _uprx66t_txi9_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI10
    .long   _uprx66t_rxi10_handler
    .long   _uprx66t_txi10_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#if defined(CONFIG_RX66T_IRQ_GROUP)/* && defined(USE_GROUP_IRQ) */
    .long   _uprx66t_groupbe0_handler
    .long   _uprx66t_groupbl2_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_RSPI2
    .long   _uprx66t_spri2_handler
    .long   _uprx66t_spti2_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#if defined(CONFIG_RX66T_IRQ_GROUP)/* && defined(USE_GROUP_IRQ) */
    .long   _uprx66t_groupbl0_handler
    .long   _uprx66t_groupbl1_handler
    .long   _uprx66t_groupal0_handler
    .long   _uprx66t_groupal1_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI11
    .long   _uprx66t_rxi11_handler
    .long   _uprx66t_txi11_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_SCI12
    .long   _uprx66t_rxi12_handler
    .long   _uprx66t_txi12_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#ifdef CONFIG_RX66T_DMAC
    .long   _uprx66t_dmac0i_handler
    .long   _uprx66t_dmac1i_handler
    .long   _uprx66t_dmac2i_handler
    .long   _uprx66t_dmac3i_handler
    .long   _uprx66t_dmac74i_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_OST
    .long   _uprx66t_ostdi_handler
#else
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_EXDMAC
    .long   _uprx66t_exdmac0i_handler
    .long   _uprx66t_exdmac1i_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_PERIB
    .long   _uprx66t_intb128_handler
    .long   _uprx66t_intb129_handler
    .long   _uprx66t_intb130_handler
    .long   _uprx66t_intb131_handler
    .long   _uprx66t_intb132_handler
    .long   _uprx66t_intb133_handler
    .long   _uprx66t_intb134_handler
    .long   _uprx66t_intb135_handler
    .long   _uprx66t_intb136_handler
    .long   _uprx66t_intb137_handler
    .long   _uprx66t_intb138_handler
    .long   _uprx66t_intb139_handler
    .long   _uprx66t_intb140_handler
    .long   _uprx66t_intb141_handler
    .long   _uprx66t_intb142_handler
    .long   _uprx66t_intb143_handler
    .long   _uprx66t_intb144_handler
    .long   _uprx66t_intb145_handler
    .long   _uprx66t_intb146_handler
    .long   _uprx66t_intb147_handler
    .long   _uprx66t_intb148_handler
    .long   _uprx66t_intb149_handler
    .long   _uprx66t_intb150_handler
    .long   _uprx66t_intb151_handler
    .long   _uprx66t_intb152_handler
    .long   _uprx66t_intb153_handler
    .long   _uprx66t_intb154_handler
    .long   _uprx66t_intb155_handler
    .long   _uprx66t_intb156_handler
    .long   _uprx66t_intb157_handler
    .long   _uprx66t_intb158_handler
    .long   _uprx66t_intb159_handler
    .long   _uprx66t_intb160_handler
    .long   _uprx66t_intb161_handler
    .long   _uprx66t_intb162_handler
    .long   _uprx66t_intb163_handler
    .long   _uprx66t_intb164_handler
    .long   _uprx66t_intb165_handler
    .long   _uprx66t_intb166_handler
    .long   _uprx66t_intb167_handler
    .long   _uprx66t_intb168_handler
    .long   _uprx66t_intb169_handler
    .long   _uprx66t_intb170_handler
    .long   _uprx66t_intb171_handler
    .long   _uprx66t_intb172_handler
    .long   _uprx66t_intb173_handler
    .long   _uprx66t_intb174_handler
    .long   _uprx66t_intb175_handler
    .long   _uprx66t_intb176_handler
    .long   _uprx66t_intb177_handler
    .long   _uprx66t_intb178_handler
    .long   _uprx66t_intb179_handler
    .long   _uprx66t_intb180_handler
    .long   _uprx66t_intb181_handler
    .long   _uprx66t_intb182_handler
    .long   _uprx66t_intb183_handler
    .long   _uprx66t_intb184_handler
    .long   _uprx66t_intb185_handler
    .long   _uprx66t_intb186_handler
    .long   _uprx66t_intb187_handler
    .long   _uprx66t_intb188_handler
    .long   _uprx66t_intb189_handler
    .long   _uprx66t_intb190_handler
    .long   _uprx66t_intb191_handler
    .long   _uprx66t_intb192_handler
    .long   _uprx66t_intb193_handler
    .long   _uprx66t_intb194_handler
    .long   _uprx66t_intb195_handler
    .long   _uprx66t_intb196_handler
    .long   _uprx66t_intb197_handler
    .long   _uprx66t_intb198_handler
    .long   _uprx66t_intb199_handler
    .long   _uprx66t_intb200_handler
    .long   _uprx66t_intb201_handler
    .long   _uprx66t_intb202_handler
    .long   _uprx66t_intb203_handler
    .long   _uprx66t_intb204_handler
    .long   _uprx66t_intb205_handler
    .long   _uprx66t_intb206_handler
    .long   _uprx66t_intb207_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
#ifdef CONFIG_RX66T_PERIA
    .long   _uprx66t_inta208_handler
    .long   _uprx66t_inta209_handler
    .long   _uprx66t_inta210_handler
    .long   _uprx66t_inta211_handler
    .long   _uprx66t_inta212_handler
    .long   _uprx66t_inta213_handler
    .long   _uprx66t_inta214_handler
    .long   _uprx66t_inta215_handler
    .long   _uprx66t_inta216_handler
    .long   _uprx66t_inta217_handler
    .long   _uprx66t_inta218_handler
    .long   _uprx66t_inta219_handler
    .long   _uprx66t_inta220_handler
    .long   _uprx66t_inta221_handler
    .long   _uprx66t_inta222_handler
    .long   _uprx66t_inta223_handler
    .long   _uprx66t_inta224_handler
    .long   _uprx66t_inta225_handler
    .long   _uprx66t_inta226_handler
    .long   _uprx66t_inta227_handler
    .long   _uprx66t_inta228_handler
    .long   _uprx66t_inta229_handler
    .long   _uprx66t_inta230_handler
    .long   _uprx66t_inta231_handler
    .long   _uprx66t_inta232_handler
    .long   _uprx66t_inta233_handler
    .long   _uprx66t_inta234_handler
    .long   _uprx66t_inta235_handler
    .long   _uprx66t_inta236_handler
    .long   _uprx66t_inta237_handler
    .long   _uprx66t_inta238_handler
    .long   _uprx66t_inta239_handler
    .long   _uprx66t_inta240_handler
    .long   _uprx66t_inta241_handler
    .long   _uprx66t_inta242_handler
    .long   _uprx66t_inta243_handler
    .long   _uprx66t_inta244_handler
    .long   _uprx66t_inta245_handler
    .long   _uprx66t_inta246_handler
    .long   _uprx66t_inta247_handler
    .long   _uprx66t_inta248_handler
    .long   _uprx66t_inta249_handler
    .long   _uprx66t_inta250_handler
    .long   _uprx66t_inta251_handler
    .long   _uprx66t_inta252_handler
    .long   _uprx66t_inta253_handler
    .long   _uprx66t_inta254_handler
    .long   _uprx66t_inta255_handler
#else
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
    .long   _uprx66t_invalid_handler
#endif
    .size   __int_vector_table, . - __int_vector_table
    .end

/****************************************************************************
 * EOF
 ***************************************************************************/
