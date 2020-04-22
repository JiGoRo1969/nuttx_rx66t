/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_definitions.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ***************************************************************************/

#ifndef __ARCH_RENESAS_SRC_RX_65N_H
#define __ARCH_RENESAS_SRC_RX_65N_H

/****************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include "rx65n/iodefine.h"
#include "arch/board/board.h"

/****************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Memory-mapped register addresses *************************************************/

#define RX65N_SCI0_BASE       (uint32_t)&SCI0
#define RX65N_SCI1_BASE       (uint32_t)&SCI1
#define RX65N_SCI2_BASE       (uint32_t)&SCI2
#define RX65N_SCI3_BASE       (uint32_t)&SCI3
#define RX65N_SCI4_BASE       (uint32_t)&SCI4
#define RX65N_SCI5_BASE       (uint32_t)&SCI5
#define RX65N_SCI6_BASE       (uint32_t)&SCI6
#define RX65N_SCI7_BASE       (uint32_t)&SCI7
#define RX65N_SCI8_BASE       (uint32_t)&SCI8
#define RX65N_SCI9_BASE       (uint32_t)&SCI9
#define RX65N_SCI10_BASE      (uint32_t)&SCI10
#define RX65N_SCI11_BASE      (uint32_t)&SCI11
#define RX65N_SCI12_BASE      (uint32_t)&SCI12

/* Serial Communications interface (SCI) */

#define RX_SCISMR_CKSMASK  (0x03)        /* Bit 0-1: Internal clock source */
#define RX_SCISMR_DIV1     (0x00)        /*   System clock (phi) */
#define RX_SCISMR_DIV4     (0x01)        /*   phi/4 */
#define RX_SCISMR_DIV16    (0x02)        /*   phi/16 */
#define RX_SCISMR_DIV64    (0x03)        /*   phi/64 */
#define RX_SCISMR_MP       (0x04)        /* Bit 2: Multiprocessor select */
#define RX_SCISMR_STOP     (0x08)        /* Bit 3: 0:One stop bit, 1:Two stop bits */
#define RX_SCISMR_OE       (0x10)        /* Bit 4: 0:Even parity, 1:Odd parity */
#define RX_SCISMR_PE       (0x20)        /* Bit 5: Parity enable */
#define RX_SCISMR_CHR      (0x40)        /* Bit 6: 0:8-bit data, 1:7-bit data */
#define RX_SCISMR_CA       (0x80)        /* Bit 7: 0:Asynchronous, 1:clocked synchronous */
#define RX_SCISCR_CKEMASK  (0x03)        /* Bit 0-1: Internal clock source */

/* Asynchronous mode: */

#define RX_SCISCR_AISIN    (0x00)        /*   Internal clock, SCK pin used for input pin */
#define RX_SCISCR_AISOUT   (0x01)        /*   Internal clock, SCK pin used for clock output */
#define RX_SCISCR_AXSIN1   (0x02)        /*   External clock, SCK pin used for clock input */
#define RX_SCISCR_AXSIN2   (0x03)        /*   External clock, SCK pin used for clock input */

/* Synchronous mode: #define RX_SCISCR_SISOUT1  (0x00)  Internal clock, SCK pin used for input pin */

#define RX_SCISCR_SISOUT2  (0x01)        /*   Internal clock, SCK pin used for clock output */
#define RX_SCISCR_SXSIN1   (0x02)        /*   External clock, SCK pin used for clock input */
#define RX_SCISCR_SXSIN2   (0x03)        /*   External clock, SCK pin used for clock input */
#define RX_SCISCR_TEIE     (0x04)        /* Bit 2: 1=Transmit end interrupt enable */
#define RX_SCISCR_MPIE     (0x08)        /* Bit 3: 1=Multiprocessor interrupt enable */
#define RX_SCISCR_RE       (0x10)        /* Bit 4: 1=Receiver enable */
#define RX_SCISCR_TE       (0x20)        /* Bit 5: 1=Transmitter enable */
#define RX_SCISCR_RIE      (0x40)        /* Bit 6: 1=Recieve-data-full interrupt enable */
#define RX_SCISCR_TIE      (0x80)        /* Bit 7: 1=Transmit-data-empty interrupt enable */
#define RX_SCISCR_ALLINTS  (0xcc)
#define RX_SCISSR_MPBT     (0x01)        /* Bit 0: Multi-processor Bit in Transmit data */
#define RX_SCISSR_MPB      (0x02)        /* Bit 1: Multi-processor Bit in receive data */
#define RX_SCISSR_TEND     (0x04)        /* Bit 2: End of transmission */
#define RX_SCISSR_PER      (0x08)        /* Bit 3: Receive parity error */
#define RX_SCISSR_FER      (0x10)        /* Bit 4: Receive framing error */
#define RX_SCISSR_ORER     (0x20)        /* Bit 5: Receive overrun error */
#define RX_SCISSR_RDRF     (0x40)        /* Bit 6: RDR contains valid received data */
#define RX_SCISSR_TDRE     (0x80)        /* Bit 7: TDR does not contain valid transmit data */
#define RX65N_CMT_CMSTR0_ADDR           (0x00088000)  /* 8-bits wide */
#define RX65N_CMT0_CMCNT_ADDR           (0x00088004)
#define RX65N_CMT0_CMCOR_ADDR           (0x00088006)
#define RX65N_CMT0_CMCR_ADDR            (0x00088002)
#define RX65N_MSTPCRA_ADDR              (0x00080010)
#define RX65N_CMT0_TICKFREQ             (100)           /* 100Hz tick frequency */
#define RX65N_CMT_DIV32                 (0x0001)
#define RX65N_CMT0_DIV_VALUE            (32)
#define RX65N_CMT0_COUNT_VALUE          ((RX_PCLKB / RX65N_CMT0_DIV_VALUE)/(RX65N_CMT0_TICKFREQ))
#define RX65N_CMT_CMCR_INIT             (RX65N_CMT_DIV32 |\
                                        RX65N_CMT_CMCR_CMIE_ENABLE |\
                                        RX65N_CMT_CMCR_DEFAULT)
#define RX65N_CMT_CMCR_DEFAULT          (0x0080)
#define RX65N_CMT_CMCR_CMIE_ENABLE      (0x0040)
#define RX65N_CMT_MSTPCRA_STOP          (0x00008000)
#define RX65N_CMTCMSTR0_STR0            (0x0001)        /* Bit 0: TCNT0 is counting */
#define RX65N_CMTCMSTR0_STR1            (0x0002)        /* Bit 1: TCNT1 is counting */
#define RX65N_PRCR_ADDR                 (0x000803fe)
#define RX65N_PRCR_VALUE                (0xa50b)
#define RX65N_GRPBE0_ADDR               (0x00087600)
#define RX65N_GRPBL0_ADDR               (0x00087630)
#define RX65N_GRPBL1_ADDR               (0x00087634)
#define RX65N_GRPBL2_ADDR               (0x00087638)
#define RX65N_GRPAL0_ADDR               (0x00087830)
#define RX65N_GRPAL1_ADDR               (0x00087834)
#define RX65N_GENBE0_ADDR               (0x00087640)
#define RX65N_GENBL0_ADDR               (0x00087670)
#define RX65N_GENBL1_ADDR               (0x00087674)
#define RX65N_GENBL2_ADDR               (0x00087678)
#define RX65N_GENAL0_ADDR               (0x00087870)
#define RX65N_GENAL1_ADDR               (0x00087874)
#define RX65N_GRPBL0_TEI0_MASK          (1U <<  0)              /* (0x00000001) */
#define RX65N_GRPBL0_ERI0_MASK          (1U <<  1)              /* (0x00000002) */
#define RX65N_GRPBL0_TEI1_MASK          (1U <<  2)              /* (0x00000004) */
#define RX65N_GRPBL0_ERI1_MASK          (1U <<  3)              /* (0x00000008) */
#define RX65N_GRPBL0_TEI2_MASK          (1U <<  4)              /* (0x00000010) */
#define RX65N_GRPBL0_ERI2_MASK          (1U <<  5)              /* (0x00000020) */
#define RX65N_GRPBL0_TEI3_MASK          (1U <<  6)              /* (0x00000040) */
#define RX65N_GRPBL0_ERI3_MASK          (1U <<  7)              /* (0x00000080) */
#define RX65N_GRPBL0_TEI4_MASK          (1U <<  8)              /* (0x00000100) */
#define RX65N_GRPBL0_ERI4_MASK          (1U <<  9)              /* (0x00000200) */
#define RX65N_GRPBL0_TEI5_MASK          (1U << 10)              /* (0x00000400) */
#define RX65N_GRPBL0_ERI5_MASK          (1U << 11)              /* (0x00000800) */
#define RX65N_GRPBL0_TEI6_MASK          (1U << 12)              /* (0x00001000) */
#define RX65N_GRPBL0_ERI6_MASK          (1U << 13)              /* (0x00002000) */
#define RX65N_GRPBL0_TEI7_MASK          (1U << 14)              /* (0x00004000) */
#define RX65N_GRPBL0_ERI7_MASK          (1U << 15)              /* (0x00008000) */
#define RX65N_GRPBL1_TEI8_MASK          (1U << 24)
#define RX65N_GRPBL1_ERI8_MASK          (1U << 25)
#define RX65N_GRPBL1_TEI9_MASK          (1U << 26)
#define RX65N_GRPBL1_ERI9_MASK          (1U << 27)
#define RX65N_GRPAL0_TEI10_MASK         (1U <<  8)
#define RX65N_GRPAL0_ERI10_MASK         (1U <<  9)
#define RX65N_GRPAL0_TEI11_MASK         (1U << 12)
#define RX65N_GRPAL0_ERI11_MASK         (1U << 13)
#define RX65N_GRPBL0_TEI12_MASK         (1U << 16)
#define RX65N_GRPBL0_ERI12_MASK         (1U << 17)

/* General Values LED: */

#if defined(CONFIG_ARCH_BOARD_RX65N_RSK1MB) || defined(CONFIG_ARCH_BOARD_RX65N_RSK2MB)
#define LED_ON          (0)
#define LED_OFF         (1)
#elif defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)
#define LED_ON          (1)
#define LED_OFF         (0)
#else
#error "No Selection for PORT definition in rx65n_port.c"
#endif

/* Bit Set Values */

#define SET_BIT_HIGH    (1)
#define SET_BIT_LOW     (0)
#define SET_BYTE_HIGH   (0xff)
#define SET_BYTE_LOW    (0x00)

/****************************************************************************
 * Public Types
 ***************************************************************************/

/****************************************************************************
 * Public Data
 ***************************************************************************/

#ifndef __ASSEMBLER__
/* Serial Communications interface (SCI) */

  enum   E_RX_SCI
  {
   RX_SCI_SMR_OFFSET = 0,
   RX_SCI_BRR_OFFSET,
   RX_SCI_SCR_OFFSET,
   RX_SCI_TDR_OFFSET,
   RX_SCI_SSR_OFFSET,
   RX_SCI_RDR_OFFSET,
   RX_SCI_SCMR_OFFSET,
   RX_SCI_SEMR_OFFSET,
   RX_SCI_SNFR_OFFSET,
   RX_SCI_SIMR1_OFFSET,
   RX_SCI_SIMR2_OFFSET,
   RX_SCI_SIMR3_OFFSET,
   RX_SCI_SISR_OFFSET,
   RX_SCI_SPMR_OFFSET,
   RX_SCI_THRHL_OFFSET,
   RX_SCI_RDRHL_OFFSET,
   RX_SCI_MDDR_OFFSET
};
#endif  /* __ASSEMBLER__ */

/****************************************************************************
 * Public Functions
 ***************************************************************************/
#endif /* __ARCH_RENESAS_SRC_RX_65N_H */
