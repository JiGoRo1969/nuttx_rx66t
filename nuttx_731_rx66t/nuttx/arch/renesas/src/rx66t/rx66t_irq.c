/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_irq.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

 /* Actually a pointer to the beginning of a uint8_t array */

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_NOINTC
void up_disable_irq(int irq)
{
  if (irq == RX66T_CMI0_IRQ)
    {
      ICU.IER[3].BIT.IEN4 = 0;
    }
#ifdef CONFIG_RX66T_SCI1
  if (irq == RX66T_RXI1_IRQ)
    {
      ICU.IER[7].BIT.IEN4 = 0;
    }
  if (irq == RX66T_TXI1_IRQ)
    {
       ICU.IER[7].BIT.IEN5 = 0;
    }
  if (irq == RX66T_ERI1_IRQ)
    {
       ICU.GRPBL0.BIT.IS3 = 0;
       ICU.GENBLO.BIT.EN3 = 0;
    }
  if (irq == RX66T_TEI1_IRQ)
    {
       ICU.GRPBL0.BIT.IS2 = 0;
       ICU.GENBLO.BIT.EN2 = 0;
    }
#endif

#ifdef CONFIG_RX66T_SCI5
  if (irq == RX66T_RXI5_IRQ)
    {
          ICU.IER[10].BIT.IEN4 = 0;
    }
  if (irq == RX66T_TXI5_IRQ)
    {
          ICU.IER[10].BIT.IEN5 = 0;
    }
  if (irq == RX66T_ERI5_IRQ)
    {
          ICU.GRPBL0.BIT.IS11 = 0;
          ICU.GENBLO.BIT.EN11 = 0;
    }
  if (irq == RX66T_TEI5_IRQ)
    {
          ICU.GRPBL0.BIT.IS10 = 0;
          ICU.GENBLO.BIT.EN10 = 0;
    }
#endif

#ifdef CONFIG_RX66T_SCI6
  if (irq == RX66T_RXI6_IRQ)
    {
          ICU.IER[10].BIT.IEN6 = 0;
    }
  if (irq == RX66T_TXI6_IRQ)
    {
          ICU.IER[10].BIT.IEN7 = 0;
    }
  if (irq == RX66T_ERI6_IRQ)
    {
          ICU.GRPBL0.BIT.IS13 = 0;
          ICU.GENBLO.BIT.EN13 = 0;
    }
  if (irq == RX66T_TEI6_IRQ)
    {
          ICU.GRPBL0.BIT.IS12 = 0;
          ICU.GENBLO.BIT.EN12 = 0;
    }
#endif

#ifdef CONFIG_RX66T_SCI8
  if (irq == RX66T_RXI8_IRQ)
    {
          ICU.IER[12].BIT.IEN4 = 0;
    }
  if (irq == RX66T_TXI8_IRQ)
    {
          ICU.IER[12].BIT.IEN5 = 0;
    }
  if (irq == RX66T_ERI8_IRQ)
    {
          ICU.GRPBL1.BIT.IS25 = 0;
          ICU.GENBL1.BIT.EN25 = 0;
    }
  if (irq == RX66T_TEI8_IRQ)
    {
          ICU.GRPBL1.BIT.IS24 = 0;
          ICU.GENBL1.BIT.EN24 = 0;
    }
#endif

#ifdef CONFIG_RX66T_SCI9
  if (irq == RX66T_RXI9_IRQ)
    {
          ICU.IER[12].BIT.IEN6 = 0;
    }
  if (irq == RX66T_TXI9_IRQ)
    {
          ICU.IER[12].BIT.IEN7 = 0;
    }
  if (irq == RX66T_ERI9_IRQ)
    {
          ICU.GRPBL1.BIT.IS27 = 0;
          ICU.GENBL1.BIT.EN27 = 0;
    }
  if (irq == RX66T_TEI9_IRQ)
    {
          ICU.GRPBL1.BIT.IS26 = 0;
          ICU.GENBL1.BIT.EN26 = 0;
    }
#endif

#ifdef CONFIG_RX66T_SCI11
  if (irq == RX66T_RXI11_IRQ)
   {
          ICU.IER[14].BIT.IEN2 = 0;
   }
  if (irq == RX66T_TXI11_IRQ)
   {
          ICU.IER[14].BIT.IEN3 = 0;
   }
  if (irq == RX66T_ERI11_IRQ)
   {
          ICU.GRPAL0.BIT.IS13 = 0;
          ICU.GRPAL0.BIT.EN13 = 0;
   }
  if (irq == RX66T_TEI11_IRQ)
   {
          ICU.GRPAL0.BIT.IS12 = 0;
          ICU.GRPAL0.BIT.EN12 = 0;
   }
#endif

#ifdef CONFIG_RX66T_SCI12
  if (irq == RX66T_RXI12_IRQ)
   {
          ICU.IER[14].BIT.IEN4 = 0;
   }
  if (irq == RX66T_TXI12_IRQ)
   {
          ICU.IER[14].BIT.IEN5 = 0;
   }
  if (irq == RX66T_ERI12_IRQ)
   {
      ICU.GRPBL0.BIT.IS17 = 0;
      ICU.GENBLO.BIT.EN17 = 0;
   }
  if (irq == RX66T_TEI12_IRQ)
   {
      ICU.GRPBL0.BIT.IS16 = 0;
      ICU.GENBLO.BIT.EN16 = 0;
   }
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq == RX66T_CMI0_IRQ)
   {
      ICU.IER[3].BIT.IEN4 = 1;
   }
#ifdef CONFIG_RX66T_SCI1
  if (irq == RX66T_RXI1_IRQ)
   {
      ICU.IER[7].BIT.IEN4 = 1;
   }
  if (irq == RX66T_TXI1_IRQ)
   {
      ICU.IER[7].BIT.IEN5 = 1;
   }
  if (irq == RX66T_ERI1_IRQ)
   {
      ICU.GRPBL0.BIT.IS3 = 1;
      ICU.GENBLO.BIT.EN3 = 1;
   }
  if (irq == RX66T_TEI1_IRQ)
   {
      ICU.GRPBL0.BIT.IS2 = 1;
      ICU.GENBLO.BIT.EN2 = 1;
   }
#endif

#ifdef CONFIG_RX66T_SCI5
  if (irq == RX66T_RXI5_IRQ)
   {
      ICU.IER[10].BIT.IEN4 = 1;
   }
  if (irq == RX66T_TXI5_IRQ)
   {
      ICU.IER[10].BIT.IEN5 = 1;
   }
  if (irq == RX66T_ERI5_IRQ)
   {
      ICU.GRPBL0.BIT.IS11 = 1;
      ICU.GENBLO.BIT.EN11 = 1;
   }
  if (irq == RX66T_TEI5_IRQ)
   {
      ICU.GRPBL0.BIT.IS10 = 1;
      ICU.GENBLO.BIT.EN10 = 1;
   }
#endif

#ifdef CONFIG_RX66T_SCI6
  if (irq == RX66T_RXI6_IRQ)
   {
      ICU.IER[10].BIT.IEN6 = 1;
   }
  if (irq == RX66T_TXI6_IRQ)
   {
      ICU.IER[10].BIT.IEN7 = 1;
   }
  if (irq == RX66T_ERI6_IRQ)
   {
      ICU.GRPBL0.BIT.IS13 = 1;
      ICU.GENBLO.BIT.EN13 = 1;
   }
  if (irq == RX66T_TEI6_IRQ)
   {
      ICU.GRPBL0.BIT.IS12 = 1;
      ICU.GENBLO.BIT.EN12 = 1;
   }
#endif

#ifdef CONFIG_RX66T_SCI8
  if (irq == RX66T_RXI8_IRQ)
   {
      ICU.IER[12].BIT.IEN4 = 1;
   }
  if (irq == RX66T_TXI8_IRQ)
   {
      ICU.IER[12].BIT.IEN5 = 1;
   }
  if (irq == RX66T_ERI8_IRQ)
   {
      ICU.GRPBL1.BIT.IS25 = 1;
      ICU.GENBL1.BIT.EN25 = 1;
   }
  if (irq == RX66T_TEI8_IRQ)
   {
      ICU.GRPBL1.BIT.IS24 = 1;
      ICU.GENBL1.BIT.EN24 = 1;
   }
#endif

#ifdef CONFIG_RX66T_SCI9
  if (irq == RX66T_RXI9_IRQ)
   {
      ICU.IER[12].BIT.IEN6 = 1;
   }
  if (irq == RX66T_TXI9_IRQ)
   {
      ICU.IER[12].BIT.IEN7 = 1;
   }
  if (irq == RX66T_ERI9_IRQ)
   {
      ICU.GRPBL1.BIT.IS27 = 1;
      ICU.GENBL1.BIT.EN27 = 1;
   }
  if (irq == RX66T_TEI9_IRQ)
   {
      ICU.GRPBL1.BIT.IS26 = 1;
      ICU.GENBL1.BIT.EN26 = 1;
   }
#endif

#ifdef CONFIG_RX66T_SCI11
  if (irq == RX66T_RXI11_IRQ)
   {
      ICU.IER[14].BIT.IEN2 = 1;
   }
  if (irq == RX66T_TXI11_IRQ)
   {
      ICU.IER[14].BIT.IEN3 = 1;
   }
  if (irq == RX66T_ERI11_IRQ)
   {
      ICU.GRPAL0.BIT.IS13 = 1;
      ICU.GRPAL0.BIT.EN13 = 1;
   }
  if (irq == RX66T_TEI11_IRQ)
   {
      ICU.GRPAL0.BIT.IS12 = 1;
      ICU.GRPAL0.BIT.EN12 = 1;
   }
#endif

#ifdef CONFIG_RX66T_SCI12
  if (irq == RX66T_RXI12_IRQ)
    {
      ICU.IER[14].BIT.IEN4 = 1;
    }
  if (irq == RX66T_TXI12_IRQ)
  {
      ICU.IER[14].BIT.IEN5 = 1;
  }
  if (irq == RX66T_ERI12_IRQ)
   {
      ICU.GRPBL0.BIT.IS17 = 1;
      ICU.GENBLO.BIT.EN17 = 1;
   }
  if (irq == RX66T_TEI12_IRQ)
   {
      ICU.GRPBL0.BIT.IS16 = 1;
      ICU.GENBLO.BIT.EN16 = 1;
   }
#endif
}

#endif /* CONFIG_ARCH_NOINTC */
