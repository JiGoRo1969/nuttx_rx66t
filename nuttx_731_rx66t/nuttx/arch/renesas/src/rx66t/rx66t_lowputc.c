/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_lowputc.c
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"
#include "rx66t_definitions.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration **********************************************************/

/* Is there a serial console? */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI0)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI1)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI2)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI3)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI4)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI5)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI6)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI7)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI8)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI9)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI10_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI10)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI11_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI11)
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_SCI12_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI12)
#  define HAVE_CONSOLE 1
#else
#  error "Serial console selected, but corresponding SCI not enabled"
#  undef HAVE_CONSOLE
#endif

/* Select UART parameters for the selected console */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI0_BASE
#  define RX_SCI_BAUD     CONFIG_SCI0_BAUD
#  define RX_SCI_BITS     CONFIG_SCI0_BITS
#  define RX_SCI_PARITY   CONFIG_SCI0_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI0_2STOP
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI1_BASE
#  define RX_SCI_BAUD     CONFIG_SCI1_BAUD
#  define RX_SCI_BITS     CONFIG_SCI1_BITS
#  define RX_SCI_PARITY   CONFIG_SCI1_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI1_2STOP
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI2_BASE
#  define RX_SCI_BAUD     CONFIG_SCI2_BAUD
#  define RX_SCI_BITS     CONFIG_SCI2_BITS
#  define RX_SCI_PARITY   CONFIG_SCI2_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI2_2STOP
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI3_BASE
#  define RX_SCI_BAUD     CONFIG_SCI3_BAUD
#  define RX_SCI_BITS     CONFIG_SCI3_BITS
#  define RX_SCI_PARITY   CONFIG_SCI3_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI3_2STOP
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI4_BASE
#  define RX_SCI_BAUD     CONFIG_SCI4_BAUD
#  define RX_SCI_BITS     CONFIG_SCI4_BITS
#  define RX_SCI_PARITY   CONFIG_SCI4_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI4_2STOP
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI5_BASE
#  define RX_SCI_BAUD     CONFIG_SCI5_BAUD
#  define RX_SCI_BITS     CONFIG_SCI5_BITS
#  define RX_SCI_PARITY   CONFIG_SCI5_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI5_2STOP
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI6_BASE
#  define RX_SCI_BAUD     CONFIG_SCI6_BAUD
#  define RX_SCI_BITS     CONFIG_SCI6_BITS
#  define RX_SCI_PARITY   CONFIG_SCI6_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI6_2STOP
#elif defined(CONFIG_SCI7_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI7_BASE
#  define RX_SCI_BAUD     CONFIG_SCI7_BAUD
#  define RX_SCI_BITS     CONFIG_SCI7_BITS
#  define RX_SCI_PARITY   CONFIG_SCI7_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI7_2STOP
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI8_BASE
#  define RX_SCI_BAUD     CONFIG_SCI8_BAUD
#  define RX_SCI_BITS     CONFIG_SCI8_BITS
#  define RX_SCI_PARITY   CONFIG_SCI8_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI8_2STOP
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI9_BASE
#  define RX_SCI_BAUD     CONFIG_SCI9_BAUD
#  define RX_SCI_BITS     CONFIG_SCI9_BITS
#  define RX_SCI_PARITY   CONFIG_SCI9_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI9_2STOP
#elif defined(CONFIG_SCI10_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI10_BASE
#  define RX_SCI_BAUD     CONFIG_SCI10_BAUD
#  define RX_SCI_BITS     CONFIG_SCI10_BITS
#  define RX_SCI_PARITY   CONFIG_SCI10_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI10_2STOP
#elif defined(CONFIG_SCI11_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI11_BASE
#  define RX_SCI_BAUD     CONFIG_SCI11_BAUD
#  define RX_SCI_BITS     CONFIG_SCI11_BITS
#  define RX_SCI_PARITY   CONFIG_SCI11_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI11_2STOP
#elif defined(CONFIG_SCI12_SERIAL_CONSOLE)
#  define RX_SCI_BASE     RX66T_SCI12_BASE
#  define RX_SCI_BAUD     CONFIG_SCI12_BAUD
#  define RX_SCI_BITS     CONFIG_SCI12_BITS
#  define RX_SCI_PARITY   CONFIG_SCI12_PARITY
#  define RX_SCI_2STOP    CONFIG_SCI12_2STOP
#else
#  error "No CONFIG_SCIn_SERIAL_CONSOLE Setting"
#endif

/* Get mode setting */

#if RX_SCI_BITS == 7
#  define RX_SMR_MODE RX_SCISMR_CHR
#elif RX_SCI_BITS == 8
#  define RX_SMR_MODE (0)
#else
#  define RX_SMR_MODE (0)
#endif

#if RX_SCI_PARITY == 0
#  define RX_SMR_PARITY (0)
#elif RX_SCI_PARITY == 1
#  define RX_SMR_PARITY (RX_SCISMR_PE|RX_SCISMR_OE)
#elif RX_SCI_PARITY == 2
#  define RX_SMR_PARITY RX_SCISMR_PE
#else
#  define RX_SMR_PARITY (0)
#endif

#if RX_SCI_2STOP != 0
#  define RX_SMR_STOP RX_SCISMR_STOP
#else
#  define RX_SMR_STOP (0)
#endif

/* The full SMR setting also includes internal clocking with no divisor,
 * aysnchronous operation and multiprocessor disabled:
 */

#define RX_SMR_VALUE (RX_SMR_MODE|RX_SMR_PARITY|RX_SMR_STOP)

/* Clocking ***************************************************************/

#define RX_DIVISOR (8 * RX_SCI_BAUD)
#define RX_BRR     ((RX_PCLKB / RX_DIVISOR) - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return TRUE of the Transmit Data Register is empty
 *
 ****************************************************************************/

#ifdef HAVE_CONSOLE
static inline int up_txready(void)
{
  /* Check the TDRE bit in the SSR.  1=TDR is empty */

  return ((getreg8(RX_SCI_BASE + RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE) != 0);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  uint8_t ssr;

  /* Wait until the TDR is avaible */

  while (!up_txready());

  /* Write the data to the TDR */

  putreg8(ch, RX_SCI_BASE + RX_SCI_TDR_OFFSET);

  /* Clear the TDRE bit in the SSR */

  ssr  = getreg8(RX_SCI_BASE + RX_SCI_SSR_OFFSET);
  ssr &= ~RX_SCISSR_TDRE;
  putreg8(ssr, RX_SCI_BASE + RX_SCI_SSR_OFFSET);
#endif
}

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_SCI_CONFIG)
  uint8_t scr;

  /* Disable the transmitter and receiver */

  scr  = getreg8(RX_SCI_BASE + RX_SCI_SCR_OFFSET);
  scr &= ~(RX_SCISCR_TE | RX_SCISCR_RE);
  putreg8(scr, RX_SCI_BASE + RX_SCI_SCR_OFFSET);

  /* Set communication to be asynchronous with the configured number of data
   * bits, parity, and stop bits.  Use the internal clock (undivided)
   */

  putreg8(RX_SMR_VALUE, RX_SCI_BASE + RX_SCI_SMR_OFFSET);

  /* Set the baud based on the configured console baud and configured
   * system clock.
   */

  putreg8(RX_BRR, RX_SCI_BASE + RX_SCI_BRR_OFFSET);

  /* Select the internal clock source as input */

  scr &= ~RX_SCISCR_CKEMASK;
  putreg8(scr, RX_SCI_BASE + RX_SCI_SCR_OFFSET);

  /* Then enable the transmitter and reciever */

  scr |= (RX_SCISCR_TE | RX_SCISCR_RE);
  putreg8(scr, RX_SCI_BASE + RX_SCI_SCR_OFFSET);
#endif
}
