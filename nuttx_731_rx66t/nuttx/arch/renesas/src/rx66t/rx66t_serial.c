/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include "rx66t_macrodriver.h"
#include "rx66t/iodefine.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "rx66t_definitions.h"
#include "rx66t_sci.h"
#include "rx66t/irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some sanity checks *******************************************************/

#ifdef USE_SERIALDRIVER

/* Which SCI with be tty0/console and which tty1? */

#if 1

#ifdef	CONFIG_RX66T_SCI1
#  define	TTYS1_DEV	g_sci1port		/* SCI1 is tty1 */
#else
#  undef	TTYS1_DEV					/* No tty1 */
#endif
#ifdef	CONFIG_RX66T_SCI5
#  define	TTYS5_DEV	g_sci5port		/* SCI5 is tty5 */
#else
#  undef	TTYS5_DEV					/* No tty5 */
#endif
#ifdef	CONFIG_RX66T_SCI6
#  define	TTYS6_DEV	g_sci6port		/* SCI6 is tty6 */
#else
#  undef	TTYS6_DEV					/* No tty6 */
#endif
#ifdef	CONFIG_RX66T_SCI8
#  define	TTYS8_DEV	g_sci8port		/* SCI8 is tty8 */
#else
#  undef	TTYS8_DEV					/* No tty8 */
#endif
#ifdef	CONFIG_RX66T_SCI9
#  define	TTYS9_DEV	g_sci9port		/* SCI9 is tty9 */
#else
#  undef	TTYS9_DEV					/* No tty9 */
#endif
#ifdef	CONFIG_RX66T_SCI11
#  define	TTYS11_DEV	g_sci11port		/* SCI11 is tty11*/
#else
#  undef	TTYS11_DEV					/* No tty11 */
#endif
#ifdef	CONFIG_RX66T_SCI12
#  define	TTYS12_DEV	g_sci12port		/* SCI12 is tty12 */
#else
#  undef	TTYS12_DEV					/* No tty12 */
#endif

#if defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI1)
#  define		HAVE_CONSOLE
#  define		CONSOLE_DEV		g_sci1port		/* SCI1 is console */
#  define		TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI5_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI5)
#  define		HAVE_CONSOLE
#  define		CONSOLE_DEV		g_sci5port		/* SCI5 is console */
#  define		TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI6_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI6)
#  define		HAVE_CONSOLE
#  define		CONSOLE_DEV		g_sci6port		/* SCI6 is console */
#  define		TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI8_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI8)
#  define		HAVE_CONSOLE
#  define		CONSOLE_DEV		g_sci8port		/* SCI8 is console */
#  define		TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI11_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI11)
#  define		HAVE_CONSOLE
#  define		CONSOLE_DEV		g_sci11port		/* SCI11 is console */
#  define		TTYS0_DEV		CONSOLE_DEV
#elif defined(CONFIG_SCI12_SERIAL_CONSOLE) && defined(CONFIG_RX66T_SCI12)
#  define		HAVE_CONSOLE
#  define		CONSOLE_DEV		g_sci12port		/* SCI12 is console */
#  define		TTYS0_DEV		CONSOLE_DEV
#else	/* No console */
#  undef		HAVE_CONSOLE
#  undef		CONSOLE_DEV
#endif

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
				uint32_t	scibase;	/* Base address of SCI registers */
				uint32_t	baud;		/* Configured baud */
	volatile	uint8_t		scr;		/* Saved SCR value */
	volatile	uint8_t		ssr;		/* Saved SR value (only used during interrupt processing) */
				uint8_t		xmitirq;	/* Base IRQ associated with xmit IRQ */
				uint8_t		recvirq;	/* Base IRQ associated with receive IRQ */
				uint8_t		eriirq;
				uint8_t		teiirq;
				uint32_t	grpibase;
				uint32_t	erimask;
				uint32_t	teimask;
				uint8_t		parity;		/* 0=none, 1=odd, 2=even */
				uint8_t		bits;		/* Number of bits (7 or 8) */
				bool		stopbits2;	/* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, FAR void *arg);
static int  up_eriinterrupt(int irq, void *context, FAR void *arg);
static int  up_teiinterrupt(int irq, void *context, FAR void *arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I/O buffers */

struct uart_ops_s g_sci_ops =
{
	.setup			= up_setup,
	.shutdown		= up_shutdown,
	.attach			= up_attach,
	.detach			= up_detach,
	.receive		= up_receive,
	.rxint			= up_rxint,
	.rxavailable	= up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
	.rxflowcontrol	= NULL,
#endif
	.send			= up_send,
	.txint			= up_txint,
	.txready		= up_txready,
	.txempty		= up_txready,
};

#ifdef CONFIG_RX66T_SCI1
static char		g_sci1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char		g_sci1txbuffer[CONFIG_SCI1_TXBUFSIZE];

static struct up_dev_s g_sci1priv =
{
	.scibase		= RX66T_SCI1_BASE,
	.baud			= CONFIG_SCI1_BAUD,
	.recvirq		= RX66T_RXI1_IRQ,
	.xmitirq		= RX66T_TXI1_IRQ,
	.eriirq			= RX66T_ERI1_IRQ,
	.teiirq			= RX66T_TEI1_IRQ,
	.grpibase		= RX66T_GRPBL0_ADDR,
	.erimask		= RX66T_GRPBL0_ERI1_MASK,
	.teimask		= RX66T_GRPBL0_TEI1_MASK,
	.parity			= CONFIG_SCI1_PARITY,
	.bits			= CONFIG_SCI1_BITS,
	.stopbits2		= CONFIG_SCI1_2STOP,
};

static uart_dev_t g_sci1port =
{
	.recv	= {
		.size		= CONFIG_SCI1_RXBUFSIZE,
		.buffer		= g_sci1rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI1_TXBUFSIZE,
		.buffer		= g_sci1txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci1priv,
};
#endif

#ifdef CONFIG_RX66T_SCI5
static char		g_sci5rxbuffer[CONFIG_SCI5_RXBUFSIZE];
static char		g_sci5txbuffer[CONFIG_SCI5_TXBUFSIZE];

static struct up_dev_s g_sci5priv =
{
	.scibase		= RX66T_SCI5_BASE,
	.baud			= CONFIG_SCI5_BAUD,
	.recvirq		= RX66T_RXI5_IRQ,
	.xmitirq		= RX66T_TXI5_IRQ,
	.eriirq			= RX66T_ERI5_IRQ,
	.teiirq			= RX66T_TEI5_IRQ,
	.grpibase		= RX66T_GRPBL0_ADDR,
	.erimask		= RX66T_GRPBL0_ERI5_MASK,
	.teimask		= RX66T_GRPBL0_TEI5_MASK,
	.parity			= CONFIG_SCI5_PARITY,
	.bits			= CONFIG_SCI5_BITS,
	.stopbits2		= CONFIG_SCI5_2STOP,
};

static uart_dev_t g_sci5port =
{
	.recv	= {
		.size		= CONFIG_SCI5_RXBUFSIZE,
		.buffer		= g_sci5rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI5_TXBUFSIZE,
		.buffer		= g_sci5txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci5priv,
};
#endif

#ifdef CONFIG_RX66T_SCI6
static char		g_sci6rxbuffer[CONFIG_SCI6_RXBUFSIZE];
static char		g_sci6txbuffer[CONFIG_SCI6_TXBUFSIZE];

static struct up_dev_s g_sci6priv =
{
	.scibase		= RX66T_SCI6_BASE,
	.baud			= CONFIG_SCI6_BAUD,
	.recvirq		= RX66T_RXI6_IRQ,
	.xmitirq		= RX66T_TXI6_IRQ,
	.eriirq			= RX66T_ERI6_IRQ,
	.teiirq			= RX66T_TEI6_IRQ,
	.grpibase		= RX66T_GRPBL0_ADDR,
	.erimask		= RX66T_GRPBL0_ERI6_MASK,
	.teimask		= RX66T_GRPBL0_TEI6_MASK,
	.parity			= CONFIG_SCI6_PARITY,
	.bits			= CONFIG_SCI6_BITS,
	.stopbits2		= CONFIG_SCI6_2STOP,
};

static uart_dev_t g_sci6port =
{
	.recv	= {
		.size		= CONFIG_SCI6_RXBUFSIZE,
		.buffer		= g_sci6rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI6_TXBUFSIZE,
		.buffer		= g_sci6txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci6priv,
};
#endif

#ifdef CONFIG_RX66T_SCI8
static char		g_sci8rxbuffer[CONFIG_SCI8_RXBUFSIZE];
static char		g_sci8txbuffer[CONFIG_SCI8_TXBUFSIZE];

static struct up_dev_s g_sci8priv =
{
	.scibase		= RX66T_SCI8_BASE,
	.baud			= CONFIG_SCI8_BAUD,
	.recvirq		= RX66T_RXI8_IRQ,
	.xmitirq		= RX66T_TXI8_IRQ,
	.eriirq			= RX66T_ERI8_IRQ,
	.teiirq			= RX66T_TEI8_IRQ,
	.grpibase		= RX66T_GRPBL1_ADDR,
	.erimask		= RX66T_GRPBL1_ERI8_MASK,
	.teimask		= RX66T_GRPBL1_TEI8_MASK,
	.parity			= CONFIG_SCI8_PARITY,
	.bits			= CONFIG_SCI8_BITS,
	.stopbits2		= CONFIG_SCI8_2STOP,
};

static uart_dev_t g_sci8port =
{
	.recv	= {
		.size		= CONFIG_SCI8_RXBUFSIZE,
		.buffer		= g_sci8rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI8_TXBUFSIZE,
		.buffer		= g_sci8txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci8priv,
};
#endif

#ifdef CONFIG_RX66T_SCI9
static char		g_sci9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
static char		g_sci9txbuffer[CONFIG_SCI9_TXBUFSIZE];

static struct up_dev_s g_sci9priv =
{
	.scibase		= RX66T_SCI9_BASE,
	.baud			= CONFIG_SCI9_BAUD,
	.recvirq		= RX66T_RXI9_IRQ,
	.xmitirq		= RX66T_TXI9_IRQ,
	.eriirq			= RX66T_ERI9_IRQ,
	.teiirq			= RX66T_TEI9_IRQ,
	.grpibase		= RX66T_GRPBL1_ADDR,
	.erimask		= RX66T_GRPBL1_ERI9_MASK,
	.teimask		= RX66T_GRPBL1_TEI9_MASK,
	.parity			= CONFIG_SCI9_PARITY,
	.bits			= CONFIG_SCI9_BITS,
	.stopbits2		= CONFIG_SCI9_2STOP,
};

static uart_dev_t g_sci9port =
{
	.recv	= {
		.size		= CONFIG_SCI9_RXBUFSIZE,
		.buffer		= g_sci9rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI9_TXBUFSIZE,
		.buffer		= g_sci9txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci9priv,
};
#endif

#if 1	// def CONFIG_RX66T_SCI11
static char		g_sci11rxbuffer[CONFIG_SCI11_RXBUFSIZE];
static char		g_sci11txbuffer[CONFIG_SCI11_TXBUFSIZE];

static struct up_dev_s g_sci11priv =
{
	.scibase		= RX66T_SCI11_BASE,
	.baud			= CONFIG_SCI11_BAUD,
	.recvirq		= RX66T_RXI11_IRQ,
	.xmitirq		= RX66T_TXI11_IRQ,
	.eriirq			= RX66T_ERI11_IRQ,
	.teiirq			= RX66T_TEI11_IRQ,
	.grpibase		= RX66T_GRPAL0_ADDR,
	.erimask		= RX66T_GRPAL0_ERI11_MASK,
	.teimask		= RX66T_GRPAL0_TEI11_MASK,
	.parity			= CONFIG_SCI11_PARITY,
	.bits			= CONFIG_SCI11_BITS,
	.stopbits2		= CONFIG_SCI11_2STOP,
};

static uart_dev_t g_sci11port =
{
	.recv	= {
		.size		= CONFIG_SCI11_RXBUFSIZE,
		.buffer		= g_sci11rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI11_TXBUFSIZE,
		.buffer		= g_sci11txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci11priv,
};
#endif

#ifdef CONFIG_RX66T_SCI12
static char		g_sci12rxbuffer[CONFIG_SCI12_RXBUFSIZE];
static char		g_sci12txbuffer[CONFIG_SCI12_TXBUFSIZE];

static struct up_dev_s g_sci12priv =
{
	.scibase		= RX66T_SCI12_BASE,
	.baud			= CONFIG_SCI12_BAUD,
	.recvirq		= RX66T_RXI12_IRQ,
	.xmitirq		= RX66T_TXI12_IRQ,
	.eriirq			= RX66T_ERI12_IRQ,
	.teiirq			= RX66T_TEI12_IRQ,
	.grpibase		= RX66T_GRPBL0_ADDR,
	.erimask		= RX66T_GRPBL0_ERI12_MASK,
	.teimask		= RX66T_GRPBL0_TEI12_MASK,
	.parity			= CONFIG_SCI12_PARITY,
	.bits			= CONFIG_SCI12_BITS,
	.stopbits2		= CONFIG_SCI12_2STOP,
};

static uart_dev_t g_sci12port =
{
	.recv	= {
		.size		= CONFIG_SCI12_RXBUFSIZE,
		.buffer		= g_sci12rxbuffer,
	},
	.xmit	= {
		.size		= CONFIG_SCI12_TXBUFSIZE,
		.buffer		= g_sci12txbuffer,
	},
	.ops			= &g_sci_ops,
	.priv			= &g_sci12priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/
static inline
uint8_t         up_serialin(struct up_dev_s *priv, int offset)
{
		return getreg8(priv->scibase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/
static inline
void	up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
	putreg8(value, priv->scibase + offset);
}

/****************************************************************************
 * Name: up_disablesciint
 ****************************************************************************/
static inline
void	up_disablesciint(struct up_dev_s *priv, uint8_t *scr)
{
	if(scr)
	{
		*scr = priv->scr;
	}
	/* The disable all interrupts */
	priv->scr &= ~RX_SCISCR_ALLINTS;
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_restoresciint
 ****************************************************************************/
static inline
void	up_restoresciint(struct up_dev_s *priv, uint8_t scr)
{
	/* Set the interrupt bits in the scr value */
	priv->scr  &= ~RX_SCISCR_ALLINTS;
	priv->scr |= (scr & RX_SCISCR_ALLINTS);
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/
#ifdef HAVE_CONSOLE
static inline
void	up_waittxready(struct up_dev_s *priv)
{
	int		tmp;

	/* Limit how long we will wait for the TDR empty condition */
	for(tmp = 1000; tmp > 0 ; tmp--) {
		/* Check if the TDR is empty.  The TDR becomes empty when:  (1) the
		 * the chip is reset or enters standby mode, (2) the TE bit in the SCR
		 * is cleared, or (3) the current TDR contents are loaded in the TSR so
		 * that new data can be written in the TDR.
		 */
		if(0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE))
		{
			/* The TDR is empty... return */
			break;
		}
	}
}
#endif

/****************************************************************************
 * Name: up_setbrr
 *
 * Description:
 *   Calculate the correct value for the BRR given the configured frequency
 *   and the desired BAUD settings.
 *
 ****************************************************************************/
static inline
void	up_setbrr(struct up_dev_s *priv, unsigned int baud)
{
	uint32_t	t_baud, t_brr1, brrdiv, t_pclk_divbrr;
	float		f_mddr;
	uint8_t		semr, mddr, brr;

	semr = up_serialin(priv, RX_SCI_SEMR_OFFSET);
	brrdiv = 32U;
	if(0U != (0x10 & semr))
	{
		brrdiv /= 2U;
	}
	if(0U != (0x40 & semr))
	{
		brrdiv /= 2U;
	}
	if(RX66T_SCI11_BASE == prev->scibase) {
		t_pclk_divbrr = RX_PCLKA / brrdiv;
	}
	else {
		t_pclk_divbrr = RX_PCLKB / brrdiv;
	}
	t_brr1 = t_pclk_divbrr / baud;
	t_baud = t_pclk_divbrr / t_brr1;
	while(t_baud < baud)
	{
		t_brr1--;
		t_baud = t_pclk_divbrr / t_brr1;
	}
	brr  = t_brr1 - 1;
	f_mddr = ((float)baud * 256.0f) / (float)t_baud + 0.5f;
	mddr = ((256.0f <= f_mddr) || (0.0f > f_mddr)) ? 0 : (uint8_t)f_mddr;
	if(0 < mddr)
	{
		semr |= 0x04; // BRME(0x04) = 1;
	}
	else
	{ // mddr == 0
		semr &= 0xF1; // BRME(0x04) = 0;
		mddr = 255U;
	}
	semr |= 0xA0; // RXDE(0x80) = NFEN(0x20) = 1;
	up_serialout(priv, RX_SCI_SEMR_OFFSET, semr);
	up_serialout(priv, RX_SCI_BRR_OFFSET, brr);
	up_serialout(priv, RX_SCI_MDDR_OFFSET, mddr);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the SCI baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/
static
int		up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_SCI_CONFIG
	struct up_dev_s	*priv;
	uint8_t			smr;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable the transmitter and receiver */
	priv->scr  = up_serialin(priv, RX_SCI_SCR_OFFSET);
	priv->scr &= ~(RX_SCISCR_TE | RX_SCISCR_RE);
	priv->scr &= ~RX_SCISCR_CKEMASK;
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	/* Set communication to be asynchronous with the configured number of data
	 * bits, parity, and stop bits.  Use the internal clock (undivided)
	 */
	smr = 0;
	if(7 == priv->bits)
	{
		smr |= RX_SCISMR_CHR;
	}
	if(1 == priv->parity)
	{
		smr |= (RX_SCISMR_PE|RX_SCISMR_OE);
	}
	else if(2 == priv->parity)
	{
		smr |= RX_SCISMR_PE;
	}
	if(priv->stopbits2)
	{
		smr |= RX_SCISMR_STOP;
	}
	up_serialout(priv, RX_SCI_SMR_OFFSET, smr);
	/* Set the baud based on the configured console baud and configured
	 * system clock.
	 */
	up_setbrr(priv, priv->baud);
	/* Then enable the transmitter and reciever */
	priv->scr |= (RX_SCISCR_TE | RX_SCISCR_RE);
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
#endif
	return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.  This method is called when the serial port is closed
 *
 ****************************************************************************/
static
void	up_shutdown(struct uart_dev_s *dev)
{
	struct up_dev_s         *priv;

	priv = (struct up_dev_s*)dev->priv;
	up_disablesciint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/
static
int		up_attach(struct uart_dev_s *dev)
{
	struct up_dev_s	*priv;
	int				ret;

	priv = (struct up_dev_s*)dev->priv;
	/* Attach the RDR full IRQ (RXI) that is enabled by the RIE SCR bit */
	ret = irq_attach(priv->recvirq, up_interrupt, dev);
	if(OK != ret)
	{
		return ret;
	}
	/* Attach the TDR empty IRQ (TXI) enabled by the TIE SCR bit */
	ret = irq_attach(priv->xmitirq, up_interrupt, dev);
	if(OK != ret)
	{
		return ret;
	}
	/* Attach the ERI IRQ */
	ret = irq_attach(priv->eriirq, up_eriinterrupt, dev);
	if(OK != ret)
	{
		return ret;
	}
	/* Attach the TEI IRQ */
	ret = irq_attach(priv->teiirq, up_teiinterrupt, dev);
	if(OK == ret)
	{
#ifdef CONFIG_ARCH_IRQPRIO
		/* All SCI0 interrupts share the same prioritization */
		up_prioritize_irq(priv->recvirq, 7);  /* Set SCI priority midway */
		up_prioritize_irq(priv->xmitirq, 7);
#endif
		/* Return OK on success */
		return OK;
	}
	(void)irq_detach(priv->recvirq);
	(void)irq_detach(priv->xmitirq);
	(void)irq_detach(priv->eriirq);
	return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach SCI interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/
static
void	up_detach(struct uart_dev_s *dev)
{
	struct up_dev_s *priv;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable all SCI interrupts */
	up_disablesciint(priv, NULL);
	/* Detach the SCI interrupts */
	(void)irq_detach(priv->recvirq);
	(void)irq_detach(priv->xmitirq);
}

static
int		up_eriinterrupt(int irq, void *context, void *arg)
{
	struct uart_dev_s	*dev;
	struct up_dev_s		*priv;
	uint32_t			grpreg;

	dev = (struct uart_dev_s *)arg;
	DEBUGASSERT((NULL != dev) && (NULL != priv));
	priv = (struct up_dev_s*)dev->priv;
	grpreg = getreg32(priv->grpibase);
	if(grpreg|priv->erimask)
	{
		/* Get the current SCI status  */
		priv->ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
		/* Clear all read related events (probably already done in up_receive)) */
		priv->ssr &= ~(RX_SCISSR_ORER|RX_SCISSR_FER|RX_SCISSR_PER);
		up_serialout(priv, RX_SCI_SSR_OFFSET, priv->ssr);
	}
	return OK;
}

static
int		up_teiinterrupt(int irq, void *context, void *arg)
{
	struct uart_dev_s	*dev;
	struct up_dev_s		*priv;
	uint32_t			grpreg;

	dev = (struct uart_dev_s *)arg;
	DEBUGASSERT((NULL != dev) && (NULL != priv));
	priv = (struct up_dev_s*)dev->priv;
	grpreg = getreg32(priv->grpibase);
	if(grpreg|priv->teimask)
	{
		/* Get the current SCI status  */
		priv->ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
		/* Clear all read related events (probably already done in up_receive)) */
		priv->ssr &= ~(RX_SCISSR_TEND);
		up_serialout(priv, RX_SCI_SSR_OFFSET, priv->ssr);
	}
	return OK;
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the SCI interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   up_dev_s structure in order to call these functions.
 *
 ****************************************************************************/
static
int		up_interrupt(int irq, void *context, FAR void *arg)
{
	struct uart_dev_s	*dev;
	struct up_dev_s		*priv;

	dev = (struct uart_dev_s *)arg;
	DEBUGASSERT((NULL != dev) && (NULL != priv));
	priv = (struct up_dev_s*)dev->priv;
	/* Get the current SCI status  */
	priv->ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
	/* Handle receive-related events with RIE is enabled.  RIE is enabled at
	 * times that driver is open EXCEPT when the driver is actively copying
	 * data from the circular buffer.  In that case, the read events must
	 * pend until RIE is set
	 */
	if(0 != (priv->scr & RX_SCISCR_RIE))
	{
		/* Handle incoming, receive bytes (RDRF: Receive Data Register Full) */
		/* Setting RDRF bit when data is entered as input */
		if ((priv->ssr & RX_SCISSR_RDRF) != 0) {
			/* Rx data register not empty ... process incoming bytes */
			uart_recvchars(dev);
		}
		/* Clear all read related events (probably already done in up_receive)) */
		priv->ssr &= ~(RX_SCISSR_RDRF|RX_SCISSR_ORER|RX_SCISSR_FER|RX_SCISSR_PER);
	}
	/* Handle outgoing, transmit bytes (TDRE: Transmit Data Register Empty)
	 * when TIE is enabled.  TIE is only enabled when the driver is waiting with
	 * buffered data.  Since TDRE is usually true,
	 */
	if((0 != (priv->ssr & RX_SCISSR_TDRE)) && (0 != (priv->scr & RX_SCISCR_TIE)))
	{
		/* Tx data register empty ... process outgoing bytes */
		uart_xmitchars(dev);
		/* Clear the TDR empty flag (Possibly done in up_send, will have not
		 * effect if the TDR is still empty)
		 */
		priv->ssr &= ~(RX_SCISSR_TDRE|RX_SCISSR_TEND|RX_SCISSR_MPBT);
	}
	/* Clear all (clear-able) status flags.  Note that that RX65-1 requires
	 * that you read the bit in the "1" then write "0" to the bit in order
	 * to clear it.  Any bits in the SSR that transitioned from 0->1 after
	 * we read the SR will not be effected by the following:
	 */
	up_serialout(priv, RX_SCI_SSR_OFFSET, priv->ssr);
	return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/
static
int		up_receive(struct uart_dev_s *dev, unsigned int *status)
{
	struct up_dev_s	*priv;
	uint8_t			rdr;
	uint8_t			ssr;

	priv = (struct up_dev_s*)dev->priv;
	/* Read the character from the RDR port */
	rdr  = up_serialin(priv, RX_SCI_RDR_OFFSET);
	/* Clear all read related status in  real ssr (so that when when rxavailable
	 * is called again, it will return false.
	 */
	ssr = up_serialin(priv, RX_SCI_SSR_OFFSET);
	ssr &= ~(RX_SCISSR_RDRF|RX_SCISSR_ORER|RX_SCISSR_FER|RX_SCISSR_PER);
	up_serialout(priv, RX_SCI_SSR_OFFSET, ssr);
	/* For status, return the SSR at the time that the interrupt was received */
	*status = (uint32_t)priv->ssr << 8 | rdr;
	/* Return the received character */
	return (int)rdr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/
static
void	up_rxint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s	*priv;
	irqstate_t		flags;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable interrupts to prevent asynchronous accesses */
	flags = enter_critical_section();
	/* Are we enabling or disabling? */
	if(enable) {
		/* Enable the RDR full interrupt */
		priv->scr |= RX_SCISCR_RIE;
	}
	else {
		/* Disable the RDR full interrupt */
		priv->scr &= ~RX_SCISCR_RIE;
	}
	/* Write the modified SCR value to hardware */
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the RDR is not empty
 *
 ****************************************************************************/
static
bool	up_rxavailable(struct uart_dev_s *dev)
{
	struct up_dev_s	*priv;

	/* Return true if the RDR full bit is set in the SSR */
	priv = (struct up_dev_s*)dev->priv;
	return (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_RDRF));
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/
static
void	up_send(struct uart_dev_s *dev, int ch)
{
	struct up_dev_s	*priv;
	uint8_t			ssr;

	priv = (struct up_dev_s*)dev->priv;
	/* Write the data to the TDR */
	up_serialout(priv, RX_SCI_TDR_OFFSET, (uint8_t)ch);
	/* Clear the TDRE bit in the SSR */
	ssr  = up_serialin(priv, RX_SCI_SSR_OFFSET);
	ssr &= ~RX_SCISSR_TDRE;
	up_serialout(priv, RX_SCI_SSR_OFFSET, ssr);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/
static
void	up_txint(struct uart_dev_s *dev, bool enable)
{
	struct up_dev_s	*priv;
	irqstate_t		flags;

	priv = (struct up_dev_s*)dev->priv;
	/* Disable interrupts to prevent asynchronous accesses */
	flags = enter_critical_section();
	/* Are we enabling or disabling? */
	if(enable)
	{
		/* Enable the TDR empty interrupt */
		priv->scr |= RX_SCISCR_TIE;
		/* If the TDR is already empty, then don't wait for the interrupt */
#if 1
		if (up_txready(dev)) {
			/* Tx data register empty ... process outgoing bytes.  Note:
			 * this could call up_txint to be called recursively.  However,
			 * in this event, priv->scr should hold the correct value upon
			 * return from uuart_xmitchars().
			 */
			uart_xmitchars(dev);
		}
#endif
	}
	else
	{
		/* Disable the TDR empty interrupt */
		priv->scr &= ~RX_SCISCR_TIE;
	}
	/* Write the modified SCR value to hardware */
	up_serialout(priv, RX_SCI_SCR_OFFSET, priv->scr);
	leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the TDR is empty
 *
 ****************************************************************************/
static
bool	up_txready(struct uart_dev_s *dev)
{
	struct up_dev_s *priv;

	priv = (struct up_dev_s*)dev->priv;
	return (0 != (up_serialin(priv, RX_SCI_SSR_OFFSET) & RX_SCISSR_TDRE));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level SCI initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_consoleinit.
 *
 ****************************************************************************/
void	up_earlyconsoleinit(void)
{
	/* NOTE:  All GPIO configuration for the SCIs was performed in
	 * up_lowsetup
	 */
	/* Disable all SCIs */
#ifdef CONFIG_RX66T_SCI1
	up_disablesciint(TTYS1_DEV.priv, NULL);
#endif
#ifdef CONFIG_RX66T_SCI5
	up_disablesciint(TTYS5_DEV.priv, NULL);
#endif
#ifdef CONFIG_RX66T_SCI6
	up_disablesciint(TTYS6_DEV.priv, NULL);
#endif
#ifdef CONFIG_RX66T_SCI8
	up_disablesciint(TTYS8_DEV.priv, NULL);
#endif
#ifdef CONFIG_RX66T_SCI9
	up_disablesciint(TTYS9_DEV.priv, NULL);
#endif
#ifdef CONFIG_RX66T_SCI11
	up_disablesciint(TTYS11_DEV.priv, NULL);
#endif
#ifdef CONFIG_RX66T_SCI12
	up_disablesciint(TTYS12_DEV.priv, NULL);
#endif
	/* Configuration whichever one is the console */
#ifdef HAVE_CONSOLE
	CONSOLE_DEV.isconsole = true;
	up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyconsoleinit was called previously.
 *
 ****************************************************************************/
void    up_serialinit(void)
{
	/* Register all SCIs */
#ifdef CONFIG_RX66T_SCI1
	R_SCI1_Create();
	up_setup(&CONSOLE_DEV);
	R_SCI1_Start();
	SCI1.SCR.BIT.TE = 1U;
#if	defined(TTYS1_DEV)
	(void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#endif
#ifdef CONFIG_RX66T_SCI5
	R_SCI5_Create();
	up_setup(&g_sci5port);
	R_SCI5_Start();
	SCI5.SCR.BIT.TE = 1U;
#if	defined(TTYS5_DEV)
	(void)uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
#endif
#ifdef CONFIG_RX66T_SCI6
	R_SCI6_Create();
	up_setup(&g_sci6port);
	R_SCI6_Start();
	SCI6.SCR.BIT.TE = 1U;
#if	defined(TTYS6_DEV)
	(void)uart_register("/dev/ttyS6", &TTYS6_DEV);
#endif
#endif
#ifdef CONFIG_RX66T_SCI8
	R_SCI8_Create();
	up_setup(&g_sci8port);
	R_SCI8_Start();
	SCI8.SCR.BIT.TE = 1U;
#if	defined(TTYS8_DEV)
	(void)uart_register("/dev/ttyS8", &TTYS8_DEV);
#endif
#endif
#ifdef CONFIG_RX66T_SCI9
	R_SCI9_Create();
	up_setup(&g_sci9port);
	R_SCI9_Start();
	SCI9.SCR.BIT.TE = 1U;
#if	defined(TTYS9_DEV)
	(void)uart_register("/dev/ttyS9", &TTYS9_DEV);
#endif
#endif
#ifdef CONFIG_RX66T_SCI11
	R_SCI11_Create();
	up_setup(&g_sci11port);
	R_SCI11_Start();
	SCI11.SCR.BIT.TE = 1U;
  #if defined(TTYS11_DEV)
	(void)uart_register("/dev/ttyS11", &TTYS11_DEV);
  #endif
#endif
#ifdef CONFIG_RX66T_SCI12
	R_SCI12_Create();
	up_setup(&g_sci12port);
	R_SCI12_Start();
	SCI12.SCR.BIT.TE = 1U;
#if	defined(TTYS12_DEV)
	(void)uart_register("/dev/ttyS12", &TTYS12_DEV);
#endif
#endif
	/* Register the console */
#ifdef HAVE_CONSOLE
	(void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/
int		up_putc(int ch)
{
#ifdef HAVE_CONSOLE
	struct up_dev_s *priv;
	uint8_t  scr;

	priv = (struct up_dev_s*)CONSOLE_DEV.priv;
	up_disablesciint(priv, &scr);
	/* Check for LF */
	if(ch == '\n') {
		/* Add CR */
		up_waittxready(priv);
		up_serialout(priv, RX_SCI_TDR_OFFSET, '\r');
	}
	up_waittxready(priv);
	up_serialout(priv, RX_SCI_TDR_OFFSET, (uint8_t)ch);
	up_waittxready(priv);
	up_restoresciint(priv, scr);
#endif
	return ch;
}
#else /* USE_SERIALDRIVER */
/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/
int		up_putc(int ch)
{
#ifdef HAVE_CONSOLE
	/* Check for LF */
	if(ch == '\n') {
		/* Add CR */
		up_lowputc('\r');
	}
	up_lowputc(ch);
#endif
	return ch;
}

#endif /* USE_SERIALDRIVER */
