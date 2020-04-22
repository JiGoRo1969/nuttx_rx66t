/****************************************************************************
 * configs/rx66t/include/board.h
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

#ifndef _CONFIGS_RX66T_BOARD_H
#define _CONFIGS_RX66T_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define	RX_CLK_1MHz			(1000UL * 1000UL)
#define	RX_FCLK				( 60 * RX_CLK_1MHz)
#define	RX_ICLK				(120 * RX_CLK_1MHz)
#define	RX_PCLKA			(120 * RX_CLK_1MHz)
#define	RX_PCLKB			( 60 * RX_CLK_1MHz)
#define	RX_PCLKC			(120 * RX_CLK_1MHz)
#define	RX_PCLKD			( 60 * RX_CLK_1MHz)
#define	RX_BCLK				( 60 * RX_CLK_1MHz)
#define	RX_UCLK				( 48 * RX_CLK_1MHz)

#if		defined(CONFIG_ARCH_BOARD_RX66T)
#define	RX_RESONATOR		( 12 * RX_CLK_1MHz)
#else
#error "Please definition RX_RESONATOR in board.h"
#endif

/* LED definitions **********************************************************/
#if		defined(CONFIG_ARCH_BOARD_RX66T)
#define	LED0				(PORT0.PODR.BIT.B1)
#define	LED1				(PORTE.PODR.BIT.B5)
#define	LED_PORTINIT(X)		{ LED0 = LED1 = (X);		\
	PORT0.ODR0.BIT.B2	=	PORTE.ODR1.BIT.B2	= 0;	\
	PORT0.DSCR.BIT.B1	=	PORTE.DSCR.BIT.B5	= 1;	\
	PORT0.PODR.BIT.B1	=	PORTE.PODR.BIT.B5	= 1;	\
	PORT0.PMR.BIT.B1	=	PORTE.PMR.BIT.B5	= 0;	\
	PORT0.PDR.BIT.B1	=	PORTE.PDR.BIT.B5	= 1;	\
}
#else
#errror "LEDs are not defined."
#endif

#define LED_STARTED       1     /* ON    OFF   NC    NC   */
#define LED_HEAPALLOCATE  0     /* NC    NC    NC    NC   */
#define LED_IRQSENABLED   0     /* NC    NC    NC    NC   */
#define LED_STACKCREATED  2     /* ON    ON    NC    NC   */
#define LED_INIRQ         0     /* NC    NC    NC    NC   */
#define LED_SIGNAL        0     /* NC    NC    NC    NC   */
#define LED_ASSERTION     0     /* NC    NC    NC    NC   */
#define LED_PANIC         3     /* OFF   ON    NC    NC   (flashing 2Hz) */

/* Button definitions *******************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* _CONFIGS_RX66T_BOARD_H */
