/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_cgc.c
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
#include "rx66t_macrodriver.h"
#include "rx66t_cgc.h"
#include "arch/board/board.h"
#include <nuttx/config.h>
#include <sys/types.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r_cgc_create
 *
 * Description:
 * Clock generator Initialization
 ****************************************************************************/

void r_cgc_create(void)
{
	volatile uint16_t	tmp16;
	volatile uint32_t	tmp32;

	/* Protect off. */
	SYSTEM.PRCR.WORD = 0xA50B;
	/* Set the oscillation source of the main clock oscillator. */
	SYSTEM.MOFCR.BIT.MOSEL = 1;		// 1 == BSP_CFG_MAIN_CLOCK_SOURCE
	/* Use HOCO if HOCO is chosen or if PLL is chosen with HOCO as source. */
	/* If HOCO is already operating, it doesn't stop. */
	if (1 == SYSTEM.HOCOCR.BIT.HCSTP)
	{
		/* Turn off power to HOCO. */
		SYSTEM.HOCOPCR.BYTE = 0x01;
	}
	else
	{
		/* WAIT_LOOP */
		while(0 == SYSTEM.OSCOVFSR.BIT.HCOVF)
		{
			/* The delay period needed is to make sure that the HOCO has stabilized. */
			__asm("nop");
		}
	}
	/* Use Main clock if Main clock is chosen or if PLL is chosen with Main clock as source. */
	/* Main clock oscillator is chosen. Start it operating. */
	/* If the main oscillator is >10MHz then the main clock oscillator forced oscillation control register (MOFCR) must
	 *	be changed. */
	/* 16 - 20MHz. */
	SYSTEM.MOFCR.BIT.MODRV2 = 1;
	/* Set the oscillation stabilization wait time of the main clock oscillator. */
	SYSTEM.MOSCWTCR.BYTE = 0x00;
	/* Set the main clock to operating. */
	SYSTEM.MOSCCR.BYTE = 0x00;
	/* Dummy read and compare. cf."5. I/O Registers", "(2) Notes on writing to I/O registers" in User's manual.
	 *	This is done to ensure that the register has been written before the next register access. The RX has a 
	 *	pipeline architecture so the next instruction could be executed before the previous write had finished.
	 */
	if(0x00 ==  SYSTEM.MOSCCR.BYTE)
	{
		__asm("nop");
	}
	/* WAIT_LOOP */
	while(0 == SYSTEM.OSCOVFSR.BIT.MOOVF)
	{
		/* The delay period needed is to make sure that the Main clock has stabilized. */
		__asm("nop");
	}
	/* Set PLL Input Divisor. */
	SYSTEM.PLLCR.BIT.PLIDIV = 1 - 1;	// 1 == BSP_CFG_PLL_DIV
	/* Clear PLL clock source if PLL clock source is Main clock. */
	SYSTEM.PLLCR.BIT.PLLSRCSEL = 0;
	/* Set PLL Multiplier. */
	SYSTEM.PLLCR.BIT.STC = ((uint8_t)((float)20.0 * 2.0)) - 1;	// 20.0 == BSP_CFG_PLL_MUL
	/* Set the PLL to operating. */
	SYSTEM.PLLCR2.BYTE = 0x00;
	/* WAIT_LOOP */
	while(0 == SYSTEM.OSCOVFSR.BIT.PLOVF)
	{
		/* The delay period needed is to make sure that the PLL has stabilized. */
		__asm("nop");
	}
	/* Set SCKCR register. */
	/* Figure out setting for FCK bits. */
	/* Figure out setting for ICK bits. */
	/* Figure out setting for BCK bits. */
	/* Configure PSTOP1 bit for BCLK output. */
	/* Figure out setting for PCKA bits. */
	/* Figure out setting for PCKB bits. */
	/* Figure out setting for PCKC bits. */
	/* Figure out setting for PCKD bits. */
	SYSTEM.SCKCR.LONG = tmp32 = 0x21821212;
	/* Dummy read and compare. cf."5. I/O Registers", "(2) Notes on writing to I/O registers" in User's manual.
	 *	This is done to ensure that the register has been written before the next register access. The RX has a 
	 *	pipeline architecture so the next instruction could be executed before the previous write had finished.
	 */
	if(tmp32 == SYSTEM.SCKCR.LONG)
	{
		__asm("nop");
	}
	/* Figure out setting for UCK bits. */
	/* Set SCKCR2 register. */
	SYSTEM.SCKCR2.WORD = tmp16 = 0x0041;
	/* Dummy read and compare. cf."5. I/O Registers", "(2) Notes on writing to I/O registers" in User's manual.
	 *	This is done to ensure that the register has been written before the next register access. The RX has a 
	 *	pipeline architecture so the next instruction could be executed before the previous write had finished.
	 */
	if(tmp16 == SYSTEM.SCKCR2.WORD)
	{
		__asm("nop");
	}
	/* Choose clock source. Default for r_bsp_config.h is PLL. */
	/* Casting is valid because it matches the type to the retern value. */
	SYSTEM.SCKCR3.WORD = tmp16 = 4 << 8;	// 4 == BSP_CFG_CLOCK_SOURCE
	/* Dummy read and compare. cf."5. I/O Registers", "(2) Notes on writing to I/O registers" in User's manual.
	 *	This is done to ensure that the register has been written before the next register access. The RX has a 
	 *	pipeline architecture so the next instruction could be executed before the previous write had finished.
	 */
	if(tmp16 == SYSTEM.SCKCR3.WORD)
	{
		__asm("nop");
	}
	/* We can now turn LOCO off since it is not going to be used. */
	SYSTEM.LOCOCR.BYTE = 0x01;
	/* Wait for five the LOCO cycles */
	/* 5 count of LOCO : (1000000/216000)*5 = 23.148148148us
	 *	23 + 2 = 25us ("+2" is overhead cycle) */
	up_udelay(25);	// == R_BSP_SoftwareDelay((uint32_t)25, BSP_DELAY_MICROSECS);
	/* Protect on. */
	SYSTEM.PRCR.WORD = 0xA500;
}
