/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_icu.c
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
#include "rx66t_icu.h"

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
 * Name: r_icu_create
 *
 * Description:
 * ICU Initialization
 ****************************************************************************/
void r_icu_create(void)
{
	/* Disable IRQ interrupts */
	ICU.IER[0x08].BYTE = 0x00U;
	ICU.IER[0x09].BYTE = 0x00U;
	/* Disable software interrupt */
	IEN(ICU,SWINT) = 0U;
	IEN(ICU,SWINT2) = 0U;
	/* Disable IRQ digital filter */
	ICU.IRQFLTE0.BYTE &= ~(_01_ICU_IRQ0_FILTER_ENABLE | _02_ICU_IRQ1_FILTER_ENABLE | _04_ICU_IRQ2_FILTER_ENABLE | 
						   _08_ICU_IRQ3_FILTER_ENABLE | _10_ICU_IRQ4_FILTER_ENABLE | _20_ICU_IRQ5_FILTER_ENABLE);
	ICU.IRQFLTE1.BYTE &= ~( _80_ICU_IRQ15_FILTER_ENABLE);
	/* Set IRQ digital filter sampling clock */
	ICU.IRQFLTC0.BIT.FCLKSEL0 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	ICU.IRQFLTC0.BIT.FCLKSEL1 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	ICU.IRQFLTC0.BIT.FCLKSEL2 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	ICU.IRQFLTC0.BIT.FCLKSEL3 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	ICU.IRQFLTC0.BIT.FCLKSEL4 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	ICU.IRQFLTC0.BIT.FCLKSEL5 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	ICU.IRQFLTC1.BIT.FCLKSEL15 = _0003_ICU_IRQ0_FILTER_PCLK_64;
	/* Set IRQ detection type */
	ICU.IRQCR[0].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ0) = 0U;
	ICU.IRQCR[1].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ1) = 0U;
	ICU.IRQCR[2].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ2) = 0U;
	ICU.IRQCR[3].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ3) = 0U;
	ICU.IRQCR[4].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ4) = 0U;
	ICU.IRQCR[5].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ5) = 0U;
	ICU.IRQCR[15].BYTE = _04_ICU_IRQ_EDGE_FALLING;
	IR(ICU,IRQ15) = 0U;
	/* Enable IRQ digital filter */
	ICU.IRQFLTE0.BYTE |= _01_ICU_IRQ0_FILTER_ENABLE | _02_ICU_IRQ1_FILTER_ENABLE | _04_ICU_IRQ2_FILTER_ENABLE | 
						 _08_ICU_IRQ3_FILTER_ENABLE | _10_ICU_IRQ4_FILTER_ENABLE | _20_ICU_IRQ5_FILTER_ENABLE;
	ICU.IRQFLTE1.BYTE |= _80_ICU_IRQ15_FILTER_ENABLE;
	/* Set SWINT/SWINT2 priority level */
	IPR(ICU,SWINT) = _0F_ICU_PRIORITY_LEVEL15;
	/* Set IRQ priority level */
	IPR(ICU,IRQ0) = _0B_ICU_PRIORITY_LEVEL11;
	IPR(ICU,IRQ1) = _0B_ICU_PRIORITY_LEVEL11;
	IPR(ICU,IRQ2) = _0B_ICU_PRIORITY_LEVEL11;
	IPR(ICU,IRQ3) = _0B_ICU_PRIORITY_LEVEL11;
	IPR(ICU,IRQ4) = _0B_ICU_PRIORITY_LEVEL11;
	IPR(ICU,IRQ5) = _0B_ICU_PRIORITY_LEVEL11;
	IPR(ICU,IRQ15) = _0B_ICU_PRIORITY_LEVEL11;
	/* Set IRQ0 pin */
	MPC.P52PFS.BYTE = 0x40U;
	PORT5.PDR.BYTE &= 0xFBU;
	PORT5.PMR.BYTE &= 0xFBU;
	/* Set IRQ1 pin */
	MPC.PE4PFS.BYTE = 0x40U;
	PORTE.PDR.BYTE &= 0xEFU;
	PORTE.PMR.BYTE &= 0xEFU;
	/* Set IRQ2 pin */
	MPC.P00PFS.BYTE = 0x40U;
	PORT0.PDR.BYTE &= 0xFEU;
	PORT0.PMR.BYTE &= 0xFEU;
	/* Set IRQ3 pin */
	MPC.PB4PFS.BYTE = 0x40U;
	PORTB.PDR.BYTE &= 0xEFU;
	PORTB.PMR.BYTE &= 0xEFU;
	/* Set IRQ4 pin */
	MPC.PB1PFS.BYTE = 0x40U;
	PORTB.PDR.BYTE &= 0xFDU;
	PORTB.PMR.BYTE &= 0xFDU;
	/* Set IRQ5 pin */
	MPC.PD6PFS.BYTE = 0x40U;
	PORTD.PDR.BYTE &= 0xBFU;
	PORTD.PMR.BYTE &= 0xBFU;
	/* Set IRQ15 pin */
	MPC.PE1PFS.BYTE = 0x40U;
	PORTE.PDR.BYTE &= 0xFDU;
	PORTE.PMR.BYTE &= 0xFDU;
}

/****************************************************************************
 * Name: r_icu_irq0_start
 *
 * Description:
 * Enable IRQ0 Interrupt
 ****************************************************************************/

void r_icu_irq0_start(void)
{
    /* Enable IRQ0 interrupt */

    IEN(ICU, IRQ0) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq0_stop
 *
 * Description:
 *Initialize IRQ0 Interrupt
 ****************************************************************************/

void r_icu_irq0_stop(void)
{
    /* Disable IRQ0 interrupt */

    IEN(ICU, IRQ0) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq1_start
 *
 * Description:
 * Enable IRQ1 Interrupt
 ****************************************************************************/

void r_icu_irq1_start(void)
{
    /* Enable IRQ1 interrupt */

    IEN(ICU, IRQ1) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq1_stop
 *
 * Description:
 *Initialize IRQ1 Interrupt
 ****************************************************************************/

void r_icu_irq1_stop(void)
{
    /* Disable IRQ1 interrupt */

    IEN(ICU, IRQ1) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq2_start
 *
 * Description:
 * Enable IRQ2 Interrupt
 ****************************************************************************/

void r_icu_irq2_start(void)
{
    /* Enable IRQ2 interrupt */

    IEN(ICU, IRQ2) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq2_stop
 *
 * Description:
 *Initialize IRQ2 Interrupt
 ****************************************************************************/

void r_icu_irq2_stop(void)
{
    /* Disable IRQ2 interrupt */

    IEN(ICU, IRQ2) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq3_start
 *
 * Description:
 * Enable IRQ3 Interrupt
 ****************************************************************************/

void r_icu_irq3_start(void)
{
    /* Enable IRQ3 interrupt */

    IEN(ICU, IRQ3) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq3_stop
 *
 * Description:
 *Initialize IRQ3 Interrupt
 ****************************************************************************/

void r_icu_irq3_stop(void)
{
    /* Disable IRQ3 interrupt */

    IEN(ICU, IRQ3) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq4_start
 *
 * Description:
 * Enable IRQ4 Interrupt
 ****************************************************************************/

void r_icu_irq4_start(void)
{
    /* Enable IRQ4 interrupt */

    IEN(ICU, IRQ4) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq4_stop
 *
 * Description:
 *Initialize IRQ4 Interrupt
 ****************************************************************************/

void r_icu_irq4_stop(void)
{
    /* Disable IRQ4 interrupt */

    IEN(ICU, IRQ4) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq5_start
 *
 * Description:
 * Enable IRQ5 Interrupt
 ****************************************************************************/

void r_icu_irq5_start(void)
{
    /* Enable IRQ5 interrupt */

    IEN(ICU, IRQ5) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq5_stop
 *
 * Description:
 *Initialize IRQ5 Interrupt
 ****************************************************************************/

void r_icu_irq5_stop(void)
{
    /* Disable IRQ5 interrupt */

    IEN(ICU, IRQ5) = 0u;
}

/****************************************************************************
 * Name: r_icu_irq15_start
 *
 * Description:
 * Enable IRQ15 Interrupt
 ****************************************************************************/

void r_icu_irq15_start(void)
{
    /* Enable IRQ15 interrupt */

    IEN(ICU, IRQ15) = 1u;
}

/****************************************************************************
 * Name: r_icu_irq15_stop
 *
 * Description:
 * Disable IRQ15 Interrupt
 ****************************************************************************/

void r_icu_irq15_stop(void)
{
    /* Disable IRQ15 interrupt */

    IEN(ICU, IRQ15) = 0u;
}

/****************************************************************************
 * Name: r_config_icu_software_start
 *
 * Description:
 * Enable S/W Interrupt
 ****************************************************************************/

void r_config_icu_software_start(void)
{
    /* Enable software interrupt */

    IEN(ICU, SWINT) = 1u;
}

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt_generate
 *
 * Description:
 * Generate S/W Interrupt
 ****************************************************************************/

void r_config_icu_softwareinterrupt_generate(void)
{
    /* Generate software interrupt */

    ICU.SWINTR.BIT.SWINT = 1u;
}

/****************************************************************************
 * Name: r_config_icu_software_stop
 *
 * Description:
 * Disable S/W Interrupt
 ****************************************************************************/

void r_config_icu_software_stop(void)
{
    /* Disable software interrupt */

    IEN(ICU, SWINT) = 0u;
}

/****************************************************************************
 * Name: r_config_icu_software2_start
 *
 * Description:
 * Enable S/W Interrupt 2
 ****************************************************************************/

void r_config_icu_software2_start(void)
{
    /* Enable software interrupt 2 */

    IEN(ICU, SWINT2) = 1u;
}

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt2_generate
 *
 * Description:
 * Generate software interrupt 2
 ****************************************************************************/

void r_config_icu_softwareinterrupt2_generate(void)
{
    /* Generate software interrupt 2 */

    ICU.SWINT2R.BIT.SWINT2 = 1u;
}

/****************************************************************************
 * Name: r_config_icu_softwareinterrupt2_stop
 *
 * Description:
 * Disable software interrupt 2
 ****************************************************************************/

void r_config_icu_software2_stop(void)
{
    /* Disable software interrupt 2 */

    IEN(ICU, SWINT2) = 0u;
}

/****************************************************************************
 * Name: r_icu_irqisfallingedge
 *
 * Description:
 * Detect if falling edge interrupt is triggered
 ****************************************************************************/

uint8_t r_icu_irqisfallingedge (const uint8_t irq_no)
{
	uint8_t falling_edge_trig = 0x0;

	if (ICU.IRQCR[irq_no].BYTE & _04_ICU_IRQ_EDGE_FALLING)
	{
		falling_edge_trig = 1;
	}
	return (falling_edge_trig);
}

/****************************************************************************
 * Name: r_icu_irqsetfallingedge
 *
 * Description:
 * Sets or unsets falling edge triggered
 ****************************************************************************/

void r_icu_irqsetfallingedge (const uint8_t irq_no, const uint8_t set_f_edge)
{
	if (1 == set_f_edge)
	{
		ICU.IRQCR[irq_no].BYTE |= _04_ICU_IRQ_EDGE_FALLING;
	}
	else
	{
		ICU.IRQCR[irq_no].BYTE &= (uint8_t) ~_04_ICU_IRQ_EDGE_FALLING;
	}
}

/****************************************************************************
 * Name: r_icu_irqsetrisingedge
 *
 * Description:
 * Sets or unsets rising edge triggered
 ****************************************************************************/

void r_icu_irqsetrisingedge (const uint8_t irq_no, const uint8_t set_r_edge)
{
	if (1 == set_r_edge)
	{
		ICU.IRQCR[irq_no].BYTE |= _08_ICU_IRQ_EDGE_RISING;
	}
	else
	{
		ICU.IRQCR[irq_no].BYTE &= (uint8_t) ~_08_ICU_IRQ_EDGE_RISING;
	}
}
