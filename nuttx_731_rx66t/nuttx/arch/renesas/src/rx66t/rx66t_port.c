/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_port.c
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
#include "rx66t_port.h"
#include "arch/board/board.h"

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
 * Name: r_port_create
 *
 * Description:
 * Port Initialization
 ****************************************************************************/
void    r_port_create(void)
{
	/* Set PORT0 registers */
	PORT0.PODR.BYTE = _02_Pm1_OUTPUT_1;
	PORT0.ODR0.BYTE = _00_Pm1_CMOS_OUTPUT;
	PORT0.DSCR.BYTE = _02_Pm1_HIDRV_ON;
	PORT0.PMR.BYTE = _00_Pm1_PIN_GPIO;
	PORT0.PDR.BYTE = _02_Pm1_MODE_OUTPUT;
	/* Set PORTE registers */
	PORTE.PODR.BYTE = _20_Pm5_OUTPUT_1;
	PORTE.ODR1.BYTE = _00_Pm5_CMOS_OUTPUT;
	PORTE.DSCR.BYTE = _20_Pm5_HIDRV_ON;
	PORTE.PMR.BYTE = _00_Pm5_PIN_GPIO;
	PORTE.PDR.BYTE = _20_Pm5_MODE_OUTPUT | _40_PDRE_DEFAULT;
}
