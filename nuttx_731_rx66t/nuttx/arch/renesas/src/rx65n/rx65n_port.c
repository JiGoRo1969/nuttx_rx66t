/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_port.c
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
#include "rx65n_macrodriver.h"
#include "rx65n_port.h"
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
#if  defined(CONFIG_ARCH_BOARD_RX65N_RSK1MB)

    /* LED_PORTINIT(0); */

    PORT0.PODR.BYTE  = _04_PM2_OUTPUT_1 | _08_PM3_OUTPUT_1 | _20_PM5_OUTPUT_1;
    PORT5.PODR.BYTE  = _40_PM6_OUTPUT_1;
    PORT7.PODR.BYTE  = _08_PM3_OUTPUT_1;
    PORT9.PODR.BYTE  = _08_PM3_OUTPUT_1;
    PORTJ.PODR.BYTE  = _20_PM5_OUTPUT_1;
    PORT0.DSCR.BYTE  = _00_PM2_HIDRV_OFF;
    PORT0.DSCR2.BYTE = _00_PM2_HISPEED_OFF;
    PORT5.DSCR.BYTE  = _20_PM5_HIDRV_ON | _00_PM6_HIDRV_OFF;
    PORT5.DSCR2.BYTE = _00_PM5_HISPEED_OFF | _00_PM6_HISPEED_OFF;
    PORT7.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
    PORT9.DSCR.BYTE  = _00_PM3_HIDRV_OFF;
    PORT9.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
    PORT0.PMR.BYTE   = 0x00u;
    PORT0.PDR.BYTE   = _04_PM2_MODE_OUTPUT | _08_PM3_MODE_OUTPUT |
                       _20_PM5_MODE_OUTPUT | _50_PDR0_DEFAULT;
    PORT5.PMR.BYTE   = 0x00u;
    PORT5.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _40_PM6_MODE_OUTPUT |
                       _80_PDR5_DEFAULT;
    PORT7.PMR.BYTE   = 0x00u;
    PORT7.PDR.BYTE   = _08_PM3_MODE_OUTPUT;
    PORT9.PMR.BYTE   = 0x00u;
    PORT9.PDR.BYTE   = _08_PM3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
    PORTJ.PMR.BYTE   = 0x00u;
    PORTJ.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
#elif   defined (CONFIG_ARCH_BOARD_RX65N_RSK2MB)

/*      LED_PORTINIT(0); */

    PORT0.PODR.BYTE  = _04_PM2_OUTPUT_1 | _08_PM3_OUTPUT_1 | _20_PM5_OUTPUT_1;
    PORT5.PODR.BYTE  = _40_PM6_OUTPUT_1;
    PORT7.PODR.BYTE  = _08_PM3_OUTPUT_1;
    PORT9.PODR.BYTE  = _08_PM3_OUTPUT_1;
    PORTJ.PODR.BYTE  = _20_PM5_OUTPUT_1;
    PORT0.DSCR.BYTE  = _00_PM2_HIDRV_OFF;
    PORT0.DSCR2.BYTE = _00_PM2_HISPEED_OFF;
    PORT5.DSCR.BYTE  = _20_PM5_HIDRV_ON | _00_PM6_HIDRV_OFF;
    PORT5.DSCR2.BYTE = _00_PM5_HISPEED_OFF | _00_PM6_HISPEED_OFF;
    PORT7.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
    PORT9.DSCR.BYTE  = _00_PM3_HIDRV_OFF;
    PORT9.DSCR2.BYTE = _00_PM3_HISPEED_OFF;
    PORT0.PMR.BYTE   = 0x00u;
    PORT0.PDR.BYTE   = _04_PM2_MODE_OUTPUT | _08_PM3_MODE_OUTPUT |
                       _20_PM5_MODE_OUTPUT | _50_PDR0_DEFAULT;
    PORT5.PMR.BYTE   = 0x00u;
    PORT5.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _40_PM6_MODE_OUTPUT |
                       _80_PDR5_DEFAULT;
    PORT7.PMR.BYTE   = 0x00u;
    PORT7.PDR.BYTE   = _08_PM3_MODE_OUTPUT;
    PORT9.PMR.BYTE   = 0x00u;
    PORT9.PDR.BYTE   = _08_PM3_MODE_OUTPUT | _F0_PDR9_DEFAULT;
    PORTJ.PMR.BYTE   = 0x00u;
    PORTJ.PDR.BYTE   = _20_PM5_MODE_OUTPUT | _D7_PDRJ_DEFAULT;
#elif   defined(CONFIG_ARCH_BOARD_RX65N_GRROSE)
    LED_PORTINIT(0);

   /* SCI0(UART)  direction */

    PORT2.PODR.BIT.B2 = 0;  PORT2.PMR.BIT.B2 = 0;   PORT2.PDR.BIT.B2 = 1;

   /* SCI2(UART)  direction */

    PORT1.PODR.BIT.B4 = 0;  PORT1.PMR.BIT.B4 = 0;   PORT1.PDR.BIT.B4 = 1;

  /* SCI5(UART)  direction */

    PORTC.PODR.BIT.B4 = 0;  PORTC.PMR.BIT.B4 = 0;   PORTC.PDR.BIT.B4 = 1;

 /* SCI6(UART)  direction */

    PORT3.PODR.BIT.B4 = 0;  PORT3.PMR.BIT.B4 = 0;   PORT3.PDR.BIT.B4 = 1;

 /* SCI8(RS485) direction */

    PORTC.PODR.BIT.B5 = 0;  PORTC.PMR.BIT.B5 = 0;   PORTC.PDR.BIT.B5 = 1;
#else
#error "No Selection for PORT definition in rx65n_port.c"
#endif
}
