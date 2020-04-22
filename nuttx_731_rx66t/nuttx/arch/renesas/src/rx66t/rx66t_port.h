/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_port.h
 *
 *   Copyright (C) 2009-2019 Gregory Nutt. All rights reserved.
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
#ifndef __ARCH_RENESAS_SRC_RX66T_PORT_H
#define __ARCH_RENESAS_SRC_RX66T_PORT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
    Port Mode Register (PMR)
*/
/* Pmn Pin Mode Control (B7 -B0) */
#define _00_Pm0_PIN_GPIO         (0x00U) /* Pm0 as general I/O pin */
#define _00_Pm1_PIN_GPIO         (0x00U) /* Pm1 as general I/O pin */
#define _00_Pm2_PIN_GPIO         (0x00U) /* Pm2 as general I/O pin */
#define _00_Pm3_PIN_GPIO         (0x00U) /* Pm3 as general I/O pin */
#define _00_Pm4_PIN_GPIO         (0x00U) /* Pm4 as general I/O pin */
#define _00_Pm5_PIN_GPIO         (0x00U) /* Pm5 as general I/O pin */
#define _00_Pm6_PIN_GPIO         (0x00U) /* Pm6 as general I/O pin */
#define _00_Pm7_PIN_GPIO         (0x00U) /* Pm7 as general I/O pin */

/*
    Port Direction Register (PDR)
*/
/* Pmn Direction Control (B7 - B0) */
#define _00_Pm0_MODE_INPUT       (0x00U) /* Pm0 as input */
#define _01_Pm0_MODE_OUTPUT      (0x01U) /* Pm0 as output */
#define _00_Pm1_MODE_INPUT       (0x00U) /* Pm1 as input */
#define _02_Pm1_MODE_OUTPUT      (0x02U) /* Pm1 as output */
#define _00_Pm2_MODE_INPUT       (0x00U) /* Pm2 as input */
#define _04_Pm2_MODE_OUTPUT      (0x04U) /* Pm2 as output */
#define _00_Pm3_MODE_INPUT       (0x00U) /* Pm3 as input */
#define _08_Pm3_MODE_OUTPUT      (0x08U) /* Pm3 as output */
#define _00_Pm4_MODE_INPUT       (0x00U) /* Pm4 as input */
#define _10_Pm4_MODE_OUTPUT      (0x10U) /* Pm4 as output */
#define _00_Pm5_MODE_INPUT       (0x00U) /* Pm5 as input */
#define _20_Pm5_MODE_OUTPUT      (0x20U) /* Pm5 as output */
#define _00_Pm6_MODE_INPUT       (0x00U) /* Pm6 as input */
#define _40_Pm6_MODE_OUTPUT      (0x40U) /* Pm6 as output */
#define _00_Pm7_MODE_INPUT       (0x00U) /* Pm7 as input */
#define _80_Pm7_MODE_OUTPUT      (0x80U) /* Pm7 as output */
#define _FC_PDR1_DEFAULT         (0xFCU) /* PDR1 default value */
#define _60_PDR2_DEFAULT         (0x60U) /* PDR2 default value */
#define _30_PDR3_DEFAULT         (0x30U) /* PDR3 default value */
#define _03_PDR5_DEFAULT         (0x03U) /* PDR5 default value */
#define _C0_PDRA_DEFAULT         (0xC0U) /* PDRA default value */
#define _80_PDRB_DEFAULT         (0x80U) /* PDRB default value */
#define _7F_PDRC_DEFAULT         (0x7FU) /* PDRC default value */
#define _03_PDRD_DEFAULT         (0x03U) /* PDRD default value */
#define _40_PDRE_DEFAULT         (0x40U) /* PDRE default value */
#define _0F_PDRF_DEFAULT         (0x0FU) /* PDRF default value */
#define _07_PDRG_DEFAULT         (0x07U) /* PDRG default value */
#define _EE_PDRH_DEFAULT         (0xEEU) /* PDRH default value */
#define _07_PDRK_DEFAULT         (0x07U) /* PDRK default value */

/*
    Port Output Data Register (PODR)
*/
/* Pmn Output Data Store (B7 - B0) */
#define _00_Pm0_OUTPUT_0         (0x00U) /* output low at B0 */
#define _01_Pm0_OUTPUT_1         (0x01U) /* output high at B0 */
#define _00_Pm1_OUTPUT_0         (0x00U) /* output low at B1 */
#define _02_Pm1_OUTPUT_1         (0x02U) /* output high at B1 */
#define _00_Pm2_OUTPUT_0         (0x00U) /* output low at B2 */
#define _04_Pm2_OUTPUT_1         (0x04U) /* output high at B2 */
#define _00_Pm3_OUTPUT_0         (0x00U) /* output low at B3 */
#define _08_Pm3_OUTPUT_1         (0x08U) /* output high at B3 */
#define _00_Pm4_OUTPUT_0         (0x00U) /* output low at B4 */
#define _10_Pm4_OUTPUT_1         (0x10U) /* output high at B4 */
#define _00_Pm5_OUTPUT_0         (0x00U) /* output low at B5 */
#define _20_Pm5_OUTPUT_1         (0x20U) /* output high at B5 */
#define _00_Pm6_OUTPUT_0         (0x00U) /* output low at B6 */
#define _40_Pm6_OUTPUT_1         (0x40U) /* output high at B6 */
#define _00_Pm7_OUTPUT_0         (0x00U) /* output low at B7 */
#define _80_Pm7_OUTPUT_1         (0x80U) /* output high at B7 */

/*
    Open Drain Control Register 0 (ODR0)
*/
/* Pmn Output Type Select (Pm0 to Pm3) */
#define _00_Pm0_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _01_Pm0_NCH_OPEN_DRAIN   (0x01U) /* N-channel open-drain output */
#define _00_Pm1_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _04_Pm1_NCH_OPEN_DRAIN   (0x04U) /* N-channel open-drain output */
#define _00_Pm2_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _10_Pm2_NCH_OPEN_DRAIN   (0x10U) /* N-channel open-drain output */
#define _00_Pm3_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _40_Pm3_NCH_OPEN_DRAIN   (0x40U) /* N-channel open-drain output */

/*
    Open Drain Control Register 1 (ODR1)
*/
/* Pmn Output Type Select (Pm4 to Pm7) */
#define _00_Pm4_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _01_Pm4_NCH_OPEN_DRAIN   (0x01U) /* N-channel open-drain output */
#define _00_Pm5_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _04_Pm5_NCH_OPEN_DRAIN   (0x04U) /* N-channel open-drain output */
#define _00_Pm6_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _10_Pm6_NCH_OPEN_DRAIN   (0x10U) /* N-channel open-drain output */
#define _00_Pm7_CMOS_OUTPUT      (0x00U) /* CMOS output */
#define _40_Pm7_NCH_OPEN_DRAIN   (0x40U) /* N-channel open-drain output */

/*
    Pull-Up Control Register (PCR)
*/
/* Pmn Input Pull-Up Resistor Control (B7 - B0) */
#define _00_Pm0_PULLUP_OFF       (0x00U) /* Pn0 pull-up resistor not connected */
#define _01_Pm0_PULLUP_ON        (0x01U) /* Pn0 pull-up resistor connected */
#define _00_Pm1_PULLUP_OFF       (0x00U) /* Pn1 pull-up resistor not connected */
#define _02_Pm1_PULLUP_ON        (0x02U) /* Pn1 pull-up resistor connected */
#define _00_Pm2_PULLUP_OFF       (0x00U) /* Pn2 Pull-up resistor not connected */
#define _04_Pm2_PULLUP_ON        (0x04U) /* Pn2 pull-up resistor connected */
#define _00_Pm3_PULLUP_OFF       (0x00U) /* Pn3 pull-up resistor not connected */
#define _08_Pm3_PULLUP_ON        (0x08U) /* Pn3 pull-up resistor connected */
#define _00_Pm4_PULLUP_OFF       (0x00U) /* Pn4 pull-up resistor not connected */
#define _10_Pm4_PULLUP_ON        (0x10U) /* Pn4 pull-up resistor connected */
#define _00_Pm5_PULLUP_OFF       (0x00U) /* Pn5 pull-up resistor not connected */
#define _20_Pm5_PULLUP_ON        (0x20U) /* Pn5 pull-up resistor connected */
#define _00_Pm6_PULLUP_OFF       (0x00U) /* Pn6 pull-up resistor not connected */
#define _40_Pm6_PULLUP_ON        (0x40U) /* Pn6 pull-up resistor connected */
#define _00_Pm7_PULLUP_OFF       (0x00U) /* Pn7 pull-up resistor not connected */
#define _80_Pm7_PULLUP_ON        (0x80U) /* Pn7 pull-up resistor connected */

/*
    Drive Capacity Control Register (DSCR)
*/
/* Pmn Drive Capacity Control (B7 - B0) */
#define _00_Pm0_HIDRV_OFF        (0x00U) /* Pm0 Normal drive output */
#define _01_Pm0_HIDRV_ON         (0x01U) /* Pm0 High-drive output */
#define _00_Pm1_HIDRV_OFF        (0x00U) /* Pm1 Normal drive output */
#define _02_Pm1_HIDRV_ON         (0x02U) /* Pm1 High-drive output */
#define _00_Pm2_HIDRV_OFF        (0x00U) /* Pm2 Normal drive output */
#define _04_Pm2_HIDRV_ON         (0x04U) /* Pm2 High-drive output */
#define _00_Pm3_HIDRV_OFF        (0x00U) /* Pm3 Normal drive output */
#define _08_Pm3_HIDRV_ON         (0x08U) /* Pm3 High-drive output */
#define _00_Pm4_HIDRV_OFF        (0x00U) /* Pm4 Normal drive output */
#define _10_Pm4_HIDRV_ON         (0x10U) /* Pm4 High-drive output */
#define _00_Pm5_HIDRV_OFF        (0x00U) /* Pm5 Normal drive output */
#define _20_Pm5_HIDRV_ON         (0x20U) /* Pm5 High-drive output */
#define _00_Pm6_HIDRV_OFF        (0x00U) /* Pm6 Normal drive output */
#define _40_Pm6_HIDRV_ON         (0x40U) /* Pm6 High-drive output */
#define _00_Pm7_HIDRV_OFF        (0x00U) /* Pm7 Normal drive output */
#define _80_Pm7_HIDRV_ON         (0x80U) /* Pm7 High-drive output */

/*
     Drive Capacity Control Register 2 (DSCR2)
*/
/* Pmn Drive Capacity Control 2 (B7 - B0) */
#define _00_Pm0_LARGECUR_OFF     (0x00U) /* Pm0 Normal drive/high-drive output */
#define _01_Pm0_LARGECUR_ON      (0x01U) /* Pm0 Large current output */
#define _00_Pm1_LARGECUR_OFF     (0x00U) /* Pm1 Normal drive/high-drive output */
#define _02_Pm1_LARGECUR_ON      (0x02U) /* Pm1 Large current output */
#define _00_Pm2_LARGECUR_OFF     (0x00U) /* Pm2 Normal drive/high-drive output */
#define _04_Pm2_LARGECUR_ON      (0x04U) /* Pm2 Large current output */
#define _00_Pm3_LARGECUR_OFF     (0x00U) /* Pm3 Normal drive/high-drive output */
#define _08_Pm3_LARGECUR_ON      (0x08U) /* Pm3 Large current output */
#define _00_Pm4_LARGECUR_OFF     (0x00U) /* Pm4 Normal drive/high-drive output */
#define _10_Pm4_LARGECUR_ON      (0x10U) /* Pm4 Large current output */
#define _00_Pm5_LARGECUR_OFF     (0x00U) /* Pm5 Normal drive/high-drive output */
#define _20_Pm5_LARGECUR_ON      (0x20U) /* Pm5 Large current output */
#define _00_Pm6_LARGECUR_OFF     (0x00U) /* Pm6 Normal drive/high-drive output */
#define _40_Pm6_LARGECUR_ON      (0x40U) /* Pm6 Large current output */
#define _00_Pm7_LARGECUR_OFF     (0x00U) /* Pm7 Normal drive/high-drive output */
#define _80_Pm7_LARGECUR_ON      (0x80U) /* Pm7 Large current output */

/****************************************************************************
 * Public Types
 ***************************************************************************/

/****************************************************************************
 * Public Data
 ***************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ***************************************************************************/

/****************************************************************************
 * Name: r_port_create
 *
 * Description:
 *   Initializes Ports of rx66t
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_port_create(void);

#endif
