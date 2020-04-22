/****************************************************************************
 * arch/renesas/src/rx65n/rx65n_cmt.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#ifndef __ARCH_RENESAS_SRC_RX65N_CMT_H
#define __ARCH_RENESAS_SRC_RX65N_CMT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compare Match Timer Control Register (CMCR) */

/* Clock Select (CKS[1:0]) */

#define _0000_CMT_CMCR_CKS_PCLK8                (0x0000U) /* PCLK/8 */
#define _0001_CMT_CMCR_CKS_PCLK32               (0x0001U) /* PCLK/32 */
#define _0002_CMT_CMCR_CKS_PCLK128              (0x0002U) /* PCLK/128 */
#define _0003_CMT_CMCR_CKS_PCLK512              (0x0003U) /* PCLK/512 */

/* Compare Match Interrupt Enable (CMIE) */

#define _0000_CMT_CMCR_CMIE_DISABLE             (0x0000U) /* Compare match interrupt (CMIn) disabled */
#define _0040_CMT_CMCR_CMIE_ENABLE              (0x0040U) /* Compare match interrupt (CMIn) enabled */
#define _0080_CMT_CMCR_DEFAULT                  (0x0080U) /* Write default value of CMCR */

/* Interrupt Source Priority Register n (IPRn) */

/* Interrupt Priority Level Select (IPR[3:0]) */

#define _00_CMT_PRIORITY_LEVEL0                 (0x00U) /* Level 0 (interrupt disabled) */
#define _01_CMT_PRIORITY_LEVEL1                 (0x01U) /* Level 1 */
#define _02_CMT_PRIORITY_LEVEL2                 (0x02U) /* Level 2 */
#define _03_CMT_PRIORITY_LEVEL3                 (0x03U) /* Level 3 */
#define _04_CMT_PRIORITY_LEVEL4                 (0x04U) /* Level 4 */
#define _05_CMT_PRIORITY_LEVEL5                 (0x05U) /* Level 5 */
#define _06_CMT_PRIORITY_LEVEL6                 (0x06U) /* Level 6 */
#define _07_CMT_PRIORITY_LEVEL7                 (0x07U) /* Level 7 */
#define _08_CMT_PRIORITY_LEVEL8                 (0x08U) /* Level 8 */
#define _09_CMT_PRIORITY_LEVEL9                 (0x09U) /* Level 9 */
#define _0A_CMT_PRIORITY_LEVEL10                (0x0aU) /* Level 10 */
#define _0B_CMT_PRIORITY_LEVEL11                (0x0bU) /* Level 11 */
#define _0C_CMT_PRIORITY_LEVEL12                (0x0cU) /* Level 12 */
#define _0D_CMT_PRIORITY_LEVEL13                (0x0dU) /* Level 13 */
#define _0E_CMT_PRIORITY_LEVEL14                (0x0eU) /* Level 14 */
#define _0F_CMT_PRIORITY_LEVEL15                (0x0fU) /* Level 15 (highest) */

/* Compare Match Values */

#define _1D4B_CMT0_CMCOR_VALUE                  (0x1d4bU)
#define _927B_CMT1_CMCOR_VALUE                  (0x927bU)
#define _5B8D_CMT2_CMCOR_VALUE                  (0x5b8dU)

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
 * Name: r_cmt0_create
 *
 * Description:
 *   Initializes CMT0 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_cmt0_create(void);

/****************************************************************************
 * Name: r_cmt0_start
 *
 * Description:
 *   Starts the CMT0 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt0_start(void);

/****************************************************************************
 * Name: r_cmt0_stop
 *
 * Description:
 *   Stops the CMT0 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt0_stop(void);

/****************************************************************************
 * Name: r_cmt1_create
 *
 * Description:
 *   Initializes the CMT1 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt1_create(void);

/****************************************************************************
 * Name: r_cmt1_start
 *
 * Description:
 *   Starts the CMT1 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt1_start(void);

/****************************************************************************
 * Name: r_cmt1_stop
 *
 * Description:
 *   Stops the CMT1 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt1_stop(void);

/****************************************************************************
 * Name: r_cmt2_create
 *
 * Description:
 *   Initializes the CMT2 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt2_create(void);

/****************************************************************************
 * Name: r_cmt2_start
 *
 * Description:
 *   Starts the CMT2 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt2_start(void);

/****************************************************************************
 * Name: r_cmt2_stop
 *
 * Description:
 *   Stops the CMT2 counter
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void r_cmt2_atop(void);

/****************************************************************************
 * Name: r_cmt_msdelay
 *
 * Description:
 *   Millisecond delay function
 *
 * Input Parameters:
 *   millisec - Value of delay in millisec
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void r_cmt_msdelay(const uint16_t millisec);

#endif
