
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include "rx66t_macrodriver.h"
#include "rx66t_cmt.h"

/****************************************************************************
 * Name: r_cmt0_create
 *
 * Description:
 *   Initializes CMT0 timer
 *
 ****************************************************************************/
void r_cmt0_create(void)
{
  /* Disable CMI interrupt */

  IEN(CMT0, CMI0) = 0U;

  /* Cancel CMT stop state in LPC */

  MSTP(CMT0) = 0U;

  /* Set control registers */

  CMT0.CMCR.WORD = _0000_CMT_CMCR_CKS_PCLK8 | _0040_CMT_CMCR_CMIE_ENABLE |
                   _0080_CMT_CMCR_DEFAULT;
  CMT0.CMCOR     = _1D4B_CMT0_CMCOR_VALUE;

  /* Set CMI0 priority level */

  IPR(CMT0, CMI0) = _0A_CMT_PRIORITY_LEVEL10;
}

/****************************************************************************
 * Name: r_cmt0_start
 *
 * Description:
 *   Start the CMT0 channel counter
 *
 ****************************************************************************/
void r_cmt0_start(void)
{
  /* Enable CMI0 interrupt in ICU */

  IEN(CMT0, CMI0) = 1U;

  /* Start CMT0 count */

  CMT.CMSTR0.BIT.STR0 = 1U;
}

/****************************************************************************
 * Name: r_cmt0_stop
 *
 * Description:
 *   Stops the CMT0 channel counter
 *
 ****************************************************************************/
void r_cmt0_stop(void)
{
  /* Disable CMI0 interrupt in ICU */

  IEN(CMT0, CMI0) = 0U;

  /* Stop CMT0 count */

  CMT.CMSTR0.BIT.STR0 = 0U;
}

/****************************************************************************
 * Name: r_cmt1_create
 *
 * Description:
 *   Initializes the CMT1 channel counter
 *
 ****************************************************************************/
void r_cmt1_create(void)
{
  /* Disable CMI interrupt */

  IEN(CMT1, CMI1) = 0U;

  /* Cancel CMT stop state in LPC */

  MSTP(CMT1) = 0U;

  /* Set control registers */

  CMT1.CMCR.WORD = _0001_CMT_CMCR_CKS_PCLK32 | _0040_CMT_CMCR_CMIE_ENABLE |
                   _0080_CMT_CMCR_DEFAULT;
  CMT1.CMCOR     = _927B_CMT1_CMCOR_VALUE;

  /* Set CMI1 priority level */

  IPR(CMT1, CMI1) = _0A_CMT_PRIORITY_LEVEL10;
}

/****************************************************************************
 * Name: r_cmt1_start
 *
 * Description:
 *   Starts the CMT1 channel counter
 *
 ****************************************************************************/
void r_cmt1_start(void)
{
  /* Enable CMI1 interrupt in ICU */

  IEN(CMT1, CMI1) = 1U;

  /* Start CMT1 count */

  CMT.CMSTR0.BIT.STR1 = 1U;
}

/****************************************************************************
 * Name: r_cmt1_stop
 *
 * Description:
 *   Stops the CMT1 channel counter
 *
 ****************************************************************************/
void r_cmt1_stop(void)
{
  /* Disable CMI1 interrupt in ICU */

  IEN(CMT1, CMI1) = 0U;

  /* Stop CMT1 count */

  CMT.CMSTR0.BIT.STR1 = 0U;
}

/****************************************************************************
 * Name: r_cmt2_create
 *
 * Description:
 *   Initializes the CMT2 channel counter
 *
 ****************************************************************************/
void r_cmt2_create(void)
{
  /* Disable CMI interrupt */

  IEN(PERIB, INTB128) = 0U;

  /* Cancel CMT stop state in LPC */

  MSTP(CMT2) = 0U;

  /* Set control registers */

  CMT2.CMCR.WORD = _0003_CMT_CMCR_CKS_PCLK512 | _0040_CMT_CMCR_CMIE_ENABLE |
                   _0080_CMT_CMCR_DEFAULT;
  CMT2.CMCOR     = _5B8D_CMT2_CMCOR_VALUE;
  ICU.SLIBXR128.BYTE = 0x01u;

  /* Set CMI2 priority level */

  IPR(PERIB, INTB128) = _0A_CMT_PRIORITY_LEVEL10;
}

/****************************************************************************
 * Name: r_cmt2_start
 *
 * Description:
 *   Starts the CMT2 channel counter
 *
 ****************************************************************************/
void r_cmt2_start(void)
{
  /* Enable CMI2 interrupt in ICU */

  IEN(PERIB, INTB128) = 1U;

  /* Start CMT2 count */

  CMT.CMSTR1.BIT.STR2 = 1U;
}

/****************************************************************************
 * Name: r_cmt2_stop
 *
 * Description:
 *   Stops the CMT2 channel counter
 *
 ****************************************************************************/
void r_cmt2_stop(void)
{
  /* Disable CMI2 interrupt in ICU */

  IEN(PERIB, INTB128) = 0U;

  /* Stop CMT2 count */

  CMT.CMSTR1.BIT.STR2 = 0U;
}
