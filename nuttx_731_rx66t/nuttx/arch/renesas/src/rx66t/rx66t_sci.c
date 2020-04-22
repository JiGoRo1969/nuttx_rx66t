/****************************************************************************
 * arch/renesas/src/rx66t/rx66t_sci.c
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

/***********************************************************************************************************************
 *	Includes
 ***********************************************************************************************************************/
#include <nuttx/config.h>

#include "sys/types.h"
#include "rx66t_macrodriver.h"
#include "rx66t_sci.h"
#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "rx66t_definitions.h"

/***********************************************************************************************************************
 *	Global variables and functions
 ***********************************************************************************************************************/
static inline
void	RX_MPC_enable( void )
{
	/* Enable writing to registers related to operating modes, LPC, CGC and software reset */
	SYSTEM.PRCR.WORD = 0xA50BU;
	/* Enable writing to MPC pin function control registers */
	MPC.PWPR.BIT.B0WI = 0U;
	MPC.PWPR.BIT.PFSWE = 1U;
}

static inline
void	RX_MPC_disable( void )
{
	/* Disable writing to MPC pin function control registers */
	MPC.PWPR.BIT.PFSWE = 0U;
	MPC.PWPR.BIT.B0WI = 1U;
	/* Enable protection */
	SYSTEM.PRCR.WORD = 0xA500U;
}

#ifdef	CONFIG_RX66T_SCI1
volatile uint8_t	*gp_sci1_tx_address;			/* SCI1 transmit buffer address		*/
volatile uint8_t	*gp_sci1_rx_address;			/* SCI1 receive buffer address		*/
volatile uint16_t	g_sci1_tx_count;				/* SCI1 transmit data number		*/
volatile uint16_t	g_sci1_rx_count;				/* SCI1 receive data number			*/
volatile uint16_t	g_sci1_rx_length;				/* SCI1 receive data length			*/

static inline
void	SCI1_init_port( void )
{
	/* Set RXD1 pin (PD5) */
	MPC.PD5PFS.BYTE		= 0x0AU;
	PORTD.PMR.BIT.BT5	= 1U;
	/* Set TXD1 pin (PD3) */
	PORTD.PODR.BIT.B3	= 1U;
	MPC.PD3PFS.BYTE		= 0x0AU;
	PORTD.PDR.BIT.B3	= 1U;
	PORTD.PMR.BIT.B3	= 1U;
}
#endif	// CONFIG_RX66T_SCI1

#ifdef	CONFIG_RX66T_SCI5
volatile uint8_t	*gp_sci5_tx_address;			/* SCI5 transmit buffer address		*/
volatile uint8_t	*gp_sci5_rx_address;			/* SCI5 receive buffer address		*/
volatile uint16_t	g_sci5_tx_count;				/* SCI5 transmit data number		*/
volatile uint16_t	g_sci5_rx_count;				/* SCI5 receive data number			*/
volatile uint16_t	g_sci5_rx_length;				/* SCI5 receive data length			*/

static inline
void	SCI5_init_port( void )
{
	/* Set RXD5 pin (PE0) */
	MPC.PE0PFS.BYTE		= 0x0AU;
	PORTE.PMR.BIT.BT0	= 1U;
	/* Set TXD5 pin (PD7) */
	PORTD.PODR.BIT.B7	= 1U;
	MPC.PD7PFS.BYTE		= 0x0AU;
	PORTD.PDR.BIT.B7	= 1U;
	PORTD.PMR.BIT.B7	= 1U;
}
#endif	// CONFIG_RX66T_SCI5

#ifdef	CONFIG_RX66T_SCI6
volatile uint8_t	*gp_sci6_tx_address;			/* SCI6 transmit buffer address		*/
volatile uint8_t	*gp_sci6_rx_address;			/* SCI6 receive buffer address		*/
volatile uint16_t	g_sci6_tx_count;				/* SCI6 transmit data number		*/
volatile uint16_t	g_sci6_rx_count;				/* SCI6 receive data number			*/
volatile uint16_t	g_sci6_rx_length;				/* SCI6 receive data length			*/

static inline
void	SCI6_init_port( void )
{
	/* Set RXD6 pin (PB1) */
	MPC.PB1PFS.BYTE		= 0x0AU;
	PORTB.PMR.BIT.B1	= 1U;
	/* Set TXD6 pin (PB2) */
	PORTB.PODR.BIT.B2	= 1U;
	MPC.PB2PFS.BYTE		= 0x0AU;
	PORTB.PDR.BIT.B2	= 1U;
	PORTB.PMR.BIT.B2	= 1U;
}
#endif	// CONFIG_RX66T_SCI6

#ifdef	CONFIG_RX66T_SCI8
volatile uint8_t	*gp_sci8_tx_address;			/* SCI8 transmit buffer address		*/
volatile uint8_t	*gp_sci8_rx_address;			/* SCI8 receive buffer address		*/
volatile uint16_t	g_sci8_tx_count;				/* SCI8 transmit data number		*/
volatile uint16_t	g_sci8_rx_count;				/* SCI8 receive data number			*/
volatile uint16_t	g_sci8_rx_length;				/* SCI8 receive data length			*/

static inline
void	SCI8_init_port( void )
{
	/* Set RXD8 pin (PA5) */
	MPC.PA5PFS.BYTE		= 0x0BU;
	PORTA.PMR.BIT.B5	= 1U;
	/* Set TXD8 pin (PA4) */
	PORTA.PODR.BIT.B4	= 1U;
	MPC.PA4PFS.BYTE		= 0x0BU;
	PORTA.PDR.BIT.B4	= 1U;
	PORTA.PMR.BIT.B4	= 1U;
}
#endif	// CONFIG_RX66T_SCI8

#ifdef	CONFIG_RX66T_SCI9
volatile uint8_t	*gp_sci9_tx_address;			/* SCI9 transmit buffer address		*/
volatile uint8_t	*gp_sci9_rx_address;			/* SCI9 receive buffer address		*/
volatile uint16_t	g_sci9_tx_count;				/* SCI9 transmit data number		*/
volatile uint16_t	g_sci9_rx_count;				/* SCI9 receive data number			*/
volatile uint16_t	g_sci9_rx_length;				/* SCI9 receive data length			*/

static inline
void	SCI9_init_port( void )
{
	/* Set RXD9 pin (PA2) */
	MPC.PA2PFS.BYTE		= 0x0BU;
	PORTA.PMR.BIT.B2	= 1U;
	/* Set TXD9 pin (PA3) */
	PORTA.PODR.BIT.B3	= 1U;
	MPC.PA3PFS.BYTE		= 0x0AU;
	PORTA.PDR.BIT.B3	= 1U;
	PORTA.PMR.BIT.B3	= 1U;
}
#endif	// CONFIG_RX66T_SCI8

#ifdef	CONFIG_RX66T_SCI11
volatile uint8_t	*gp_sci11_tx_address;			/* SCI11 transmit buffer address		*/
volatile uint8_t	*gp_sci11_rx_address;			/* SCI11 receive buffer address		*/
volatile uint16_t	g_sci11_tx_count;				/* SCI11 transmit data number		*/
volatile uint16_t	g_sci11_rx_count;				/* SCI11 receive data number			*/
volatile uint16_t	g_sci11_rx_length;				/* SCI11 receive data length			*/

static inline
void	SCI11_init_port( void )
{
	/* Set RXD11 pin (PA1) */
	MPC.PA1PFS.BYTE		= 0x0BU;
	PORTA.PMR.BIT.B1	= 1U;
	/* Set TXD11 pin (PA0) */
	PORTA.PODR.BIT.BT0	= 1U;
	MPC.PA0PFS.BYTE		= 0x0BU;
	PORTA.PDR.BIT.BT0	= 1U;
	PORTA.PMR.BIT.BT0	= 1U;
}
#endif	// CONFIG_RX66T_SCI11

#ifdef	CONFIG_RX66T_SCI12
volatile uint8_t	*gp_sci12_tx_address;			/* SCI12 transmit buffer address		*/
volatile uint8_t	*gp_sci12_rx_address;			/* SCI12 receive buffer address		*/
volatile uint16_t	g_sci12_tx_count;				/* SCI12 transmit data number		*/
volatile uint16_t	g_sci12_rx_count;				/* SCI12 receive data number			*/
volatile uint16_t	g_sci12_rx_length;				/* SCI12 receive data length			*/

static inline
void	SCI12_init_port( void )
{
	/* Set RXD12 pin (P22) */
	MPC.P22PFS.BYTE		= 0x0AU;
	PORT2.PMR.BIT.B2	= 1U;
	/* Set TXD12 pin (P23) */
	PORT2.PODR.BIT.B3	= 1U;
	MPC.P23PFS.BYTE		= 0x0AU;
	PORT2.PDR.BIT.B3	= 1U;
	PORT2.PMR.BIT.B3	= 1U;
}
#endif	// CONFIG_RX66T_SCI12


#ifdef	CONFIG_RX66T_SCI1
/***********************************************************************************************************************
 * Function Name: R_SCI1_Create
 * Description  : This function initializes SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI1_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI1)			= 0U;			/* Cancel SCI1 module stop state */
	IPR(SCI1,RXI1)		= 15;			/* Set interrupt priority */
	IPR(SCI1,TXI1)		= 15;			/* Set interrupt priority */
	SCI1.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI1.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI1.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI1.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI1.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI1.SPMR.BYTE		= _00_SCI_RTS;
	SCI1.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI1.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI1.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI1.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI1.BRR			= 0x40U;
	SCI1.MDDR			= 255U;
	/* Set SCI1 pin */
	SCI1_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI1_Start
 * Description  : This function starts SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI1_Start(void)
{
	IR(SCI1,TXI1)		= 0U;	/* Clear interrupt flag */
	IR(SCI1,RXI1)		= 0U;	/* Clear interrupt flag */
	IEN(SCI1,TXI1)		= 1U;	/* Enable SCI interrupt */
	IEN(SCI1,RXI1)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN2	= 0U;
	ICU.GENBL0.BIT.EN3	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI1_Stop
 * Description  : This function stops SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI1_Stop(void)
{
	SCI1.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI1.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI1.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI1.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI1,TXI1)		= 0U;
	ICU.GENBL0.BIT.EN2	= 0U;
	IR(SCI1,TXI1)		= 0U;
	IEN(SCI1,RXI1)		= 0U;
	ICU.GENBL0.BIT.EN3	= 0U;
	IR(SCI1,RXI1)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI1_Serial_Receive
 * Description  : This function receives SCI1 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI1_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci1_rx_count		= 0U;
	g_sci1_rx_length	= rx_num;
	gp_sci1_rx_address	= rx_buf;
	SCI1.SCR.BIT.RIE	= 1U;
	SCI1.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI1_Serial_Send
 * Description  : This function transmits SCI1 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI1_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci1_tx_address	= tx_buf;
	g_sci1_tx_count		= tx_num;
	/* Set TXD1 pin */
	SCI1.SCR.BIT.TIE	= 1U;
	SCI1.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX66T_SCI5
/***********************************************************************************************************************
 * Function Name: R_SCI5_Create
 * Description  : This function initializes SCI5.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI5_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI5)			= 0U;			/* Cancel SCI0 module stop state */
	IPR(SCI5,RXI5)		= 15;			/* Set interrupt priority */
	IPR(SCI5,TXI5)		= 15;			/* Set interrupt priority */
	SCI5.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI5.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI5.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI5.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI5.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI5.SPMR.BYTE		= _00_SCI_RTS;
	SCI5.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI5.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI5.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI5.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI5.BRR			= 0x40U;
	SCI5.MDDR			= 255U;
	/* Set SCI5 pin */
	SCI5_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI5_Start
 * Description  : This function starts SCI5.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI5_Start(void)
{
	IR(SCI5,TXI5)		= 0U;	/* Clear interrupt flag */
	IR(SCI5,RXI5)		= 0U;	/* Clear interrupt flag */
	IEN(SCI5,TXI5)		= 1U;	/* Enable SCI interrupt */
	IEN(SCI5,RXI5)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN10	= 0U;
	ICU.GENBL0.BIT.EN11	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI5_Stop
 * Description  : This function stops SCI5.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI5_Stop(void)
{
	SCI5.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI5.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI5.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI5.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI5,TXI5)		= 0U;
	ICU.GENBL0.BIT.EN10	= 0U;
	IR(SCI5,TXI5)		= 0U;
	IEN(SCI5,RXI5)		= 0U;
	ICU.GENBL0.BIT.EN11	= 0U;
	IR(SCI5,RXI5)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI5_Serial_Receive
 * Description  : This function receives SCI5 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI5_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci5_rx_count		= 0U;
	g_sci5_rx_length	= rx_num;
	gp_sci5_rx_address	= rx_buf;
	SCI5.SCR.BIT.RIE	= 1U;
	SCI5.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI5_Serial_Send
 * Description  : This function transmits SCI5 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI5_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci5_tx_address	= tx_buf;
	g_sci5_tx_count		= tx_num;
	/* Set TXD5 pin */
	SCI5.SCR.BIT.TIE	= 1U;
	SCI5.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX66T_SCI6
/***********************************************************************************************************************
 * Function Name: R_SCI6_Create
 * Description  : This function initializes SCI6.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI6_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI6)			= 0U;			/* Cancel SCI0 module stop state */
	IPR(SCI6,RXI6)		= 15;			/* Set interrupt priority */
	IPR(SCI6,TXI6)		= 15;			/* Set interrupt priority */
	SCI6.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI6.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI6.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI6.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI6.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI6.SPMR.BYTE		= _00_SCI_RTS;
	SCI6.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI6.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI6.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI6.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI6.BRR			= 0x40U;
	SCI6.MDDR			= 255U;
	/* Set SCI6 pin */
	SCI6_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI6_Start
 * Description  : This function starts SCI6.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI6_Start(void)
{
	IR(SCI6,TXI6)		= 0U;	/* Clear interrupt flag */
	IR(SCI6,RXI6)		= 0U;	/* Clear interrupt flag */
	IEN(SCI6,TXI6)		= 1U;	/* Enable SCI interrupt */
	IEN(SCI6,RXI6)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN12	= 0U;
	ICU.GENBL0.BIT.EN13	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI6_Stop
 * Description  : This function stops SCI6.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI6_Stop(void)
{
	SCI6.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI6.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI6.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI6.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI6,TXI6)		= 0U;
	ICU.GENBL0.BIT.EN12	= 0U;
	IR(SCI6,TXI6)		= 0U;
	IEN(SCI6,RXI6)		= 0U;
	ICU.GENBL0.BIT.EN13	= 0U;
	IR(SCI6,RXI6)		= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI6_Serial_Receive
 * Description  : This function receives SCI6 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI6_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci6_rx_count		= 0U;
	g_sci6_rx_length	= rx_num;
	gp_sci6_rx_address	= rx_buf;
	SCI6.SCR.BIT.RIE	= 1U;
	SCI6.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI6_Serial_Send
 * Description  : This function transmits SCI6 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI6_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci6_tx_address	= tx_buf;
	g_sci6_tx_count		= tx_num;
	/* Set TXD0 pin */
	SCI6.SCR.BIT.TIE	= 1U;
	SCI6.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX66T_SCI8
/***********************************************************************************************************************
 * Function Name: R_SCI8_Create
 * Description  : This function initializes SCI8.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI8_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI8)			= 0U;			/* Cancel SCI8 module stop state */
	IPR(SCI8,RXI8)		= 15;			/* Set interrupt priority */
	IPR(SCI8,TXI8)		= 15;			/* Set interrupt priority */
	SCI8.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI8.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI8.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI8.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI8.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI8.SPMR.BYTE		= _00_SCI_RTS;
	SCI8.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI8.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI8.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI8.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI8.BRR			= 0x40U;
	SCI8.MDDR			= 255U;
	/* Set SCI8 pin */
	SCI8_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI8_Start
 * Description  : This function starts SCI8.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI8_Start(void)
{
	IR(SCI8,TXI8)		= 0U;	/* Clear interrupt flag */
	IR(SCI8,RXI8)		= 0U;	/* Clear interrupt flag */
	IEN(SCI8,TXI8)		= 1U;	/* Enable SCI interrupt */
	IEN(SCI8,RXI8)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL1.BIT.EN24	= 0U;
	ICU.GENBL1.BIT.EN25	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI8_Stop
 * Description  : This function stops SCI8.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI8_Stop(void)
{
	SCI8.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI8.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI8.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI8.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI8,TXI8)		= 0U;
	IR(SCI8,TXI8)		= 0U;
	IEN(SCI8,RXI8)		= 0U;
	IR(SCI8,RXI8)		= 0U;
	ICU.GENBL1.BIT.EN24	= 0U;
	ICU.GENBL1.BIT.EN25	= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI8_Serial_Receive
 * Description  : This function receives SCI8 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI8_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci8_rx_count		= 0U;
	g_sci8_rx_length	= rx_num;
	gp_sci8_rx_address	= rx_buf;
	SCI8.SCR.BIT.RIE	= 1U;
	SCI8.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI8_Serial_Send
 * Description  : This function transmits SCI8 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI8_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci8_tx_address	= tx_buf;
	g_sci8_tx_count		= tx_num;
	/* Set TXD0 pin */
	SCI8.SCR.BIT.TIE	= 1U;
	SCI8.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX66T_SCI9
/***********************************************************************************************************************
 * Function Name: R_SCI9_Create
 * Description  : This function initializes SCI9.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI9_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI9)			= 0U;			/* Cancel SCI8 module stop state */
	IPR(SCI9,RXI9)		= 15;			/* Set interrupt priority */
	IPR(SCI9,TXI9)		= 15;			/* Set interrupt priority */
	SCI9.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI9.SCR.BYTE		= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI9.SIMR1.BIT.IICM	= 0U;			/* Clear SIMR1.IICM bit */
	SCI9.SPMR.BIT.CKPH	= 0U;			/* Clear SPMR.CKPH bit */
	SCI9.SPMR.BIT.CKPOL	= 0U;			/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI9.SPMR.BYTE		= _00_SCI_RTS;
	SCI9.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI9.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI9.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI9.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI9.BRR			= 0x40U;
	SCI9.MDDR			= 255U;
	/* Set SCI8 pin */
	SCI9_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI9_Start
 * Description  : This function starts SCI9.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI9_Start(void)
{
	IR(SCI9,TXI9)		= 0U;	/* Clear interrupt flag */
	IR(SCI9,RXI9)		= 0U;	/* Clear interrupt flag */
	IEN(SCI9,TXI9)		= 1U;	/* Enable SCI interrupt */
	IEN(SCI9,RXI9)		= 1U;	/* Enable SCI interrupt */
	ICU.GENBL1.BIT.EN26	= 0U;
	ICU.GENBL1.BIT.EN27	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI9_Stop
 * Description  : This function stops SCI9.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI9_Stop(void)
{
	SCI9.SCR.BIT.TE		= 0U;	/* Disable serial transmit */
	SCI9.SCR.BIT.RE		= 0U;	/* Disable serial receive */
	SCI9.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI9.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI9,TXI9)		= 0U;
	IR(SCI9,TXI9)		= 0U;
	IEN(SCI9,RXI9)		= 0U;
	IR(SCI9,RXI9)		= 0U;
	ICU.GENBL1.BIT.EN26	= 0U;
	ICU.GENBL1.BIT.EN27	= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI9_Serial_Receive
 * Description  : This function receives SCI9 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI9_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci9_rx_count		= 0U;
	g_sci9_rx_length	= rx_num;
	gp_sci9_rx_address	= rx_buf;
	SCI9.SCR.BIT.RIE	= 1U;
	SCI9.SCR.BIT.RE		= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI9_Serial_Send
 * Description  : This function transmits SCI9 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI9_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci9_tx_address	= tx_buf;
	g_sci9_tx_count		= tx_num;
	/* Set TXD0 pin */
	SCI9.SCR.BIT.TIE	= 1U;
	SCI9.SCR.BIT.TE		= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX66T_SCI11
/***********************************************************************************************************************
 * Function Name: R_SCI11_Create
 * Description  : This function initializes SCI11.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI11_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI11)			= 0U;			/* Cancel SCI11 module stop state */
	IPR(SCI11,RXI11)	= 15;			/* Set interrupt priority */
	IPR(SCI11,TXI11)	= 15;			/* Set interrupt priority */
	SCI11.SCR.BYTE		= 0U;			/* Clear the control register */
#if 1
	/* Set clock enable */
	SCI11.SCR.BYTE			= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI11.SIMR1.BIT.IICM	= 0U;		/* Clear SIMR1.IICM bit */
	SCI11.SPMR.BIT.CKPH		= 0U;		/* Clear SPMR.CKPH bit */
	SCI11.SPMR.BIT.CKPOL	= 0U;		/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI11.SPMR.BYTE		= _00_SCI_RTS;
	SCI11.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI11.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI11.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI11.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI11.BRR			= 0x40U;
	SCI11.MDDR			= 255U;
#else
	/* Set clock enable */
	SCI11.SCR.BYTE			= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI11.SIMR1.BIT.IICM	= 0U;
	SCI11.SPMR.BYTE			= 0U;
	/* Set control registers */
	SCI11.SMR.BYTE		= _01_SCI_CLOCK_PCLK_4 | _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_STOP_1 |
						  _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI11.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_INVERT_NONE | _00_SCI_DATA_LSB_FIRST |
						  _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI11.SEMR.BYTE		= _00_SCI_BIT_MODULATION_DISABLE | _08_SCI_6_BASE_CLOCK | _20_SCI_NOISE_FILTER_ENABLE |
						  _80_SCI_FALLING_EDGE_START_BIT;
	SCI11.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI11.BRR			= 0x04U;
#endif
	/* Set SCI1 pin */
	SCI11_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI11_Start
 * Description  : This function starts SCI11.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI11_Start(void)
{
	IR(SCI11,TXI11)		= 0U;	/* Clear interrupt flag */
	IR(SCI11,RXI11)		= 0U;	/* Clear interrupt flag */
	IEN(SCI11,TXI11)	= 1U;	/* Enable SCI interrupt */
	IEN(SCI11,RXI11)	= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN12	= 0U;
	ICU.GENBL0.BIT.EN13	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI11_Stop
 * Description  : This function stops SCI1.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI11_Stop(void)
{
	SCI11.SCR.BIT.TE	= 0U;	/* Disable serial transmit */
	SCI11.SCR.BIT.RE	= 0U;	/* Disable serial receive */
	SCI11.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI11.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI11,TXI11)	= 0U;
	IEN(SCI11,RXI11)	= 0U;
	IR(SCI11,TXI11)		= 0U;
	IR(SCI11,RXI11)		= 0U;
	ICU.GENBL0.BIT.EN12	= 0U;
	ICU.GENBL0.BIT.EN13	= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI11_Serial_Receive
 * Description  : This function receives SCI11 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI11_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci11_rx_count	= 0U;
	g_sci11_rx_length	= rx_num;
	gp_sci11_rx_address	= rx_buf;
	SCI11.SCR.BIT.RIE	= 1U;
	SCI11.SCR.BIT.RE	= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI11_Serial_Send
 * Description  : This function transmits SCI11 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI11_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci11_tx_address	= tx_buf;
	g_sci11_tx_count	= tx_num;
	/* Set TXD11 pin */
	SCI11.SCR.BIT.TIE	= 1U;
	SCI11.SCR.BIT.TE	= 1U;
	return OK;
}
#endif

#ifdef	CONFIG_RX66T_SCI12
/***********************************************************************************************************************
 * Function Name: R_SCI12_Create
 * Description  : This function initializes SCI12.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI12_Create(void)
{
	RX_MPC_enable();
	MSTP(SCI12)			= 0U;			/* Cancel SCI1 module stop state */
	IPR(SCI12,RXI12)	= 15;			/* Set interrupt priority */
	IPR(SCI12,TXI12)	= 15;			/* Set interrupt priority */
	SCI12.SCR.BYTE		= 0U;			/* Clear the control register */
	/* Set clock enable */
	SCI12.SCR.BYTE			= _00_SCI_INTERNAL_SCK_UNUSED;
	SCI12.SIMR1.BIT.IICM	= 0U;		/* Clear SIMR1.IICM bit */
	SCI12.SPMR.BIT.CKPH		= 0U;		/* Clear SPMR.CKPH bit */
	SCI12.SPMR.BIT.CKPOL	= 0U;		/* Clear SPMR.CKPOL bit */
	/* Set control registers */
	SCI12.SPMR.BYTE		= _00_SCI_RTS;
	SCI12.SMR.BYTE		= _00_SCI_CLOCK_PCLK | _00_SCI_STOP_1 | _00_SCI_PARITY_DISABLE | _00_SCI_DATA_LENGTH_8 |
						  _00_SCI_MULTI_PROCESSOR_DISABLE | _00_SCI_ASYNCHRONOUS_OR_I2C_MODE;
	SCI12.SCMR.BYTE		= _00_SCI_SERIAL_MODE | _00_SCI_DATA_LSB_FIRST | _10_SCI_DATA_LENGTH_8_OR_7 | _62_SCI_SCMR_DEFAULT;
	SCI12.SEMR.BYTE		= _80_SCI_FALLING_EDGE_START_BIT | _20_SCI_NOISE_FILTER_ENABLE | _10_SCI_8_BASE_CLOCK |
						  _40_SCI_BAUDRATE_DOUBLE | _04_SCI_BIT_MODULATION_ENABLE;
	SCI12.SNFR.BYTE		= _00_SCI_ASYNC_DIV_1;
	/* Set bit rate */
	SCI12.BRR			= 0x40U;
	SCI12.MDDR			= 255U;
	/* Set SCI1 pin */
	SCI12_init_port();
	RX_MPC_disable();
}

/***********************************************************************************************************************
 * Function Name: R_SCI12_Start
 * Description  : This function starts SCI12.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI12_Start(void)
{
	IR(SCI12,TXI12)		= 0U;	/* Clear interrupt flag */
	IR(SCI12,RXI12)		= 0U;	/* Clear interrupt flag */
	IEN(SCI12,TXI12)	= 1U;	/* Enable SCI interrupt */
	IEN(SCI12,RXI12)	= 1U;	/* Enable SCI interrupt */
	ICU.GENBL0.BIT.EN16	= 0U;
	ICU.GENBL0.BIT.EN17	= 1U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI12_Stop
 * Description  : This function stops SCI12.
 * Arguments    : None
 * Return Value : None
 ***********************************************************************************************************************/
void	R_SCI12_Stop(void)
{
	SCI12.SCR.BIT.TE	= 0U;	/* Disable serial transmit */
	SCI12.SCR.BIT.RE	= 0U;	/* Disable serial receive */
	SCI12.SCR.BIT.TIE	= 0U;	/* disable TXI interrupt */
	SCI12.SCR.BIT.RIE	= 0U;	/* disable RXI and ERI interrupt */
	IEN(SCI12,TXI12)	= 0U;
	IEN(SCI12,RXI12)	= 0U;
	IR(SCI12,TXI12)		= 0U;
	IR(SCI12,RXI12)		= 0U;
	ICU.GENBL0.BIT.EN16	= 0U;
	ICU.GENBL0.BIT.EN17	= 0U;
}

/***********************************************************************************************************************
 * Function Name: R_SCI12_Serial_Receive
 * Description  : This function receives SCI12 data.
 * Arguments    : rx_buf - receive buffer pointer
 *                rx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI12_Serial_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
	if(1U > rx_num) {
		return MD_ARGERROR;
	}
	g_sci12_rx_count	= 0U;
	g_sci12_rx_length	= rx_num;
	gp_sci12_rx_address	= rx_buf;
	SCI12.SCR.BIT.RIE	= 1U;
	SCI12.SCR.BIT.RE	= 1U;
	return OK;
}
/***********************************************************************************************************************
 * Function Name: R_SCI12_Serial_Send
 * Description  : This function transmits SCI12 data.
 * Arguments    : tx_buf - transfer buffer pointer
 *                tx_num - buffer size
 * Return Value : status - MD_OK or MD_ARGERROR
 ***********************************************************************************************************************/
MD_STATUS	R_SCI12_Serial_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
	if(1U > tx_num) {
		return MD_ARGERROR;
	}
	gp_sci12_tx_address	= tx_buf;
	g_sci12_tx_count	= tx_num;
	/* Set TXD12 pin */
	SCI12.SCR.BIT.TIE	= 1U;
	SCI12.SCR.BIT.TE	= 1U;
	return OK;
}
#endif
