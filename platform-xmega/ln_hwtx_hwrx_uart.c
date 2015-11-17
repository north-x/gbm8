/****************************************************************************
    Copyright (C) 2006 Stefan Bormann

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

 Title :   LocoNet Software UART Access library
 Author:   Stefan Bormann <stefan.bormann@gmx.de>
 Date:     17-Aug-2006
 Software:  AVR-GCC
 Target:    megaAVR

 DESCRIPTION
  Basic routines for interfacing to the LocoNet via the hardware USART.

  The current version is only capable of LocoNet master operation,
  sending out replies, as it neither implements collision detection
  nor collision avoidance.
       
 USAGE
  See the C include ln_sw_uart.h file for a description of each function.
  This should be restructure in the near future!!!
       
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>

#include "ln_interface.h"

#define	LN_TICKS_PER_BIT	(F_CPU / 16666 / 8)

typedef enum
{
	FALSE_E = 0,
	TRUE_E  = 1
}
Bool_t;


LnBuf *pstLnRxBuffer;      // this queue eats received LN messages

volatile Bool_t bCdTimerActive;     // set on reception, reset on CD timer overflow

static unsigned short usCdTimerStarted;  // state of hardware timer when the last byte was received

volatile Bool_t bBusySending;              // Signal for RX interrupt to keep hands of CD timer

volatile uint8_t rxIsrStatus;
volatile uint8_t rxIsrData;

/*
//#define COLLISION_MONITOR
#ifdef COLLISION_MONITOR
#define COLLISION_MONITOR_PORT PORTB
#define COLLISION_MONITOR_DDR DDRB
#define COLLISION_MONITOR_BIT PB4
#endif

//#define STARTBIT_MONITOR
#ifdef STARTBIT_MONITOR
#define STARTBIT_MONITOR_PORT PORTB
#define STARTBIT_MONITOR_DDR DDRB
#define STARTBIT_MONITOR_BIT PB4
#endif
*/

#if defined GBM8_20121212
#define LN_HW_UART_ID			USARTE0
#define LN_HW_UART_RX_SIGNAL    USARTE0_RXC_vect
#define LN_RX_PORT				PORTE
#define LN_RX_BIT				2

#define LN_TX_PORT				PORTE
#define LN_TX_DDR				PORTE.DIR
#define LN_TX_BIT				3
#define LN_TX_PINCTRL			PORTE.PIN3CTRL

#elif defined GBM8_20120104
#define LN_HW_UART_ID			USARTD1
#define LN_HW_UART_RX_SIGNAL    USARTD1_RXC_vect
#define LN_RX_PORT				PORTD
#define LN_RX_BIT				6

#define LN_TX_PORT				PORTD
#define LN_TX_DDR				PORTD.DIR
#define LN_TX_BIT				7
#define LN_TX_PINCTRL			PORTD.PIN7CTRL
#else
	#error "No valid hardware version defined"
#endif

#define LN_SW_UART_SET_TX_LOW  LN_TX_PORT.OUTCLR = (1<<LN_TX_BIT);  // to pull down LN line to drive low level
#define LN_SW_UART_SET_TX_HIGH LN_TX_PORT.OUTSET = (1<<LN_TX_BIT);  // master pull up will take care of high LN level

#define LN_TMR					TCC1

#define LN_HW_UART_CONTROL_REGA LN_HW_UART_ID.CTRLA
#define LN_HW_UART_CONTROL_REGB LN_HW_UART_ID.CTRLB
#define LN_HW_UART_CONTROL_REGC LN_HW_UART_ID.CTRLC
#define LN_HW_UART_BAUDRATE_REG LN_HW_UART_ID.BAUDCTRLA
#define LN_HW_UART_DATA_REG     LN_HW_UART_ID.DATA


void initLocoNetHardware(LnBuf *pstRxBuffer)
{	
#if defined GBM8_20121212
	// PORTE 2:RX 3:TX
	PORTE.PIN2CTRL = PORT_OPC_TOTEM_gc;
	PORTE.PIN3CTRL = PORT_OPC_TOTEM_gc | PORT_INVEN_bm;
	PORTE.OUTSET = (1<<2)|(1<<3);
	PORTE.DIRSET = (1<<2)|(1<<3);
	
	// Set voltage reference
	ACA.CTRLB = ((2500*10/78*64/3300)-1)&0x3F; // 2.5 V Threshold
	
	// connect comparators to pins
	ACA.AC0MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_SCALER_gc;
	ACA.AC1MUXCTRL = AC_MUXPOS_PIN5_gc | AC_MUXNEG_SCALER_gc;
	
	// and enable them
	//AC0 generates event on falling edge
	ACA.AC0CTRL = AC_ENABLE_bm | AC_INTMODE_FALLING_gc | AC_INTLVL_OFF_gc | AC_HYSMODE_SMALL_gc;
	//AC1 generates event on rising edge
	ACA.AC1CTRL = AC_ENABLE_bm | AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc | AC_HYSMODE_SMALL_gc;

	// clear pending interrupt flags (net free detection)
	ACA.STATUS = AC_AC0IF_bm | AC_AC1IF_bm;
	
	EVSYS.CH0MUX = EVSYS_CHMUX_ACA_CH0_gc;
	EVSYS.CH1MUX = EVSYS_CHMUX_ACA_CH1_gc;
	
	// Timer E0: Event0 clears the output Event 1 sets the output
	TCE0.CTRLB = TC0_CCCEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCE0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH0_gc;
	TCE0.PER = 0xFFFF;
	TCE0.CCC = 0xFFFF;
	TCE0.CTRLA = TC_CLKSEL_EVCH1_gc;
	
#ifdef DEBUG_ENABLE_LN_OUTPUT_ON_SERVO22_PWM
	PORTC.DIRSET = 1<<3;
	PORTC.PIN3CTRL = PORT_OPC_TOTEM_gc;
	
	TCC0.CTRLB = TC0_CCDEN_bm | TC_WGMODE_SINGLESLOPE_gc;
	TCC0.CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH0_gc;
	TCC0.PER = 0xFFFF;
	TCC0.CCD = 0xFFFF;
	TCC0.CTRLA = TC_CLKSEL_EVCH1_gc;
#endif

	// Use event to set initial conditions
	EVSYS.DATA = (1<<1);
	EVSYS.STROBE = (1<<1);
		
#elif defined GBM8_20120104
	LN_TX_PINCTRL = PORT_OPC_TOTEM_gc | PORT_INVEN_bm;
	PORTD.OUTSET = 1<<LN_TX_BIT;
	PORTD.DIRSET = 1<<LN_TX_BIT;

	// Enable Pull-down on RX line (voltage divider), input sense both edges
	PORTD.PIN6CTRL = PORT_ISC_BOTHEDGES_gc | PORT_OPC_PULLDOWN_gc;
	
	// Input sense toghether with Port interrupt 0 are used to detect if the bus is free.
	LN_RX_PORT.INT0MASK = (1<<LN_RX_BIT);
	// Clear any pending flags
	LN_RX_PORT.INTFLAGS = PORT_INT0IF_bm | PORT_INT1IF_bm;
#endif
	// Set the TX line to IDLE
	LN_SW_UART_SET_TX_HIGH

	pstLnRxBuffer = pstRxBuffer ;

	// enable UART receiver and transmitter
	LN_HW_UART_CONTROL_REGB = USART_RXEN_bm | USART_TXEN_bm;

	// enable UART interrupt
	LN_HW_UART_CONTROL_REGA = USART_RXCINTLVL_LO_gc;
	
	// set baud rate
	//LN_HW_UART_BAUDRATE_REG = ((F_CPU)/(16666l*16l)-1);
	LN_HW_UART_BAUDRATE_REG = (int)((F_CPU)/(16.0/0.00006)-0.5);

	// Timer C1: Prescaler 8
	LN_TMR.PER = 0xFFFF;
	LN_TMR.CTRLA = TC_CLKSEL_DIV8_gc;
	LN_TMR.CTRLB = 0;			// Normal mode PER = TOP
	LN_TMR.CTRLD = 0;			// No action for incoming events
	
	/*
	PMIC.CTRL = 0x07;	// Enable High, Medium and Low level interrupts
    sei();				// Enable global interrupts
	
	uint8_t txchar = 0xAA;
	for (;;)
	{
		_delay_ms(100);
		if (LN_HW_UART_ID.STATUS&USART_DREIF_bm)
		{
			LN_HW_UART_DATA_REG = txchar;
		}
	}
*/
}


/////////////////////////// Network idle detection ///////////////////////////
// 
static void ResetPinChange(void)
{
#if defined GBM8_20121212
	// Reset the interrupt flag
	ACA.STATUS = AC_AC0IF_bm|AC_AC1IF_bm;
#elif defined GBM8_20120104
	LN_RX_PORT.INTFLAGS = PORT_INT0IF_bm | PORT_INT1IF_bm;
#endif
}

static Bool_t IsPinChanged(void)
{
#if defined GBM8_20121212
	if (ACA.STATUS&(AC_AC0IF_bm|AC_AC1IF_bm))
		return TRUE_E;
	else
		return FALSE_E;
#elif defined GBM8_20120104
	if (LN_RX_PORT.INTFLAGS!=0)
		return TRUE_E;
	else
		return FALSE_E;
#endif
}

////////////////////////////// CD backoff timer //////////////////////////////
// The idea is to use a free running 16 bit timer as basis.
// When a busy net is detected, the current counter state is captured by software
// Possible triggers:
// - UART receive interrupt
// - break generator
// - polling of network state
// The difference of the counter state and the captured start time calculates
// the count of bits since the trigger.
// Additionally the same timer is used for bit timing in the transmit function.


static inline unsigned short BitCount2Ticks(unsigned short usBitCount)
{
	return usBitCount * LN_TICKS_PER_BIT;
}


static void StartCdTimer(void)
{
	// Timestamp
	usCdTimerStarted = LN_TMR.CNT;
	
	// not yet wraped around -> h/w timer value may be used to determine bits since trigger
	bCdTimerActive = TRUE_E;
	
	// Setup compare unit to see when timer wraps around
	LN_TMR.CCA = usCdTimerStarted-1;
	LN_TMR.INTFLAGS = TC1_CCAIF_bm;
	LN_TMR.INTCTRLB |= TC_CCAINTLVL_LO_gc;
	
	// Want to see if anything happens on the net. If not, we count the time since now to declare the net free
	ResetPinChange();
}

static Bool_t IsBitCountReached(unsigned char ucBitCount)
{
	if (!bCdTimerActive)
		return TRUE_E;

	unsigned short usDelta = LN_TMR.CNT - usCdTimerStarted;

	return usDelta >= BitCount2Ticks(ucBitCount);
}


static void WaitForBitCountReached(unsigned char ucBitCount)
{
	while (!IsBitCountReached(ucBitCount));
}

ISR(TCC1_CCA_vect)
{
	bCdTimerActive = FALSE_E;	// reset flag -> net is declared totally free now
	LN_TMR.INTCTRLB &= ~(TC_CCAINTLVL_HI_gc);
}

///////////////////////////////// Receive ////////////////////////////////////
// We are receiving data with the UART hardware interrupt driven.
// This is as trivial as it can be.

static unsigned char ucDataOverrunError;

ISR(LN_HW_UART_RX_SIGNAL)
{
	rxIsrStatus = (1<<0);
	if (LN_HW_UART_ID.STATUS & USART_FERR_bm)
	{
		// should invalidate current message in buffer, but how?
		pstLnRxBuffer->Stats.RxErrors++;
		rxIsrStatus |= (1<<1);
	}
	if (LN_HW_UART_ID.STATUS & USART_BUFOVF_bm)
	{
		ucDataOverrunError++;
		rxIsrStatus |= (1<<2);
	}

	// must read this register whether we have an error flag or not
	unsigned char ucRxByte = LN_HW_UART_DATA_REG;

	// want to know when CD was *started*
	if (!bBusySending)  // not messing with timer, while transmitting, because
		StartCdTimer(); // same timer is used for bit timing

	// store incoming data in queue
	addByteLnBuf(pstLnRxBuffer, ucRxByte);
	
	rxIsrData = ucRxByte;
}

#define LN_COLLISION_TICKS 15


static void HandleCollision(void)
{
	StartCdTimer();  // counting bits for break from here

	// Invert TX pin
	LN_TX_PINCTRL ^= PORT_INVEN_bm;
	sei();
	WaitForBitCountReached(LN_COLLISION_TICKS);

	LN_TX_PINCTRL ^= PORT_INVEN_bm;	

	StartCdTimer();  // starting timer, now used for backoff
	bBusySending = FALSE_E;
	pstLnRxBuffer->Stats.Collisions++;
}


LN_STATUS sendLocoNetPacketTry(lnMsg *TxData, unsigned char ucPrioDelay)
{
	unsigned char ucCheckSum;
	unsigned char ucTxIndex;
	unsigned char ucTxLength = getLnMsgSize(TxData);

	// First calculate the checksum as it may not have been done
	ucCheckSum = 0xFF ;

	for(ucTxIndex = 0; ucTxIndex < ucTxLength-1; ucTxIndex++)
		ucCheckSum ^= TxData->data[ ucTxIndex ];

	TxData->data[ ucTxLength-1 ] = ucCheckSum; 

	// clip maximum prio delay???????????????????????????????????????
	if (ucPrioDelay > LN_BACKOFF_MAX)
		ucPrioDelay = LN_BACKOFF_MAX;

	// Check for current traffic
	if (IsPinChanged())
	{
		StartCdTimer();  // there was traffic, start counting backoff timer
		return LN_NETWORK_BUSY;
	}

	// If the Network is not Idle, don't start the packet
	if (!IsBitCountReached(LN_CARRIER_TICKS))
		return LN_CD_BACKOFF;

	// if priority delay was waited now, declare net as free for this try
	if (!IsBitCountReached(ucPrioDelay))
		return LN_PRIO_BACKOFF;

	StartCdTimer();  // either we or somebody else will start a packet now

	for (ucTxIndex=0; ucTxIndex < ucTxLength; ucTxIndex++)
	{
		volatile unsigned char ucDataByte = TxData->data[ucTxIndex];
		//volatile Bool_t bLastBit = FALSE_E;  // start bit is always zero

		// We need to do this with interrupts off.
		// The last time we check for free net until sending our start bit
		// must be as short as possible, not interrupted.
		cli();

		// start bit
		if (!bit_is_set(LN_RX_PORT.IN, LN_RX_BIT)) //-->---must be <2us by spec.-->-+
		{	                                    //                                  |
			sei();  // reenable interrupts, UART RX shall work...                   |
			if (ucTxIndex==0)            // first bit disturbed -> don't touch      |
			    return LN_NETWORK_BUSY;  // somebody else was faster                V
			HandleCollision();           // not first bit -> collision              |
			return LN_COLLISION;         //                                         |
		}	                      //                                                |
		LN_HW_UART_DATA_REG = ucDataByte;// Seizure of network:  <------------------+
		
		// generating bits relative to this time stamp
		usCdTimerStarted = LN_TMR.CNT;

		// avoid that HW UART Receive ISR messes with the timer
		bBusySending = TRUE_E;

		rxIsrStatus = 0;		
		sei();
		
		while (rxIsrStatus==0); // && !(IsBitCountReached(14)));

		// check eighth bit for collision
		WaitForBitCountReached(9);
		
		if (rxIsrData!=ucDataByte)
		{	// last bit sent was one and we see a zero on the net -> collision detected
			HandleCollision();
			return LN_COLLISION;
		}
	}
	StartCdTimer();
	bBusySending = FALSE_E;

	pstLnRxBuffer->Stats.TxPackets++;

	return LN_DONE;
}
