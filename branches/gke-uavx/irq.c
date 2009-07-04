// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
// =======================================================================

//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.


// Interrupt routine
// Major changes to irq.c including removal of redundant source by Ing. Greg Egan - 
// use at your own risk - see GPL.

// The 64uS assumed for the maximum interrupt latency in the generation of the 
// output pulse preamble in utils.c  most likely applies to the Timer0 interrupt. 
// The original version of irq.c had a much longer path through the receiving of bit5 
// exacerbated by the stick filter code. This caused the edge of the 1mS preamble to 
// be peridocally missed and for the OutSignals routine to emit preambles greater 
// than 1mS. GKE

#include "uavx.h"

// Interrupt Routine

#define MIN_PPM_SYNC_PAUSE 2500  	/* 2500 *2us = 5ms */

// Prototypes

void ReceivingGPSOnly(uint8);
void InitTimersAndInterrupts(void);
void MapRC(void);
void low_isr_handler(void);
void high_isr_handler(void);

// Variables

uint8 RxCheckSum;


void ReceivingGPSOnly(uint8 r)
{
	if ( r != _ReceivingGPS )
	{
		PIE1bits.RCIE = false;
		_ReceivingGPS = r;
		if ( _ReceivingGPS )
		{
			// reset origin
			InitGPS();
			InitNavigation();

			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B9600);
		}
		else
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);

   		PIE1bits.RCIE = r;
		Delay1mS(10);				// switch bounce
	}
} // ReceivingGPSOnly

void MapRC(void)
{  // re-maps captured PPM to Rx channel sequence
	uint8 c;
	#define RCFilter NoFilter

	for (c = 0 ; c < CONTROLS ; c++)
		RC[c] = PPM[Map[TxRxType][c]-1]& 0xff;

	RC[RollC] -= _Neutral;
	RC[PitchC] -= _Neutral;
	RC[YawC] -= _Neutral;

} // MapRC

void InitTimersAndInterrupts(void)
{
	int8 i;

	OpenTimer0(TIMER_INT_OFF&T0_8BIT&T0_SOURCE_INT&T0_PS_1_16);
	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);

	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge
	CCP1CONbits.CCP1M0 = _NegativePPM;

//	OpenTimer2(TIMER_INT_ON&T2_PS_1_16&T2_POST_1_16);		
//	PR2 = TMR2_5MS;		// set compare reg to 9ms

	for (i = 0; i<= CompassUpdate; i++)
		mS[i] = 0;

	for (i = 0; i < CONTROLS; i++)
		PPM[i] = RC[i] = 0;

	RC[RollC] = RC[PitchC] = RC[YawC] = RC_NEUTRAL;

	PPM_Index =0;
	PrevEdge = 0;
	RCGlitches = 0;
	RxCheckSum = 0;
	_Signal = _NewValues = false;
   	ReceivingGPSOnly(false);
} // InitTimersAndInterrupts


#pragma interrupt low_isr_handler
void low_isr_handler(void)
{
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void)
{	
	static uint8 ch;
	
	if( PIR1bits.CCP1IF && PIE1bits.CCP1IE ) // An Rx PPM pulse edge has been detected
	{
		CurrCCPR1 = CCPR1;
		if ( CurrCCPR1 < PrevEdge )
			PrevEdge -= 0xffff;	// Deal with wraparound

		Width = CurrCCPR1 - PrevEdge;
		PrevEdge = CurrCCPR1;		

		if ( Width > MIN_PPM_SYNC_PAUSE ) 		// A pause in 2us ticks > 5ms 
		{
			PPM_Index = 0;						// Sync pulse detected - next CH is CH1
			RCFrameOK = true;
			PauseTime = Width;
		}
		else // An actual channel -- Record the variable part of the PWM time 
			if (PPM_Index < CONTROLS)
			{	
				Width >>= 1; 					// Width in 4us ticks.
	
				if ( (Width & 0xff00) == 0x0100 ) 	// Check pulse is 1.024 to 2.048mS
				{
					PPM[PPM_Index] = Width;
					_NewValues = false; 		// Strictly after PPM_Index == 0
				}	
				else
				{
					// preserve old value i.e. default hold
					RCGlitches++;
					RCFrameOK = false;
				}
				PPM_Index++;
				// MUST demand rock solid RC frames for autonomous functions not
				// to be cancelled by noise-generated partially correct frames
				if ( PPM_Index == CONTROLS )
				{	  
					_NewValues = RCFrameOK;
					_Signal = true;
					mS[RCSignalTimeout] = mS[Clock] + RC_SIGNAL_TIMEOUT;
					CCP1CONbits.CCP1M0 = _NegativePPM; // reset in case Tx/Rx combo has changed
				}	
			}

		if ( !RxPPM )							
			CCP1CONbits.CCP1M0 ^= 1;				// For composite PPM signal not using wired OR

		PIR1bits.CCP1IF = false;
	}	

	if (PIR1bits.RCIF && PIE1bits.RCIE)	// GPS and commands
	{
		if ( RCSTAbits.OERR || RCSTAbits.FERR )
		{
			ch = RCREG; // flush
			RCSTAbits.CREN = false;
			RCSTAbits.CREN = true;
		}
		else
			PollGPS(RCREG);
		PIR1bits.RCIF = false;
	}

	if ( INTCONbits.T0IF && INTCONbits.T0IE )
	{ 
		mS[Clock]++;
		if ( (mS[Clock] > mS[RCSignalTimeout]) && _Signal )
		{
			CCP1CONbits.CCP1M0 = _NegativePPM; // Reset in case Tx/Rx combo has changed
			_Signal = false;
		}	
		INTCONbits.TMR0IF = false;	
	}
 	
} // high_isr_handler
	
#pragma code high_isr = 0x08
void high_isr (void)
{
  _asm goto high_isr_handler _endasm
} // high_isr
#pragma code

#pragma code low_isr = 0x18
void low_isr (void)
{
  _asm goto low_isr_handler _endasm
} // low_isr
#pragma code

