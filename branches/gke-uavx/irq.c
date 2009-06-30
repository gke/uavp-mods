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

// Prototypes

void ReceivingGPSOnly(uint8);
void InitTimersAndInterrupts(void);
void low_isr_handler(void);
void high_isr_handler(void);

// Variables

#pragma udata isrvars
int16 	NewK1, NewK2, NewK3, NewK4, NewK5, NewK6, NewK7;
int8	RCState;
int24	PrevEdge, CurrEdge;
int16 	Width;
#pragma udata

#pragma udata rxvars
uint8 RxCheckSum;
#pragma udata 

void ReceivingGPSOnly(uint8 r)
{
	if ( r != _ReceivingGPS )
	{
		PIE1bits.RCIE = false;
		_ReceivingGPS = r;
		if ( _ReceivingGPS )
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B9600);
		else
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);

   		PIE1bits.RCIE = r;
		Delay1mS(10);				// switch bounce
	}
} // ReceivingGPSOnly

void InitTimersAndInterrupts(void)
{
	OpenTimer0(TIMER_INT_OFF&T0_8BIT&T0_SOURCE_INT&T0_PS_1_16);
	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);

	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge
	CCP1CONbits.CCP1M0 = NegativePPM;

	OpenTimer2(TIMER_INT_ON&T2_PS_1_16&T2_POST_1_16);		
	PR2 = TMR2_5MS;		// set compare reg to 9ms

	RCGlitches = 0;
	RxCheckSum = 0;
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
		
	// For 2.4GHz systems see README_DSM2_ETC.
	if( PIR1bits.TMR2IF )	// 5 or 14 ms have elapsed without an active edge
	{
		PIR1bits.TMR2IF = false;	// quit int

		if ( !RxPPM )				// single PPM pulse train from receiver
		{
			if( _FirstTimeout )			// 5 ms have been gone by...
			{
				PR2 = TMR2_5MS;			// set compare reg to 5ms
				goto ErrorRestart;
			}
			_FirstTimeout = true;
			PR2 = TMR2_14MS;			// set compare reg to 14ms
		}

		RCState = 0;
	}

	if( PIR1bits.CCP1IF )
	{
		TMR2 = 0;					// re-set timer and postscaler
		PIR1bits.TMR2IF = false;	// quit int
		_FirstTimeout = false;

		CurrEdge = CCPR1;
		if ( CurrEdge < PrevEdge )
			PrevEdge -= 0xffff;		// Deal with Timer1 wraparound
		Width = (CurrEdge - PrevEdge) >> 1;
		PrevEdge = CurrEdge;
		
		switch ( RCState )
		{
			case 0: break;
			case 1: NewK1 = Width; break;
			case 2: NewK2 = Width; break;
			case 3: NewK3 = Width; break;
			case 4: NewK4 = Width; break;
			case 5:
			{
				NewK5 = Width;
				// sanity check - NewKx has values in 4us units now. 
				// content must be 256..511 (1024-2047us)
				if( ((NewK1 >>8 ) == 1) &&
				    ((NewK2 >>8 ) == 1) &&
				    ((NewK3 >>8 ) == 1) &&
				    ((NewK4 >>8 ) == 1) &&
				    ((NewK5 >>8 ) == 1) )
				{
					if ( !DSM2Mode)
					{									
						if( FutabaMode ) // Ch3 set for Throttle on UAPSet
						{
							IGas = NewK3 & 0xff;
							IRoll = (NewK1 & 0xff) - (int16)_Neutral;
							IPitch = (NewK2 & 0xff) - (int16)_Neutral;
						}
						else
						{
							IGas  = NewK1  & 0xff;
							IRoll = (NewK2 & 0xff) - (int16)_Neutral;
							IPitch = (NewK3 & 0xff) - (int16)_Neutral;
						}
						IYaw = (NewK4 & 0xff) - (int16)_Neutral;						
						IK5 = NewK5 & 0xff;
	
						_Signal = _NewValues = true; // potentially IK6 & IK7 are still about to change ???
					}

					break;
				}
				else	// values are unsafe
					goto ErrorRestart;
			}
			case 6:
			{
				NewK6 = Width;		
				if ( DSM2Mode )
				{
					if ( (NewK6>>8) !=1) 	// add glitch detection to 6 & 7
						goto ErrorRestart;
				}
				else
					IK6 = NewK6 & 0xff;
				break;	
			}
			case 7:
			{
				NewK7 = Width;	
				if ( DSM2Mode )
				{
					if ( (NewK7>>8) !=1)	
						goto ErrorRestart;
	
					if( FutabaMode ) // Ch3 set for Throttle on UAPSet
					{
				//EDIT FROM HERE ->
				// CURRENTLY Futaba 9C with Spektrum DM8 / JR 9XII with DM9 module
						IGas = NewK5 & 0xff;
	
						IRoll = (NewK3 & 0xff) - (int16)_Neutral; 
						IPitch = (NewK2 & 0xff) - (int16)_Neutral;
						IYaw = (NewK1 & 0xff) - (int16)_Neutral;
	
						IK5 = NewK6 & 0xff; // do not filter
						IK6 = NewK4 & 0xff;
						IK7 = NewK7 & 0xff;
				// TO HERE
					}
					else // Reference 2.4GHz configuration DX7 Tx and AR7000 Rx
					{
						IGas = NewK6 & 0xff;
	
						IRoll = (int16)(NewK1 & 0xff) - (int16)_Neutral; 
						IPitch = (int16)(NewK4 & 0xff) - (int16)_Neutral;
						IYaw = (int16)(NewK7 & 0xff) - (int16)_Neutral;
	
						IK5 = NewK3 & 0xff; // do not filter
						IK6 = NewK5 & 0xff;
						IK7 = NewK2 & 0xff;
					}	
	
					_Signal = _NewValues = true;
				}
				else				
					IK7 = NewK7 & 0xff;

				RCState = -1;
				break;
			}
			default: 
			ErrorRestart:
			{	
				_Signal = _NewValues = false;	
				RCGlitches++;
				RCState = -1;

				if ( !RxPPM )
					CCP1CONbits.CCP1M0 = NegativePPM;

				break;
			}
		} // switch
	
		if ( !RxPPM )
		{				
			CCP1CONbits.CCP1M0 ^= 1;
			PR2 = TMR2_5MS;
		}

		PIR1bits.CCP1IF = false;				// quit int
		RCState++;
	}

	if (PIR1bits.RCIF && PIE1bits.RCIE)	// GPS and commands
	{
		PIR1bits.RCIF = false;

		if ( RCSTAbits.OERR || RCSTAbits.FERR )
		{
			ch = RCREG; // flush
			RCSTAbits.CREN = false;
			RCSTAbits.CREN = true;
		}
		else
			PollGPS(RCREG);
	} 	

	if( INTCONbits.TMR0IE && INTCONbits.TMR0IF )
	{
		INTCONbits.TMR0IF = false;				// quit int
		TimeSlot--;
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

