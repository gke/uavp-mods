// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =      Rewritten and ported to 18F2520 2008 by Prof. Greg Egan        =
// =                          http://www.uavp.org                        =
// =======================================================================
//
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

#include "c-ufo.h"
#include "bits.h"

// Prototypes
void InitTimersAndInterrupts(void);
void MapRC(void);
void ErrorRestart(void);

int8	PPMBit;
uint16	Width, Edge, PrevEdge;

uint8	RxBuff[RXBUFFMASK+1];
uint8	RCValid;
uint8	RxHead, RxTail;
uint8	RxCheckSum;

void InitTimersAndInterrupts()
{
	#ifdef CLOCK_16MHZ
	OpenTimer0(TIMER_INT_OFF&T0_8BIT&T0_SOURCE_INT&T0_PS_1_16);
	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);
	OpenTimer2(TIMER_INT_ON&T2_PS_1_16&T2_POST_1_16);
	#else // CLOCK_32MHZ
	OpenTimer0(TIMER_INT_OFF&T0_8BIT&T0_SOURCE_INT&T0_PS_1_16);
	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);	
	OpenTimer2(TIMER_INT_ON&T2_PS_1_32&T2_POST_1_16);
    #endif

	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge		
	CCP1CONbits.CCP1M0 = NegativePPM;
	PR2 = TMR2_5MS;
	_FirstTimeout = 0;
	PrevEdge = 0;

	// Serial I/O
	RxCheckSum = RxHead = RxTail = 0;
    PIE1bits.RCIE = true;

	ClockMilliSec = 0;
	LostTimer0Clicks = 0;

	INTCONbits.TMR0IE = true;
	INTCONbits.PEIE = true;

	EnableInterrupts;	
} // InitTimersAndInterrupts

void MapRC(void)
{  // re-maps captured PPM to Rx channel sequence
	// possible interference to PPM by irq.c?
	uint8 c;

	for (c = FirstC; c<= LastC; c++)
		RC[c] = StickFilter(RC[c], PPM[Map[c]]);

} // MapRC

void ErrorRestart(void)
{
	BadRCFrames++;
	_NewValues = false;
	_Signal = false;		// Signal lost
	PPMBit = -1;

	#ifndef RX_PPM
	CCP1CONbits.CCP1M0 = NegativePPM;
	#endif

} // ErrorRestart

#pragma interrupt low_isr_handler
void low_isr_handler(void)
{
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void)
{	// For 2.4GHz systems see README_DSM2_ETC.
	uint8 ch, c;

	if( PIR1bits.CCP1IF )
	{
		TMR2 = 0;								// re-set timer and postscaler
		PIR1bits.TMR2IF = false;
		_FirstTimeout = false;

		#ifndef RX_PPM							// single PPM pulse train from receiver
		CCP1CONbits.CCP1M0 ^= 1;				// toggle edge bit
		PR2 = TMR2_5MS;				
		#endif  // !RX_PPM

		if( PPMBit == 0 )
		{
			PrevEdge = CCPR1;
			RCValid = true;
		}
		else
		{
			Edge = CCPR1;
			Width = (Edge - PrevEdge) >> 1;
			RCValid &= (Width & 0xff00) == 0x0100; 	// 256..511 (1024-2047us)
			PPM[PPMBit] = Width & 0x00ff;			// 0..127 (0-1023uS)
			PrevEdge = Edge;
		}

		#ifdef RX_PPM
		if ( PPMBit >= 5 )
			if ( RCValid )
			{
				if (PPMBit >= 7 )
					_NewValues = _Signal = true;
			}
			else
				ErrorRestart();
		#else
		if ( PPMBit == 5 ) 							// preserve original semantics
			if ( RCValid )
				_NewValues = _Signal = true;
			else
				ErrorRestart();
		#endif // RX_PPM		
	

		// MapRC is used outside interrupt to filter and map PPM[c] to RC[c]
	
		PIR1bits.CCP1IF = false;
		PPMBit++;
	}
	else
	if( PIR1bits.TMR2IF )						// 5 OR 14 ms  without an active edge
	{
		#ifndef RX_PPM							// single PPM pulse train from receiver
		if( _FirstTimeout )						// 5 ms time out
		{
			PR2 = TMR2_5MS;						// set compare reg to 5ms
			ErrorRestart();
		}
		_FirstTimeout = true;
		PR2 = TMR2_14MS;						// set compare reg to 14ms
		#endif // RX_PPM
		PPMBit = 0;
		PIR1bits.TMR2IF = false;
	}
	else
	if( INTCONbits.T0IF && INTCONbits.T0IE )
	{
		ClockMilliSec++;						// time since start
		TimeSlot--;
		INTCONbits.T0IF = false;
	}
	else
	if (PIR1bits.RCIF )
	{
		if (RCSTAbits.OERR)
		{
			ch = RCREG; // flush
			RCSTAbits.CREN = false;
			RCSTAbits.CREN = true;
		}
		else
		{
			RxTail = (RxTail+1) & RXBUFFMASK;	// no check for overflow yet
			RxBuff[RxTail] = RCREG;
		}
		PIR1bits.RCIF = false;
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



