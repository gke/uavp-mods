// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =           Extensively modified 2008-9 by Prof. Greg Egan            =
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
// Major changes to irq.c including removal of redundant source by Ing. Greg Egan - 
// use at your own risk - see GPL.

#include "pu-test.h"
#include "bits.h"

// Interrupt Routine

#pragma udata irqvars
int16 	NewK1, NewK2, NewK3, NewK4, NewK5, NewK6, NewK7;
uns8	RecFlags;
#pragma udata

#pragma interrupt low_isr_handler
void low_isr_handler(void)
{
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void)
{
int8	NewRoll, NewPitch, NewYaw;	
int16 	Temp;

	// For 2.4GHz systems see README_DSM2_ETC.
	if( INTCONbits.TMR0IF && INTCONbits.TMR0IE )
	{
		INTCONbits.TMR0IF = 0;				// quit int
		TimeSlot--;
	}

	if( PIR1bits.CCP1IF )
	{
		TMR2 = 0;				// re-set timer and postscaler
		PIR1bits.TMR2IF = 0;				// quit int
		_FirstTimeout = 0;

#ifndef RX_PPM						// single PPM pulse train from receiver
							// standard usage (PPM, 3 or 4 channels input)
		CCP1CONbits.CCP1M0 ^= 1;	// toggle edge bit
		PR2 = TMR2_5MS;				// set compare reg to 5ms

		if( NegativePPM ^ CCP1CONbits.CCP1M0  )		// a negative edge
		{
#endif
			// could be replaced by a switch ???

			if( RecFlags == 0 )
			{
				NewK1 = CCPR1;
			}
			else
			if( RecFlags == 2 )
			{
				NewK3 = CCPR1;
				NewK2 = NewK3 - NewK2;
				NewK2 >>= 1;
			}
			else
			if( RecFlags == 4 )
			{
				NewK5 = CCPR1;
				NewK4 = NewK5 - NewK4;
				NewK4 >>= 1;
			}
			else
			if( RecFlags == 6 )
			{
				NewK7 = CCPR1;
				NewK6 = NewK7 - NewK6;
				CurrK6 = NewK6 >> 1;
			}
#ifdef RX_PPM
			else
#else
			else	// values are unsafe
				goto ErrorRestart;
		}
		else	// a positive edge
		{
#endif // RX_PPM 
			if( RecFlags == 1 )
			{
				NewK2 = CCPR1;
				NewK1 = NewK2 - NewK1;
				NewK1 >>= 1;
			}
			else
			if( RecFlags == 3 )
			{
				NewK4 = CCPR1;
				NewK3 = NewK4 - NewK3;
				NewK3 >>= 1;
			}
			else
			if( RecFlags == 5 )
			{
				NewK6 = CCPR1;
				NewK5 = NewK6 - NewK5;
				NewK5 >>= 1;
				CurrK1 = NewK1;
				CurrK2 = NewK2;
				CurrK3 = NewK3;
				CurrK4 = NewK4;
				CurrK5 = NewK5;
				_NewValues = 1;
// sanity check
// NewKx has values in 4us units now. content must be 256..511 (1024-2047us)
				if( ((NewK1>>8) == 1) &&
				    ((NewK2>>8) == 1) &&
				    ((NewK3>>8) == 1) &&
				    ((NewK4>>8) == 1) &&
				    ((NewK5>>8) == 1) )
				{
					_NoSignal = 0;
				}
				else	// values are unsafe
					goto ErrorRestart;
			}
			else
			if( RecFlags == 7 )
			{
				NewK7 = CCPR1 - NewK7;
				CurrK7 = NewK7 >> 1;
				RecFlags = -1;
			}
			else
			{
ErrorRestart:
				_NewValues = 0;
				_NoSignal = 1;		// Signal lost
				RecFlags = -1;
#ifndef RX_PPM
				if( NegativePPM )
					CCP1CONbits.CCP1M0 = 1;	// wait for positive edge next
				else
					CCP1CONbits.CCP1M0 = 0;	// wait for negative edge next
#endif
			}	
#ifndef RX_PPM
		}
#endif
		PIR1bits.CCP1IF = 0;				// quit int
		RecFlags++;
	}

	if( PIR1bits.TMR2IF )	// 5 or 14 ms have elapsed without an active edge
	{
		PIR1bits.TMR2IF = 0;	// quit int
#ifndef RX_PPM	// single PPM pulse train from receiver
		if( _FirstTimeout )			// 5 ms have been gone by...
		{
			PR2 = TMR2_5MS;			// set compare reg to 5ms
			goto ErrorRestart;
		}
		_FirstTimeout = 1;
		PR2 = TMR2_14MS;			// set compare reg to 14ms
#endif
		RecFlags = 0;
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

