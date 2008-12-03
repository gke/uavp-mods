// ==============================================
// =      U.A.V.P Brushless UFO Controller      =
// =           Professional Version             =
// = Copyright (c) 2007 Ing. Wolfgang Mahringer =
// =      Rewritten 2008 Ing. Greg Egan         =
// ==============================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// ==============================================
// =  please visit http://www.uavp.org          =
// ==============================================

// Interrupt routine
// Major changes to irq.c including removal of redundant source by Ing. Greg Egan - 
// use at your own risk - see GPL.

#include "c-ufo.h"
#include "bits.h"

#include <int16cxx.h>	//interrupt support

#pragma origin 4

// Interrupt Routine

bank1 int16 	NewK1, NewK2, NewK3, NewK4, NewK5, NewK6, NewK7;

uns8	RecFlags;

#pragma interruptSaveCheck w

#define Filter(O,N) (( O + N + 1) >> 1)

interrupt irq(void)
{
int8	NewRoll, NewNick, NewTurn;	
int16 	Temp;
uns16 	CCPR1 @0x15;

// For 2.4GHz systems see README_DSM2_ETC.
 
	int_save_registers;	// save W and STATUS

	if( TMR2IF )	// 5 or 14 ms have elapsed without an active edge
	{
		TMR2IF = 0;	// quit int
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
	if( CCP1IF )
	{
		TMR2 = 0;				// re-set timer and postscaler
		TMR2IF = 0;				// quit int
			
		if ( _IntIsMasked )
			_RxFrameOK = 0;


		_FirstTimeout = 0;

#ifndef RX_PPM	// single PPM pulse train from receiver
		// standard usage (PPM, 3 or 4 channels input)
		CCPR1.low8 = CCPR1L;
		CCPR1.high8 = CCPR1H;
		CCP1M0 ^= 1;	// toggle edge bit
		PR2 = TMR2_5MS;				// set compare reg to 5ms

		if( NegativePPM ^ CCP1M0  )		// a negative edge
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
				NewK6 >>= 1; 		
#ifdef RX_DSM2
				if (NewK6.high8 !=1) 	// add glitch detection to 6 & 7
					goto ErrorRestart;
#else
				IK6 = NewK6.low8;
#endif // RX_DSM2	
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

				// sanity check - NewKx has values in 4us units now. 
				// content must be 256..511 (1024-2047us)
				if( (NewK1.high8 == 1) &&
				    (NewK2.high8 == 1) &&
				    (NewK3.high8 == 1) &&
				    (NewK4.high8 == 1) &&
				    (NewK5.high8 == 1) )
				{
#ifndef RX_DSM2									
					if( FutabaMode ) // Ch3 set for Throttle on UAPSet
					{
						IGas = NewK3.low8;
#ifdef EXCHROLLNICK
						NewRoll = NewK2.low8;
						NewNick = NewK1.low8;
#else
						NewRoll = NewK1.low8;
						NewNick = NewK2.low8;
#endif // EXCHROLLNICK
					}
					else
					{
						IGas  = NewK1.low8;
						NewRoll = NewK2.low8;
						NewNick = NewK3.low8;
					}					

					IRoll = NewRoll - _Neutral; 
					INick = NewNick - _Neutral;
					ITurn = NewK4.low8 - _Neutral;
					
					IK5 = NewK5.low8;
					IK6 = 0;
					IK7 = 0;

					_NoSignal = 0;
					_NewValues = _RxFrameOK; // potentially IK6 & IK7 are still about to change ???
#ifdef DEBUG_RXERRORS
					if ( _RxFrameOK )
					{
						LedBlue_OFF;
					}
					else
					{
						LedBlue_ON;
					}
#endif // DEBUG_RXERRORS
					_RxFrameOK = 1;
#endif // !RX_DSM2
				}
				else	// values are unsafe
					goto ErrorRestart;
			}
			else
			if( RecFlags == 7 )
			{
				NewK7 = CCPR1 - NewK7;
				NewK7 >>= 1;	
#ifdef RX_DSM2
				if (NewK7.high8 !=1)	
					goto ErrorRestart;

				if( FutabaMode ) // Ch3 set for Throttle on UAPSet
				{

//EDIT FROM HERE ->
// CURRENTLY Futaba 9C with Spektrum DM8 / JR 9XII with DM9 module
					IGas = NewK5.low8;

					IRoll = NewK3.low8 - _Neutral; 
					INick = NewK2.low8 - _Neutral;
					ITurn = NewK1.low8 - _Neutral;
					IK5 = NewK6.low8; // do not filter
					IK6 = NewK4.low8;
					IK7 = NewK7.low8;
// TO HERE
				}
				else // Reference 2.4GHz configuration DX7 and AR7000 Rx

				{
					IGas = NewK6.low8;

					IRoll = NewK1.low8 - _Neutral; 
					INick = NewK4.low8 - _Neutral;
					ITurn = NewK7.low8 - _Neutral;

					IK5 = NewK3.low8; // do not filter
					IK6 = NewK5.low8;
					IK7 = NewK2.low8;
				}

				_NoSignal = 0;
				_NewValues = _RxFrameOK;
#ifdef DEBUG_RXERRORS
				if ( _RxFrameOK )
				{
					LedBlue_OFF;
				}
				else
				{
				LedBlue_ON;
				}
#endif // DEBUG_RXERRORS
				_RxFrameOK = 1;
#else				
				IK7 = NewK7.low8;
#endif // RX_DSM2 
				RecFlags = -1;
			}
			else
			{
ErrorRestart:
				_NewValues = 0;
				_RxFrameOK = 1;
				_NoSignal = 1;		// Signal lost
				RecFlags = -1;
#ifndef RX_PPM
				if( NegativePPM )
					CCP1M0 = 1;	// wait for positive edge next
				else
					CCP1M0 = 0;	// wait for negative edge next
#endif
			}	
#ifndef RX_PPM
		}
#endif
		CCP1IF = 0;				// quit int
		RecFlags++;
	}
	else
	if( T0IF && T0IE )
	{
		T0IF = 0;				// quit int
		TimeSlot--;
	}
	
	int_restore_registers;
}

