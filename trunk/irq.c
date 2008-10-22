// ==============================================
// =      U.A.V.P Brushless UFO Controller      =
// =           Professional Version             =
// = Copyright (c) 2007 Ing. Wolfgang Mahringer =
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
// =               http://www.mahringer.co.at   =
// ==============================================

// Interrupt routine

#include "c-ufo.h"
#include "bits.h"

#include <int16cxx.h>	//interrupt support

#pragma origin 4

// Interrupt Routine

bank0	uns16	NewK1, NewK2, NewK3, NewK4, NewK5;
uns16	NewK6@NewK1;
uns16	NewK7@NewK2;
uns8	RecFlags;
uns8	NewRoll@NewK2.high8;
uns8	NewNick@NewK3.high8;
uns8	NewTurn@NewK4.high8;

#pragma interruptSaveCheck w

interrupt irq(void)
{
	uns16 CCPR1 @0x15;

	int_save_registers;	// save W and STATUS

	if( TMR2IF )	// 5 or 14 ms have elapsed without an active edge
	{
		TMR2IF = 0;	// quit int
#ifndef RX_PPM	// single PPM pulse train from receiver
		if( _FirstTimeout )	// 5 ms have been gone by...
		{
			PR2 = TMR2_5MS;		// set compare reg to 5ms
			goto ErrorRestart;
		}
		_FirstTimeout = 1;
		PR2 = TMR2_14MS;		// set compare reg to 14ms
#endif
		RecFlags = 0;
	}
	if( CCP1IF )
	{

#ifdef RX_PPM	// single PPM pulse train from receiver

		TMR2 = 0;	// re-set timer and postscaler
		TMR2IF = 0;	// quit int
		_FirstTimeout = 0;

		if( RecFlags == 0 )
			NewK1 = CCPR1;
		else
		if( RecFlags == 1 )
		{
			NewK2 = CCPR1;
			NewK1 = CCPR1 - NewK1;
		}
		else
		if( RecFlags == 2 )
		{
			NewK3 = CCPR1;
			NewK2 = CCPR1 - NewK2;
		}
		else
		if( RecFlags == 3 )
		{
			NewK4 = CCPR1;
			NewK3 = CCPR1 - NewK3;
		}
		else
		if( RecFlags == 4 )
		{
			NewK5 = CCPR1;
			NewK4 = CCPR1 - NewK4;
		}
		else
		if( RecFlags == 5 )
		{
			NewK5 = CCPR1 - NewK5;
// all complete! Now copy all the values at once
			NewK1 >>= 1;
			NewK2 >>= 1;
			NewK3 >>= 1;
			NewK4 >>= 1;
			NewK5 >>= 1;
// sanity check
// NewKx has values in 4us units now. content must be 256..511 (1024-2047us)
			if( (NewK1.high8 == 1) &&
			    (NewK2.high8 == 1) &&
			    (NewK3.high8 == 1) &&
			    (NewK4.high8 == 1) &&
			    (NewK5.high8 == 1) )
			{
				if( FutabaMode )
				{
					IGas  = NewK3.low8;
#ifdef EXCHROLLNICK
					NewRoll = NewK2.low8;
					NewNick = NewK1.low8;
#else	// REAL transmitters use this
					NewRoll = NewK1.low8;
					NewNick = NewK2.low8;
#endif
				}
				else
				{
					IGas  = NewK1.low8;
					NewRoll = NewK2.low8;
					NewNick = NewK3.low8;
				}
				NewTurn = NewK4.low8;

				IK5 = NewK5.low8;

				W = _Neutral;
				NewRoll -= W;
				NewNick -= W;
				NewTurn -= W;

				if( DoubleRate )
				{
					(int)NewRoll >>= 1;
					(int)NewNick >>= 1;
				}

		// New Roll/Nick = (3*Old_Roll + New_Roll)/4
		//
		// smooths away jitter from the transmitter
		// very good idea from ufo_hans (Hans Haider), THANK YOU!

				NewK5 = (long)IRoll;
				NewK5 <<= 1;
				NewK5 += (long)IRoll;
				(long)NewK5 += (int)NewRoll;
				NewK5 >>= 2;
				IRoll = NewK5.low8;

				NewK5 = (long)INick;
				NewK5 <<= 1;
				NewK5 += (long)INick;
				(long)NewK5 += (int)NewNick;
				NewK5 >>= 2;
				INick = NewK5.low8;
				
				NewK5 = (long)ITurn;
				NewK5 <<= 1;
				NewK5 += (long)ITurn;
				(long)NewK5 += (int)NewTurn;
				NewK5 >>= 2;
				ITurn = NewK5.low8;
								
				NewK6 = CCPR1;
				
				_NoSignal = 0;
				_NewValues = 1;
			}
			else	// values are unsafe
				goto ErrorRestart;
		}
		else
		if( RecFlags == 6 )
		{
			NewK7 = CCPR1;
			NewK6 = CCPR1 - NewK6;
			IK6 = NewK6 >> 1;
		}
		else
		if( RecFlags == 7 )
		{
			NewK7 = CCPR1 - NewK7;
			IK7 = NewK7 >> 1;
			RecFlags = -1;
		}
		else	// values are unsafe
		{
ErrorRestart:
			_NewValues = 0;
			_NoSignal = 1;		// Signal lost
			RecFlags = -1;
		}

		CCP1IF = 0;		// quit int
		RecFlags++;

#else	// standard usage (PPM, 3 or 4 channels input)
		CCPR1.low8 = CCPR1L;
		CCPR1.high8 = CCPR1H;
		CCP1M0 ^= 1;	// toggle edge bit
		PR2 = TMR2_5MS;		// set compare reg to 5ms
		TMR2 = 0;	// re-set timer and postscaler
		TMR2IF = 0;	// quit int
		_FirstTimeout = 0;

		if( NegativePPM ^ CCP1M0  )	// a negative edge
		{
			if( RecFlags == 0 )
			{
				NewK1 = CCPR1;
			}
			else
			if( RecFlags == 2 )
			{
				NewK3 = CCPR1;
				NewK2 = CCPR1 - NewK2;
			}
			else
			if( RecFlags == 4 )
			{
				NewK5 = CCPR1;
				NewK4 = CCPR1 - NewK4;
			}
			else
			if( RecFlags == 6 )
			{
				NewK7 = CCPR1;
				NewK6 = CCPR1 - NewK6;
				IK6 = NewK6 >> 1;
			}
			else	// values are unsafe
			{
				goto ErrorRestart;
			}
		}
		else	// a positive edge
		{
			if( RecFlags == 1 )
			{
				NewK2 = CCPR1;
				NewK1 = CCPR1 - NewK1;
			}
			else
			if( RecFlags == 3 )
			{
				NewK4 = CCPR1;
				NewK3 = CCPR1 - NewK3;
			}
			else
			if( RecFlags == 5 )
			{
				NewK5 = CCPR1 - NewK5;
// all complete! Now copy all the values at once
				NewK1 >>= 1;
				NewK2 >>= 1;
				NewK3 >>= 1;
				NewK4 >>= 1;
				NewK5 >>= 1;
// sanity check
// NewKx has values in 4us units now. content must be 256..511 (1024-2047us)
				if( (NewK1.high8 == 1) &&
				    (NewK2.high8 == 1) &&
				    (NewK3.high8 == 1) &&
				    (NewK4.high8 == 1) &&
				    (NewK5.high8 == 1) )
				{
					if( FutabaMode )
					{
						IGas  = NewK3.low8;
#ifdef EXCHROLLNICK
						NewRoll = NewK2.low8;
						NewNick = NewK1.low8;
#else	// REAL transmitters use this
						NewRoll = NewK1.low8;
						NewNick = NewK2.low8;
#endif
					}
					else
					{
						IGas  = NewK1.low8;
						NewRoll = NewK2.low8;
						NewNick = NewK3.low8;
					}
					NewTurn = NewK4.low8;

					IK5 = NewK5.low8;

					W = _Neutral;
					NewRoll -= W;
					NewNick -= W;
					NewTurn -= W;
					
					if( DoubleRate )
					{
						(int)NewRoll >>= 1;
						(int)NewNick >>= 1;
					}

		// New Roll/Nick = (3*Old_Roll + New_Roll)/4
		//
		// smooths away jitter from the transmitter
		// very good idea from ufo_hans (Hans Haider), THANK YOU!

					NewK5 = (long)IRoll;
					NewK5 <<= 1;
					NewK5 += (long)IRoll;
					(long)NewK5 += (int)NewRoll;
					NewK5 >>= 2;
					IRoll = NewK5.low8;

					NewK5 = (long)INick;
					NewK5 <<= 1;
					NewK5 += (long)INick;
					(long)NewK5 += (int)NewNick;
					NewK5 >>= 2;
					INick = NewK5.low8;
		
					NewK5 = (long)ITurn;
					NewK5 <<= 1;
					NewK5 += (long)ITurn;
					(long)NewK5 += (int)NewTurn;
					NewK5 >>= 2;
					ITurn = NewK5.low8;

		//			ITurn = NewK4.low8 - _Neutral;

					_NoSignal = 0;
					_NewValues = 1;
					NewK6 = CCPR1;
				}
				else	// values are unsafe
				{
					goto ErrorRestart;
				}
			}
			else
			if( RecFlags == 7 )
			{
				NewK7 = CCPR1 - NewK7;
				IK7 = NewK7 >> 1;
				RecFlags = -1;
			}
			else
			{
ErrorRestart:
				_NewValues = 0;
				_NoSignal = 1;		// Signal lost
				RecFlags = -1;
				if( NegativePPM )
					CCP1M0 = 1;	// wait for positive edge next
				else
					CCP1M0 = 0;	// wait for negative edge next
			}	

		}
		CCP1IF = 0;		// quit int
		RecFlags++;

#endif // !RX_PPM
	}
	if( T0IF && T0IE )
	{
		T0IF = 0;	// quit int
		TimeSlot--;
	}
	
	int_restore_registers;
}

