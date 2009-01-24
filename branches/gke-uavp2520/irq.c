// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =             Ported 2008 to 18F2520 by Prof. Greg Egan               =
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

uint8	RecFlags;
uint16	NewK1, NewK2, NewK3, NewK4, NewK5, NewK6, NewK7;
int16	NewRoll, NewPitch, NewYaw, NewThrottle, OldThrottle = 0;

void InitTimersAndInterrupts()
{
	OpenTimer0(TIMER_INT_OFF&T0_8BIT&T0_SOURCE_INT&T0_PS_1_16);

	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);

	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge		
	CCP1CONbits.CCP1M0 = NegativePPM;
	
	OpenTimer2(TIMER_INT_ON&T2_PS_1_16&T2_POST_1_16);
	PR2 = TMR2_5MS;
	_FirstTimeout = 0;

	ClockMilliSec = 0;

	INTCONbits.TMR0IE = true;
	INTCONbits.PEIE = true;

	EnableInterrupts;	
} // InitTimersAndInterrupts

#pragma interrupt low_isr_handler
void low_isr_handler(void)
{
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void)
{
// For 2.4GHz systems see README_DSM2_ETC.
	uint8 ch;

	if( PIR1bits.TMR2IF )				// 5 or 14 ms have elapsed without an active edge
	{
		PIR1bits.TMR2IF = false;		// quit int
#ifndef RX_PPM	// single PPM pulse train from receiver
		if( _FirstTimeout )				// 5 ms have been gone by...
		{
			PR2 = TMR2_5MS;				// set compare reg to 5ms
			goto ErrorRestart;
		}
		_FirstTimeout = true;
		PR2 = TMR2_14MS;				// set compare reg to 14ms
#endif
		RecFlags = 0;
	}
	if( PIR1bits.CCP1IF )
	{
		TMR2 = 0;						// re-set timer and postscaler
		PIR1bits.TMR2IF = false;		// quit int
		_FirstTimeout = false;

#ifndef RX_PPM							// single PPM pulse train from receiver
										// standard usage (PPM, 3 or 4 channels input)
		CCP1CONbits.CCP1M0 ^= 1;		// toggle edge bit
		PR2 = TMR2_5MS;					// set compare reg to 5ms

		if( NegativePPM ^ CCP1CONbits.CCP1M0  )	// a negative edge
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
				if (Upper8(NewK6) !=1) 	// add glitch detection to 6 & 7
					goto ErrorRestart;
#else
				IK6 = Lower8(NewK6);
#endif // RX_DSM2	
			}
#ifdef RX_PPM
			else
#else
			else						// values are unsafe
				goto ErrorRestart;
		}
		else							// a positive edge
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
				// this test is fast but has a loophole!
				if( ( NewK1 & NewK2 & NewK3 & NewK4 & NewK5 & 0x0100 ) == 0x0100 ) 
				{
#ifndef RX_DSM2									
					if( FutabaMode ) 	// Ch3 set for Throttle on UAPSet
					{
						NewThrottle = Lower8(NewK3);
#ifdef EXCHROLLNICK
						NewRoll = Lower8(NewK2) - _Neutral;
						NewPitch = Lower8(NewK1)- _Neutral;
#else
						NewRoll = Lower8(NewK1)- _Neutral;
						NewPitch = Lower8(NewK2)- _Neutral;
#endif // EXCHROLLNICK
					}
					else
					{
						NewThrottle  = Lower8(NewK1);
						NewRoll = Lower8(NewK2)- _Neutral;
						NewPitch = Lower8(NewK3)- _Neutral;
					}
					NewYaw = Lower8(NewK4) - _Neutral;

					IThrottle = StickFilter(OldThrottle, NewThrottle);
					OldThrottle = IThrottle; 
					
					IRoll = StickFilter(IRoll, NewRoll); 
					IPitch = StickFilter(IPitch, NewPitch);
					IYaw = StickFilter(IYaw, NewYaw);
	
					IK5 = Lower8(NewK5);
					IK6 = - _Neutral;
					IK7 = - _Neutral;
				
					_Signal = true;
					_NewValues = true; 	// potentially IK6 & IK7 are still about to change ???
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
				if (Upper8(NewK7) !=1)	
					goto ErrorRestart;

				if( FutabaMode ) 		// Ch3 set for Throttle on UAPSet
				{

//EDIT FROM HERE ->
// CURRENTLY Futaba 9C with Spektrum DM8 / JR 9XII with DM9 module
					NewThrottle = NewK5;

					NewRoll = Lower8(NewK3) - _Neutral; 
					NewPitch = Lower8(NewK2) - _Neutral;
					NewYaw = Lower8(NewK1) - _Neutral;

					IK5 = Lower8(NewK6); 
					IK6 = Lower8(NewK4);
					IK7 = Lower8(NewK7);
// TO HERE
				}
				else // Reference 2.4GHz configuration DX7 and AR7000 Rx

				{
					NewThrottle = Lower8(NewK6);

					NewRoll = Lower8(NewK1) - _Neutral; 
					NewPitch = Lower8(NewK4) - _Neutral;
					NewYaw = Lower8(NewK7) - _Neutral;
					if( DoubleRate )
					{
						NewRoll >>= 1;
						NewPitch >>= 1;
					}
					IK5 = Lower8(NewK3); // do not filter
					IK6 = Lower8(NewK5);
					IK7 = Lower8(NewK2);
				}

				IThrottle = StickFilter(OldThrottle, NewThrottle);
				OldThrottle = IThrottle;

				IRoll = StickFilter(IRoll, NewRoll); 
				IPitch = StickFilter(IPitch, NewPitch);
				IYaw = StickFilter(IYaw, NewYaw);		

				_Signal = true;
				_NewValues = true;
#else				
				IK7 = Lower8(NewK7);
#endif // RX_DSM2 
				RecFlags = -1;
			}
			else
			{
ErrorRestart:
				BadRCFrames++;
				_NewValues = false;
				_Signal = false;		// Signal lost
				RecFlags = -1;
#ifndef RX_PPM
				if( NegativePPM )
					CCP1CONbits.CCP1M0 = true;	// wait for positive edge next
				else
					CCP1CONbits.CCP1M0 = false;	// wait for negative edge next
#endif
			}	
#ifndef RX_PPM
		}
#endif
		PIR1bits.CCP1IF = false;
		RecFlags++;
	}
	if( INTCONbits.T0IF && INTCONbits.T0IE )
	{
		INTCONbits.T0IF = false;
		ClockMilliSec++;						// time since start
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



