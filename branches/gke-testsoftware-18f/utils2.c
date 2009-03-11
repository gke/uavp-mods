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


// Utilities and subroutines


#include "pu-test.h"
#include "bits.h"


static int i;

// wait blocking for "dur" * 0.1 seconds
// Motor and servo pulses are still output every 10ms
void Delay100mS(uint8 dur)
{
	uint8 k, j;

	// a TMR0 Timeout is 0,25us * 256 * 16 (presc) = 1024 us
	WriteTimer0(0);

	for(k = 0; k < 10; k++)
	{
		for(i = 0; i < dur; i++)
		{
// wait ca. 10ms (10*1024us (see _Prescale0)) before outputting
			for( j = 10; j != 0; j-- )
			{
				while( INTCONbits.T0IF == 0 );
				INTCONbits.T0IF = 0;
			}
			// break loop if a serial command is in FIFO
			if( PIR1bits.RCIF )
				return;
		}
	}
}

void nop2()
{
	Delay1TCY();
	Delay1TCY();
}

//
// The LED routines, only needed for 
// PCB revision 3.1 (registered power driver TPIC6B595N)
//
void SendLeds(void)
{
	uint8 s;

	/* send LedShadow byte to TPIC */

	LISL_CS = 1;	// CS to 1
	LISL_IO = 0;	// SDA is output
	LISL_SCL = 0;	// because latch on positive edge
	
	i = LedShadow;
	for(s=8; s!=0; s--)
	{
		if( i & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		LISL_SCL = 1;
		i<<=1;
		LISL_SCL = 0;
	}

	PORTCbits.RC1 = 1;
	PORTCbits.RC1 = 0;	// latch into drivers
}

void SwitchLedsOn(uint8 l)
{
	LedShadow |= l;
	SendLeds();
}

void SwitchLedsOff(uint8 l)
{
	LedShadow &= ~l;
	SendLeds();
}



