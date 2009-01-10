// ==============================================
// =    U.A.V.P Brushless UFO Test-Software     =
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
// =  please visit http://www.uavp.de           =
// =               http://www.mahringer.co.at   =
// ==============================================

// Power output test

#pragma codepage = 1

#include "pu-test.h"
#include "bits.h"

// flash output for a second, then return to its previous state
void PowerOutput(uns8 niout)
{
	bank1 uns8 nii, nij;

	for( nij=0; nij < 10; nij++ )	// 10 flashes (count MUST be even!)
	{
		switch(niout)
		{
#ifdef BOARD_3_0
			case 0:	PORTB.7 ^= 1; break;	// toggle AUX
			case 1:	PORTC.3 ^= 1; break;	// toggle BLUE
			case 2:	PORTC.4 ^= 1; break;	// toggle RED
			case 3:	PORTC.1 ^= 1; break;	// toggle GREEN
			case 4:	PORTC.5 ^= 1; break;	// toggle AUX
			case 5:	PORTC.0 ^= 1; break;	// toggle YELLOW
			case 6:	PORTB.6 ^= 1; break;	// toggle BEEPER
#endif
#ifdef BOARD_3_1
			case 0:	LedShadow ^= 0x01; break;	// toggle AUX
			case 1:	LedShadow ^= 0x02; break;	// toggle BLUE
			case 2:	LedShadow ^= 0x04; break;	// toggle RED
			case 3:	LedShadow ^= 0x08; break;	// toggle GREEN
			case 4:	LedShadow ^= 0x10; break;	// toggle AUX
			case 5:	LedShadow ^= 0x20; break;	// toggle YELLOW
			case 6:	LedShadow ^= 0x40; break;	// toggle AUX
			case 7:	LedShadow ^= 0x80; break;	// toggle BEEPER
#endif
		}	
#ifdef BOARD_3_1
		SendLeds();	// send LEDs to bus
#endif
		for( nii=0; nii < 200; nii++)
		{
			T0IF = 0;
			while( T0IF == 0 )	;
		}
	}		
}
