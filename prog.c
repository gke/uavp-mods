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

#include "c-ufo.h"
#include "bits.h"

int8 ReadEE(uns8 addr)
{
	int8 b;

	EEADR = addr;
	EECON1bits.EEPGD = false;
	EECON1bits.RD = true;
	b=EEDATA;
	EECON1 = 0;
	return(b);	
} // ReadEE


void ReadParametersEE(void)
{
	int8 *p, c; 
	uns16 addr;

	if( IK5 > _Neutral )
		addr = _EESet2;	
	else
		addr = _EESet1;
	
	for(p = &FirstProgReg; p <= &LastProgReg; p++)
		*p = ReadEE(addr++);

	// modified Ing. Greg Egan
	BatteryVolts = LowVoltThres;

	// Sanity check
	//if timing value is lower than 1, set it to 10ms!
	if( TimeSlot < 2 )
		TimeSlot = 2;
	else
	if ( TimeSlot > 20 )
		TimeSlot = 20;
} // ReadParametersEE

void WriteEE(uns8 addr, int8 d)
{
	int8 rd;
	
	rd = ReadEE(addr);
	if ( rd != d )						// avoid redundant writes
	{
		EEDATA = d;				
		EEADR = addr;
		EECON1bits.EEPGD = false;
		EECON1bits.WREN = true;

		DisableInterrupts;
		EECON2 = 0x55;
		EECON2 = 0xaa;
		EECON1bits.WR = true;
		while(EECON1bits.WR);
		EnableInterrupts;

		EECON1bits.WREN = false;
	}

} // WriteEE

void WriteParametersEE(uns8 s)
{
	int8 *p;
	uns8 b;
	uns16 addr;
	
	if( s == 1 )
		addr = _EESet1;	
	else
		addr = _EESet2;

	p = &FirstProgReg; 
	while ( p <= &LastProgReg)
		WriteEE(addr++, *p++);
} // WriteParametersEE
