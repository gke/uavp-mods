// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =           Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer          =
// =              Copyright 2008, 2009 by Prof. Greg Egan                =
// =                            http://uavp.ch                           =
// =======================================================================
//
//    UAVP is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVP is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "c-ufo.h"
#include "bits.h"


// read the current parameter set into the RAM variables
void ReadEEdata(void) // 245 uSec @ 16MHz
{
	int *p;
	EEADR = _EESet1;	// default 1st parameter set
	if( IK5 > _Neutral )
		EEADR = _EESet2;	// user selected 2nd parameter set

	for(p = &FirstProgReg; p <= &LastProgReg; p++)
	{
		EEPGD = 0;
		RD = 1;
		*p = EEDATA;
		EEADR++;
	}

	// modified Ing. Greg Egan
	BatteryVolts = LowVoltThres;

	// Sanity check
	//if timing value is lower than 1, set it to 10ms!
	if( TimeSlot < 2 )
		TimeSlot = 2;
	else
	if ( TimeSlot > 20 )
		TimeSlot = 20;
}


