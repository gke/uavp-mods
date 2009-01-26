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

// Autonomous flight routines

#include "c-ufo.h"
#include "bits.h"

// Prototypes
uint8 Descend(uint8);
void DoAutonomous(void);
void Navigate(void);
void ParseGPSSentence(void);
uint8 RCLinkRestored(int32);

uint8 RCLinkRestored(int32 d)
{
	// checks for good RC frames for a period d mS
	if ( !_Signal )
		TimerMilliSec = ClockMilliSec + d;
	return(ClockMilliSec > TimerMilliSec );
} // RCLinkRestored

uint8 Descend(uint8 T)
{
	if ( _UseBaro )
	{
		DesiredBaroAltitude = Limit(CurrBaroAltitude - 10, 0, 32000);				// ??? needs more thought
	}
	else
	{	
	// need to use accelerometer based descent control
	//	if (((ClockMilliSec & 0x000000ff) == 0 ) && ( T > 0 ))
		if ( T > 0 )
			T -= 1;
	}
		return(T);
} // Descend

void Navigate()
{
#define MaxPitch 10
#define MaxRoll 10

	int16 NorthDiff, EastDiff;

	if ( GPSSentenceReceived )
	{
		GPSSentenceReceived=false; 
		ParseGPSSentence();
	}

	if ( _GPSValid )
	{ 
		// QC point north

		NorthDiff=(GPSOriginLatitude-GPSLatitude)*(real32)(EarthR);
		EastDiff=(GPSOriginLongitude-GPSLongitude)*(real32)(EarthR); // * LongitudeCorrection

		IPitch = - MaxPitch * Limit(NorthDiff*NorthDiff+EastDiff*EastDiff, 0, 10);
		IRoll = 0;

		IYaw = 333; // DesiredHeading - GPSHeading

	// return home ???

	}
} // Navigate

void DoAutonomous(void)
{
	if (_GPSValid )
	{
		DesiredThrottle = THR_HOVER;
		DesiredBaroAltitude = 100;
		Navigate();
	}
	else
	{
		DesiredThrottle = Descend(DesiredThrottle);
		IRoll = IPitch = IYaw = 0;
	}
} // DoAutonomous
