// =======================================================================
// =                                 UAVX                                =
// =                         Quadrocopter Control                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
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
void Descend(void);
void ReturnHome(void);
void ParseGPSSentence(void);
void ResetTimeOuts(void);
void Navigate(int16, int16);
void HoldStation(void);

// Defines



// Variables
extern boolean FirstGPSSentence;
extern int16 GPSNorth, GPSEast, GPSNorthHold, GPSEastHold;
extern boolean GPSSentenceReceived;

void Descend(void)
{	
	if( (BlinkCount & 0x000f) == 0 )
		DesiredThrottle = Limit(DesiredThrottle--, 0, _Maximum); // safest	
} // Descend

#define GPSFilter SoftFilter
#define ANGLE_SCALE ((PROXIMITY * 256L + MAX_ANGLE/2)/MAX_ANGLE)

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// To avoid cos/sin/arctan calls a simple look up process is used.

	int16 Angle, Temp;
	int16 RangeApprox;
	int16 EastDiff, NorthDiff;
	int16 RollCorrection, PitchCorrection;
	int24 Temp2;

	EastDiff = GPSEastWay - GPSEast;
	NorthDiff = GPSNorthWay - GPSNorth;

	Temp2 = (int24)NorthDiff*(int24)NorthDiff + (int24)EastDiff*(int24)EastDiff;
	RangeApprox = Limit(Temp2, 0, PROXIMITY);
		
	Angle = Make2Pi(int16atan2(-EastDiff, NorthDiff) - CompassHeading);

	RollCorrection = (-int16sin(Angle) * RangeApprox + ANGLE_SCALE/2)/ (int16)ANGLE_SCALE;
	DesiredRoll += RollCorrection;
	DesiredRoll = Limit(DesiredRoll, -_Maximum, _Maximum);

	PitchCorrection = (-int16cos(Angle) * RangeApprox + ANGLE_SCALE/2)/(int16)ANGLE_SCALE;
	DesiredPitch += PitchCorrection;
	DesiredPitch = Limit(DesiredPitch, -_Maximum, _Maximum);

} // Navigate

void HoldStation()
{
	if ( _GPSValid && _UseCompass ) 
	{ 
		if ( !_HoldingStation )
		{
			GPSNorthHold = GPSNorth;
			GPSEastHold = GPSEast;
			_HoldingStation = true;
		}		
		Navigate(GPSNorthHold, GPSEastHold);
		//AltitudeHold(CurrAltitude);
	}

} // HoldStation

void ReturnHome(void)
{
	#ifdef TEST_SOFTWARE
	if ( _GPSValid )
	#else
	if ( _GPSValid && _UseCompass )
	#endif
		Navigate(0, 0);
		//AltitudeHold(RETURN_ALT); // rely on Baro hold for now
} // ReturnHome

void CheckAutonomous(void)
{
	DesiredThrottle = IGas;
	DesiredRoll = IRoll;
	DesiredPitch = IPitch;
	DesiredYaw = IYaw;

	#ifdef ENABLE_AUTONOMOUS

	UpdateGPS();

	if ( _NoSignal && _Flying )
	{ // NO AUTONOMY ON LOST SIGNAL IN THIS VERSION
	  // do nothing - use Wolfgang's failsafe
	}
	else
		if ( _Hovering )
			HoldStation();
		else
		{
			_HoldingStation = false;
			if ( IK5 > _Neutral )
				ReturnHome();
		}
	#endif // ENABLE_AUTONOMOUS

} // CheckAutonomous

