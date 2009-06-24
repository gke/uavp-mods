// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =                          http://uavp.ch                             =
// =======================================================================

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVXP is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Autonomous flight routines

#include "uavx.h"

// Prototypes

void Descend(void);
void Navigate(int16, int16);
void DoNavigation(void);
void DoFailsafe(void);

// Variables
extern int16 ValidGPSSentences;
extern boolean GPSSentenceReceived;

int16 NavRCorr, SumNavRCorr, NavPCorr, SumNavPCorr, NavYCorr, SumNavYCorr;

void GPSAltitudeHold(int16 DesiredAltitude)
{
	int16 Temp;

	if ( _GPSAltitudeValid && _UseRTHGPSAlt )
	{
		AE = Limit(DesiredAltitude - GPSRelAltitude, -50, 50); // 5 metre band
		AltSum += AE;
		AltSum = Limit(AltSum, -10, 10);	
		Temp = SRS16(AE*NavAltKp + AltSum*NavAltKi, 5);
	
		DesiredThrottle = HoverThrottle + Limit(Temp, -10, 30);
		DesiredThrottle = Limit(DesiredThrottle, 0, _Maximum);
	}
	else
	{
		// use baro
	}
} // GPSAltitudeHold

void Descend(void)
{	
} // Descend

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer artihmetic

	static int32 Temp;
	static int16 NavKp, GPSGain;
	static int16 Range, EastDiff, NorthDiff, WayHeading, RelHeading;

	if ( _NavComputed ) // maintain previous corrections
	{
		DesiredRoll = Limit(DesiredRoll + NavRCorr, -_Neutral, _Neutral);
		DesiredPitch = Limit(DesiredPitch + NavPCorr, -_Neutral, _Neutral);
		DesiredYaw = Limit(DesiredYaw + NavYCorr, -_Neutral, _Neutral);
	}
	else
	{
		EastDiff = GPSEastWay - GPSEast;
		NorthDiff = GPSNorthWay - GPSNorth;

		if ( (Abs(EastDiff) >= 1 ) || (Abs(NorthDiff) >=1 ))
		{ 
			Range = Max(Abs(NorthDiff), Abs(EastDiff)); 
			_Proximity = Range < NavClosingRadius;
			if ( _Proximity )
				Range = int16sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff); 
			else
				Range = NavClosingRadius;

			#define NavKi 1
			#ifdef GPS_IK7_GAIN
			GPSGain = Limit(NavSensitivity, 0, 256); // compensate for EPA offset of up to 20
			NavKp = ( GPSGain * MAX_ANGLE ) / NavClosingRadius; // /_Maximum) * 256L
			#else
			#define NavKp (((int32)MAX_ANGLE * 256L ) / NavClosingRadius )
			#endif // GPS_IK7_GAIN
	
			Temp = ((int32)Range * NavKp )>>8; // allways +ve so can use >>
			WayHeading = int16atan2(EastDiff, NorthDiff);

			RelHeading = Make2Pi(WayHeading - Heading);
			NavRCorr = SRS32((int32)int16sin(RelHeading) * Temp, 8);			
			NavPCorr = SRS32(-(int32)int16cos(RelHeading) * Temp, 8);

			#ifdef TURN_TO_HOME
			if ( Range >= NavClosingRadius )
			{
				RelHeading = MakePi(WayHeading - Heading); // make +/- MilliPi
				NavYCorr = -(RelHeading * NAV_YAW_LIMIT) / HALFMILLIPI;
				NavYCorr = Limit(NavYCorr, -NAV_YAW_LIMIT, NAV_YAW_LIMIT); // gently!
			}
			else
			#endif // TURN_TO_HOME
				NavYCorr = 0;
		
			SumNavRCorr = Limit (SumNavRCorr + Range, -NavIntLimit256, NavIntLimit256);
			DesiredRoll += NavRCorr + (SumNavRCorr * NavKi) / 256L;
			DesiredRoll = Limit(DesiredRoll , -_Neutral, _Neutral);
	
			SumNavPCorr = Limit (SumNavPCorr + Range, -NavIntLimit256, NavIntLimit256);
			DesiredPitch += NavPCorr + (SumNavPCorr * NavKi) / 256L;
			DesiredPitch = Limit(DesiredPitch , -_Neutral, _Neutral);

			DesiredYaw += NavYCorr;
		}
		else
			NavRCorr = NavPCorr = NavYCorr = 0;

		_NavComputed = true;
	}
} // Navigate

void DoNavigation(void)
{

	UpdateGPS();

	if ( _GPSValid && ( NavSensitivity > 20 ))
		if ( _CompassValid )
			switch ( NavState ) {
			case PIC:
			case HoldingStation:
				if ( (Abs(DesiredRoll) > MAX_CONTROL_CHANGE) 
						|| (Abs(DesiredPitch) > MAX_CONTROL_CHANGE) )
				{
					NavState = PIC;
					_Proximity = false;
					GPSNorthHold = GPSNorth;
					GPSEastHold = GPSEast;
					SumNavRCorr = SumNavPCorr = SumNavYCorr = 0;
					_NavComputed = false;
				}
				else
				{	
					NavState = HoldingStation;
					Navigate(GPSNorthHold, GPSEastHold);
				}
				
				if ( _ReturnHome )
				{
					AltSum = 0; 
					NavState = ReturningHome;
				}
				break;
			case ReturningHome:
				GPSAltitudeHold(NavRTHAlt * 10L);
				Navigate(0, 0);
				if ( !_ReturnHome )
				{
					GPSNorthHold = GPSNorth;
					GPSEastHold = GPSEast;
					SumNavRCorr = SumNavPCorr = SumNavYCorr = 0;
					_NavComputed = false;
					NavState = PIC;
				} 
				break;
			case Navigating:
				// not implemented yet
				break;
			} // switch NavState
		else
		{
			DesiredRoll = DesiredPitch = DesiredYaw = 0;
			GPSAltitudeHold(-50); // Compass not responding - land
		}


} // DoNavigation

void DoFailsafe(void)
{ // only relevant to PPM Rx

	ALL_LEDS_OFF;
	DesiredRoll = DesiredPitch = DesiredYaw = 0;
	if ( _Failsafe )
	{
		_LostModel = true;		
		DesiredThrottle = 0;
	}
	else
		if( --DropoutCycles == 0 )// timeout - immediate landing
			_Failsafe = true;
		else
			DesiredThrottle = Limit(DesiredThrottle, DesiredThrottle, THR_HOVER);					
} // DoFailsafe


void InitNavigation(void)
{
	GPSNorthHold = GPSEastHold = 0;
	NavRCorr = SumNavRCorr = NavPCorr = SumNavPCorr = NavYCorr = SumNavYCorr = 0;
	NavState = PIC;
	_NavComputed = false;
} // InitNavigation

