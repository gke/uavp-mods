// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.

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
void CheckThrottleMoved(void);
void DoFailsafe(void);

// Variables
extern int16 ValidGPSSentences;
extern boolean GPSSentenceReceived;

int16 NavRCorr, SumNavRCorr, NavPCorr, SumNavPCorr, NavYCorr, SumNavYCorr;

void AltitudeHold(int16 DesiredAltitude)
{
	int16 Temp;

	if ( _RTHAltitudeHold )
		if ( UseGPSAlt && _GPSAltitudeValid )
		{
			AE = Limit(DesiredAltitude - GPSRelAltitude, -50, 50); // 5 metre band
			AltSum += AE;
			AltSum = Limit(AltSum, -10, 10);	
			Temp = SRS16(AE*NavAltKp + AltSum*NavAltKi, 5);
		
			DesiredThrottle = HoverThrottle + Limit(Temp, -10, 30);
			DesiredThrottle = Limit(DesiredThrottle, 0, OUT_MAXIMUM);
		}
		else
		{	
			DesiredThrottle = HoverThrottle;
			BaroAltitudeHold(-DesiredAltitude);
		}	
	else
	{
		// manual control of altitude
	}
} // AltitudeHold

void Descend(void)
{	
} // Descend

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer arithmetic

	#define NavKi 1

	static int32 Temp;
	static int16 NavKp, GPSGain;
	static int16 Range, EastDiff, NorthDiff, WayHeading, RelHeading;

	if ( _NavComputed ) // maintain previous corrections
	{
		DesiredRoll = Limit(DesiredRoll + NavRCorr, -RC_NEUTRAL, RC_NEUTRAL);
		DesiredPitch = Limit(DesiredPitch + NavPCorr, -RC_NEUTRAL, RC_NEUTRAL);
		DesiredYaw = Limit(DesiredYaw + NavYCorr, -RC_NEUTRAL, RC_NEUTRAL);
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

			GPSGain = Limit(NavSensitivity, 0, 256);
			NavKp = ( GPSGain * MAX_ANGLE ) / NavClosingRadius; // /OUT_MAXIMUM) * 256L
		
			Temp = ((int32)Range * NavKp )>>8; // allways +ve so can use >>
			WayHeading = int16atan2(EastDiff, NorthDiff);

			RelHeading = Make2Pi(WayHeading - Heading);
			NavRCorr = SRS32((int32)int16sin(RelHeading) * Temp, 8);			
			NavPCorr = SRS32(-(int32)int16cos(RelHeading) * Temp, 8);

			if ( _TurnToHome && (Range >= NavClosingRadius) )
			{
				RelHeading = MakePi(WayHeading - Heading); // make +/- MilliPi
				NavYCorr = -(RelHeading * NAV_YAW_LIMIT) / HALFMILLIPI;
				NavYCorr = Limit(NavYCorr, -NAV_YAW_LIMIT, NAV_YAW_LIMIT); // gently!
			}
			else
				NavYCorr = 0;
		
			SumNavRCorr = Limit (SumNavRCorr + Range, -NavIntLimit256, NavIntLimit256);
			DesiredRoll += NavRCorr + (SumNavRCorr * NavKi) / 256L;
			DesiredRoll = Limit(DesiredRoll , -RC_NEUTRAL, RC_NEUTRAL);
	
			SumNavPCorr = Limit (SumNavPCorr + Range, -NavIntLimit256, NavIntLimit256);
			DesiredPitch += NavPCorr + (SumNavPCorr * NavKi) / 256L;
			DesiredPitch = Limit(DesiredPitch , -RC_NEUTRAL, RC_NEUTRAL);

			DesiredYaw += NavYCorr;
		}
		else
			NavRCorr = NavPCorr = NavYCorr = 0;

		_NavComputed = true;
	}
} // Navigate

void DoNavigation(void)
{
	if ( _GPSValid && ( NavSensitivity > NAV_GAIN_THRESHOLD ))
	{
		if ( _CompassValid )
			switch ( NavState ) {
			case PIC:
			case HoldingStation:
				if ( (Abs(DesiredRoll - RollTrim) > MAX_CONTROL_CHANGE) 
						|| (Abs(DesiredPitch - PitchTrim) > MAX_CONTROL_CHANGE) )
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
					#ifdef EMIT_TONE
					TxChar('H'); // why not H? as the frequency is determined control cycle time
					#endif // EMIT_TONE
					NavState = HoldingStation;
					Navigate(GPSNorthHold, GPSEastHold);
				}
				
				if ( _ReturnHome )
				{
					AltSum = 0; 
					NavState = ReturningHome;
				}
				CheckForHover();
				break;
			case ReturningHome:
				AltitudeHold(NavRTHAlt * 10L);
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
			AltitudeHold(-50); // Compass not responding - land
		}
	} 
	else // else no GPS
		CheckForHover();
} // DoNavigation

void DoFailsafe(void)
{ // only relevant to PPM Rx or Quad NOT synchronising with Rx

	ALL_LEDS_OFF;
	if ( _Failsafe )
	{
		LEDRed_ON;
		_LostModel = true;
		DesiredRoll = DesiredPitch = DesiredYaw = DesiredThrottle = 0;		
	}
	else
		if( mS[Clock] > mS[AbortTimeout] ) // timeout - immediate shutdown/abort
			_Failsafe = true;
		else
			if ( mS[Clock] > mS[FailsafeTimeout] ) 
			{
				DesiredRoll = DesiredPitch = DesiredYaw = 0;
				// use last "good" throttle
				// DesiredThrottle = Limit(DesiredThrottle, 0, HoverThrottle*3/4); 
			}
			else
			{
				// continue on last "good" signals
			}
					
} // DoFailsafe

void CheckThrottleMoved(void)
{
	if( mS[Clock] < mS[ThrottleUpdate] )
		ThrNeutral = DesiredThrottle;
	else
	{
		ThrLow = ThrNeutral - THR_MIDDLE;
		ThrLow = Max(ThrLow, THR_HOVER);
		ThrHigh = ThrNeutral + THR_MIDDLE;
		if ( ( DesiredThrottle <= ThrLow ) || ( DesiredThrottle >= ThrHigh ) )
		{
			mS[ThrottleUpdate] = mS[Clock] + THROTTLE_UPDATE;
			_ThrottleMoving = true;
		}
		else
			_ThrottleMoving = false;
	}
} // CheckThrottleMoved


void InitNavigation(void)
{
	GPSNorthHold = GPSEastHold = 0;
	NavRCorr = SumNavRCorr = NavPCorr = SumNavPCorr = NavYCorr = SumNavYCorr = 0;
	NavState = PIC;
	_NavComputed = false;
} // InitNavigation

