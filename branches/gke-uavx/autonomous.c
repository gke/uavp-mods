// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =           http://code.google.com/p/uavp-mods/ http://uavp.ch        =
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

void Navigate(int16, int16);
void AltitudeHold(int16);
void Descend(void);
void AcquireHoldPosition(void);
void DoNavigation(void);
void CheckThrottleMoved(void);
void DoFailsafe(void);
void InitNavigation(void);

// Variables
int16 NavRCorr, SumNavRCorr, NavPCorr, SumNavPCorr, NavYCorr, SumNavYCorr;

WayPoint WP[NAV_MAX_WAYPOINTS];

void AltitudeHold(int16 DesiredAltitude) // Decimetres
{
	static int16 Temp;

	if ( _RTHAltitudeHold )
		if ( P[ConfigBits] & UseGPSAltMask )
		{
			AE = Limit(DesiredAltitude - GPSRelAltitude, -GPS_ALT_BAND_DM, GPS_ALT_BAND_DM);
			AltSum += AE;
			AltSum = Limit(AltSum, -10, 10);	
			Temp = SRS16(AE*P[GPSAltKp] + AltSum*P[GPSAltKi], 5);
		
			DesiredThrottle = HoverThrottle + Limit(Temp, GPS_ALT_LOW_THR_COMP, GPS_ALT_HIGH_THR_COMP);
			DesiredThrottle = Limit(DesiredThrottle, 0, OUT_MAXIMUM);
		}
		else
		{	
			DesiredThrottle = HoverThrottle;
			BaroPressureHold(-SRS32( (int32)DesiredAltitude * BARO_SCALE, 8) );
		}
	else
	{
		// manual control of altitude
	}
} // AltitudeHold

void Descend(void)
{ // uses Baro only
	static int16 DesiredBaroPressure;

	if (  mS[Clock] > mS[AltHoldUpdate] )
		if ( InTheAir ) 							//  micro switch RC0 Pin 11 to ground when landed
		{
			DesiredThrottle = HoverThrottle;

			if ( CurrentBaroPressure < -( BARO_DESCENT_TRANS_DM * BARO_SCALE)/256L )
				DesiredBaroPressure = CurrentBaroPressure + (BARO_MAX_DESCENT_DMPS * BARO_SCALE)/256L;
			else
				DesiredBaroPressure = CurrentBaroPressure + (BARO_FINAL_DESCENT_DMPS * BARO_SCALE)/256L; 

			BaroPressureHold( DesiredBaroPressure );	
			mS[AltHoldUpdate] += 1000L;
		}
		else
			DesiredThrottle = 0;

} // Descend

void AcquireHoldPosition(void)
{
	SumNavRCorr = SumNavPCorr = SumNavYCorr = 0;
	_NavComputed = false;

	GPSNorthHold = GPSNorth;
	GPSEastHold = GPSEast;
	_Proximity = true;

	NavState = HoldingStation;
} // AcquireHoldPosition

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer arithmetic

	#define NavKi 1

	static int16 Temp;
	static int16 NavKp, GPSGain;
	static int16 Range, EastDiff, NorthDiff, WayHeading, RelHeading;

	if ( _NavComputed ) // maintain previous corrections
	{
		Temp = DesiredRoll + NavRCorr;
		DesiredRoll = Limit(Temp, -RC_NEUTRAL, RC_NEUTRAL);
		Temp = DesiredPitch + NavPCorr;
		DesiredPitch = Limit(Temp, -RC_NEUTRAL, RC_NEUTRAL);
		Temp = DesiredYaw + NavYCorr;
		DesiredYaw = Limit(Temp, -RC_NEUTRAL, RC_NEUTRAL);
	}
	else
	{
		EastDiff = GPSEastWay - GPSEast;
		NorthDiff = GPSNorthWay - GPSNorth;

		if ( (Abs(EastDiff) > NavNeutralRadius ) || (Abs(NorthDiff) > NavNeutralRadius )) 
		{ 
			Range = Max(Abs(NorthDiff), Abs(EastDiff)); 
			_Proximity = Range < NavClosingRadius;
			if ( _Proximity )
				Range = int16sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff); 
			else
				Range = NavClosingRadius;

			GPSGain = Limit(NavSensitivity, 0, 256);
			NavKp = ( GPSGain * NAV_MAX_ANGLE ) / ( NavClosingRadius - NavNeutralRadius ); // /OUT_MAXIMUM) * 256L
		
			Temp = SRS16((Range - NavNeutralRadius) * NavKp, 8);
			Temp = Limit(Temp, 0, NAV_MAX_ANGLE );
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
	
			DesiredRoll += NavRCorr;
			SumNavRCorr = Limit (SumNavRCorr + Range, -NavIntLimit256, NavIntLimit256);
			#ifdef NAV_ZERO_INT
			if ( Sign(SumNavRCorr) == Sign(NavRCorr) )
				DesiredRoll += (SumNavRCorr * NavKi) / 256L;
			else
				SumNavRCorr = 0;
			#else
			DesiredRoll += (SumNavRCorr * NavKi) / 256L;
			#endif //NAV_ZERO_INT
			DesiredRoll = Limit(DesiredRoll , -RC_NEUTRAL, RC_NEUTRAL);
	
			DesiredPitch += NavPCorr;
			SumNavPCorr = Limit (SumNavPCorr + Range, -NavIntLimit256, NavIntLimit256);
			#ifdef NAV_ZERO_INT
			if ( Sign(SumNavPCorr) == Sign(NavPCorr) )
				DesiredPitch += (SumNavPCorr * NavKi) / 256L;
			else
				SumNavPCorr = 0;
			#else
			DesiredPitch += (SumNavPCorr * NavKi) / 256L;
			#endif //NAV_ZERO_INT
			DesiredPitch = Limit(DesiredPitch , -RC_NEUTRAL, RC_NEUTRAL);

			DesiredYaw += NavYCorr;
		}
		else
		{
			// bump compensation with Acc here
			NavRCorr = SumNavRCorr = NavPCorr = SumNavPCorr = NavYCorr = 0;
		}

		_NavComputed = true;
	}
} // Navigate

void DoNavigation(void)
{
	if ( _GPSValid && _CompassValid  && ( NavSensitivity > NAV_GAIN_THRESHOLD ) && ( mS[Clock] > mS[NavActiveTime]) )
		switch ( NavState ) {
		case PIC:
		case HoldingStation:

			if ( !_AttitudeHold )
				AcquireHoldPosition();
			
			NavState = HoldingStation;			// Keep GPS hold active regardless
			Navigate(GPSNorthHold, GPSEastHold);
	
			if ( _ReturnHome )
			{
				AltSum = 0; 
				NavState = ReturningHome;
			}

			CheckForHover();

			break;
		case ReturningHome:
			Navigate(WP[0].N, WP[0].E);
			AltitudeHold(WP[0].A);
			if ( _ReturnHome )
			{
				if ( _Proximity )
				{
					mS[RTHTimeout] = mS[Clock] + NAV_RTH_TIMEOUT_MS;					
					NavState = AtHome;
				}
			}
			else
				AcquireHoldPosition();					
			break;
		case AtHome:
			Navigate(WP[0].N, WP[0].E);
			if ( _ReturnHome )
				if ( (( P[ConfigBits] & UseRTHDescendMask ) != 0 ) && ( mS[Clock] > mS[RTHTimeout] ) )
				{
					mS[AltHoldUpdate] = mS[Clock];
					NavState = Descending;
				}
				else
					AltitudeHold(WP[0].A);
			else
				AcquireHoldPosition();
			break;
		case Descending:
			Navigate(WP[0].N, WP[0].E);
			if ( _ReturnHome )
				Descend();
			else
				AcquireHoldPosition();
			break;
		case Navigating:
			// not implemented yet
			break;
		} // switch NavState
	else // no Navigation
		CheckForHover();
} // DoNavigation

void DoFailsafe(void)
{ // only relevant to PPM Rx or Quad NOT synchronising with Rx
	if ( State == InFlight )
		switch ( FailState ) {
		case Terminated:
			DesiredRoll = DesiredPitch = DesiredYaw = 0;
			if ( CurrentBaroPressure < -(BARO_FAILSAFE_MIN_ALT_DM * BARO_SCALE)/256L )
			{
				Descend();							// progressively increase desired baro pressure
				if ( mS[Clock ] > mS[AbortTimeout] )
					if ( _Signal )
					{
						LEDRed_OFF;
						LEDGreen_ON;
						FailState = Waiting;
					}
					else
						mS[AbortTimeout] += ABORT_UPDATE_MS;
			}
			else
			{ // shutdown motors to avoid prop injury
				DesiredThrottle = 0;
				StopMotors();
			}
			break;
		case Returning:
			DesiredRoll = DesiredPitch = DesiredYaw = DesiredThrottle = 0;
			NavSensitivity = RC_NEUTRAL;		// 50% gain
			_ReturnHome = _TurnToHome = true;
			NavState = ReturningHome;
			DoNavigation();						// if GPS is out then check for hover will see zero throttle
			if ( mS[Clock ] > mS[AbortTimeout] )
				if ( _Signal )
				{
					LEDRed_OFF;
					LEDGreen_ON;
					FailState = Waiting;
				}
				else
					mS[AbortTimeout] += ABORT_UPDATE_MS;
			break;
		case Aborting:
			if( mS[Clock] > mS[AbortTimeout] )
			{
				_LostModel = true;
				LEDGreen_OFF;
				LEDRed_ON;

				mS[AltHoldUpdate] = mS[Clock];
				AltitudeHold(WP[0].A);
				mS[AbortTimeout] += ABORT_TIMEOUT_MS;

				#ifdef NAV_PPM_FAILSAFE_RTH
				FailState = Returning;
				#else
				FailState = Terminated;
				#endif // PPM_FAILSAFE_RTH
			}
			break;
		case Waiting:
			if ( mS[Clock] > mS[FailsafeTimeout] ) 
			{
				LEDRed_ON;
				mS[AbortTimeout] = mS[Clock] + ABORT_TIMEOUT_MS;
				DesiredRoll = DesiredPitch = DesiredYaw = 0;
				FailState = Aborting;
				// use last "good" throttle; 
			}
			break;
		} // Switch FailState
	else
		DesiredRoll = DesiredPitch = DesiredYaw = DesiredThrottle = 0;
			
} // DoFailsafe

void CheckThrottleMoved(void)
{
	if( mS[Clock] < mS[ThrottleUpdate] )
		ThrNeutral = DesiredThrottle;
	else
	{
		ThrLow = ThrNeutral - THROTTLE_MIDDLE;
		ThrLow = Max(ThrLow, THROTTLE_HOVER);
		ThrHigh = ThrNeutral + THROTTLE_MIDDLE;
		if ( ( DesiredThrottle <= ThrLow ) || ( DesiredThrottle >= ThrHigh ) )
		{
			mS[ThrottleUpdate] = mS[Clock] + THROTTLE_UPDATE_MS;
			_ThrottleMoving = true;
		}
		else
			_ThrottleMoving = false;
	}
} // CheckThrottleMoved

void InitNavigation(void)
{
	static uint8 w;

	for (w = 0; w < NAV_MAX_WAYPOINTS; w++)
	{
		WP[w].N = WP[w].E = 0; 
		WP[w].A = P[NavRTHAlt]*10L; // Decimetres
	}

	GPSNorthHold = GPSEastHold = 0;
	NavRCorr = SumNavRCorr = NavPCorr = SumNavPCorr = NavYCorr = SumNavYCorr = 0;
	NavState = PIC;
	AttitudeHoldResetCount = 0;
	_NavComputed = false;
} // InitNavigation

