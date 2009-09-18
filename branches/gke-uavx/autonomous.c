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
void NavGainSchedule(int16);
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
			AE = DesiredAltitude - GPSRelAltitude;
			AE = Limit(AE, -GPS_ALT_BAND_DM, GPS_ALT_BAND_DM);
			AltSum += AE;
			AltSum = Limit(AltSum, -10, 10);	
			Temp = SRS16(AE*P[GPSAltKp] + AltSum*P[GPSAltKi], 7);
		
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
	_Proximity = _CloseProximity = true;

	NavState = HoldingStation;
} // AcquireHoldPosition

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer arithmetic

	static int16 Temp, Correction, RWeight, PWeight, RangeS, RSign, PSign;
	static int16 Range, RangeToNeutral, EastDiff, NorthDiff, WayHeading, RelHeading, DiffHeading;

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
			_CloseProximity = false;
			_Proximity = Range < NavClosingRadius;
			if ( _Proximity )
				RangeToNeutral = int16sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff) - NavNeutralRadius; 
			else
				RangeToNeutral = NavClosingRadius - NavNeutralRadius;

			NavKp = ( NavSensitivity * NAV_APPROACH_ANGLE ) / RangeToNeutral;
		
			WayHeading = int16atan2(EastDiff, NorthDiff);
			DiffHeading = WayHeading - Heading;
			RelHeading = Make2Pi(DiffHeading);

			RWeight = int16sin(RelHeading);
			PWeight = -(int32)int16cos(RelHeading);

		#ifdef NEW_HOLD

			// Roll
			RSign = Sign(RWeight);
			RWeight = Abs(RWeight);

			RangeS = RangeToNeutral * RSign;

			NavRCorr = RangeS * NavKp;

			Temp = SumNavRCorr + RangeS;
			Temp = SRS16(Temp * P[NavKi], 2);
			SumNavRCorr = Limit (Temp, -NAV_INT_LIMIT*256L, NAV_INT_LIMIT*256L);
			NavRCorr += SumNavPCorr;

		//	NavRCorr += GPSVel * P[NavKd];

			NavRCorr = Limit(NavRCorr, 1, NAV_APPROACH_ANGLE );
			NavRCorr = SRS16(RWeight * NavRCorr, 8);
			Temp = DesiredRoll + NavRCorr;
			DesiredRoll = Limit(Temp , -RC_NEUTRAL, RC_NEUTRAL);

			// Pitch
			PSign = Sign(PWeight);			
			PWeight = Abs(PWeight);

			RangeS = RangeToNeutral * PSign;

			NavPCorr = RangeS * NavKp;

			Temp = SumNavPCorr + RangeS;
			Temp = SRS16(Temp * P[NavKi], 2);
			SumNavPCorr = Limit (Temp, -NAV_INT_LIMIT*256L, NAV_INT_LIMIT*256L);
			NavPCorr += SumNavPCorr;

		//	NavPCorr += GPSVel * P[NavKd];

			NavPCorr = Limit(NavPCorr, 1, NAV_APPROACH_ANGLE );
			NavPCorr = SRS16(PWeight * NavPCorr, 8);
			Temp = DesiredRoll + NavPCorr;
			DesiredRoll = Limit(Temp, -RC_NEUTRAL, RC_NEUTRAL);

		#else

			Correction = SRS16(RangeToNeutral * NavKp, 8);
			Correction = Limit(Correction, 1, NAV_APPROACH_ANGLE );

			// Roll
			NavRCorr = SRS16(RWeight * Correction, 8);
			DesiredRoll += NavRCorr;
			Temp = SumNavRCorr + Range;
			SumNavRCorr = Limit (Temp, -NavIntLimit32, NavIntLimit32);
			DesiredRoll += (SumNavRCorr * 1) / 256L;
			DesiredRoll = Limit(DesiredRoll , -RC_NEUTRAL, RC_NEUTRAL);

			// Pitch
			NavPCorr = SRS16(PWeight * Correction, 8);
			DesiredPitch += NavPCorr;
			Temp = SumNavPCorr + Range;
			SumNavPCorr = Limit (Temp, -NavIntLimit32, NavIntLimit32);
			DesiredPitch += (SumNavPCorr * 1) / 256L;
			DesiredPitch = Limit(DesiredPitch , -RC_NEUTRAL, RC_NEUTRAL);

		#endif // EXP_HOLD

			// Yaw
			if ( _TurnToHome && !_Proximity )
			{
				RelHeading = MakePi(DiffHeading); // make +/- MilliPi
				NavYCorr = -(RelHeading * NAV_YAW_LIMIT) / HALFMILLIPI;
				NavYCorr = Limit(NavYCorr, -NAV_YAW_LIMIT, NAV_YAW_LIMIT); // gently!
			}
			else
				NavYCorr = 0;	
			DesiredYaw += NavYCorr;
		}
		else
		{
			// Neutral Zone - no GPS influence
			NavPCorr = NavRCorr = NavPCorr = SumNavRCorr = SumNavPCorr = NavYCorr = 0;
			_CloseProximity = true;
		}

		_NavComputed = true;
	}
} // Navigate

void DoNavigation(void)
{
	if ( _GPSValid && _CompassValid  && ( NavSensitivity > 0 ) && ( mS[Clock] > mS[NavActiveTime]) )
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
	_Proximity = _CloseProximity = true;
	_NavComputed = false;
} // InitNavigation

