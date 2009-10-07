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
int16 NavRCorr, NavRCorrP, SumNavRCorr, NavPCorr, NavPCorrP, SumNavPCorr, NavYCorr, SumNavYCorr;
int16 NavKp, NavVel, Range, RangeToNeutral;  
int16 RWeight, PWeight, PSign, RSign, RollP, RollI, RollD, PitchP, PitchI, PitchD; 

WayPoint WP[NAV_MAX_WAYPOINTS];

void AltitudeHold(int16 DesiredAltitude) // Decimetres
{
	static int16 Temp;

	if ( _RTHAltitudeHold )
		if ( (P[ConfigBits] & UseGPSAltMask) || !_BaroAltitudeValid )
		{
			AE = DesiredAltitude - GPSRelAltitude;
			AE = Limit(AE, -GPS_ALT_BAND_DM, GPS_ALT_BAND_DM);
			AltSum += AE;
			AltSum = Limit(AltSum, -10, 10);	
			Temp = SRS16(AE*(int16)P[GPSAltKp] + AltSum*(int16)P[GPSAltKi], 7);
		
			DesiredThrottle = HoverThrottle + Limit(Temp, GPS_ALT_LOW_THR_COMP, GPS_ALT_HIGH_THR_COMP);
			DesiredThrottle = Limit(DesiredThrottle, 0, OUT_MAXIMUM);
		}
		else
		{
			DesiredThrottle = HoverThrottle;
			Temp = -SRS32( (int32)DesiredAltitude * BARO_SCALE, 8);
			BaroPressureHold(Temp);	
		}
	else
	{
		// manual control of altitude
	}
} // AltitudeHold

void Descend(void)
{ // uses Baro only
	static int16 DesiredRelBaroPressure;

	if (  mS[Clock] > mS[AltHoldUpdate] )
		if ( InTheAir ) 							//  micro switch RC0 Pin 11 to ground when landed
		{
			DesiredThrottle = HoverThrottle;

			if ( CurrentRelBaroPressure < -( BARO_DESCENT_TRANS_DM * BARO_SCALE)/256L )
				DesiredRelBaroPressure = CurrentRelBaroPressure + (BARO_MAX_DESCENT_DMPS * BARO_SCALE)/256L;
			else
				DesiredRelBaroPressure = CurrentRelBaroPressure + (BARO_FINAL_DESCENT_DMPS * BARO_SCALE)/256L; 

			BaroPressureHold( DesiredRelBaroPressure );	
			mS[AltHoldUpdate] += 1000L;
		}
		else
			DesiredThrottle = 0;

} // Descend

void AcquireHoldPosition(void)
{
	SumNavRCorr = SumNavPCorr = SumNavYCorr = 0;
	RangeP = MAXINT16;
	_NavComputed = false;

	GPSNorthHold = GPSNorth;
	GPSEastHold = GPSEast;
	_Proximity = _CloseProximity = false;

	NavState = HoldingStation;
} // AcquireHoldPosition

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer arithmetic

	#define OLD_NAVKI	2

	static int16 Temp, Correction, SignedRange;
	static int16 EastDiff, NorthDiff, WayHeading, RelHeading, DiffHeading;

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
			NavKp = ( NavSensitivity * NAV_MAX_ROLL_PITCH ) / NavCloseToNeutralRadius;
			WayHeading = int16atan2(EastDiff, NorthDiff);
			DiffHeading = WayHeading - Heading;
			RelHeading = Make2Pi(DiffHeading);

 		#ifdef SIMPLE_POS_HOLD			// Minimal reference proportional control only 

			_CloseProximity = false;
			Range = Max(Abs(NorthDiff), Abs(EastDiff));
			_Proximity = Range < NavClosingRadius;

			if ( _Proximity )
				Range = int32sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff ); 
			else
				Range = NavClosingRadius;
			RangeToNeutral = Range - NavNeutralRadius;

			NavVel = 0; // for FakeFlight
		
			Temp = SRS16(RangeToNeutral * NavKp, 8);
			Temp = Limit(Temp, 0, NAV_MAX_ROLL_PITCH );

			NavRCorr = SRS32((int32)int16sin(RelHeading) * Temp, 8);			
			DesiredRoll += NavRCorr;
			DesiredRoll = Limit(DesiredRoll , -RC_NEUTRAL, RC_NEUTRAL);
	
			NavPCorr = SRS32(-(int32)int16cos(RelHeading) * Temp, 8);
			DesiredPitch += NavPCorr;
			DesiredPitch = Limit(DesiredPitch , -RC_NEUTRAL, RC_NEUTRAL);

		#else // !SIMPLE_POS_HOLD

			// Expectation is ~4Hz

			Range = int32sqrt( (int32)NorthDiff*(int32)NorthDiff + (int32)EastDiff*(int32)EastDiff );

			if ( RangeP == MAXINT16 )
				RangeP = Range;
			NavVel = RangeP - Range;
			if ( Abs(NavVel) < 5 ) 
				NavVel = 0;
			RangeP = Range;
		
			_Proximity = Range < NavClosingRadius; 
			_CloseProximity = false;

			if ( !_Proximity )
				Range = NavClosingRadius;

			RangeToNeutral = Range - NavNeutralRadius;

			// Roll
			RWeight = int16sin(RelHeading);
			RSign = Sign(RWeight);
			SignedRange = RangeToNeutral * RSign;

			RollP = SRS16(SignedRange * NavKp, 8);

			Temp = SumNavRCorr + RSign * 2;
			SumNavRCorr = Limit (Temp, -NAV_INT_LIMIT, NAV_INT_LIMIT);
			RollI = SRS16(SumNavRCorr * (int16)P[NavKi], 4);
			SumNavRCorr = DecayX(SumNavRCorr, 1);

			RollD = SRS16(NavVel * P[NavKd] * RSign, 6);
			RollD = Limit(RollD, -NAV_DIFF_LIMIT, NAV_DIFF_LIMIT);

			NavRCorr = RollP + RollD;

			NavRCorr = SRS16(Abs(RWeight) * NavRCorr, 8) + RollI;
			NavRCorr = Limit(NavRCorr, -NAV_MAX_ROLL_PITCH, NAV_MAX_ROLL_PITCH );

			NavRCorr = SlewLimit(NavRCorrP, NavRCorr, NAV_CORR_SLEW_LIMIT);
			NavRCorrP = NavRCorr;

			Temp = DesiredRoll + NavRCorr;
			DesiredRoll = Limit(Temp , -RC_MAX_ROLL_PITCH, RC_MAX_ROLL_PITCH);

			// Pitch
			PWeight = -(int32)int16cos(RelHeading);
			PSign = Sign(PWeight);	
			SignedRange = RangeToNeutral * PSign;

			PitchP = SRS16(SignedRange * NavKp, 8);

			Temp = SumNavPCorr + PSign * 2;
			SumNavPCorr = Limit (Temp, -NAV_INT_LIMIT, NAV_INT_LIMIT);
			PitchI = SRS16(SumNavPCorr * (int16)P[NavKi], 4);
			SumNavPCorr = DecayX(SumNavPCorr, 1);

			PitchD = SRS16(NavVel * P[NavKd] * PSign, 6);
			PitchD = Limit(PitchD, -NAV_DIFF_LIMIT, NAV_DIFF_LIMIT);

			NavPCorr = PitchP + PitchD;

			NavPCorr = SRS16(Abs(PWeight) * NavPCorr, 8) + PitchI;
			NavPCorr = Limit(NavPCorr, -NAV_MAX_ROLL_PITCH, NAV_MAX_ROLL_PITCH );

			NavPCorr = SlewLimit(NavPCorrP, NavPCorr, NAV_CORR_SLEW_LIMIT);
			NavPCorrP = NavPCorr;

			Temp = DesiredPitch + NavPCorr;
			DesiredPitch = Limit(Temp, -RC_MAX_ROLL_PITCH, RC_MAX_ROLL_PITCH);

		#endif // SIMPLE_POS_HOLD

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
			NavPCorr = NavPCorrP = NavRCorr = NavRCorrP = SumNavRCorr = SumNavPCorr = NavYCorr =  0;
			_CloseProximity = true;
		}

		_NavComputed = true;
	}
} // Navigate

void FakeFlight()
{
	#ifdef FAKE_FLIGHT

	static int16 CosH, SinH, A;

	#define FAKE_NORTH_WIND 	0
	#define FAKE_EAST_WIND 		0
    #define SCALEVEL			256L

	if ( Armed )
	{
		Heading = 0;

		RangeP = MAXINT16;	
		GPSEast = 200L; GPSNorth = 200L;
		InitNavigation();
	
		TxString("\r\n Sens Kp East North DRoll DPitch Range Vel ");
		TxString("RP RI RD RSign RWt | PP PI PD PSign PWt\r\n");
	
		while ( Armed )
		{
			UpdateControls();

			DesiredRoll = DesiredPitch = 0;

			_NavComputed = false;
			Navigate(0,0);

			Delay100mSWithOutput(5);
			CosH = int16cos(Heading);
			SinH = int16sin(Heading);
			GPSEast += ((int32)(-DesiredPitch) * SinH)/SCALEVEL;
			GPSNorth += ((int32)(-DesiredPitch) * CosH)/SCALEVEL;
			
			A = Make2Pi(Heading + HALFMILLIPI);
			CosH = int16cos(A);
			SinH = int16sin(A);
			GPSEast += ((int32)DesiredRoll * SinH) / SCALEVEL;
			GPSEast += FAKE_EAST_WIND; // wind	
			GPSNorth += ((int32)DesiredRoll * CosH) / SCALEVEL;
			GPSNorth += FAKE_NORTH_WIND; // wind
		
			TxVal32((int32)NavSensitivity,0,' ');TxVal32((int32)NavKp,0,' ');
			TxVal32(GPSEast, 0, ' '); TxVal32(GPSNorth, 0, ' '); 
			TxVal32((int32)DesiredRoll, 0, ' '); TxVal32((int32)DesiredPitch, 0, ' ');
			TxVal32((int32)Range, 0, ' '); 
			TxVal32((int32)NavVel, 0, ' ');
			TxVal32((int32)RollP, 0, ' '); TxVal32((int32)RollI, 0, ' '); TxVal32((int32)RollD, 0, ' '); 
			TxVal32((int32)RSign, 0, ' '); TxVal32((int32)RWeight, 0, ' ');
			TxString("| ");
			TxVal32((int32)PitchP, 0, ' '); TxVal32((int32)PitchI, 0, ' '); TxVal32((int32)PitchD, 0, ' '); 
			TxVal32((int32)PSign, 0, ' '); TxVal32((int32)PWeight, 0, ' ');
			TxNextLine();
		}
	}
	#endif // FAKE_FLIGHT
} // FakeFlight

void DoNavigation(void)
{
	if ( _GPSValid && _CompassValid  && ( NavSensitivity > NAV_GAIN_THRESHOLD ) && ( mS[Clock] > mS[NavActiveTime]) )
	{
		switch ( NavState ) {
		case HoldingStation:
			if ( _AttitudeHold )
			{
				#ifdef NAV_ACQUIRE_BEEPER
				if ( _HoldBeeperArmed && !_BeeperInUse )
				{
					mS[BeeperTimeout] = mS[Clock] + 500;
					Beeper_ON;
					_HoldBeeperArmed = false;
				} 
				#endif // NAV_ACQUIRE_BEEPER
			}
			else
			{
				_HoldBeeperArmed = true;
				AcquireHoldPosition();
			}
			
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
	}
	else // no Navigation
	{
		Beeper_OFF;
		CheckForHover();
	}
} // DoNavigation

void DoFailsafe(void)
{ // only relevant to PPM Rx or Quad NOT synchronising with Rx
	if ( State == InFlight )
		switch ( FailState ) {
		case Terminated:
			DesiredRoll = DesiredPitch = DesiredYaw = 0;
			if ( CurrentRelBaroPressure < -(BARO_FAILSAFE_MIN_ALT_DM * BARO_SCALE)/256L )
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
		WP[w].A = (int16)P[NavRTHAlt]*10L; // Decimetres
	}

	GPSNorthHold = GPSEastHold = 0;
	NavPCorr = NavPCorrP = NavRCorr = NavRCorrP = SumNavRCorr = SumNavPCorr = NavYCorr = 0;

	RollP, RollI = RollD = PitchP = PitchI = PitchD = 0;
	RangeP = MAXINT16;

	NavState = HoldingStation;
	AttitudeHoldResetCount = 0;
	_Proximity = _CloseProximity = true;
	_NavComputed = false;
} // InitNavigation

