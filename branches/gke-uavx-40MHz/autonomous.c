// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

// Autonomous flight routines

#include "uavx.h"

void DoShutdown(void);
void DecayNavCorr(void);
void DoCompScaling(void);
void Navigate(int24 DesiredNorth, int24 DesiredEast);
void AcquireHoldPosition(void);
void NavGainSchedule(int16);
void DoNavigation(void);
void DoFailsafe(void);
void UAVXNavCommand(void);
void GetWayPointEE(int8);
void InitNavigation(void);

#pragma udata nav_vars
NavStruct Nav;
#pragma udata

boolean StartingNav = true;
int16 DescentComp;
int16 RollYawMixFrac;
int16 RelHeading;
int16 EffNavSensitivity;

#pragma udata uavxnav_waypoints
typedef union { 
	uint8 b[256];
	struct {
		uint8 u0;
		int16 u1;
		int8 u3;
		int8 u4;
		uint16 u5;
		uint16 u6;
		int8 NoOfWPs;
		uint8 ProximityAltitude;
		uint8 ProximityRadius;
		int16 OriginAltitude;
		int32 OriginLatitude;
		int32 OriginLongitude;
		int16 RTHAltitude;
		struct {
			int32 Latitude;
			int32 Longitude;
			int16 Altitude;
			int8 Loiter;
			} WP[23];
		};
	} UAVXNav;
#pragma udata

#pragma udata eebuffer
uint8 BufferEE[256];
#pragma udata
		
#pragma udata navvars
uint8 	CurrWP;
int8 	NoOfWayPoints;
int24	WPAltitude;
int32 	WPLatitude, WPLongitude;
int24 	WPLoiter;
int24 	DistE;
int16	OriginalWayHeading, WayHeading;
uint24 	NavRTHTimeoutmS;
int24 	NavdT;
#pragma udata

boolean DoLanding(void) { // InTheAir micro switch RC0 Pin 11 to ground when landed
	static boolean HasLanded;

	HasLanded = false;
	F.AltHoldEnabled = F.AllowNavAltitudeHold = true;

	#ifdef NAV_WING

		DesiredThrottle = 0; // Glide in!!!
		HasLanded = Altitude < LAND_CM;

	#else
		SetDesiredAltitude(-200);
		if (F.BaroAltitudeValid) {
			DesiredThrottle = CruiseThrottle;
			if (Altitude > LAND_CM)
				mSTimer(NavStateTimeout, NAV_RTH_LAND_TIMEOUT_MS);

			//TODO: bump detection for landing
			//|| ((AccVertical < (-GRAVITY * 3) && (Altitude <= LAND_M)))
			// AltComp ALT_MAX_THR_COMP_STICK

			HasLanded = !InTheAir || ((mSClock() > mS[NavStateTimeout])
					&& (F.ForceFailsafe || (NavState == Touchdown)
							|| (FailState == Terminated)));
		} else {
			DesiredThrottle = Limit(CruiseThrottle - DescentComp, 0, OUT_MAXIMUM);
			if (mSClock() > mS[DescentUpdate]) {
				mSTimer(DescentUpdate, ALT_DESCENT_UPDATE_MS);
				HasLanded = DesiredThrottle < IdleThrottle;
				if (!HasLanded)
					DescentComp += ALT_DESCENT_STEP;
			}
		}
	#endif // NAV_WING

	return (HasLanded);
} // DoLanding

void DoShutdown(void) {
	StopMotors();
	ALL_LEDS_OFF;
	NavState = HoldingStation;
	State = Shutdown;
} // DoShutdown

void DecayNavCorr(void) {
	A[Roll].NavCorr = DecayX(A[Roll].NavCorr, 5);
	A[Pitch].NavCorr = DecayX(A[Pitch].NavCorr, 5);
	A[Yaw].NavCorr = 0;
} // DecayNavCorr

#ifndef TESTING

void FailsafeHoldPosition(void) {
	A[Roll].Desired = A[Pitch].Desired = A[Yaw].Desired = 0;
	Navigate(Nav.C[NorthC].Desired, Nav.C[EastC].Desired);
} // FailsafeHoldPosition

void DoFailsafeShutdown(void) {

	StopMotors();
	F.LostModel = true;
	State = Shutdown;
	FailState = Terminated;

	while (true) {
		CheckBatteries();
		CheckAlarms();
		GetBaroAltitude(); 
		UpdateGPS();
		CheckTelemetry();
	}
} // DoFailsafeShutdown

void AcquireHoldPosition(void) {
	F.NavComputed = false;

	Nav.C[NorthC].Desired = Nav.C[NorthC].Pos;
	Nav.C[EastC].Desired = Nav.C[EastC].Pos;

	F.CrossTrackActive = F.WayPointAchieved = F.WayPointCentred = F.AcquireNewPosition = false;

	NavState = HoldingStation;
} // AcquireHoldPosition

#endif // !TESTING

int16 CompensateCrossTrackError(int16 WayHeading) {
#ifdef USE_CROSSTRACK
	static int16 HeadingDiff, CorrectedHeading;

	if (!F.CrossTrackActive) {
		Nav.CrossTrackE = 0;
		OriginalWayHeading = WayHeading;
		F.CrossTrackActive = true;
	}

	HeadingDiff = WayHeading - OriginalWayHeading;
	if ((Nav.Distance > Nav.ProximityRadius)
			&& (Abs(MakePi(HeadingDiff))< DegreesToRadians(45))) {
		Nav.CrossTrackE = SRS32((int32)int16sin(HeadingDiff) * (Nav.Distance
				* Nav.CrossTrackKp),12);
		CorrectedHeading = WayHeading + Limit1(Nav.CrossTrackE, DegreesToRadians(30));
		return (Make2Pi(CorrectedHeading));
	} else
#endif  // USE_CROSSTRACK
	{
		Nav.CrossTrackE = 0;
		return (WayHeading);
	}
} // CompensateCrossTrackError


int16 CalculateTurnCorr(void) {
	static int16 A;

	A = MakePi(RelHeading - OrientationMRad);
	A = Limit1(A, DegreesToRadians(30));
	A = SRS32((int32)A * Nav.YawKp * EffNavSensitivity, 9);

	return(A);
} // CalculateTurnCorr

void Navigate(int24 DesiredNorth, int24 DesiredEast) {
	// F.GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer arithmetic

	#ifndef TESTING // not used for testing - make space!

	static uint32 NowmS;
	static int16 AltE, VelE, DesiredVel;
	static int16 NewCorr;
	static int24 AbsNorthE, AbsEastE;

	static uint8 a;

	NowmS = mSClock();
	NavdT = NowmS - mS[LastNavUpdate];
	mS[LastNavUpdate] = NowmS;

	EffNavSensitivity = Nav.Sensitivity - FromPercent(NAV_SENS_THRESHOLD, RC_MAXIMUM); 
	EffNavSensitivity = Limit(EffNavSensitivity, 0, 256);

	Nav.C[NorthC].Desired = DesiredNorth; // for telemetry tracking
	Nav.C[EastC].Desired = DesiredEast;

	Nav.C[NorthC].PosE = Nav.C[NorthC].Desired - Nav.C[NorthC].Pos;
	Nav.C[EastC].PosE = Nav.C[EastC].Desired - Nav.C[EastC].Pos;

	AbsNorthE = Abs(Nav.C[NorthC].PosE);
	AbsEastE = Abs(Nav.C[EastC].PosE);

	DistE = Max(AbsNorthE, AbsEastE);
	if (DistE < 2896L)
		DistE = int32sqrt(Sqr(AbsEastE) + Sqr(AbsNorthE));

	WayHeading = CompensateCrossTrackError(int32atan2(Nav.C[EastC].PosE, Nav.C[NorthC].PosE));

	#ifdef NAV_WING
		F.WayPointCentred = Nav.Distance < Nav.ProximityRadius;
	#else
		F.WayPointCentred = Nav.Distance < (GPS.HDOP * 2);
	#endif
	F.WayPointAchieved = F.WayPointCentred && (Abs(DesiredAltitude - Altitude)
			< Nav.ProximityAltitude);

	if (EffNavSensitivity > 0) {
		RelHeading = MakePi(WayHeading - Heading); // make +/- Pi

		#ifdef NAV_WING
			NewCorr = SRS16(-NewCorr * RollYawMixFrac, 8);
			if (Sign(A[Roll].NavCorr) != Sign(NewCorr))
				A[Roll].NavCorr = NewCorr;
			A[Roll].NavCorr = SlewLimit(A[Roll].NavCorr, NewCorr, Nav.RollPitchSlewRate);

			NewCorr *= - RollYawMixFrac;
			if (Sign(A[Yaw].NavCorr) != Sign(NewCorr))
				A[Yaw].NavCorr = NewCorr;
			A[Yaw].NavCorr = SlewLimit(A[Yaw].NavCorr, NewCorr,	Nav.YawSlewRate);

			A[Pitch].NavCorr = 0;
		#else
			DesiredVel = Limit1(DistE * Nav.PosKp, Nav.MaxVelocitydMpS);
			VelE = DesiredVel - GPS.Vel;

			NewCorr = SRS32((int32)VelE * Nav.VelKp * EffNavSensitivity, 9); 
			NewCorr = Limit1(NewCorr, NAV_MAX_ROLL_PITCH);
			Nav.Corr = SlewLimit(Nav.Corr, NewCorr, Nav.RollPitchSlewRate);

			A[Pitch].NavCorr = -SRS32((int32)Nav.Corr * int16cos(RelHeading), 8);
			A[Roll].NavCorr = SRS32((int32)Nav.Corr * int16sin(RelHeading), 8); 

			if (F.AllowTurnToWP && (DistE > Nav.ProximityRadius)) {
				NewCorr = -CalculateTurnCorr();
				if (Sign(A[Yaw].NavCorr) != Sign(NewCorr))
					A[Yaw].NavCorr = NewCorr;
				A[Yaw].NavCorr = SlewLimit(A[Yaw].NavCorr, NewCorr, Nav.YawSlewRate);
			} else
				A[Yaw].NavCorr = 0;
		#endif // NAV_WING

	} else
	#endif // !TESTING
		DecayNavCorr();

	F.NavComputed = true;

} // Navigate

void DoNavigation(void)
{
	#ifndef TESTING // not used for testing - make space!

	F.NavigationActive = F.GPSValid && F.NavValid && F.MagnetometerValid && F.AccelerationsValid && ( mSClock() > mS[NavActiveTime]);
	
	if ( F.NavigationActive ) {
		F.LostModel = F.ForceFailsafe = false;

		if (( !F.NavComputed ) && SpareSlotTime ) {

			#ifdef INC_POLAR
			Nav.Distance = int32sqrt(Sqr(Nav.C[EastC].Pos) + Sqr(Nav.C[NorthC].Pos));
			Nav.Bearing = int32atan2(Nav.C[EastC].Pos, Nav.C[NorthC].Pos);
			//Nav.Elevation = atan2f(Altitude, Nav.Distance);
			#endif

			switch ( NavState ) { // most frequent case last - switches in C18 are IF chains not branch tables!
			case Touchdown:
				Navigate(0, 0);
				DoFailsafeLanding();
				break;
			case Descending:
				Navigate( 0, 0 );
				if ( F.ReturnHome || F.Navigate )
				#ifdef NAV_WING
				{
					// needs more thought - runway direction?
					DoFailsafeLanding();
				}
				#else
				{
					if ( Altitude < LAND_CM )
						NavState = Touchdown;
					DoFailsafeLanding();
				}
				#endif // NAV_WING
				else
					AcquireHoldPosition();
				break;
			case AtHome:
				Navigate(0, 0);

				if ( F.ReturnHome || F.Navigate )
					if ( F.WayPointAchieved ) { // check still @ Home
						#ifdef NAV_WING
							
							// no automatic landing zzz
							
						#else
							if ( F.UsingRTHAutoDescend && ( mSClock() > mS[NavStateTimeout] )) {
								mSTimer(NavStateTimeout, NAV_RTH_LAND_TIMEOUT_MS);
								NavState = Descending;
							}
						#endif // NAV_WING
					} else {
						F.CrossTrackActive = false;
						NavState = ReturningHome;
					} else
					AcquireHoldPosition();
				break;
			case ReturningHome:		
				Navigate(0, 0);				
				if ( F.ReturnHome || F.Navigate ) {
					if ( F.WayPointAchieved ) {
						mSTimer(NavStateTimeout, NavRTHTimeoutmS);
						SetDesiredAltitude(RTHAltitude);			
						NavState = AtHome;
					}	
				} else
					AcquireHoldPosition();					
				break;
			case Loitering:
				Navigate(WPLatitude, WPLongitude);
				if ( F.Navigate ) {
					if ( F.WayPointAchieved && (mSClock() > mS[NavStateTimeout]) )
						if ( CurrWP == NoOfWayPoints ) {
							CurrWP = 1;
							SetDesiredAltitude(RTHAltitude);
							F.CrossTrackActive = false;
							NavState = ReturningHome;	
						} else {
							GetWayPointEE(++CurrWP); 
							SetDesiredAltitude(WPAltitude);	
							NavState = Navigating;
						}
				} else
					AcquireHoldPosition();
				break;
			case Navigating:
				Navigate(WPLatitude, WPLongitude);	
				if ( F.Navigate ) {
					if ( F.WayPointAchieved ) {
						mSTimer(NavStateTimeout, WPLoiter);
						SetDesiredAltitude(WPAltitude);
						NavState = Loitering;
					}		
				} else
					AcquireHoldPosition();					
				break;
			case HoldingStation:
				#ifdef NAV_WING

					// no orbit for fixed wing!!!

				#else 
					if ( F.AttitudeHold )		
						Navigate(Nav.C[NorthC].Desired, Nav.C[EastC].Desired);
					else
						AcquireHoldPosition();					
				#endif
			
				if ( F.Navigate ) {
					GetWayPointEE(CurrWP); // resume from previous WP
					SetDesiredAltitude(WPAltitude);
					NavState = Navigating;
				} else
					if ( F.ReturnHome ) {
						SetDesiredAltitude(RTHAltitude);
						F.CrossTrackActive;
						NavState = ReturningHome;
					}										
				break;
			} // switch NavState
		}
	}
	#ifdef ENABLE_STICK_CHANGE_FAILSAFE
	else 
    	if ( F.ForceFailsafe ) //&& F.NewCommands  && ( StickThrottle >= IdleThrottle ) )
			DoFailsafeLanding();
	#endif // ENABLE_STICK_CHANGE_FAILSAFE
		else 
			DecayNavCorr();

	F.NewCommands = false;	// Navigate modifies Desired Roll, Pitch and Yaw values.

	#endif // !TESTING
} // DoNavigation

void CheckFailsafeAbort(void) {

	#ifndef TESTING

	if ( mSClock() > mS[AbortTimeout] ) {
		if ( F.Signal )	{
			LEDGreen_ON;
			mS[NavStateTimeout] = 0;
			mSTimer(FailsafeTimeout, FAILSAFE_TIMEOUT_MS); // may be redundant?
			NavState = HoldingStation;
			FailState = MonitoringRx;
		}
	} else
		mS[AbortTimeout] += ABORT_UPDATE_MS;

	#endif // !TESTING
} // CheckFailsafeAbort

void DoFailsafe(void) { // only relevant to PPM Rx or Quad NOT synchronising with Rx and sticks unchanged

	#ifndef TESTING // not used for testing - make space!

		AltitudeHold();

		switch ( FailState ) { // FailStates { MonitoringRx, Aborting, Terminating, Terminated, RxTerminate }
		case Terminated: // Basic assumption is that aircraft is being flown over a safe area!
			FailsafeHoldPosition();
			DoFailsafeLanding();	
			break;
		case Terminating:
			FailsafeHoldPosition();
			if ( Altitude < LAND_CM )
				FailState = Terminated;
			DoFailsafeLanding();
			break;
		case Aborting:
			FailsafeHoldPosition();
			F.AltHoldEnabled = true;
			SetDesiredAltitude(RTHAltitude);
			if (mSClock() > mS[NavStateTimeout]) {
				F.LostModel = true;
				mSTimer(NavStateTimeout, NAV_RTH_LAND_TIMEOUT_MS);
				NavState = Descending;
				FailState = Terminating;
			} 
			else
				CheckFailsafeAbort();		
			break;
		case MonitoringRx:
			if ( mSClock() > mS[FailsafeTimeout] ) 
			{
				Stats[RCFailsafesS]++;
				if ( F.GPSValid && F.MagnetometerValid && F.AccelerationsValid )
					mSTimer(NavStateTimeout, ABORT_TIMEOUT_GPS_MS);
				else
					mSTimer(NavStateTimeout, ABORT_TIMEOUT_NO_GPS_MS);
				mSTimer(AbortTimeout, ABORT_UPDATE_MS);
				if (( Altitude < LAND_CM ) && F.BaroAltitudeValid )
					DoFailsafeShutdown();
				else
					FailState = Aborting; 
			}
			break;
		case RxTerminate: break; // invoked if Rx controls are not changing
		default:;
		} // Switch FailState

	#endif // !TESTING
			
} // DoFailsafe

void UAVXNavCommand(void) { // NavPlan adapted from ArduPilot ConfigTool GUI - quadrocopter must be disarmed

	#ifndef TESTING

	static int16 b;
	static uint8 c, d, csum;

	c = RxChar();
	LEDBlue_ON;

	switch ( c ) {
	case '0': // hello
		TxChar(ACK);
		break;
	case '1': // write
		csum = 0;
		for ( b = 0; b < 256; b++) { // cannot write fast enough so buffer
			d = RxChar();
			csum ^= d;
			BufferEE[b] = d;
		}
		if ( csum == (uint8)0 ) {
			for ( b = 0; b < 256; b++)
				WriteEE(NAV_ADDR_EE + b, BufferEE[b]);
			TxChar(ACK);
		} else
			TxChar(NAK);

		InitNavigation();
	
		break;
	case '2':
		csum = 0;
		for ( b = 0; b < 255; b++) {	
			d = ReadEE(NAV_ADDR_EE + b);
			csum ^= d;
			BufferEE[b] = d;
		}
		BufferEE[255] = csum;
		for ( b = 0; b < 256; b++)
			TxChar(BufferEE[b]);
		TxChar(ACK);
		break;
	case '3':
		csum = 0;
		for ( b = 0; b < 63; b++) {	
			d = ReadEE(STATS_ADDR_EE + b);
			csum ^= d;
			BufferEE[b] = d;
		}
		BufferEE[63] = csum;
		for ( b = 0; b < 64; b++)
			TxChar(BufferEE[b]);
		TxChar(ACK);
		break;
	default:
		break;
	} // switch

	LEDBlue_OFF;

	#endif // !TESTING

} // UAVXNavCommand

void GetWayPointEE(int8 wp) { 
	#ifndef TESTING

	static uint16 w;
	
	if ( wp > NoOfWayPoints )
		CurrWP = wp = 0;
	if ( wp == 0 ) {  // force to Origin
		WPLatitude = 0;
		WPLongitude = 0;
		WPAltitude = RTHAltitude;
		WPLoiter = 30000; // mS
	} else {	
		w = NAV_WP_START + (wp-1) * WAYPOINT_REC_SIZE;
		WPLatitude = Read32EE(w + 0);
		WPLongitude = Read32EE(w + 4);			
		WPAltitude = (int24)Read16EE(w + 8) * 100;

		#ifdef NAV_ENFORCE_ALTITUDE_CEILING
		if ( WPAltitude > NAV_CEILING )
			WPAltitude = NAV_CEILING;
		#endif // NAV_ENFORCE_ALTITUDE_CEILING
		WPLoiter = (int16)ReadEE(w + 10) * 1000L; // mS
	}

	#endif // !TESTING

	F.CrossTrackActive = F.WayPointCentred =  F.WayPointAchieved = false;

} // GetWaypointEE

void InitNavigation(void) {

	WayHeading = 0;		
	A[Roll].NavCorr = A[Pitch].NavCorr = A[Yaw].NavCorr = 0;

	SetDesiredAltitude(0);
	NavState = HoldingStation;
	AttitudeHoldResetCount = 0;
	CurrMaxRollPitch = 0;
	F.WayPointAchieved = F.WayPointCentred = false;
	F.NavComputed = false;
//	if ( ReadEE(NAV_NO_WP) <= 0 )
	{
		Nav.ProximityRadius = NAV_PROXIMITY_RADIUS * 10L; 
		Nav.ProximityAltitude = NAV_PROXIMITY_ALTITUDE * 10L; // Decimetres
	}
/* zzz
	else
	{
		// need minimum values in UAVXNav?
		Nav.ProximityRadius = ReadEE(NAV_PROX_RADIUS) * 10L;
		Nav.ProximityAltitude = ReadEE(NAV_PROX_ALT) * 10L; // Decimetres
	}
*/

	NoOfWayPoints = ReadEE(NAV_NO_WP);
	if ( NoOfWayPoints <= 0 )
		CurrWP = 0;
	else
		CurrWP = 1;
	GetWayPointEE(0);

} // InitNavigation

