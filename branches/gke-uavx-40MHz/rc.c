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
#include "uavx.h"

void InitRC(void);
void MapRC(void);
void CheckSticksHaveChanged(void);
void UpdateControls(void);
void CaptureTrims(void);
void CheckThrottleMoved(void);

uint8 Map[CONTROLS], RMap[CONTROLS];
int16 RC[CONTROLS], RCp[CONTROLS];
int16 CruiseThrottle, NewCruiseThrottle, MaxCruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
int16 DesiredCamPitchTrim;
int16 ThrLow, ThrHigh, ThrNeutral;

void UpdateRCMap(void) {
	static uint8 i;

	for (i = 0; i < (uint8) CONTROLS; i++)
		Map[i] = i;

	Map[ThrottleRC] = P[RxThrottleCh];
	Map[RollRC] = P[RxRollCh];
	Map[PitchRC] = P[RxPitchCh];
	Map[YawRC] = P[RxYawCh];
	Map[RTHRC] = P[RxGearCh];
	Map[CamPitchRC] = P[RxAux1Ch];
	Map[NavGainRC] = P[RxAux2Ch];
	Map[BypassRC]= P[RxAux3Ch];
	Map[PolarCoordsRC] = P[RxAux4Ch];

	for (i = ThrottleRC; i <= (uint8) PolarCoordsRC; i++)
		Map[i] -= 1;

	for ( i = 0; i < (uint8)CONTROLS; i++) // make reverse map
		RMap[Map[i]] = i;

} // UpdateRCMap

void InitRC(void) {
	static uint8 c;
	
	for (c = 0; c < (uint8)CONTROLS; c++)
		PPM[c].i16 = RC[c] = RCp[c] = 0;

	for (c = RollRC; c <= YawRC; c++)
		RC[c] = RCp[c] = RC_NEUTRAL;
	RC[CamPitchRC] = RCp[CamPitchRC] = RC_NEUTRAL;
	
	F.UsingCompoundPPM = P[RCType] == CompoundPPM;
	PIE1bits.CCP1IE = false;
	CCP1CONbits.CCP1M0 = true;
	PPM_Index = PrevEdge = RCGlitches = 0;
	PIE1bits.CCP1IE = true;

	SignalCount = -RC_GOOD_BUCKET_MAX;
	F.Signal = F.RCNewValues = false;

	UpdateRCMap();

	for (c = Roll; c<=Yaw; c++)
		A[c].Desired = A[c].Trim = A[c].Hold = 0;
 
	DesiredThrottle = StickThrottle = 0; 
	DesiredCamPitchTrim = 0;
	Nav.Sensitivity = 0;
	F.ReturnHome = F.Navigate = F.AltHoldEnabled = false;

	mS[StickChangeUpdate] = mSClock();
	mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);
	SignalCount = -RC_GOOD_BUCKET_MAX;
	F.ForceFailsafe = false;

} // InitRC

void MapRC(void) {  // re-maps captured PPM to Rx channel sequence
	static uint8 c, cc;
	static int16 LastThrottle, Temp;
	static i32u Temp2;

	LastThrottle = RC[ThrottleRC];

	for (c = 0 ; c < (uint8)NoOfControls ; c++) {
		cc = RMap[c];
		//Temp = ((int32)PPM[i].i16 * RC_MAXIMUM + 625L)/1250L; // scale to 4uS res. for now
		Temp2.i32 = ((int32)PPM[c].i16 * (RC_MAXIMUM*53L)  );
		RC[cc] = RxFilter(RC[cc], Temp2.iw1);					
	}
} // MapRC

void CheckSticksHaveChanged(void)
{
	#ifdef TESTING

	F.ForceFailsafe = false;

	#else

	static uint32 Now;
	static boolean Change;
	static uint8 c;

	if ( F.FailsafesEnabled )
	{
		Now = mSClock();
		if ( F.ReturnHome || F.Navigate  ) {
			Change = true;
			mS[RxFailsafeTimeout] = Now + RC_NO_CHANGE_TIMEOUT_MS;			
			F.ForceFailsafe = false;
		} else {
			if ( Now > mS[StickChangeUpdate] ) {
				mS[StickChangeUpdate] = Now + 500;
		
				Change = false;
				for ( c = ThrottleC; c <= (uint8)RTHRC; c++ ) {
					Change |= Abs( RC[c] - RCp[c]) > RC_STICK_MOVEMENT;
					RCp[c] = RC[c];
				}
			}
		
			if ( Change ) {
				mS[RxFailsafeTimeout] = Now + RC_NO_CHANGE_TIMEOUT_MS;
				mS[NavStateTimeout] = Now;
				F.ForceFailsafe = false;
	
				if ( FailState == MonitoringRx ) {
					if ( F.LostModel ) {
						Beeper_OFF;
						F.LostModel = false;
						DescentComp = 1;
					}
				}
			} else
				if ( Now > mS[RxFailsafeTimeout] ) {
					if ( !F.ForceFailsafe && ( State == InFlight ) )
					{
						//Stats[RCFailsafesS]++;
						mS[NavStateTimeout] = Now + NAV_RTH_LAND_TIMEOUT_MS;
						mS[DescentUpdate]  = Now + ALT_DESCENT_UPDATE_MS;
						DescentComp = 1; // for no Baro case
						F.ForceFailsafe = true;
					}
				}
		}
	} else
		F.ForceFailsafe = false;

	#endif 
} // CheckSticksHaveChanged

void UpdateControls(void) {
	static int16 Temp;
	static uint8 c;

	F.RCNewValues = false;
	MapRC();								// remap channel order for specific Tx/Rx
	StickThrottle = RC[ThrottleRC];

	//_________________________________________________________________________________________

	// Stick Processing

	F.ReturnHome = F.Navigate = false;
	if (NoOfControls > Map[RTHRC]) {
		if (RC[RTHRC] > ((2L*RC_MAXIMUM)/3L))
			F.ReturnHome = true;
		else if (RC[RTHRC] > ((RC_MAXIMUM)/3L))
			F.Navigate = true;
	}

	if (NoOfControls > Map[NavGainRC]) {
		Nav.Sensitivity = RC[NavGainRC];
		Nav.Sensitivity = Limit(Nav.Sensitivity, 0, RC_MAXIMUM);
		F.AltHoldEnabled = Nav.Sensitivity > FromPercent(NAV_SENS_ALTHOLD_THRESHOLD, RC_MAXIMUM);	
	} else {
		Nav.Sensitivity = FromPercent(50, RC_MAXIMUM);
		F.AltHoldEnabled = true;
	}

	if (NoOfControls > Map[CamPitchRC])
		DesiredCamPitchTrim = RC[CamPitchRC] - RC_NEUTRAL;
	else
		DesiredCamPitchTrim = RC_NEUTRAL;

	#ifdef GKE_TUNE
		TuneTrim = SRS16(DesiredCamPitchTrim, 4);
	#endif

	#ifdef INC_POLAR
		#if defined(MULTICOPTER) | defined(HELICOPTER)
		F.PolarCoords = (NoOfControls > Map[PolarCoordsRC]) && (RC[PolarCoordsRC]
				> (RC_MAXIMUM/2L));
		#else
		F.PolarCoords = false;
		#endif
	#else
		F.PolarCoords = false;
	#endif

	#ifdef MULTICOPTER
		F.Bypass = false;
	#else
		F.Bypass = (NoOfControls > Map[BypassRC]) && (RC[BypassRC] > (RC_MAXIMUM/2L));
	#endif

	//_________________________________________________________________________________________

	// Altitude Hold

	if ( NavState == HoldingStation )
	{ // Manual
		if ( StickThrottle < RC_THRES_STOP )	// to deal with usual non-zero EPA
			StickThrottle = 0;
	}
	else // Autonomous
		if ( F.AllowNavAltitudeHold &&  F.AltHoldEnabled )
			StickThrottle = CruiseThrottle;

	if ( (! F.HoldingAlt) && !(F.ReturnHome || F.Navigate) ) // cancel any current altitude hold setting 
		SetDesiredAltitude(Altitude);	

	//_________________________________________________________________________________________
			
	// Attitude
		
	A[Roll].Desired = RC[RollRC] - RC_NEUTRAL;
	A[Pitch].Desired = RC[PitchRC] - RC_NEUTRAL;		
	A[Yaw].Desired = RC[YawRC] - RC_NEUTRAL;
	
	for (c = Roll; c <= Yaw; c++) {				
		Temp = A[c].Desired - A[c].Trim;
		A[c].Hold = Abs(Temp);
	}
	CurrMaxRollPitch = Max(A[Roll].Hold, A[Pitch].Hold);
		
	if ( CurrMaxRollPitch > ATTITUDE_HOLD_LIMIT )
		if ( AttitudeHoldResetCount > ATTITUDE_HOLD_RESET_INTERVAL )
			F.AttitudeHold = false;
		else {
			AttitudeHoldResetCount++;
			F.AttitudeHold = true;
		} else {
			F.AttitudeHold = true;	
			if ( AttitudeHoldResetCount > 1 )
				AttitudeHoldResetCount -= 2; // Faster decay
	}

	//_________________________________________________________________________________________
			
	// Rx has gone to failsafe

	CheckSticksHaveChanged();
	
	F.NewCommands = true;

} // UpdateControls

void CaptureTrims(void) { 
	// only used in detecting movement from neutral in hold GPS position
	// Trims are invalidated if Nav sensitivity is changed - Answer do not use trims ?
	#ifndef TESTING
	static uint8 c;

	for (c = Roll; c <= Yaw; c++)
		A[c].Trim = Limit1(A[c].Desired, NAV_MAX_TRIM);
	#endif // TESTING
} // CaptureTrims

void CheckThrottleMoved(void)
{
	if( mSClock() < mS[ThrottleUpdate] )
		ThrNeutral = DesiredThrottle;
	else {
		ThrLow = ThrNeutral - THROTTLE_MIDDLE;
		ThrLow = Max(ThrLow, THROTTLE_MIN_ALT_HOLD);
		ThrHigh = ThrNeutral + THROTTLE_MIDDLE;
		if ( ( DesiredThrottle <= ThrLow ) || ( DesiredThrottle >= ThrHigh ) ) {
			mSTimer(ThrottleUpdate, THROTTLE_UPDATE_MS);
			F.ThrottleMoving = true;
		} else
			F.ThrottleMoving = false;
	}
} // CheckThrottleMoved




