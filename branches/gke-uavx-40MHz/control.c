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

void DoAltitudeHold(void);
void AltitudeHold(void);
void DoOrientationTransform(void);
void GainSchedule(void);
void DoControl(void);

void CheckThrottleMoved(void);
void InitControl(void);

#pragma udata axisvars			
AxisStruct A[3];
#pragma udata

#pragma udata hist
uint32 CycleHist[16];
#pragma udata

uint8 PIDCyclemS, PIDCycleShift;
		
int16 CameraAngle[3];

int16 Ylp;				

int32 AngleE, RateE;
int16 YawRateIntE;
int16 HoldYaw;

int16 CurrMaxRollPitch;

int16 AttitudeHoldResetCount;
int24 DesiredAltitude, Altitude, Altitudep; 
int16 AltFiltComp, AltComp, BaroROC, BaroROCp, RangefinderROC, ROC, ROCIntE, MinROCCmpS;
int24 RTHAltitude;
int32 GS;

int8 BeepTick = 0;

void DoAltitudeHold(void) { 	// Syncronised to baro intervals independant of active altitude source
	
	static int24 AltE;
	static int16 ROCE, pROC, iROC, DesiredROC;
	static int16 NewAltComp;

	#ifdef ALT_SCRATCHY_BEEPER
	if ( (--BeepTick <= 0) && !F.BeeperInUse ) 
		Beeper_TOG;
	#endif
					
	AltE = DesiredAltitude - Altitude;

	DesiredROC = Limit(AltE, MinROCCmpS, ALT_MAX_ROC_CMPS);
		
	ROCE = DesiredROC - ROC;		
	pROC = ROCE * (int16)P[AltKp]; 
		
//zzz	ROCIntE += ROCE  * (int16)P[AltKi];
//zzz	ROCIntE = Limit1(ROCIntE, (int16)P[AltIntLimit]);
//zzz	iROC = ROCIntE;
	iROC = 0;
				
	NewAltComp = SRS16(pROC + iROC, 6);
	AltComp = Limit1(NewAltComp, ALT_MAX_THR_COMP);
				
	#ifdef ALT_SCRATCHY_BEEPER
	if ( (BeepTick <= 0) && !F.BeeperInUse) 
	{
		Beeper_TOG;
		BeepTick = 5;
	}
	#endif

} // DoAltitudeHold	

void AltitudeHold() {  // relies upon good cross calibration of baro and rangefinder!!!!!!
	static int16 ActualThrottle;

	if (F.NewBaroValue) {
		F.NewBaroValue = false;
		GetRangefinderAltitude();
		CheckThrottleMoved();
		if (F.AltHoldEnabled) {
			if (F.UsingRangefinderAlt) {
				Altitude = RangefinderAltitude;
				ROC = RangefinderROC;
			} else {
				Altitude = BaroAltitude;
				ROC = BaroROC;
			}
			if (F.ForceFailsafe || ((NavState != HoldingStation)
					&& F.AllowNavAltitudeHold)) { // Navigating - using CruiseThrottle
				F.HoldingAlt = true;
				DoAltitudeHold();
			} else if (F.ThrottleMoving) {
				F.HoldingAlt = false;
				SetDesiredAltitude(Altitude);
				AltComp = Decay1(AltComp);
			} else {
				F.HoldingAlt = true;
				DoAltitudeHold(); // not using cruise throttle
				#ifndef SIMULATE
					ActualThrottle = DesiredThrottle + AltComp;
					if ((Abs(ROC) < ALT_HOLD_MAX_ROC_CMPS) && (ActualThrottle
							> THROTTLE_MIN_CRUISE)) {
						NewCruiseThrottle
								= HardFilter(NewCruiseThrottle, ActualThrottle);
						NewCruiseThrottle
								= Limit(NewCruiseThrottle, THROTTLE_MIN_CRUISE, THROTTLE_MAX_CRUISE );
						CruiseThrottle = NewCruiseThrottle;
					}
				#endif // SIMULATE
			}
		} else {
			Altitude = BaroAltitude;
			ROC = 0;
			AltComp = Decay1(AltComp);
			F.HoldingAlt = false;
		}
	}
} // AltitudeHold

void DoOrientationTransform(void) {

	if ( !F.NavigationActive )
		A[Roll].NavCorr = A[Pitch].NavCorr = A[Yaw].NavCorr = 0;

	FastRotate(&A[Pitch].Control, &A[Roll].Control, A[Pitch].Desired
			+ A[Pitch].NavCorr, A[Roll].Desired + A[Roll].NavCorr, Orientation);

	A[Pitch].Control = A[Pitch].Desired + A[Pitch].NavCorr;
	A[Roll].Control = A[Roll].Desired + A[Roll].NavCorr;

	//A[Pitch].Control -= A[FB].Damping;
	//A[Roll].Control += A[LR].Damping;

	A[Yaw].Control = A[Yaw].Desired + A[Yaw].NavCorr;

} // DoOrientationTransform

#ifdef TESTING

void GainSchedule(void) {
	GS = 256;
} // GainSchedule

#else

void GainSchedule(void)
{  // rudimentary gain scheduling (linear)
	static int16 AttDiff, ThrDiff;

	GS = 256;

	if ( (!F.NavigationActive) || ( F.NavigationActive && (NavState == HoldingStation ) ) )
	{
		// also density altitude?
	
		if ( P[Acro] > 0) // due to Foliage 2009 and Alexinparis 2010
		{
		 	AttDiff = CurrMaxRollPitch  - ATTITUDE_HOLD_LIMIT;
			GS = (int32)GS * ( 2048L - (AttDiff * (int16)P[Acro]) );
			GS = SRS32(GS, 11);
			GS = Limit(GS, 0, 256);
		}
	
		if ( P[GSThrottle] > 0 ) 
		{
		 	ThrDiff = DesiredThrottle - CruiseThrottle;
			GS = (int32)GS * ( 2048L + (ThrDiff * (int16)P[GSThrottle]) );
			GS = SRS32(GS, 11);
		}	
	}
	
} // GainSchedule

#endif // TESTING

void Do_Wolf_Rate(AxisStruct *C) { // Origins Dr. Wolfgang Mahringer
	static int32 Temp, r;
	static i24u Temp24;

	r =  SRS32((int32)C->Rate * C->RateKp - (int32)(C->Rate - C->Ratep) * C->RateKd, 5);

    Temp = SRS32((int32)C->Angle * C->RateKi , 9);
	r += Limit1(Temp, C->IntLimit);

	Temp24.i24 = r * GS;
	r = Temp24.i2_1;

	C->Out = r - C->Control;

	C->Ratep = C->Rate;
	
} // Do_Wolf_Rate

#define D_LIMIT 32*32

void Do_PD_P_Angle(AxisStruct *C) {	// with Dr. Ming Liu
	static int32 p, d, DesRate, AngleE, AngleEp, RateE;

	AngleEp = C->AngleE;
	AngleE = C->Control * RC_STICK_ANGLE_SCALE - C->Angle;
	AngleE = Limit1(AngleE, MAX_BANK_ANGLE_DEG * DEG_TO_ANGLE_UNITS); // limit maximum demanded angle

	DesRate = SRS32(AngleE * C->RateKp, 10);

	RateE = DesRate - C->Rate; 	

	C->Out = -SRS32(RateE * C->AngleKp, 5);

	C->AngleE = AngleE;

} // Do_PD_P_Angle

#ifdef OLD_YAW

void CompensateYawGyro(void) {
	static int16 HE;

	if ( F.MagnetometerValid )
	{
		// + CCW
		if ( A[Yaw].Hold > COMPASS_MIDDLE ) // acquire new heading
		{
			DesiredHeading = Heading;
			A[Yaw].DriftCorr = 0;
		}
		else
			if ( F.NewCompassValue )
			{
				F.NewCompassValue = false;
				HE = MinimumTurn(DesiredHeading - Heading);
				HE = Limit1(HE, SIXTHMILLIPI); // 30 deg limit
				A[Yaw].DriftCorr = SRS32((int24)HE * (int24)P[YawAngleKp], 7);
				A[Yaw].DriftCorr = Limit1(A[Yaw].DriftCorr, YAW_COMP_LIMIT); // yaw gyro drift compensation
			}		
	}
	else
		A[Yaw].DriftCorr = 0;

	A[Yaw].Rate -= A[Yaw].DriftCorr;

} // CompensateYawGyro

void YawControl(void) { // with Dr. Ming Liu
	static int32 Temp;

	A[Yaw].RateE = Smooth16x16(&YawF, A[Yaw].Rate);	
	Temp  = -SRS32((int24)A[Yaw].RateE * (int16)A[Yaw].RateKp, 4);

	Temp = SlewLimit(A[Yaw].Outp, Temp, 1);
	A[Yaw].Outp = Temp;
			
	A[Yaw].Out = Limit1(Temp, (int16)P[YawLimit]);

	if ( Abs(A[Yaw].Control) > 5 )
		A[Yaw].Out -= A[Yaw].Control;

} // YawControl

#else

void UpdateDesiredHeading(void) {

	if (F.MagnetometerValid) {
		if (A[Yaw].Hold > COMPASS_MIDDLE)
			DesiredHeading = Heading;
	} else
		DesiredHeading = Heading;
} // UpdateDesiredHeading

void YawControl(void) {
	int24 r, HeadingE;

	UpdateDesiredHeading();

	HeadingE = MinimumTurn(DesiredHeading - Heading);
	HeadingE = Limit1(HeadingE, DegreesToRadians(30));
	A[Yaw].RateE =HeadingE;
	r = SRS32((int24)HeadingE * A[Yaw].AngleKp, 7);

	r -= A[Yaw].Rate;
	r  = SRS32((int24)r * A[Yaw].RateKp, 4);
	r = Limit1(r, A[Yaw].Limiter);

	if (Abs(A[Yaw].Control) > FromPercent(2, RC_NEUTRAL))
		r -= A[Yaw].Control;

	A[Yaw].Out = r;

} // YawControl

#endif //OLD_YAW

void DoControl(void){

	#ifdef SIMULATE

		DoOrientationTransform();
		GainSchedule();

		DoEmulation();

	#else

		static uint8 a;
		static AxisStruct *C;
	
		GetAttitude();

		GetHeading();

		DoOrientationTransform();
	
			GainSchedule();
	
			for ( a = Roll; a<=(uint8)Pitch; a++)
			{
				C = &A[a];
				C->Control += C->NavCorr;
				
				if ( F.UsingAltControl )	
					Do_PD_P_Angle(C);
				else
					Do_Wolf_Rate(C);
			}			
				
		YawControl();
			
		Rl = A[Roll].Out; Pl = A[Pitch].Out; Yl = A[Yaw].Out;

	#endif // SIMULATE

	F.NearLevel = Max(Abs(A[Roll].Angle), Abs(A[Pitch].Angle)) < NAV_RTH_LOCKOUT;

	OutSignals(); 

} // DoControl

void InitControl(void)
{
	static uint8 a;
	static AxisStruct *C;

	Ylp = AltComp = ROCIntE = 0;
	YawRateIntE = 0;

	for ( a = Roll; a<=(uint8)Yaw; a++)
	{
		C = &A[a];
		C->Outp = C->Ratep = C->RateE = 0;
	}

} // InitControl



