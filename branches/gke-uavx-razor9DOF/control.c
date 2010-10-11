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

void DoAltitudeHold(int24, int16);
void UpdateAltitudeSource(void);
void AltitudeHold(void);
void InertialDamping(void);
void DoHeadingLock(void);
void DoOrientationTransform(void);
void DoControl(void);

void CheckThrottleMoved(void);
void LightsAndSirens(void);
void InitControl(void);

int16 RC[CONTROLS];

int16 RE, PE, YE, HE;					// gyro rate error	
int16 REp, PEp, YEp;				// previous error for derivative
int16 Rl, Pl, Yl, Ylp;						// PID output values 	

int16 RollTrim, PitchTrim, YawTrim;
int16 HoldYaw;
int16 RollIntLimit256, PitchIntLimit256, YawIntLimit256;

int16 CruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
int16 DesiredRoll, DesiredPitch, DesiredYaw, DesiredHeading, DesiredCamPitchTrim, Heading;
int16 ControlRoll, ControlPitch, ControlRollP, ControlPitchP;
int16 CurrMaxRollPitch;

int16 ThrLow, ThrHigh, ThrNeutral;

int16 AttitudeHoldResetCount;
int16 AltComp, AltDiffSum, AltD, AltDSum;
int24 DesiredAltitude, Altitude;
int16 ROC;

int16 DUComp, DUVel, LRVel, LRComp, FBVel, FBComp;

boolean	FirstPass;
int8 BeepTick = 0;

void DoAltitudeHold(int24 Altitude, int16 ROC)
{ // Syncronised to baro intervals independant of active altitude source
	
	static int16 NewAltComp, LimBE, AltP, AltI, AltD;
	static int24 Temp, BE;

	#ifdef ALT_SCRATCHY_BEEPER
	if ( (--BeepTick <= 0) && !F.BeeperInUse ) 
		Beeper_TOG;
	#endif
			
	BE = DesiredAltitude - Altitude;
	LimBE = Limit(BE, -ALT_BAND_DM, ALT_BAND_DM);
		
	AltP = SRS16(LimBE * (int16)P[AltKp], 4);
	AltP = Limit(AltP, -ALT_MAX_THR_COMP, ALT_MAX_THR_COMP);
		
	AltDiffSum += LimBE;
	AltDiffSum = Limit(AltDiffSum, -ALT_INT_WINDUP_LIMIT, ALT_INT_WINDUP_LIMIT);
	AltI = SRS16(AltDiffSum * (int16)P[AltKi], 3);
	AltI = Limit(AltDiffSum, -(int16)P[AltIntLimit], (int16)P[AltIntLimit]);
				 
	AltD = SRS16(ROC * (int16)P[AltKd], 2);
	AltD = Limit(AltD, -ALT_MAX_THR_COMP, ALT_MAX_THR_COMP); 
			 
	if ( ROC < (int16)P[MaxDescentRateDmpS] )
	{
		AltDSum += 1;
		AltDSum = Limit(AltDSum, 0, ALT_MAX_THR_COMP * 2); 
	}
	else
		AltDSum = DecayX(AltDSum, 1);
			
	NewAltComp = AltP + AltI + AltD + AltDSum;
	NewAltComp = Limit(NewAltComp, -ALT_MAX_THR_COMP, ALT_MAX_THR_COMP);
	
	AltComp = SlewLimit(AltComp, NewAltComp, 1);
		
	#ifdef ALT_SCRATCHY_BEEPER
	if ( (BeepTick <= 0) && !F.BeeperInUse) 
	{
		Beeper_TOG;
		BeepTick = 5;
	}
	#endif

} // DoAltitudeHold	

void UpdateAltitudeSource(void)
{
	if ( F.UsingRangefinderAlt )
	{
		Altitude = RangefinderAltitude / 10; 	// Decimetres for now
		ROC = RangefinderROC / 10; 				// Decimetres/Sec.
	}
	else
	{
		Altitude = BaroRelAltitude;
		ROC = BaroROC;
	}
} // UpdateAltitudeSource

void AltitudeHold()
{  // relies upon good cross calibration of baro and rangefinder!!!!!!
	static int16 NewCruiseThrottle;

	GetBaroAltitude();
	GetRangefinderAltitude();
	CheckThrottleMoved();

	if ( F.AltHoldEnabled )
	{
		if ( F.NewBaroValue  ) // sync on Baro which MUST be working
		{		
			F.NewBaroValue = false;

			UpdateAltitudeSource();

			if ( F.ThrottleMoving )
			{
				F.HoldingAlt = false;
				DesiredAltitude = Altitude;
				AltComp = Decay1(AltComp);
			}
			else
			{
				F.HoldingAlt = true;
				if ( Abs(BaroROC) < ALT_HOLD_MAX_ROC_DMPS  ) // Use Baro NOT GPS
				{
					NewCruiseThrottle = DesiredThrottle + AltComp;
					CruiseThrottle = HardFilter(CruiseThrottle, NewCruiseThrottle);
				}
				DoAltitudeHold(Altitude, ROC);
			}	
		}
	}
	else
	{
		AltComp = Decay1(AltComp);
		F.HoldingAlt = false;
	}
} // AltitudeHold

void InertialDamping(void)
{ // Uses accelerometer to damp disturbances while holding altitude
	static int16 Temp;

	if ( F.AccelerationsValid ) 
	{
		// Down - Up
		// Empirical - acceleration changes at ~approx Angle/8 for small angles
		DUVel += Attitude.DUAcc + SRS16( Abs(Attitude.RollAngle) + Abs(Attitude.PitchAngle), 3);		
		DUVel = Limit(DUVel , -16384, 16383); 			
		Temp = SRS32(SRS16(DUVel, 4) * (int32)P[VertDampKp], 13);
		if( Temp > DUComp ) 
			DUComp++;
		else
			if( Temp < DUComp )
				DUComp--;			
		DUComp = Limit(DUComp, DAMP_VERT_LIMIT_LOW, DAMP_VERT_LIMIT_HIGH); 
		DUVel = DecayX(DUVel, (int16)P[VertDampDecay]);
	
		// Lateral compensation only when holding altitude?	
		if ( F.HoldingAlt && F.AttitudeHold ) 
		{
	 		if ( F.WayPointCentred )
			{
				// Left - Right
				LRVel += Attitude.LRAcc;
				LRVel = Limit(LRVel , -16384, 16383);  	
				Temp = SRS32(SRS16(LRVel, 4) * (int32)P[HorizDampKp], 13);
				if( Temp > LRComp ) 
					LRComp++;
				else
					if( Temp < LRComp )
						LRComp--;
				LRComp = Limit(LRComp, -DAMP_HORIZ_LIMIT, DAMP_HORIZ_LIMIT);
				LRVel = DecayX(LRVel, (int16)P[HorizDampDecay]);
		
				// Front - Back
				FBVel += Attitude.FBAcc;
				FBVel = Limit(FBVel , -16384, 16383);  
				Temp = SRS32(SRS16(FBVel, 4) * (int32)P[HorizDampKp], 13);
				if( Temp > FBComp ) 
					FBComp++;
				else
					if( Temp < FBComp )
						FBComp--;
				FBComp = Limit(FBComp, -DAMP_HORIZ_LIMIT, DAMP_HORIZ_LIMIT);
				FBVel = DecayX(FBVel, (int16)P[HorizDampDecay]);
			}
			else
			{
				LRVel = FBVel = 0;
				LRComp = Decay1(LRComp);
				FBComp = Decay1(FBComp);
			}
		}
		else
		{
			LRVel = FBVel = 0;
	
			LRComp = Decay1(LRComp);
			FBComp = Decay1(FBComp);
		}
	}
	else
		LRComp = FBComp = DUComp = LRVel = FBVel = DUVel = 0;

} // InertialDamping	

#define COMPASS_MIDDLE 10
#define COMPASS_MAXDEV 30

void DoHeadingLock(void)
{
	static int16 Temp;

	if ( F.CompassValid )
	{
		// + CCW
		Temp = DesiredYaw - YawTrim;
		if ( Abs(Temp) > COMPASS_MIDDLE ) // acquire new heading
		{
			DesiredHeading = Heading;
			HE = 0; 
		}
		else
		{
			HE = MakePi(DesiredHeading - Heading);
			HE = Limit(HE, -SIXTHMILLIPI, SIXTHMILLIPI); // 30 deg limit
			HE = SRS32( (int32)HE * (int32)P[CompassKp], 12); 
			YE -= Limit(HE, -COMPASS_MAXDEV, COMPASS_MAXDEV);
		}
	}

	Attitude.YawAngle = Limit(Attitude.YawAngle, -YawIntLimit256, YawIntLimit256);

} // DoHeadingLock

void DoOrientationTransform(void)
{
	static i24u Temp;
	static int24 OSO, OCO;

	if ( F.UsingPolar )
	{
		OSO = OSin[PolarOrientation];
		OCO = OCos[PolarOrientation];
	}
	else
	{
		OSO = OSin[Orientation];
		OCO = OCos[Orientation];
	}

	// -PS+RC
	Temp.i24 = -DesiredPitch * OSO + DesiredRoll * OCO;
	ControlRoll = Temp.i2_1 - LRComp;
		
	// PC+RS
	Temp.i24 = DesiredPitch * OCO + DesiredRoll * OSO;
	ControlPitch = Temp.i2_1 - FBComp; 

} // DoOrientationTransform

void DoControl(void)
{
	static i24u Temp;

    InertialDamping();

	#ifdef SIMULATE

	FakeDesiredRoll = DesiredRoll;
	FakeDesiredPitch = DesiredPitch;
	FakeDesiredYaw =  DesiredYaw;
	Attitude.RollAngle = SlewLimit(Attitude.RollAngle, -FakeDesiredRoll * 16, 4);
	Attitude.PitchAngle = SlewLimit(Attitude.PitchAngle, -FakeDesiredPitch * 16, 4);
	Attitude.YawAngle = SlewLimit(Attitude.YawAngle, FakeDesiredYaw, 4);
	Rl = -FakeDesiredRoll;
	Pl = -FakeDesiredPitch;
	Yl = DesiredYaw;
	 
    #else

	DoOrientationTransform();

	// Roll
				
	RE = Attitude.RollRate;
	Attitude.RollAngle = Limit(Attitude.RollAngle, -RollIntLimit256, RollIntLimit256);
 
	Rl  = SRS16(RE *(int16)P[RollKp] + (REp-RE) * (int16)P[RollKd], 4);
	Rl -= SRS16(Attitude.RollAngle * (int16)P[RollKi], 9); 
	Rl -= ControlRoll + SRS16((ControlRoll - ControlRollP) * ATTITUDE_FF_DIFF, 4);
	ControlRollP = ControlRoll;

	// Pitch

	PE = Attitude.PitchRate;
	Attitude.PitchAngle = Limit(Attitude.PitchAngle, -PitchIntLimit256, PitchIntLimit256);

	Pl  = SRS16(PE *(int16)P[PitchKp] + (PEp-PE) * (int16)P[PitchKd], 4);
	Pl -= SRS16(Attitude.PitchAngle * (int16)P[PitchKi], 9);
	Pl -= ControlPitch + SRS16((ControlPitch - ControlPitchP) * ATTITUDE_FF_DIFF, 4);
	ControlPitchP = ControlPitch;

	// Yaw
	
	YE = Attitude.YawRate;
	DoHeadingLock();

	YE += DesiredYaw;

	Yl  = SRS16(YE *(int16)P[YawKp] + (YEp-YE) * (int16)P[YawKd], 4);
	Yl += SRS16(Attitude.YawAngle * (int16)P[YawKi], 8);

	#ifdef TRICOPTER
		Yl = SlewLimit(Ylp, Yl, 1);
		Ylp = Yl;
	#endif // TRICOPTER

	Yl = Limit(Yl, -(int16)P[YawLimit], (int16)P[YawLimit]);
		
	REp = RE;
	PEp = PE;
	YEp = YE;

	#endif // SIMULATE		

	F.NearLevel = Max(Abs(Attitude.RollAngle), Abs(Attitude.PitchAngle)) < NAV_RTH_LOCKOUT;

} // DoControl

static int8 RCStart = RC_INIT_FRAMES;

void LightsAndSirens(void)
{
	static int24 Ch5Timeout, AccTimeout;

	LEDYellow_TOG;
	if ( F.Signal ) LEDGreen_ON; else LEDGreen_OFF;

	Beeper_OFF;
	Ch5Timeout = AccTimeout = mSClock()+ 500; 					// mS.
	do
	{
		ProcessCommand();
		if( F.Signal )
		{
			LEDGreen_ON;
			if( F.RCNewValues )
			{
				UpdateControls();
				if ( --RCStart == 0 ) // wait until RC filters etc. have settled
				{
					UpdateParamSetChoice();
					MixAndLimitCam();
					RCStart = 1;
				}
				GetBaroAltitude();
				InitialThrottle = StickThrottle;
				StickThrottle = 0; 
				OutSignals();
				if( mSClock() > Ch5Timeout )
				{
					if ( F.Navigate || F.ReturnHome || !F.ParametersValid )
					{
						Beeper_TOG;					
						LEDRed_TOG;
					}
					else
						if ( Armed )
							LEDRed_TOG;
							
					Ch5Timeout += 500;
				}	
			}
		}
		else
		{
			LEDRed_ON;
			LEDGreen_OFF;
		}	
		ReadParametersEE();


		if ( RazorReady() )
		{
			GetAttitude(0);
			Razor('?');
		}
		
	}
	while( (!F.Signal) || (Armed && FirstPass) || F.Ch5Active || F.GyroFailure || 
		( InitialThrottle >= RC_THRES_START ) || (!F.ParametersValid)  );
				
	FirstPass = false;

	Beeper_OFF;
	LEDRed_OFF;
	LEDGreen_ON;
	LEDYellow_ON;

	mS[LastBattery] = mSClock();
	mS[UpdateTimeout] = mSClock() + (uint24)P[TimeSlots];

	F.LostModel = false;

} // LightsAndSirens

void InitControl(void)
{
	RollTrim = PitchTrim = YawTrim = 0;
	ControlRollP = ControlPitchP = 0;
	Ylp = 0;
	AltComp = 0;
	DUComp = DUVel = LRVel = LRComp = FBVel = FBComp = 0;	
} // InitControl



