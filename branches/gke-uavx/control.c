// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "uavx.h"

// Prototypes

void GyroCompensation(void);
void AltitudeDamping(void);
void LimitRollSum(void);
void LimitPitchSum(void);
void LimitYawSum(void);
void GetGyroValues(void);
void ErectGyros(void);
void CalcGyroRates(void);
void DoControl(void);

void WaitThrottleClosed(void);
void CheckThrottleMoved(void);
void WaitForRxSignal(void);
void UpdateControls(void);
void CaptureTrims(void);
void StopMotors(void);
void InitControl(void);

void GyroCompensation(void)
{
	static int16 GravComp, AbsRollSum, AbsPitchSum, Temp;
	static int16 LRAcc, LRGrav, LRDyn, FBAcc, FBGrav, FBDyn;

	#define GYRO_COMP_STEP 1

	if ( P[GyroType] == IDG300 )
		GravComp = 15;
	else
		GravComp = 11;

	if( _AccelerationsValid )
	{
		ReadAccelerations();

		LRAcc = Ax.i16;
		UDAcc = Ay.i16;
		FBAcc = Az.i16;
		
		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.

		#ifdef REVERSE_OFFSET_SIGNS	
		// Jim's solution
		LRAcc += P[MiddleLR];
		FBAcc += P[MiddleFB];
		#else
		LRAcc -= P[MiddleLR];
		FBAcc -= P[MiddleFB];
		#endif
		UDAcc -= P[MiddleUD];

		UDAcc -= 1024;	// subtract 1g - not corrrect for other than level
	
		// Roll

		// static compensation due to Gravity
		LRGrav = -SRS16(RollSum * GravComp, 5); 
	
		// dynamic correction of moved mass
		#ifdef DISABLE_DYNAMIC_MASS_COMP_ROLL
		LRDyn = 0;
		#else
		LRDyn = RollRate;	
		#endif

		// correct DC level of the integral
		LRIntKorr = (LRAcc + LRGrav + LRDyn) / 10;
		LRIntKorr = Limit(LRIntKorr, -GYRO_COMP_STEP, GYRO_COMP_STEP); 
	
		// Pitch

		// static compensation due to Gravity
		FBGrav = -SRS16(PitchSum * GravComp, 5); 
	
		// dynamic correction of moved mass		
		#ifdef DISABLE_DYNAMIC_MASS_COMP_PITCH
		FBDyn = 0;
		#else
		FBDyn = PitchRate;
		#endif

		// correct DC level of the integral	
		FBIntKorr = (FBAcc + FBGrav + FBDyn) / 10;
		FBIntKorr = Limit(FBIntKorr, -GYRO_COMP_STEP, GYRO_COMP_STEP); 

		#ifdef DEBUG_SENSORS
		Trace[TAx]= LRAcc;
		Trace[TAz] = FBAcc;
		Trace[TAy] = UDAcc;

		Trace[TLRIntKorr] = LRIntKorr * 8; // scale up for UAVPSet
		Trace[TFBIntKorr] = FBIntKorr * 8;	
		#endif // DEBUG_SENSORS	
	}	
	else
		LRIntKorr = FBIntKorr = UDAcc = 0;

} // GyroCompensation

void AltitudeDamping(void)
{ // Uses vertical accelerometer to damp altitude changes due mainly to turbulence
	static int16 Temp, AbsRollSum, AbsPitchSum;

	#ifdef ENABLE_VERTICAL_VELOCITY_DAMPING

	#ifdef SLOW_DAMPING
	if ( mS[Clock] >= mS[VerticalDampingUpdate] )
	{
	#endif // SLOW_DAMPING
		mS[VerticalDampingUpdate] = mS[Clock] + VERT_DAMPING_UPDATE;

		AbsRollSum = Abs(RollSum);
		AbsPitchSum = Abs(PitchSum);
	
		// Empirical - vertical acceleration decreases at ~approx Sum/8
		if ( (AbsRollSum < 200) && ( AbsPitchSum < 200) ) // ~ 10deg
			UDSum += UDAcc + SRS16( AbsRollSum + AbsPitchSum, 3);
	
		UDSum = Limit(UDSum , -16384, 16384); 
		UDSum = DecayBand(UDSum, -10, 10, 10);
		
		Temp = SRS16(SRS16(UDSum, 4) * (int16) P[VertDampKp], 8);	
		if( Temp > VUDComp ) 
			VUDComp++;
		else
			if( Temp < VUDComp )
				VUDComp--;
		
		VUDComp = Limit(VUDComp, -5, 20);  // -20, 20
	#ifdef SLOW_DAMPING
	}
	#endif // SLOW_DAMPING

	#endif // ENABLE_VERTICAL_VELOCITY_DAMPING
} // AltitudeDamping

void LimitRollSum(void)
{
	RollSum += SRS16(RollRate, 1);		// use 9 bit res. for I controller
	RollSum = Limit(RollSum, -RollIntLimit256, RollIntLimit256);
	RollSum = Decay(RollSum);			// damps to zero even if still rolled
	RollSum += LRIntKorr;				// last for accelerometer compensation
} // LimitRollSum

void LimitPitchSum(void)
{
	PitchSum += SRS16(PitchRate, 1);
	PitchSum = Limit(PitchSum, -PitchIntLimit256, PitchIntLimit256);
	PitchSum = Decay(PitchSum);
	PitchSum += FBIntKorr;
} // LimitPitchSum

void LimitYawSum(void)
{
	static int16 Temp;

	if ( _CompassValid )
	{
		// + CCW
		HoldYaw = HardFilter(HoldYaw, Abs(DesiredYaw - YawTrim));
		if ( HoldYaw > COMPASS_MIDDLE ) // acquire new heading
		{
			DesiredHeading = Heading;
			HE = HEp = 0; 
		}
		else
		{
			HE = MakePi(DesiredHeading - Heading);
			HE = Limit(HE, -(MILLIPI/6), MILLIPI/6); // 30 deg limit
			HE = SRS16( (HEp * 3 + HE) * (int16)P[CompassKp], 10); // CompassKp < 16 
			YE -= Limit(HE, -COMPASS_MAXDEV, COMPASS_MAXDEV);
		}
	}

	YawSum += (int16)YE;
	YawSum = Limit(YawSum, -YawIntLimit256, YawIntLimit256);

	YawSum = Decay(YawSum); 				// GKE added to kill gyro drift
	YawSum = Decay(YawSum); 

} // LimitYawSum

void GetGyroValues(void)
{
	if ( P[GyroType] == IDG300 )
	{
		RollRate += (int16)ADC(IDGADCRollChan, ADCVREF3V3);
		PitchRate += (int16)ADC(IDGADCPitchChan, ADCVREF3V3);
	}
	else
	{
		RollRate += (int16)ADC(NonIDGADCRollChan, ADCVREF5V);
		PitchRate += (int16)ADC(NonIDGADCPitchChan, ADCVREF5V);
	}
} // GetGyroValues

void ErectGyros(void)
{
	static int8 i;

	RollSum = PitchSum = YawSum = 0;
	
	DesiredThrottle = 0;					// prevent motors from starting

    for ( i=16; i; i-- )
	{
		LEDRed_TOG;
		Delay100mSWithOutput(GYRO_ERECT_DELAY);

		if ( P[GyroType] == IDG300 )
		{
			RollSum += (int16)ADC(IDGADCRollChan, ADCVREF3V3);
			PitchSum += (int16)ADC(IDGADCPitchChan, ADCVREF3V3);	
		}
		else
		{
			RollSum += (int16)ADC(NonIDGADCRollChan, ADCVREF5V);
			PitchSum += (int16)ADC(NonIDGADCPitchChan, ADCVREF5V);
		}
		YawSum += ADC(ADCYawChan, ADCVREF5V);
	}
		
	if ( P[GyroType] == ADXRS150 )
	{
		RollSum = (RollSum + 1) >> 1; 
		PitchSum = (PitchSum + 1) >> 1;
	}

	if( !_AccelerationsValid )
	{
		RollSum += P[MiddleLR];
		PitchSum += P[MiddleFB];
	}

	GyroMidRoll = (RollSum + 8) >> 4;	
	GyroMidPitch = (PitchSum + 8) >> 4;
	GyroMidYaw = (YawSum + 32) >> 6;

	RollSum = PitchSum = YawSum = 0;
	REp = PEp = YEp = 0;

	_GyrosErected = true;

} // ErectGyros

void CalcGyroRates(void)
{
	static int16 Temp;

	// RollRate & Pitchsamples hold the sum of 2 consecutive conversions
	// Approximately 4 bits of precision are discarded in this and related 
	// calculations presumably because of the range of the 16 bit arithmetic.
	// The upside, if any, is a reduction in ADC noise!

	if ( P[GyroType] == ADXRS150 )
	{
		RollRate = (RollRate + 2) >> 2; 
		PitchRate = (PitchRate + 2) >> 2;
	}
	else
	{
		RollRate = RollRate >> 1;	
		PitchRate = PitchRate >> 1;
	}
	
	// standard flight mode
	RollRate -= GyroMidRoll;
	PitchRate -= GyroMidPitch;
	if ( P[GyroType] == IDG300 )
		RollRate = -RollRate;

	if ( IsSet(P[ConfigBits], FlyXMode) )
	{
		// "Real" Roll = 0.707 * (P + R), Pitch = 0.707 * (P - R)
		Temp = RollRate + PitchRate;	
		PitchRate -= RollRate;	
		RollRate = (Temp * 7)/10;
		PitchRate = (PitchRate * 7)/10; 
	}
	
	// Yaw is sampled only once every frame, 8 bit A/D resolution
	YawRate = ADC(ADCYawChan, ADCVREF5V) >> 2;	
	YawRate -= GyroMidYaw;

} // CalcGyroRates

void DoControl(void)
{				
	CalcGyroRates();
	GyroCompensation();	
	AltitudeDamping();

	// Roll
				
	RE = SRS16(RollRate, 2);
	LimitRollSum();
 
	Rl  = SRS16(RE *(int16)P[RollKp] + (REp-RE) * P[RollKd], 4);
	Rl += SRS16(RollSum * (int16)P[RollKi], 8); 
	Rl -= DesiredRoll;

	// Pitch

	PE = SRS16(PitchRate, 2);
	LimitPitchSum();

	Pl  = SRS16(PE *(int16)P[PitchKp] + (PEp-PE) * P[PitchKd], 4);
	Pl += SRS16(PitchSum * (int16)P[PitchKi], 8);
	Pl -= DesiredPitch;

	// Yaw

	YE = YawRate;
	YE += DesiredYaw;
	LimitYawSum();

	Yl  = SRS16(YE *(int16)P[YawKp] + (YEp-YE) * P[YawKd], 4);
	Yl += SRS16(YawSum * (int16)P[YawKi], 8);
	Yl = Limit(Yl, -P[YawLimit], P[YawLimit]);		// effective slew limit

	REp = RE;
	PEp = PE;
	YEp = YE;
	HEp = HE;

	RollRate = PitchRate = 0;

	#ifdef DEBUG_SENSORS
	Trace[THE] = HE;
	Trace[TRollRate] = RollRate;
	Trace[TPitchRate] = PitchRate;
	Trace[TYE] = YE;
	Trace[TRollSum] = RollSum;
	Trace[TPitchSum] = PitchSum;
	Trace[TYawSum] = YawSum;
	#endif

} // DoControl

void WaitThrottleClosedAndRTHOff(void)
{
	uint24 Timeout;
	
	Timeout = mS[Clock] + 500; 					// mS.
	do
	{
		if( _NewValues )
		{
			UpdateControls();
			DesiredThrottle = 0; 				// prevent motors from starting
			OutSignals();
			if( mS[Clock] > Timeout )
			{
				LEDRed_TOG;	
				if ( _ReturnHome )
					Beeper_TOG;					// toggle red LED every 0.5Sec 
				Timeout = mS[Clock] + 500;
			}
		}
	}
	while( (RC[ThrottleC] >= RC_THRES_STOP) || _ReturnHome );
	Beeper_OFF;
	LEDRed_OFF;
} // WaitThrottleClosedAndRTHOff

void WaitForRxSignalAndArmed(void)
{
	do
	{		
		ProcessCommand();
		if( _Signal )
			if ( _NewValues )
			{
				UpdateParamSetChoice();
				ReadParametersEE();
				DesiredThrottle = 0;
				OutSignals();
			}	
	}
	while( !( _Signal && Armed ) );				// no signal or switch is off
} // WaitForRXSignalAndArmed

void UpdateControls(void)
{
	if ( _NewValues )
	{
		_NewValues = false;

		MapRC();								// remap channel order for specific Tx/Rx

		DesiredThrottle = RC[ThrottleC];
		DesiredRoll = RC[RollC];
		DesiredPitch = RC[PitchC];
		DesiredYaw = RC[YawC];
		NavSensitivity = RC[NavGainC];
		_ReturnHome = RC[RTHC] > RC_NEUTRAL;
	}
	else
		if ( (mS[Clock] > mS[RCSignalTimeout]) && _Signal ) 
		{ // does not need to be precise so polling OK
			CCP1CONbits.CCP1M0 = _NegativePPM; // Reset in case Tx/Rx combo has changed
			_Signal = false;
		}
} // UpdateControls

void CaptureTrims(void)
{ // only used in detecting movement from neutral in hold GPS position
	RollTrim = Limit(DesiredRoll, -MAX_TRIM, MAX_TRIM);
	PitchTrim = Limit(DesiredPitch, -MAX_TRIM, MAX_TRIM);
	YawTrim = Limit(DesiredYaw, -MAX_TRIM, MAX_TRIM);

	HoldRoll = HoldPitch = HoldYaw = 0;
} // CaptureTrims

void StopMotors(void)
{
	int8 i;

	for (i = 0; i < NoOfMotors; i++)
		Motor[i] = OUT_MINIMUM;
	MCamPitch = MCamRoll = OUT_NEUTRAL;
} // StopMotors

void InitControl(void)
{
	RollRate = PitchRate = 0;
	RollTrim = PitchTrim = YawTrim = 0;	
	VUDComp = VBaroComp = 0;	
	UDSum = 0;
	AE = AltSum = 0;
} // InitControl

