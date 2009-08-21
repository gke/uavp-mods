// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =           http://code.google.com/p/uavp-mods/ http://uavp.ch        =
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
void WaitForRxSignalAndDisarmed(void);
void UpdateControls(void);
void CaptureTrims(void);
void StopMotors(void);
void InitControl(void);

void GyroCompensation(void)
{
	static int16 GravComp, AbsRollSum, AbsPitchSum, Temp;
	static int16 LRAcc, LRGrav, LRDyn, FBAcc, FBGrav, FBDyn;

	#define GYRO_COMP_STEP 1

	if( _AccelerationsValid )
	{
		if ( P[GyroType] == IDG300 )
			GravComp = 8; 		// -1/6 of reference
		else
			GravComp = 11; 		// 9..11   

		ReadAccelerations();

		LRAcc = Ax.i16;
		UDAcc = Ay.i16;
		FBAcc = Az.i16;
		
		// NeutralLR, NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.

		LRAcc -= P[MiddleLR];
		FBAcc -= P[MiddleFB];
		UDAcc -= P[MiddleUD];

		UDAcc -= 1024;	// subtract 1g - not corrrect for other than level
						// ??? could check for large negative Acc => upside down?
	
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
	static uint8 i;

	RollSum = PitchSum = YawSum = 0;
	
	DesiredThrottle = 0;					// prevent motors from starting

    for ( i = 16; i ; i-- )
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
	// 300 Deg/Sec is the "reference" gyro full scale rate

	if ( P[GyroType] == IDG300 )
	{ // 500 Deg/Sec or 1.66 ~= 2 so do not average readings 
		RollRate -= GyroMidRoll * 2;
		PitchRate -= GyroMidPitch * 2;
		RollRate = -RollRate;			// adjust for reversed rool gyro sense
 	}
	else
	{ // 1.0
		// Average of two readings
		RollRate = ( RollRate >> 1 ) - GyroMidRoll;	
		PitchRate = ( PitchRate >> 1 ) - GyroMidPitch;

		if ( P[GyroType] == ADXRS150 )	// 150 deg/sec (0.5)
		{ // 150 Deg/Sec or 0.5
			RollRate /= 2; 
			PitchRate /= 2;
		}
	}

	if ( P[ConfigBits] & FlyXModeMask )
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
		ProcessCommand();
		if( _NewValues )
		{
			UpdateControls();
			DesiredThrottle = 0; 				// prevent motors from starting
			OutSignals();
			if( mS[Clock] > Timeout )
			{
				LEDRed_TOG;	
				if ( _ReturnHome )
					Beeper_TOG;					// toggle Beeper every 0.5Sec 
				Timeout += 500;
			}
		}
	}
	while( (RC[ThrottleC] >= RC_THRES_STOP) || _ReturnHome );
	Beeper_OFF;
	LEDRed_OFF;
} // WaitThrottleClosedAndRTHOff

void WaitForRxSignalAndDisarmed(void)
{
	uint24 Timeout;
	uint8 SaveLEDShadow;
	
	Timeout = mS[Clock] + 500; 					// mS.
	do
	{		
		ProcessCommand();
		if( _Signal )
		{
			LEDGreen_ON;

			if ( _NewValues )
			{
				UpdateParamSetChoice();
				ReadParametersEE();
				DesiredThrottle = 0;
				OutSignals();
				if ( Armed )
					if( mS[Clock] > Timeout )
					{
						LEDRed_TOG;
						Timeout += 500;			// Toggle LEDs every 0.5Sec
					}
			}
		}
		else
		{
			LEDRed_ON;
			LEDGreen_OFF;
		}	
	}
	while( ( Armed && FirstPass) || !_Signal );

	FirstPass = false;

	LEDRed_OFF;
	LEDGreen_ON;

} // WaitForRxSignalAndDisarmed

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
		FailState = Waiting;
	}
} // UpdateControls

void CaptureTrims(void)
{ // only used in detecting movement from neutral in hold GPS position
	RollTrim = Limit(DesiredRoll, -NAV_MAX_TRIM, NAV_MAX_TRIM);
	PitchTrim = Limit(DesiredPitch, -NAV_MAX_TRIM, NAV_MAX_TRIM);
	YawTrim = Limit(DesiredYaw, -NAV_MAX_TRIM, NAV_MAX_TRIM);

	HoldRoll = HoldPitch = HoldYaw = 0;
} // CaptureTrims

void StopMotors(void)
{
	static uint8 m;

	for (m = 0; m < NoOfMotors; m++)
		Motor[m] = OUT_MINIMUM;
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

