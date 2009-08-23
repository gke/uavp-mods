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

void CheckThrottleMoved(void);
void UpdateControls(void);
void CaptureTrims(void);
void StopMotors(void);
void LightsAndSirens(void);
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
		DUAcc = Ay.i16;
		FBAcc = Az.i16;
		
		// NeutralLR, NeutralFB, NeutralDU pass through UAVPSet 
		// and come back as MiddleLR etc.

		LRAcc -= P[MiddleLR];
		FBAcc -= P[MiddleFB];
		DUAcc -= P[MiddleDU];

		DUAcc -= 1024;	// subtract 1g - not corrrect for other than level
						// ??? could check for large negative Acc => upside down?

		if ( LRAcc > MaxLRAcc ) MaxLRAcc = LRAcc; 
		if ( FBAcc > MaxFBAcc ) MaxFBAcc = FBAcc; 
		if ( DUAcc > MaxDUAcc ) MaxDUAcc = DUAcc; 
		
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
		Trace[TAy] = DUAcc;

		Trace[TLRIntKorr] = LRIntKorr * 8; // scale up for UAVPSet
		Trace[TFBIntKorr] = FBIntKorr * 8;	
		#endif // DEBUG_SENSORS	
	}	
	else
		LRIntKorr = FBIntKorr = DUAcc = 0;

} // GyroCompensation

void AltitudeDamping(void)
{ // Uses vertical accelerometer to damp altitude changes due mainly to turbulence
	static int16 Temp, AbsRollSum, AbsPitchSum;

	AbsRollSum = Abs(RollSum);
	AbsPitchSum = Abs(PitchSum);
	
	// Empirical - vertical acceleration decreases at ~approx Sum/8
	if ( (AbsRollSum < 200) && ( AbsPitchSum < 200) ) // ~ 10deg
		DUSum += DUAcc + SRS16( AbsRollSum + AbsPitchSum, 3);
	
	DUSum = Limit(DUSum , -16384, 16384); 
	DUSum = DecayBand(DUSum, -10, 10, 10);
		
	Temp = SRS16(SRS16(DUSum, 4) * (int16) P[VertDampKp], 8);	
	if( Temp > VDUComp ) 
		VDUComp++;
	else
		if( Temp < VDUComp )
			VDUComp--;
		
	VDUComp = Limit(VDUComp, -5, 20);  // -20, 20

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
	static int16 Rate;

	if ( P[GyroType] == IDG300 )
	{
		Rate = (int16)ADC(IDGADCRollChan, ADCVREF3V3);
		RollRate += Rate;
		if ( Rate > MaxRollRate ) MaxRollRate = Rate;
		Rate = (int16)ADC(IDGADCPitchChan, ADCVREF3V3);
		PitchRate += Rate;
		if ( Rate > MaxPitchRate ) MaxPitchRate = Rate;
	}
	else
	{
		Rate = (int16)ADC(NonIDGADCRollChan, ADCVREF5V);
		RollRate += Rate;
		if ( Rate > MaxRollRate ) MaxRollRate = Rate;
		Rate = (int16)ADC(NonIDGADCPitchChan, ADCVREF5V);
		PitchRate += Rate;
		if ( Rate > MaxPitchRate ) MaxPitchRate = Rate;
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
	if ( YawRate > MaxYawRate) MaxYawRate = YawRate;	
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

void LightsAndSirens(void)
{
	uint24 Timeout;
	int16 CurrentThrottle;

	if( _AccelerationsValid ) LEDYellow_ON; else LEDYellow_OFF;
	if ( _Signal ) LEDGreen_ON; else LEDGreen_OFF;

	Beeper_OFF; 
	CurrentThrottle = RC_MAXIMUM;
	Timeout = mS[Clock] + 500; 					// mS.
	do
	{
		ProcessCommand();
		if( _Signal )
		{
			LEDGreen_ON;
			if( _NewValues )
			{
				UpdateControls();
				UpdateParamSetChoice();
				CurrentThrottle = DesiredThrottle;
				DesiredThrottle = 0; 				// prevent motors from starting
				OutSignals();
				if( mS[Clock] > Timeout )
				{
					if ( _ReturnHome )
					{
						Beeper_TOG;					
						LEDRed_TOG;
					}
					else
						if ( Armed )
							LEDRed_TOG;
						
					Timeout += 500;
				}
			}
		}
		else
		{
			LEDRed_ON;
			LEDGreen_OFF;
		}	
		ReadParametersEE();	
	}
	while( (!_Signal) || (Armed && FirstPass) || _ReturnHome || ( CurrentThrottle >= RC_THRES_STOP) );

	FirstPass = false;

	Beeper_OFF;
	LEDRed_OFF;
	LEDGreen_ON;

	_LostModel = false;
	mS[FailsafeTimeout] = mS[Clock] + FAILSAFE_TIMEOUT_S*1000L;
	mS[AbortTimeout] = mS[Clock] + ABORT_TIMEOUT_S*1000L;
	mS[UpdateTimeout] = mS[Clock] + P[TimeSlots];
	FailState = Waiting;

} // LightsAndSirens


void InitControl(void)
{
	RollRate = PitchRate = 0;
	RollTrim = PitchTrim = YawTrim = 0;	
	VDUComp = VBaroComp = 0;	
	DUSum = 0;
	AE = AltSum = 0;
} // InitControl



