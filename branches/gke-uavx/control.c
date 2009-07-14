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
void CalcGyroValues(void);
void DoControl(void);

void WaitThrottleClosed(void);
void CheckThrottleMoved(void);
void WaitForRxSignal(void);
void UpdateControls(void);
void CaptureTrims(void);

void GyroCompensation(void)
{
	static int16 GravComp, AbsRollSum, AbsPitchSum, Temp;
	static int16 LRAcc, LRGrav, LRDyn, FBAcc, FBGrav, FBDyn;

	#define GYRO_COMP_STEP 1

	if ( GyroType == IDG300 )
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

		// 1 unit is 1/4096 of 2g = 1/2048g
		#ifdef REVERSE_OFFSET_SIGNS	
		// Jim's solution
		LRAcc += MiddleLR;
		FBAcc += MiddleFB;
		#else
		LRAcc -= MiddleLR;
		FBAcc -= MiddleFB;
		#endif
		UDAcc -= MiddleUD;

		UDAcc -= 1024;	// subtract 1g - not corrrect for other than level
	
		#ifdef DEBUG_SENSORS
		Trace[TAx]= LRAcc;
		Trace[TAz] = FBAcc;
		Trace[TAy] = UDAcc;

		Trace[TUDSum] = UDSum;
		Trace[TVUDComp] = VUDComp;
		#endif
	
		// Roll

		// static compensation due to Gravity
		LRGrav = -SRS16(RollSum * GravComp, 5); 
	
		// dynamic correction of moved mass
		#ifdef DISABLE_DYNAMIC_MASS_COMP_ROLL
		LRDyn = 0;
		#else
		LRDyn = RollRate * 2;	
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
		FBDyn = PitchRate * 2;
		#endif

		// correct DC level of the integral	
		FBIntKorr = (FBAcc + FBGrav + FBDyn) / 10;
		FBIntKorr = Limit(FBIntKorr, -GYRO_COMP_STEP, GYRO_COMP_STEP); 

		#ifdef DEBUG_SENSORS
		Trace[TLRAcc] = LRAcc;
		Trace[TLRGrav] = LRGrav;
		Trace[TLRDyn] = LRDyn;
		Trace[TLRIntKorr] = LRIntKorr;

		Trace[TFBAcc] = FBAcc;
		Trace[TFBGrav] = FBGrav;
		Trace[TFBDyn] = FBDyn;
		Trace[TFBIntKorr] = FBIntKorr;	
		#endif // DEBUG_SENSORS	
	}	
	else
	{
		VUDComp = 0;
		#ifdef DEBUG_SENSORS
		Trace[TAx] = 0;
		Trace[TAz] = 0;
		Trace[TAy] = 0;

		Trace[TUDSum] = 0;
		Trace[TVUDComp] = 0;
		#endif
	}
} // GyroCompensation

void AltitudeDamping(void)
{
	int16 Temp, AbsRollSum, AbsPitchSum;

	#ifdef ENABLE_VERTICAL_VELOCITY_DAMPING
	// UDSum rises if ufo climbs
	// Empirical - vertical acceleration decreases at ~approx Angle/8

	if ( mS[Clock] >= mS[VerticalDampingUpdate] )
	{
		mS[VerticalDampingUpdate] = mS[Clock] + VERT_DAMPING_UPDATE;

		AbsRollSum = Abs(RollSum);
		AbsPitchSum = Abs(PitchSum);
	
		if ( (AbsRollSum < 200) && ( AbsPitchSum < 200) ) // ~ 10deg
			UDSum += UDAcc + SRS16( AbsRollSum + AbsPitchSum, 3);
	
		UDSum = Limit(UDSum , -16384, 16384); 
		UDSum = DecayBand(UDSum, -10, 10, 10);
		
		Temp = SRS16(SRS16(UDSum, 4) * (int16) VertDampKp, 8);	
		if( Temp > VUDComp ) 
			VUDComp++;
		else
			if( Temp < VUDComp )
				VUDComp--;
		
		VUDComp = Limit(VUDComp, -20, 20);

	}

	#ifdef DEBUG_SENSORS
	Trace[TUDSum] = UDSum;
	Trace[TVUDComp] = VUDComp;
	#endif

	#endif // ENABLE_VERTICAL_VELOCITY_DAMPING
} // AltitudeDamping

void LimitRollSum(void)
{
	RollSum += RollRate;

	if( IntegralCount == 0 )
	{
		RollSum = Limit(RollSum, -RollIntLimit256, RollIntLimit256);
		RollSum = Decay(RollSum);	// damps to zero even if still rolled
		RollSum += LRIntKorr;		// last for accelerometer compensation
	}

} // LimitRollSum

void LimitPitchSum(void)
{
	PitchSum += PitchRate;

	if( IntegralCount == 0 )
	{
		PitchSum = Limit(PitchSum, -PitchIntLimit256, PitchIntLimit256);
		PitchSum = Decay(PitchSum);	// damps to zero even if still pitched
		PitchSum += FBIntKorr;		// last for accelerometer compensation
	}
} // LimitPitchSum

void LimitYawSum(void)
{
	static int16 Temp;

	YE += DesiredYaw;						// add the yaw stick value

	if ( _CompassValid )
	{
		// CurDeviation is negative if quadrocopter has yawed to the right (go back left)
		if ( Abs(DesiredYaw) > COMPASS_MIDDLE )
			AbsDirection = COMPASS_INVAL; // acquire new heading
		else		
			if( CurDeviation > COMPASS_MAXDEV )
				YE -= COMPASS_MAXDEV;
			else
				if( CurDeviation < -COMPASS_MAXDEV )
					YE += COMPASS_MAXDEV;
				else
					YE -= CurDeviation;
	}

	#ifdef YAW_RESET
	if ( (Sign(YawSum) != Sign(YE)) && (Abs(YawSum) > 0) )
		YawSum = 0;
	#endif // YAW_RESET

	YawSum += (int16)YE;
	YawSum = Limit(YawSum, -YawIntLimit256, YawIntLimit256);

	YawSum = Decay(YawSum); // GKE added to kill gyro drift
	YawSum = Decay(YawSum); 

} // LimitYawSum

void GetGyroValues(void)
{
	if ( GyroType == IDG300 )
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

#ifdef NEW_ERECT_GYROS

void ErectGyros(void)
{
	static int8 i;

	RollSum = PitchSum = YawSum = 0;

    for ( i=16; i; i-- )
	{
		if ( GyroType == IDG300 )
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
		
	if ( GyroType == ADXRS150 )
	{
		RollSum = (RollSum + 1) >> 1; 
		PitchSum = (PitchSum + 1) >> 1;
	}

	if( !_AccelerationsValid )
	{
		RollSum += MiddleLR;
		PitchSum += MiddleFB;
	}

	GyroMidRoll = (RollSum + 8) >> 4;	
	GyroMidPitch = (PitchSum + 8) >> 4;
	GyroMidYaw = (YawSum + 32) >> 6;

	RollSum = PitchSum = YawSum = 0;

	_GyrosErected = true;

} // ErectGyros

#endif // NEW_ERECT_GYROS

// Calc the gyro values from added RollRate and PitchRate
void CalcGyroValues(void)
{
	static int16 Temp;

	// RollRate & Pitchsamples hold the sum of 2 consecutive conversions
	// Approximately 4 bits of precision are discarded in this and related 
	// calculations presumably because of the range of the 16 bit arithmetic.

	if ( GyroType == ADXRS150 )
	{
		RollRate = (RollRate + 2) >> 2; // recreate the 10 bit resolution
		PitchRate = (PitchRate + 2) >> 2;
	}
	else
	{
		RollRate = RollRate >> 1;	
		PitchRate = PitchRate >> 1;
	}
	
	if( IntegralCount > 0 )
	{
		// pre-flight auto-zero mode
		RollSum += RollRate;
		PitchSum += PitchRate;

		if( IntegralCount == 1 )
		{
			if( !_AccelerationsValid )
			{
				RollSum += MiddleLR;
				PitchSum += MiddleFB;
			}
			GyroMidRoll = (RollSum + 8) >> 4;	
			GyroMidPitch = (PitchSum + 8) >> 4;
			GyroMidYaw = 0;
			RollSum = PitchSum = LRIntKorr = FBIntKorr = 0;
		}
		IntegralCount--;
	}
	else
	{
		// standard flight mode
		if ( GyroType == IDG300 )
		{
			RollRate = -RollRate + GyroMidRoll ;
			PitchRate = PitchRate - GyroMidPitch;
		}
		else
		{
			RollRate =  RollRate - GyroMidRoll;
			PitchRate =  PitchRate - GyroMidPitch;
		}

		// calc Cross flying mode
		if( FlyCrossMode )
		{
			// Real Roll = 0.707 * (P + R)
			//      Pitch = 0.707 * (P - R)
			// the constant factor 0.667 is used instead
			Temp = RollRate + PitchRate;	
			PitchRate -= RollRate;	
			RollRate = (Temp * 2)/3;
			PitchRate = (PitchRate * 2)/3; 	// 7/10 with int24
		}

		#ifdef DEBUG_SENSORS
		Trace[TRollRate] = RollRate;
		Trace[TPitchRate] = PitchRate;
		#endif
	
		// Roll
		RE = SRS16(RollRate, 2); 			// use 8 bit res. for PD controller

		RollRate = SRS16(RollRate, 1);		// use 9 bit res. for I controller	
		LimitRollSum();

		// Pitch
		PE = SRS16(PitchRate, 2);

		PitchRate = SRS16(PitchRate, 1); 	// use 9 bit res. for I controller	
		LimitPitchSum();					// for pitch integration

		// Yaw is sampled only once every frame, 8 bit A/D resolution
		YE = ADC(ADCYawChan, ADCVREF5V) >> 2;

		#ifndef NEW_ERECT_GYROS
		if( GyroMidYaw == 0 )
			GyroMidYaw = YE;
		#endif // !NEW_ERECT_GYROS

		YE -= GyroMidYaw;
		YawRate = YE;

		LimitYawSum();

		#ifdef DEBUG_SENSORS
		Trace[TYE] = YE;
		Trace[TRollSum] = RollSum;
		Trace[TPitchSum] = PitchSum;
		Trace[TYawSum] = YawSum;
		#endif
	}
} // CalcGyroValues

void DoControl(void)
{				
	CalcGyroValues();
	GyroCompensation();	
	AltitudeDamping();

	// Roll

	Rl  = SRS16(RE *(int16)RollKp + (REp-RE) * RollKd, 4);

	if( IntegralCount == 0 )
		Rl += SRS16(RollSum * (int16)RollKi, 8); 
	Rl -= DesiredRoll;						// subtract stick signal

	// Pitch

	Pl  = SRS16(PE *(int16)PitchKp + (PEp-PE) * PitchKd, 4);

	if( IntegralCount == 0 )
		Pl += SRS16(PitchSum * (int16)PitchKi, 8);

	Pl -= DesiredPitch;						// subtract stick signal

	// Yaw

	// the yaw stick signal is already added in LimitYawSum() !
	//	YE += IYaw;

	// Differential and Proportional for Yaw
	Yl  = SRS16(YE *(int16)YawKp + (YEp-YE) * YawKd, 4);
	Yl += SRS16(YawSum * (int16)YawKi, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);	// effective slew limit

	REp = RE;
	PEp = PE;
	YEp = YE;

} // DoControl

void WaitThrottleClosed(void)
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
				LEDRed_TOG;						// toggle red LED every 0.5Sec 
				Timeout = mS[Clock] + 500;
			}
		}
	}
	while( RC[ThrottleC] >= RC_THRES_STOP );

	LEDRed_OFF;
} // WaitThrottleClosed

void WaitForRxSignal(void)
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
} // WaitForRXSignal

void UpdateControls(void)
{
	if ( _NewValues )
	{
		_NewValues = false;

		MapRC();

		DesiredThrottle = RC[ThrottleC];
		DesiredRoll = RC[RollC];
		DesiredPitch = RC[PitchC];
		DesiredYaw = RC[YawC];
		NavSensitivity = RC[NavGainC];
		_ReturnHome = RC[RTHC] > RC_NEUTRAL;
	}
	else
		if ( (mS[Clock] > mS[RCSignalTimeout]) && _Signal ) // does not need to be precise so polling OK
		{
			CCP1CONbits.CCP1M0 = _NegativePPM; // Reset in case Tx/Rx combo has changed
			_Signal = false;
		}
} // UpdateControls

void CaptureTrims(void)
{
	#ifdef CAPTURE_TRIMS
	if ( !_TrimsCaptured )
	{
		RollTrim = Limit(DesiredRoll, -20, 20);
		PitchTrim = Limit(DesiredPitch, -20, 20);
		_TrimsCaptured = true;
	}
	#endif // CAPTURE_TRIMS
} // CaptureTrims

