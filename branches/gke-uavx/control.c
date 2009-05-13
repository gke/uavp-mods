// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
// =======================================================================

//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include "uavx.h"

// Prototypes

extern void GyroCompensation(void);
extern void LimitRollSum(void);
extern void LimitPitchSum(void);
extern void LimitYawSum(void);
extern void GetGyroValues(void);
extern void CalcGyroValues(void);
extern void PID(void);

extern void WaitThrottleClosed(void);
extern void CheckThrottleMoved(void);
extern void WaitForRxSignal(void);

void GyroCompensation(void)
{
	#define GYRO_COMP_STEP 1
	#ifdef OPT_ADXRS
		#define GRAV_COMP 11
	#else
		#define GRAV_COMP 15
	#endif

	static int16 AbsRollSum, AbsPitchSum, Temp;
	static int16 UDAcc, LRAcc, LRGrav, LRDyn, FBAcc, FBGrav, FBDyn;

	if( _AccelerationsValid )
	{
		ReadAccelerations();

		LRAcc = Ax.i16;
		UDAcc = Ay.i16;
		FBAcc = Az.i16;
		
		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.
	
		// 1 unit is 1/4096 of 2g = 1/2048g
		LRAcc -= MiddleLR;
		FBAcc -= MiddleFB;
		UDAcc -= MiddleUD;
	
		UDAcc -= 1024;	// subtract 1g - not corrrect for other than level
	
		#ifdef ENABLE_VERTICAL_VELOCITY_DAMPING
		// UDSum rises if ufo climbs
		// Empirical - vertical acceleration decreases at ~approx Angle/8

		AbsRollSum = Abs(RollSum);
		AbsPitchSum = Abs(PitchSum);

		if ( (AbsRollSum < 200) && ( AbsPitchSum < 200) ) // ~ 10deg
			UDSum += UDAcc + SRS16( AbsRollSum + AbsPitchSum, 3);

		UDSum = Limit(UDSum , -16384, 16384); 
		UDSum = DecayBand(UDSum, -10, 10, 10);
	
		Temp = SRS16(SRS16(UDSum, 4) * (int16) LinUDIntFactor, 8);
		if( (BlinkCount & 0x0003) == 0 )	
			if( Temp > Vud ) 
				Vud++;
			else
				if( Temp < Vud )
					Vud--;
	
		Vud = Limit(Vud, -20, 20);
	
		#endif // ENABLE_VERTICAL_VELOCITY_DAMPING

		#ifdef DEBUG_SENSORS
		if( IntegralCount == 0 )
		{
			Trace[TAx]= LRAcc;
			Trace[TAz] = FBAcc;
			Trace[TAy] = UDAcc;

			Trace[TUDSum] = UDSum;
			Trace[TVud] = Vud;
		}
		#endif
	
		// Roll

		// static compensation due to Gravity
		LRGrav = -SRS16(RollSum * GRAV_COMP, 5); 
	
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
		FBGrav = -SRS16(PitchSum * GRAV_COMP, 5); 
	
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
		Vud = 0;
		#ifdef DEBUG_SENSORS
		Trace[TAx] = 0;
		Trace[TAz] = 0;
		Trace[TAy] = 0;

		Trace[TUDSum] = 0;
		Trace[TVud] = 0;
		#endif
	}
} // GyroCompensation

void LimitRollSum(void)
{
	RollSum += RollRate;

	if( IntegralCount == 0 )
	{
		RollSum = Limit(RollSum, -RollIntLimit*256, RollIntLimit*256);
		RollSum = Decay(RollSum);	// damps to zero even if still rolled
		RollSum += LRIntKorr;		// last for accelerometer compensation
	}

} // LimitRollSum

void LimitPitchSum(void)
{
	PitchSum += PitchRate;

	if( IntegralCount == 0 )
	{
		PitchSum = Limit(PitchSum, -PitchIntLimit*256, PitchIntLimit*256);
		PitchSum = Decay(PitchSum);	// damps to zero even if still pitched
		PitchSum += FBIntKorr;		// last for accelerometer compensation
	}
} // LimitPitchSum

void LimitYawSum(void)
{
	static int16 Temp;

	YawRate += DesiredYaw;						// add the yaw stick value

	if ( _CompassValid )
	{
		// CurDeviation is negative if quadrocopter has yawed to the right (go back left)
		if ( Abs(DesiredYaw) > COMPASS_MIDDLE )
			AbsDirection = COMPASS_INVAL; // acquire new heading
		else		
			if( CurDeviation > COMPASS_MAXDEV )
				YawRate -= COMPASS_MAXDEV;
			else
				if( CurDeviation < -COMPASS_MAXDEV )
					YawRate += COMPASS_MAXDEV;
				else
					YawRate -= CurDeviation;
	}

	YawSum += (int16)YawRate;
	YawSum = Limit(YawSum, -YawIntLimit*256, YawIntLimit*256);

	YawSum = Decay(YawSum); // GKE added to kill gyro drift
	YawSum = Decay(YawSum); 

} // LimitYawSum

void GetGyroValues(void)
{
	RollRate += (int16)ADC(ADCRollChan, ADCEXTVREF_PITCHROLL);
	PitchRate += (int16)ADC(ADCPitchChan, ADCEXTVREF_PITCHROLL);
} // GetGyroValues

//int16 DummyPitch = 0;//zz

// Calc the gyro values from added RollRate and PitchRate
void CalcGyroValues(void)
{
	static int16 Temp;

	// RollRate & Pitchsamples hold the sum of 2 consecutive conversions
	// Approximately 4 bits of precision are discarded in this and related 
	// calculations presumably because of the range of the 16 bit arithmetic.

	#ifdef OPT_ADXRS150
	RollRate = (RollRate + 2) >> 2; // recreate the 10 bit resolution
	PitchRate = (PitchRate + 2) >> 2;
	#else // IDG300 and ADXRS300
	RollRate = RollRate >> 1;	
	PitchRate = PitchRate >> 1;
	#endif
	
	if( IntegralCount > 0 )
	{
		// pre-flight auto-zero gyro rates
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
			GyroMidYaw = RollSum = PitchSum = LRIntKorr = FBIntKorr = 0;
		}
	}
	else
	{
		// flight mode
		RollRate = GYROSIGN_ROLL * ( RollRate - GyroMidRoll );
		PitchRate = GYROSIGN_PITCH * ( PitchRate - GyroMidPitch );

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
	
		LimitRollSum();
		LimitPitchSum();

		// Yaw is sampled only once every frame
		YawRate = ADC(ADCYawChan, ADCVREF5V);	
		if( GyroMidYaw == 0 )
			GyroMidYaw = YawRate;
		YawRate -= GyroMidYaw;
		YawRate = YawRate;
		LimitYawSum();

		#ifdef DEBUG_SENSORS
		Trace[TRollRate] = RollRate;
		Trace[TPitchRate] = PitchRate;
		Trace[TYawRate] = YawRate;
		Trace[TRollSum] = RollSum;
		Trace[TPitchSum] = PitchSum;
		Trace[TYawSum] = YawSum;
		#endif
	}
} // CalcGyroValues

void PID(void)
{
	CalcGyroValues();
	GyroCompensation();	
	
	// PID controller
	// E0 = current gyro error
	// E1 = previous gyro error
	// Sum(Ex) = integrated gyro error, sinst start of ufo!
	// A0 = current correction value
	// fx = programmable controller factors
	//
	// for Roll and Pitch:
	//       E0*Kp + E1*Kd     Sum(Ex)*Ki
	// A0 = --------------- + ------------
	//            16               256

	// Roll

	// Differential and Proportional for Roll axis
	Rl  = SRS16(RollRate *RollPropFactor + (PrevRollRate-RollRate) * RollDiffFactor, 4);

	// Integral part for Roll
	if( IntegralCount == 0 )
		Rl += SRS16(RollSum * RollIntFactor, 8); 
	Rl -= DesiredRoll;						// subtract stick signal

	// Pitch

	// Differential and Proportional for Pitch
	Pl  = SRS16(PitchRate * PitchPropFactor + (PrevPitchRate-PitchRate) * PitchDiffFactor, 4);

	// Integral part for Pitch
	if( IntegralCount == 0 )
		Pl += SRS16(PitchSum * (int16)PitchIntFactor, 8);

	Pl -= DesiredPitch;						// subtract stick signal

	// Yaw

	// the yaw stick signal is already added in LimitYawSum() !
	//	YE += IYaw;

	// Differential and Proportional for Yaw
	Yl  = SRS16(YawRate *(int16)YawPropFactor + (PrevYawRate-YawRate) * YawDiffFactor, 4);
	Yl += SRS16(YawSum * (int16)YawIntFactor, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);	// effective slew limit

	DoPIDDisplays();

} // PID

void WaitThrottleClosed(void)
{
	DropoutCount = 1;
	while( (IGas >= _ThresStop) )
	{
		if ( !_Signal)
			break;
		if( _NewValues )
		{
			OutSignals();
			_NewValues = false;
			if( --DropoutCount <= 0 )
			{
				LEDRed_TOG;	// toggle red LED 
				DropoutCount = 10;				// to signal: THROTTLE OPEN
			}
		}
		ProcessComCommand();
	}
	LEDRed_OFF;
} // WaitThrottleClosed

void CheckThrottleMoved(void)
{
	static int16 Temp;

	if( _NewValues )
	{
		if( ThrDownCount > 0 )
		{
			if( (LEDCount & 1) == 0 )
				ThrDownCount--;
			if( ThrDownCount == 0 )
				ThrNeutral = DesiredThrottle;	// remember current Throttle level
		}
		else
		{
			if( ThrNeutral < THR_MIDDLE )
				Temp = 0;
			else
				Temp = ThrNeutral - THR_MIDDLE;

			if( DesiredThrottle < THR_HOVER ) 	// no hovering below this throttle setting
				ThrDownCount = THR_DOWNCOUNT;	// left dead area

			if( DesiredThrottle < Temp )
				ThrDownCount = THR_DOWNCOUNT;	// left dead area
			if( DesiredThrottle > ThrNeutral + THR_MIDDLE )
				ThrDownCount = THR_DOWNCOUNT;	// left dead area
		}
	}
} // CheckThrottleMoved

void WaitForRxSignal(void)
{
	DropoutCount = MODELLOSTTIMER;
	do
	{
		Delay100mSWithOutput(2);	// wait 2/10 sec until signal is there
		ProcessComCommand();
		if( !_Signal )
			if( Armed )
			{
				if( --DropoutCount == 0 )
				{
					_LostModel = true;
					DropoutCount = MODELLOSTTIMERINT;
				}
			}
			else
				_LostModel = false;
	}
	while( !( _Signal && Armed ) );				// no signal or switch is off
} // WaitForRXSignal

