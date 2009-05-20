// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

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
void LimitRollSum(void);
void LimitPitchSum(void);
void LimitYawSum(void);
void GetGyroValues(void);
void ErectGyros(void);
void CalcGyroValues(void);
void AltitudeDamping(void);
void DoControl(void);

void WaitThrottleClosed(void);
void CheckThrottleMoved(void);
void WaitForRxSignal(void);

void GyroCompensation(void)
{
	#define GYRO_COMP_STEP 1
	#ifdef OPT_ADXRS
		#define GRAV_COMP 11
	#else
		#define GRAV_COMP 15
	#endif

	static int16 AbsRollSum, AbsPitchSum, Temp;
	static int16 LRAcc, LRGrav, LRDyn, FBAcc, FBGrav, FBDyn;

	if( _AccelerationsValid )
	{
		ReadAccelerations();

		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.

		LRAcc = Ax.i16 - MiddleLR;
		UDAcc = Ay.i16 - MiddleUD - 1024; // 1g - not corrrect for other than level
		FBAcc = Az.i16 - MiddleFB;
		
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
	
} // GyroCompensation

void LimitRollSum(void)
{
	RollSum += RollRate;

	RollSum = Limit(RollSum, -RollIntLimit256, RollIntLimit256);
	RollSum = Decay(RollSum);	// damps to zero even if still rolled
	RollSum += LRIntKorr;		// last for accelerometer compensation
} // LimitRollSum

void LimitPitchSum(void)
{
	PitchSum += PitchRate;

	PitchSum = Limit(PitchSum, -PitchIntLimit256, PitchIntLimit256);
	PitchSum = Decay(PitchSum);	// damps to zero even if still pitched
	PitchSum += FBIntKorr;		// last for accelerometer compensation
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

	YawSum += YawRate;
	YawSum = Limit(YawSum, -YawIntLimit256, YawIntLimit256);

	YawSum = Decay(YawSum); // GKE added to kill gyro drift
	YawSum = Decay(YawSum); 

} // LimitYawSum

void GetGyroValues(void)
{
	RollRate += (int16)ADC(ADCRollChan, ADCEXTVREF_PITCHROLL);
	PitchRate += (int16)ADC(ADCPitchChan, ADCEXTVREF_PITCHROLL);
} // GetGyroValues

void ErectGyros(void)
{
	static int8 i;

	RollSum = PitchSum = 0;

    for (i=0; i<16; i++)
	{
		GetGyroValues();

		// pre-flight auto-zero gyro rates
		RollSum += RollRate;
		PitchSum += PitchRate;
	}
		
	if( !_AccelerationsValid )
	{
		RollSum += MiddleLR;
		PitchSum += MiddleFB;
	}

	GyroMidRoll = (RollSum + 8) >> 4;	
	GyroMidPitch = (PitchSum + 8) >> 4;
	GyroMidYaw = 0;
	RollSum = PitchSum = 0;

} // ErectGyros

// Calc the gyro values from added RollRate and PitchRate
void CalcGyroValues(void)
{
	static int16 Temp;

	// average of two readings always positive so can use >>
	RollRate = (RollRate + 1) >> 1;
	PitchRate = (PitchRate + 1) >> 1;

	RollRate = GYROSIGN_ROLL * ( RollRate - GyroMidRoll );
	PitchRate = GYROSIGN_PITCH * ( PitchRate - GyroMidPitch );

	#ifdef OPT_ADXRS150
	RollRate = RollRate / 2;
	PitchRate = PitchRate / 2;
	#endif // OPT_ADXRS150

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

	// Yaw is sampled only once every frame - 8 bits for now
	YawRate = ADC(ADCYawChan, ADCVREF5V) >> 2;	
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
} // CalcGyroValues
	
void AltitudeDamping(void)
{
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
	if( (BlinkTick & 0x0003) == 0 )	
		if( Temp > VUDComp ) 
			VUDComp++;
		else
			if( Temp < VUDComp )
				VUDComp--;
	
	VUDComp = Limit(VUDComp, -20, 20);

	#ifdef DEBUG_SENSORS
	Trace[TUDAcc] = UDAcc;
	Trace[TUDSum] = UDSum;
	Trace[TVUDComp] = VUDComp;
	#endif

	#endif // ENABLE_VERTICAL_VELOCITY_DAMPING
} // AltitudeDamping

void DoControl(void)
{
	CheckThrottleMoved();				
	CalcGyroValues();
	GyroCompensation();	
	AltitudeDamping();
	
	// PID controller
	// E0 = current gyro error
	// E1 = previous gyro error
	// Sum(Ex) = integrated gyro error, sinst start of ufo!
	// A0 = current correction value
	// fx = programmable controller factors
	//
	// for Roll and Pitch:
	//       Rate*Kp + (PrevRate-Rate)*Kd     Sum(Rate)*Ki
	// A0 = ------------------------------- + ------------
	//                      64                   512

	// reduced divisors for Yaw

	// Roll
	Rl  = SRS16(RollRate * RollPropFactor, 6 )
		+ SRS16(RollSum * (int16)RollIntFactor, 9) 
		+ (int16)SRS32((int32)(PrevRollRate-RollRate) * RollDiffFactor, 6)
		- DesiredRoll; 

	// Pitch
	Pl  = SRS16(PitchRate * PitchPropFactor, 6)
		 + SRS16(PitchSum * (int16)PitchIntFactor, 9)
		 + (int16)SRS32((int32)(PrevPitchRate-PitchRate) * PitchDiffFactor, 6)
		 - DesiredPitch;

	// Yaw
	Yl  = SRS16(YawRate *(int16)YawPropFactor + (PrevYawRate-YawRate) * YawDiffFactor, 4) +
		SRS16(YawSum * (int16)YawIntFactor, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);	// effective slew limit

	DoPIDDisplays();

	PrevRollRate = RollRate;
	PrevPitchRate = PitchRate;
	PrevYawRate = YawRate;

	MixAndLimitMotors();
	
} // DoControl

void WaitThrottleClosed(void)
{
	static int8 d;

	d = 10;
	while( _Signal && (IGas >= _ThresStop) )
	{
		if( _NewValues )
		{
			OutSignals();
			_NewValues = false;
			if( d-- <= 0 )
			{
				LEDRed_TOG;			// toggle red LED 
				d = 10;				// to signal: THROTTLE OPEN
			}
		}
		ProcessComCommand();
	}

	ThrNeutral = 0xFF;
	LEDRed_OFF;

} // WaitThrottleClosed

void CheckThrottleMoved(void)
{
	static int16 ThrLow, ThrHigh;

	if( _NewValues )
		if( ThrCycles > 0 )
		{ // sample every ~sec
			if( (--ThrCycles) == 0 )
				ThrNeutral = DesiredThrottle;	// remember current Throttle
		}
		else
		{
			ThrLow = ThrNeutral - THR_MIDDLE;
			ThrLow = Max(THR_HOVER, ThrLow);
			ThrHigh = ThrNeutral + THR_MIDDLE;

			_ThrottleMoving = (DesiredThrottle < ThrLow) || (DesiredThrottle > ThrHigh);
			if ( _ThrottleMoving )
				ThrCycles = THR_DOWNCOUNT;	// left dead area
		}

} // CheckThrottleMoved

void WaitForRxSignal(void)
{
	static int16 d;
	
	d = MODELLOSTTIMER;
	do
	{
		Delay100mSWithOutput(2);	// wait 2/10 sec until signal is there
		ProcessComCommand();
		if( !_Signal )
			if( Armed )
			{
				if( --d <= 0 )
				{
					_LostModel = true;
					d = MODELLOSTTIMERINT;
				}
			}
			else
				_LostModel = false;
	}
	while( !( _Signal && Armed ) );				// no signal or switch is off
} // WaitForRXSignal

