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

extern void AccelerationCompensation(void);
extern void LimitRollSum(void);
extern void LimitPitchSum(void);
extern void LimitYawSum(void);
extern void GetGyroValues(void);
extern void CalcGyroValues(void);
extern void PID(void);

extern void WaitThrottleClosed(void);
extern void CheckThrottleMoved(void);
extern void WaitForRxSignal(void);

void AccelerationCompensation(void)
{
	int16 AbsRollSum, AbsPitchSum, Temp;
	int16 Rp, Pp, Yp;

	if( _UseLISL )
	{
		ReadAccelerations();

		Rp = Ax;
		Yp = Ay;
		Pp = Az;;
		
		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.
	
		// 1 unit is 1/4096 of 2g = 1/2048g
		Rp -= MiddleLR;
		Pp -= MiddleFB;
		Yp -= MiddleUD;
	
		Yp -= 1024;	// subtract 1g
	
		#ifdef ENABLE_VERTICAL_VELOCITY_DAMPING
		// UDSum rises if ufo climbs
		// Empirical - vertical acceleration decreases at ~approx Angle/8

		AbsRollSum = Abs(RollSum);
		AbsPitchSum = Abs(PitchSum);

		if ( (AbsRollSum < 200) && ( AbsPitchSum < 200) ) // ~ 10deg
			UDSum += Yp + SRS16( AbsRollSum + AbsPitchSum, 3);

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
			Trace[TAx]= Rp;
			Trace[TAz] = Pp;
			Trace[TAy] = Yp;

			Trace[TUDSum] = UDSum;
			Trace[TVud] = Vud;
		}
		#endif
	
		// Roll

		// Static compensation due to Gravity
		#ifdef OPT_ADXRS
		Rp -= SRS16(RollSum * 11, 5);
		#else // OPT_IDG
		Rp -= SRS16(RollSum * (-15), 5); 
		#endif
	
		#ifndef ENABLE_DYNAMIC_MASS_COMP_ROLL
		// dynamic correction of moved mass
		#ifdef OPT_ADXRS
		Rp += (int16)RollRate << 1;
		#else // OPT_IDG
		Rp -= (int16)RollRate;
		#endif	
		#else
		// no dynamic correction of moved mass
		#endif

		// correct DC level of the integral
		LRIntKorr = 0;
		#ifdef OPT_ADXRS
		if( Rp > 10 ) LRIntKorr =  1;
		else
			if( Rp < 10 ) LRIntKorr = -1;
		#else // OPT_IDG
		if( Rp > 10 ) LRIntKorr = -1;
		else
			if( Rp < 10 ) LRIntKorr =  1;
		#endif
	
		// Pitch

		Pp = -Pp;				// long standing BUG!

		// Static compensation due to Gravity
		#ifdef OPT_ADXRS
		Pp -= SRS16(PitchSum * 11, 5);	
		#else // OPT_IDG
		Pp -= SRS16(PitchSum * (-15), 5);
		#endif
		
		#ifndef ENABLE_DYNAMIC_MASS_COMP_PITCH
		// dynamic correction of moved mass
		#ifdef OPT_ADXRS
		Pp += (int16)PitchRate << 1;
		#else // OPT_IDG
		Pp -= (int16)PitchRate;
		#endif	
		#else
		// no dynamic correction of moved mass
		#endif
	
		// correct DC level of the integral
		FBIntKorr = 0;
		#ifdef OPT_ADXRS
		if( Pp > 10 ) FBIntKorr =  1; 
		else 
			if( Pp < 10 ) FBIntKorr = -1;
		#endif
		#ifdef OPT_IDG
		if( Pp > 10 ) FBIntKorr = -1;
		else
			if( Pp < 10 ) FBIntKorr =  1;
		#endif		
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
} // AccelerationCompensation

void LimitRollSum(void)
{
	RollSum += RollRate;

	if( IntegralCount == 0 )
	{
		RollSum = Limit(RollSum, -RollIntLimit*256, RollIntLimit*256);
		RollSum += LRIntKorr;
		RollSum = Decay(RollSum);		// damps to zero even if still rolled
	}

} // LimitRollSum

void LimitPitchSum(void)
{
	PitchSum += PitchRate;

	if( IntegralCount == 0 )
	{
		PitchSum = Limit(PitchSum, -PitchIntLimit*256, PitchIntLimit*256);
		PitchSum += FBIntKorr;		
		PitchSum = Decay(PitchSum);	// damps to zero even if still pitched
	}
} // LimitPitchSum

void LimitYawSum(void)
{
	int16 Temp;

	YE += DesiredYaw;						// add the yaw stick value

	if ( _UseCompass )
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

	YawSum += (int16)YE;
	YawSum = Limit(YawSum, -YawIntLimit*256, YawIntLimit*256);

	YawSum = Decay(YawSum); // GKE added to kill gyro drift
	YawSum = Decay(YawSum); 

} // LimitYawSum

void GetGyroValues(void)
{
	#ifdef OPT_IDG
	RollRate += ADC(ADCRollChan, ADCVREF);
	PitchRate += ADC(ADCPitchChan, ADCVREF);
	#else
	RollRate += ADC(ADCRollChan, ADCVREF5V);
	PitchRate += ADC(ADCPitchChan, ADCVREF5V);
	#endif // OPT_IDG
} // GetGyroValues

// Calc the gyro values from added RollRate and PitchRate
void CalcGyroValues(void)
{
	int16 Temp;

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
		// pre-flight auto-zero mode
		RollSum += RollRate;
		PitchSum += PitchRate;

		if( IntegralCount == 1 )
		{
			RollSum += 8;
			PitchSum += 8;
			if( !_UseLISL )
			{
				RollSum = RollSum + MiddleLR;
				PitchSum = PitchSum + MiddleFB;
			}
			GyroMidRoll = RollSum >> 4;	
			GyroMidPitch = PitchSum >> 4;
			GyroMidYaw = 0;
			RollSum = PitchSum = LRIntKorr = FBIntKorr = 0;
		}
	}
	else
	{
		// standard flight mode
		RollRate -= GyroMidRoll;
		PitchRate -= GyroMidPitch;

		// calc Cross flying mode
		if( FlyCrossMode )
		{
			// Real Roll = 0.707 * (P + R)
			//      Pitch = 0.707 * (P - R)
			// the constant factor 0.667 is used instead
			Temp = RollRate + PitchRate;	
			PitchRate -= RollRate;	
			RollRate = (Temp * 2)/3;
			PitchRate = (PitchRate * 2)/3; // 7/10 with int24
		}

		#ifdef DEBUG_SENSORS
		Trace[TRollRate] = RollRate;
		Trace[TPitchRate] = PitchRate;
		#endif
	
		// Roll
		#ifdef OPT_ADXRS
		RE = SRS16(RollRate, 2);
		#else // OPT_IDG
		RE = SRS16(RollRate, 1); // use 8 bit res. for PD controller
		#endif	

		#ifdef OPT_ADXRS
		RollRate = SRS16(RollRate, 1); // use 9 bit res. for I controller	
		#endif

		LimitRollSum();		// for roll integration

		// Pitch
		#ifdef OPT_ADXRS
		PE = SRS16(PitchRate, 2);
		#else // OPT_IDG
		PE = SRS16(PitchRate, 1);
		#endif

		#ifdef OPT_ADXRS
		PitchRate = SRS16(PitchRate, 1); // use 9 bit res. for I controller	
		#endif

		LimitPitchSum();		// for pitch integration

		// Yaw is sampled only once every frame, 8 bit A/D resolution
		YE = ADC(ADCYawChan, ADCVREF5V) >> 2;	
		if( GyroMidYaw == 0 )
			GyroMidYaw = YE;
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

void PID(void)
{

	AccelerationCompensation();	
	
	// PID controller
	// E0 = current gyro error
	// E1 = previous gyro error
	// Sum(Ex) = integrated gyro error, sinst start of ufo!
	// A0 = current correction value
	// fx = programmable controller factors
	//
	// for Roll and Pitch:
	//       E0*fP + E1*fD     Sum(Ex)*fI
	// A0 = --------------- + ------------
	//            16               256

	// Roll

	// Differential and Proportional for Roll axis
	Rl  = SRS16(RE *(int16)RollPropFactor + (REp-RE) * RollDiffFactor, 4);

	// Integral part for Roll
	if( IntegralCount == 0 )
		Rl += SRS16(RollSum * (int16)RollIntFactor, 8); // thanks Jim aka Jesolins
	Rl -= DesiredRoll;								// subtract stick signal

	// Pitch

	// Differential and Proportional for Pitch
	Pl  = SRS16(PE *(int16)PitchPropFactor + (PEp-PE) * PitchDiffFactor, 4);

	// Integral part for Pitch
	if( IntegralCount == 0 )
		Pl += SRS16(PitchSum * (int16)PitchIntFactor, 8);

	Pl -= DesiredPitch;								// subtract stick signal

	// Yaw

	// the yaw stick signal is already added in LimitYawSum() !
	//	YE += IYaw;

	// Differential and Proportional for Yaw
	Yl  = SRS16(YE *(int16)YawPropFactor + (YEp-YE) * YawDiffFactor, 4);
	Yl += SRS16(YawSum * (int16)YawIntFactor, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);		// effective slew limit

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
				DropoutCount = 10;		// to signal: THROTTLE OPEN
			}
		}
		ProcessComCommand();
	}
	LEDRed_OFF;
} // WaitThrottleClosed

void CheckThrottleMoved(void)
{
	int16 Temp;

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

			if( DesiredThrottle < THR_HOVER ) // no hovering below this throttle setting
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
	while( !( _Signal && Armed ) );	// no signal or switch is off
} // WaitForRXSignal

