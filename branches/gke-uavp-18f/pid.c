// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =           Extensively modified 2008-9 by Prof. Greg Egan            =
// =                          http://www.uavp.org                        =
// =======================================================================
//
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

// The PID controller algorithm

#include "c-ufo.h"
#include "bits.h"

// Limit integral sum of Roll gyros
// it must be limited to avoid numeric overflow
// which would cause a serious flip -> crash
void LimitRollSum(void)
{
	RollSum += RollSamples;

	if( IntegralCount == 0 )
	{
		RollSum = Limit(RollSum, -RollIntLimit*256, RollIntLimit*256);
		RollSum += LRIntKorr;
		RollSum = Decay(RollSum);		// damps to zero even if still rolled
	}

} // LimitRollSum

// Limit integral sum of Pitch gyros
// it must be limited to avoid numeric overflow
// which would cause a serious flip -> crash
void LimitPitchSum(void)
{
	PitchSum += PitchSamples;

	if( IntegralCount == 0 )
	{
		PitchSum = Limit(PitchSum, -PitchIntLimit*256, PitchIntLimit*256);
		PitchSum += FBIntKorr;		
		PitchSum = Decay(PitchSum);	// damps to zero even if still pitched
	}
} // LimitPitchSum

// Limit integral sum of Yaw gyros
// it must be limited to avoid numeric overflow
// which would cause a uncontrolled yawing -> crash
void LimitYawSum(void)
{
	int16 Temp;
	// add the yaw stick value
	YE += IYaw;

	if ( _UseCompass )
	{
		// add compass heading correction
		// CurDeviation is negative if Ufo has yawed to the right (go back left)

		// this double "if" is necessary because of dumb CC5X compiler
		Temp = YawNeutral + COMPASS_MIDDLE;
		if ( IYaw > Temp )
			// yaw stick is not in neutral zone, learn new desired heading
			AbsDirection = COMPASS_INVAL;
		else		
		{
			Temp = YawNeutral - COMPASS_MIDDLE;
			if ( IYaw < Temp )
				// yaw stick is not in neutral zone, learn new desired heading
				AbsDirection = COMPASS_INVAL;
			else
				// yaw stick is in neutral zone, hold heading
				if( CurDeviation > COMPASS_MAXDEV )
					YE -= COMPASS_MAXDEV;
				else
					if( CurDeviation < -COMPASS_MAXDEV )
						YE += COMPASS_MAXDEV;
					else
						YE -= CurDeviation;
		}
	}

	YawSum += (int16)YE;
	YawSum = Limit(YawSum, -YawIntLimit*256, YawIntLimit*256);

} // LimitYawSum

void GetGyroValues(void)
{
	#ifdef OPT_IDG
	RollSamples += ADC(ADCRollChan, ADCVREF);
	PitchSamples += ADC(ADCPitchChan, ADCVREF);
	#else
	RollSamples += ADC(ADCRollChan, ADCVREF5V);
	PitchSamples += ADC(ADCPitchChan, ADCVREF5V);
	#endif // OPT_IDG
} // GetGyroValues

// Calc the gyro values from added RollSamples and PitchSamples
void CalcGyroValues(void)
{
	int16 Temp;

	// RollSamples & Pitchsamples hold the sum of 2 consecutive conversions
	// Approximately 4 bits of precision are discarded in this and related 
	// calculations presumably because of the range of the 16 bit arithmetic.

	#ifdef OPT_ADXRS150
	RollSamples = (RollSamples + 2)>>2; // recreate the 10 bit resolution
	PitchSamples = (PitchSamples + 2)>>2;
	#else // IDG300 and ADXRS300
	RollSamples = (RollSamples + 1)>>1;	
	PitchSamples = (PitchSamples + 1)>>1;
	#endif
	
	if( IntegralCount > 0 )
	{
		// pre-flight auto-zero mode
		RollSum += RollSamples;
		PitchSum += PitchSamples;

		if( IntegralCount == 1 )
		{
			RollSum += 8;
			PitchSum += 8;
			if( !_UseLISL )
			{
				RollSum = RollSum + MiddleLR;
				PitchSum = PitchSum + MiddleFB;
			}
			MidRoll = RollSum >> 4;	
			MidPitch = PitchSum >> 4;
			MidYaw = 0;
			RollSum = PitchSum = LRIntKorr = FBIntKorr = 0;
		}
	}
	else
	{
		// standard flight mode
		RollSamples -= MidRoll;
		PitchSamples -= MidPitch;

		// calc Cross flying mode
		if( FlyCrossMode )
		{
			// Real Roll = 0.707 * (P + R)
			//      Pitch = 0.707 * (P - R)
			// the constant factor 0.667 is used instead
			Temp = RollSamples + PitchSamples;	
			PitchSamples -= RollSamples;	
			RollSamples = (Temp * 2)/3;
			PitchSamples = (PitchSamples * 2)/3; // 7/10 with int24
		}

		#ifdef DEBUG_SENSORS
		TxValH16(RollSamples);
		TxChar(';');
		TxValH16(PitchSamples);
		TxChar(';');
		#endif
	
		// Roll
		#ifdef OPT_ADXRS
		RE = SRS16(RollSamples + 2, 2);
		#else // OPT_IDG
		RE = SRS16(RollSamples + 1, 1); // use 8 bit res. for PD controller
		#endif	

		#ifdef OPT_ADXRS
		RollSamples = SRS16(RollSamples + 1, 1); // use 9 bit res. for I controller	
		#endif

		LimitRollSum();		// for roll integration

		// Pitch
		#ifdef OPT_ADXRS
		PE = SRS16(PitchSamples + 2, 2);
		#else // OPT_IDG
		PE = SRS16(PitchSamples + 1, 1);
		#endif

		#ifdef OPT_ADXRS
		PitchSamples = SRS16(PitchSamples + 1, 1); // use 9 bit res. for I controller	
		#endif

		LimitPitchSum();		// for pitch integration

		// Yaw is sampled only once every frame, 8 bit A/D resolution
		YE = ADC(ADCYawChan, ADCVREF5V)>>2;
		if( MidYaw == 0 )
			MidYaw = YE;
		YE -= MidYaw;

		LimitYawSum();

		#ifdef DEBUG_SENSORS
		TxValH(YE);
		TxChar(';');
		TxValH16(RollSum);
		TxChar(';');
		TxValH16(PitchSum);
		TxChar(';');
		TxValH16(YawSum);
		TxChar(';');
		#endif
	}
} // CalcGyroValues

// compute the correction adders for the motors
// using the gyro values (PID controller)
// for the axes Roll and Pitch
void PID(void)
{

	CheckLISL();	// get accelerations, if available

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
	Rl  = SRS16(RE *(int16)RollPropFactor + (REp-RE) * RollDiffFactor + 8, 4);

	// Integral part for Roll
	if( IntegralCount == 0 )
		Rl += SRS16(RollSum * (int16)RollIntFactor + 128, 8); // thanks Jim

	// subtract stick signal
	Rl -= IRoll;

	// Pitch

	// Differential and Proportional for Pitch
	Pl  = SRS16(PE *(int16)PitchPropFactor + (PEp-PE) * PitchDiffFactor + 8, 4);

	// Integral part for Pitch
	if( IntegralCount == 0 )
		Pl += SRS16(PitchSum * (int16)PitchIntFactor +128, 8);

	// subtract stick signal
	Pl -= IPitch;

	// Yaw

	// the yaw stick signal is already added in LimitYawSum() !
	//	YE += IYaw;

	// Differential for Yaw (for quick and stable reaction)
	Yl  = SRS16(YE *(int16)YawPropFactor + (YEp-YE) * YawDiffFactor + 8, 4);
	Yl += SRS16(YawSum * (int16)YawIntFactor + 128, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);

	// Camera

	// use only roll/pitch angle
	if( (IntegralCount == 0) && (CamRollFactor != 0) && (CamPitchFactor != 0) )
	{
		Rp = RollSum / (int16)CamRollFactor;
		Pp = PitchSum / (int16)CamPitchFactor;
	}
	else
		Rp = Pp = 0;

	DoPIDDisplays();

} // PID
