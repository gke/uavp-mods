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

// Utilities and subroutines

#include "c-ufo.h"
#include "bits.h"

// Limit integral sum of Roll gyros
// it must be limited to avoid numeric overflow
// which would cause a serious flip -> crash
void LimitRollSum(void)
{

	RollSum += (int16)RollSamples;

	if( IntegralCount == 0 )
	{
		RollSum = Limit(RollSum, -RollIntLimit*256, RollIntLimit*256);

		RollSum += LRIntKorr;
		RollSum = Decay(RollSum);
	}

} // LimitRollSum

// Limit integral sum of Pitch gyros
// it must be limited to avoid numeric overflow
// which would cause a serious flip -> crash
void LimitPitchSum(void)
{

	PitchSum += (int16)PitchSamples;

	if( IntegralCount == 0 )
	{
		PitchSum = Limit(PitchSum, -PitchIntLimit*256, PitchIntLimit*256);

		PitchSum += FBIntKorr;
		PitchSum = Decay(PitchSum);

		#ifdef NADA
		SendComValH16(PitchSamples);
		SendComValH16(PitchSum);
		SendComValH16(MidPitch);
		SendComChar(0x0d);SendComChar(0x0a);
		#endif

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

	if( CompassTest )
	{
		if( CurDeviation > 0 )
			LedGreen_ON;
		else
			if( CurDeviation < 0 )
				LedRed_ON;
		if( AbsDirection > COMPASS_MAX )
			LedYellow_ON;
	}

} // LimitYawSum

// to avoid stopping motors in the air, the
// motor values are limited to a minimum and
// a maximum
// the eventually corrected value is returned
int8 SaturInt(int16 l)
{
	#if defined ESC_PPM || defined ESC_HOLGER || defined ESC_YGEI2C
	if( l > _Maximum )
		return(_Maximum);
	if( l < MotorLowRun )
		return(MotorLowRun);
	// just for safety
	if( l < _Minimum )
		return(_Minimum);
	#endif

	#ifdef ESC_X3D
	l -= _Minimum;
	if( l > 200 )
		return(200);
	if( l < 1 )
		return(1);
	#endif
	return((int8)l);
} // SaturInt

// mix the PID-results (Rl, Pl and Yl) and the throttle
// on the motors and check for numerical overrun
void MixAndLimit(void)
{
	uns8 CurrGas;
    int16 Temp;

	CurrGas = IGas;	// to protect against IGas being changed in interrupt
 
	#ifndef TRICOPTER
	if( FlyCrossMode )
	{	// "Cross" Mode
		Ml = CurrGas + Pl; Ml -= Rl;
		Mr = CurrGas - Pl; Mr += Rl;
		Mf = CurrGas - Pl; Mf -= Rl;
		Mb = CurrGas + Pl; Mb += Rl;
	}
	else
	{	// "Plus" Mode
		#ifdef MOUNT_45
		Ml = CurrGas - Rl; Ml -= Pl;	// K2 -> Front right
		Mr = CurrGas + Rl; Mr += Pl;	// K3 -> Rear left
		Mf = CurrGas + Rl; Mf -= Pl;	// K1 -> Front left
		Mb = IGas - Rl; Mb += Pl;	// K4 -> Rear rigt
		#else
		Ml = CurrGas - Rl;	// K2 -> Front right
		Mr = CurrGas + Rl;	// K3 -> Rear left
		Mf = CurrGas - Pl;	// K1 -> Front left
		Mb = CurrGas + Pl;	// K4 -> Rear rigt
		#endif
	}

	Mf += Yl;
	Mb += Yl;
	Ml -= Yl;
	Mr -= Yl;

	// Altitude stabilization factor
	Mf += Vud;
	Mb += Vud;
	Ml += Vud;
	Mr += Vud;

	Mf += VBaroComp;
	Mb += VBaroComp;
	Ml += VBaroComp;
	Mr += VBaroComp;

	// if low-throttle limiting occurs, must limit other motor too
	// to prevent flips!

	if( CurrGas > MotorLowRun )
	{
		if( (Mf > Mb) && (Mb < MotorLowRun) )
		{
			Temp = Mb - MotorLowRun;
			Mf += Temp;
			Ml += Temp;
			Mr += Temp;
		}
		if( (Mb > Mf) && (Mf < MotorLowRun) )
		{
			Temp = Mf - MotorLowRun;
			Mb += Temp;
			Ml += Temp;
			Mr += Temp;
		}
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			Temp = Mr - MotorLowRun;
			Ml += Temp;
			Mf += Temp;
			Mb += Temp;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{	
			Temp = Ml - MotorLowRun;
			Mr += Temp;
			Mf += Temp;
			Mb += Temp;
		}
	}
	#else	// TRICOPTER
	Mf = CurrGas + Pl;	// front motor
	Ml = CurrGas + Rl;
	Mr = CurrGas - Rl;
	Rl >>= 1;
	Ml -= Rl;	// rear left
    Mr -= Pl;	// rear right
	Mb = Yl + _Neutral;	// yaw servo

	if( CurrGas > MotorLowRun )
	{
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			// Mf += Mb - MotorLowRun
			Ml += Mr;
			Ml -= MotorLowRun;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{
			// Mb += Mf - MotorLowRun
			Mr += Ml;
			Mr -= MotorLowRun;
		}
	}
	#endif

	// Ergebnisse auf Überlauf testen und korrigieren
	MFront = SaturInt(Mf);
	MLeft = SaturInt(Ml);
	MRight = SaturInt(Mr);
	MBack = SaturInt(Mb);
} // MixAndLimit

