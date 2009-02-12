// ==============================================
// =      U.A.V.P Brushless UFO Controller      =
// =           Professional Version             =
// = Copyright (c) 2007 Ing. Wolfgang Mahringer =
// ==============================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITPESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// ==============================================
// =  please visit http://www.uavp.org          =
// =               http://www.mahringer.co.at   =
// ==============================================

// The PID controller algorithm

#pragma codepage=1
#include "c-ufo.h"
#include "bits.h"

// Math Library
#include "mymath16.h"


// compute the correction adders for the motors
// using the gyro values (PID controller)
// for the axes Roll and Pitch
void PID(void)
{

	if( IntegralTest || CompassTest )
		ALL_LEDS_OFF;

// Roll/Pitch Linearsensoren
	Rp = 0;
	Pp = 0;
//	Vud = 0;

	if( _UseLISL )
	{
		CheckLISL();	// get the linear sensors data, if available
	}
#ifdef DEBUG_SENSORS
	else
	{
		SendComChar(';');
		SendComChar(';');
		SendComChar(';');
	}
#endif

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


// ####################################
// Roll

// Differential and Proportional for Roll axis

	Rl  = -RE;
	Rl += REp;
	Rl *= (long)RollDiffFactor;
	Rl += (long)RE   * (long)RollPropFactor;

	Rl += 8;
	Rl >>= 4;

	Rl += Rp;	// add proportional part

	if( CompassTest )
	{
		if( CurDeviation > 0 )
			LedGreen_ON;
		if( CurDeviation < 0 )
			LedRed_ON;
		if( AbsDirection > COMPASS_MAX )
			LedYellow_ON;
	}

	if( IntegralTest )
	{
		if( (int)RollSum.high8 > 0 )
		{
			LedRed_ON;
		}
		if( (int)RollSum.high8 < -1 )
		{
			LedGreen_ON;
		}
	}

// Integral part for Roll

	if( IntegralCount == 0 )
	{
		Rp = RollSum * (long)RollIntFactor;
		Rp += 128;
		Rl += (int)Rp.high8;
	}

// muss so gemacht werden, weil CC5X kein if(Pl < -RollPitchLimit) kann!
//	NegFact = -RollLimit;
//	if( Rl < NegFact ) Rl = NegFact;
//	if( Rl > RollLimit ) Rl = RollLimit;

// subtract stick signal
	Rl -= IRoll;


// ####################################
// Pitch

// Differential and Proportional for Pitch

	Pl  = -PE;
	Pl += PEp;
	Pl *= (long)PitchDiffFactor;
	Pl += (long)PE   * (long)PitchPropFactor;
	
	Pl += 8;
	Pl >>= 4;	// divide rounded by 16

	Pl += Pp;	// add proportional part

	if( IntegralTest )
	{
		if( (int)PitchSum.high8 >  0 )
		{
			LedYellow_ON;
		}
		if( (int)PitchSum.high8 < -1 )
		{
			LedBlue_ON;
		}
	}

// Integral part for Pitch
	if( IntegralCount == 0 )
	{
		Pp = PitchSum * (long)PitchIntFactor;
		Pp += 128;
		Pl += (int)Pp.high8;
	}

// muss so gemacht werden, weil CC5X kein if(Pl < -RollPitchLimit) kann!
//	NegFact = -PitchLimit;
//	if( Pl < NegFact ) Pl = NegFact;
//	if( Pl > PitchLimit ) Pl = PitchLimit;

// subtract stick signal
	Pl -= IPitch;

// PID controller for Yaw (Heading Lock)
//       E0*fp + E1*fD     Sum(Ex)*fI
// A0 = --------------- + ------------
//             16              256

// ####################################
// Yaw

// the yaw stick signal is already added in LimitYawSum() !
//	YE += IYaw;

	Yl = YawSum * (long)YawIntFactor;
	Yl += 128;		// divide rounded by 256
	Yl = (int)Yl.high8;

// Differential for Yaw (for quick and stable reaction)

	Yp  = -YE;
	Yp += YEp;
	Yp *= (long)YawDiffFactor;
	Yp += (long)YE  * (long)YawPropFactor;
	Yp += 8;
	Yp >>= 4;	// divide rounded by 16
	Yl += Yp;

	NegFact = -YawLimit;
	if( Yl < NegFact ) Yl = NegFact;
	if( Yl > YawLimit ) Yl = YawLimit;

//
// calculate camera servos
//
// use only integral part (direct angle)
//
	if( (IntegralCount == 0) && 
		(CamRollFactor != 0) && (CamPitchFactor != 0) )
	{
		Rp = RollSum / (long)CamRollFactor;
		Pp = PitchSum / (long)CamPitchFactor;
	}
	else
	{
		Rp = 0;
		Pp = 0;
	}
}
