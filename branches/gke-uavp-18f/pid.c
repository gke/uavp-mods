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
	Vud = 0;

	if( _UseLISL )
		CheckLISL();	// get the linear sensors data, if available
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
	Rl  = SRS16(RE *(int16)RollPropFactor + (REp-RE) * RollDiffFactor + 8, 4);

	if( IntegralTest )
		if( (int8)(RollSum>>8) > 0 )
			LedRed_ON;
		else
			if( (int8)(RollSum>>8) < -1 )
				LedGreen_ON;

	// Integral part for Roll
	if( IntegralCount == 0 )
		Rl += SRS16(RollSum * (int16)RollIntFactor +128, 8);

	// subtract stick signal
	Rl -= IRoll;


	// ####################################
	// Pitch

	// Differential and Proportional for Pitch
	Pl  = SRS16(PE *(int16)PitchPropFactor + (PEp-PE) * PitchDiffFactor + 8, 4);

	if( IntegralTest )
		if( (int8)(PitchSum>>8) >  0 )
			LedYellow_ON;
		else
			if( (int8)(PitchSum>>8) < -1 )
				LedBlue_ON;

	// Integral part for Pitch
	if( IntegralCount == 0 )
		Pl += SRS16(PitchSum * (int16)PitchIntFactor +128, 8);

	// Old test for pitch limits
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

	// Differential for Yaw (for quick and stable reaction)
	Yl  = SRS16(YE *(int16)YawPropFactor + (YEp-YE) * YawDiffFactor + 8, 4);
	Yl += SRS16(YawSum * (int16)YawIntFactor +128, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);

	// ####################################
	// Camera

	// use only integral part (direct angle)
	if( (IntegralCount == 0) && (CamRollFactor != 0) && (CamPitchFactor != 0) )
	{
		Rp = RollSum / (int16)CamRollFactor;
		Pp = PitchSum / (int16)CamPitchFactor;
	}
	else
	{
		Rp = 0;
		Pp = 0;
	}
} // PID
