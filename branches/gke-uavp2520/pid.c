// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =             Ported 2008 to 18F2520 by Prof. Greg Egan               =
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

// PID compensation

#include "c-ufo.h"
#include "bits.h"

void DoIntTestLEDS(void)
{
	if( IntegralTest )
	{
		ALL_LEDS_OFF;
		if( RollAngle > 255 )
			LedRed_ON;
		else
			if( RollAngle < -256 )
				LedGreen_ON;
		if( PitchAngle >  255 )
			LedYellow_ON;
        else
			if( PitchAngle < -256 )
				LedBlue_ON;
	}
} // DoLEDS

void PID(void)
{
	// General Form of PID controller
	// xE = current error
	// xEP = previous error
	// xAngle = sum of all errors since restart!
	// xl = current correction value
	// Kz = programmable controller factors

	//       xE*Kp + xEP*Kd     Angle(xAngle)*Ki
	// xl =  --------------  +  ------------
	//             16                256

	// Roll
	Rl  = SRS32(RE*(int32) RollPropFactor+(REp-RE)*(int32) RollDiffFactor + 8, 4);	// P & D
	Rl += SRS32(RollAngle * (int32) RollIntFactor + 128, 8);						// I	
	Rl -= IRoll;

	// Pitch
	Pl  = SRS32(PE*(int32) PitchPropFactor+(PEp-PE)*(int32) PitchDiffFactor + 8, 4);
	Pl += SRS32(PitchAngle * (int32) PitchIntFactor + 128, 8);
	Pl -= IPitch;

	// Yaw
	Yl  = SRS32(YE*(int32) YawPropFactor+(YEp-YE)*(int32) YawDiffFactor + 8, 4);
	Yl += SRS32(YawAngle*(int32) YawIntFactor + 128, 8);
	Yl = Limit(Yl, -YawLimit, YawLimit);

	DoIntTestLEDS();

	REp = RE;						// remember old gyro values
	PEp = PE;
	YEp = YE;
} // PID

