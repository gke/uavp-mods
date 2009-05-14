// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =           Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer          =
// =              Copyright 2008, 2009 by Prof. Greg Egan                =
// =                            http://uavp.ch                           =
// =======================================================================
//
//    UAVP is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVP is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

// Accelerator sensor routine

#pragma codepage=2
#include "c-ufo.h"
#include "bits.h"

// Math Library
#include "mymath16.h"

// read all acceleration values from LISL sensor
// and compute correction adders (Rp, Pp, Vud)

#ifdef OPT_ADXRS
	#define GRAV_COMP 11
#else
	#define GRAV_COMP 15
#endif

void CheckLISL(void)
{
	int16 nila1@nilarg1;

	if( _UseLISL )
	{
		ReadLISL(LISL_STATUS + LISL_READ);
		// the LISL registers are in order here!!
		Rp.low8  = (int8)ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
		Rp.high8 = (int8)ReadLISLNext();
		Yp.low8  = (int8)ReadLISLNext();
		Yp.high8 = (int8)ReadLISLNext();
		Pp.low8  = (int8)ReadLISLNext();
		Pp.high8 = (int8)ReadLISLNext();
		LISL_CS = 1;	// end transmission
			
		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.
	
		// 1 unit is 1/4096 of 2g = 1/2048g
		Rp -= MiddleLR;
		Pp -= MiddleFB;
		Yp -= MiddleUD;
		
		Yp -= 1024;	// subtract 1g

		#ifdef DEBUG_SENSORS
		if( IntegralCount == 0 )
		{
			SendComValH(Rp.high8);
			SendComValH(Rp.low8);
			SendComChar(';');
			SendComValH(Pp.high8);
			SendComValH(Pp.low8);
			SendComChar(';');
			SendComValH(Yp.high8);
			SendComValH(Yp.low8);
			SendComChar(';');
		}
		#endif
	
		// DO NOT USE - Requires roll & pitch angle compensation
		#ifdef NADA
		// UDSum rises if ufo climbs
		UDSum += Yp - RollPitchComp;
	
		Yp = UDSum;
		Yp += 8;
		Yp >>= 4;
		Yp *= LinUDIntFactor;
		Yp += 128;
	
		if( (BlinkCount & 0x03) == 0 )	
		{
			if( (int8)Yp.high8 > Vud ) 
				Vud++;
			else
				if( (int8)Yp.high8 < Vud )
					Vud--;
	
			if( Vud >  10 ) // was 20
				Vud =  10; 
			else
				if( Vud < -10 ) 
					Vud = -10;
		}
		if( UDSum >  10 ) 
			UDSum -= 10;
		else
			if( UDSum < -10 ) 
				UDSum += 10;
		#endif // ACCEL_VUD
	
		// =====================================
		// Roll-Axis
		// =====================================
		// Static compensation due to Gravity
		Yl = RollSum * GRAV_COMP;	
		Yl += 16;
		Yl >>= 5;
		Rp -= Yl;
	
		// dynamic correction of moved mass
		Rp += (int16)RollSamples;
		Rp += (int16)RollSamples;
	
		// correct DC level of the integral
		LRIntKorr = 0;
		if ( Rp > 10 )
			LRIntKorr++;
		else
			if ( Rp < -10 )
				LRIntKorr--;
	
		// =====================================
		// Pitch-Axis
		// =====================================
		// Static compensation due to Gravity
		Yl = PitchSum * GRAV_COMP;
		Yl += 16;
		Yl >>= 5;
		Pp -= Yl;

		// dynamic correction of moved mass
		Pp += (int16)PitchSamples;
		Pp += (int16)PitchSamples;

		// correct DC level of the integral
		FBIntKorr = 0;
		if ( Pp > 10 )
			FBIntKorr++;
		else
			if ( Pp < -10 )
				FBIntKorr--;

	}
	else
	{
		Vud = 0;

		#ifdef DEBUG_SENSORS
		SendComChar(';');
		SendComChar(';');
		SendComChar(';');
		#endif
	}

} // CheckLISL
