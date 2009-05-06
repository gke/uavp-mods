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

// Accelerator sensor routine

#pragma codepage=2
#include "c-ufo.h"
#include "bits.h"

// Math Library
#include "mymath16.h"

// read all acceleration values from LISL sensor
// and compute correction adders (Rp, Pp, Vud)
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
	
		#if 0
		// calc the angles for roll matrices
		// rw = arctan( x*16/z/4 )
		niltemp = Yp * 16;
		niltemp1 = niltemp / Rp;
		niltemp1 >>= 2;
		nila1 = niltemp1;
		Rw = Arctan();
		
		SendComValS(Rw);
		SendComChar(';');
		
		niltemp1 = niltemp / Pp;
		niltemp1 >>= 2;
		nila1 = niltemp1;
		Pw = Arctan();
		
		SendComValS(Pw);
		SendComChar(0x13);
		SendComChar(0x10);
	
		#endif
		
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
	
		#ifdef OPT_ADXRS
		Yl = RollSum * 11;	// Rp um RollSum*11/32 korrigieren
		#endif
	
		#ifdef OPT_IDG
		Yl = RollSum * -15; // Rp um RollSum* -15/32 korrigieren
		#endif
		Yl += 16;
		Yl >>= 5;
		Rp -= Yl;
	
		// dynamic correction of moved mass
		#ifdef OPT_ADXRS
		Rp += (int16)RollSamples;
		Rp += (int16)RollSamples;
		#else  // OPT_IDG
		Rp -= (int16)RollSamples;
		#endif
	
		// correct DC level of the integral
		#ifdef OPT_ADXRS
		LRIntKorr =  Rp/10;
		#else // OPT_IDG
		LRIntKorr =  -Rp/10;
		#endif
		//LRIntKorr = Limit(LRIntKorr, -1, 1);
		if ( LRIntKorr > 5 )
			LRIntKorr = 5;
		else
			if  ( LRIntKorr < -5 )
				LRIntKorr = -5;
	
		// =====================================
		// Pitch-Axis
		// =====================================
		// Static compensation due to Gravity

		Pp =-Pp; // long standing error???
	
		#ifdef OPT_ADXRS
		Yl = PitchSum * 11;	// Pp um RollSum* 11/32 korrigieren
		#else // OPT_IDG
		Yl = PitchSum * -15;	// Pp um RollSum* -14/32 korrigieren
		#endif
		Yl += 16;
		Yl >>= 5;
	
		Pp -= Yl;
		// no dynamic correction of moved mass necessary
	
		// correct DC level of the integral
		#ifdef OPT_ADXRS
		FBIntKorr =  Pp/10; 
		#else // OPT_IDG
		FBIntKorr = -Pp/10;
		#endif
		//FBIntKorr = Limit(FBIntKorr, -1, 1);
		if ( FBIntKorr > 5 )
			FBIntKorr = 5;
		else
			if  ( FBIntKorr < -5 )
				FBIntKorr = -5;
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
