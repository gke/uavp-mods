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

#include "c-ufo.h"
#include "bits.h"

// read all acceleration values from LISL sensor
// and compute correction adders (Rp, Pp, Vud)
void CheckLISL(void)
{
	int16 nila1;

	if( _UseLISL )
	{
		while( (ReadLISL(LISL_STATUS+LISL_READ) & 0x08) == 0 ); //wait until ready
		// 0.903mS
		Rp = (ReadLISL(LISL_OUTX_H|LISL_READ)*256)|ReadLISL(LISL_OUTX_L|LISL_READ);
		Yp = (ReadLISL(LISL_OUTY_H|LISL_READ)*256)|ReadLISL(LISL_OUTY_L|LISL_READ);
		Pp = (ReadLISL(LISL_OUTZ_H|LISL_READ)*256)|ReadLISL(LISL_OUTZ_L|LISL_READ);
		LISL_IO = 1; // read
		
		#ifdef DEBUG_SENSORS
		if( IntegralCount == 0 )
		{
			SendComValH16(Rp);
			SendComChar(';');
			SendComValH16(Pp);
			SendComChar(';');
			SendComValH16(Yp);
			SendComChar(';');
		}
		#endif
	
		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.
	
		// 1 unit is 1/4096 of 2g = 1/2048g
		Rp -= MiddleLR;
		Pp -= MiddleFB;
		Yp -= MiddleUD;
	
		Yp -= 1024;	// subtract 1g
	
		#ifdef ACCEL_VUD
		// UDSum rises if ufo climbs
		UDSum += Yp;
	
		Yp = SRS16(SRS16(UDSum + 8, 4) * (int16) LinUDIntFactor + 128, 8);
	
		if( (BlinkCount & 0x03) == 0 )	
		{
			if( Yp > Vud ) 
				Vud++;
			else
				if( Yp < Vud )
					Vud--;
	
			Vud = Limit(Vud, -20, 20);
		}
	
		UDSum = DecayBand(UDSum, -10, 10, 10);
	
		#endif // ACCEL_VUD
	
		// =====================================
		// Roll-Achse
		// =====================================
		// Static compensation due to Gravity
	
		#ifdef OPT_ADXRS
		Rp -= SRS16(RollSum * 11 + 16, 5);	// Rp um RollSum*11/32 korrigieren
		#endif
	
		#ifdef OPT_IDG
		Rp -= SRS16(RollSum * (-15) + 16, 5); // Rp um RollSum* -15/32 korrigieren
		#endif
	
		// dynamic correction of moved mass
		#ifdef OPT_ADXRS
		Rp += (int16)RollSamples;
		Rp += (int16)RollSamples;
		#endif
	
		#ifdef OPT_IDG
		Rp -= (int16)RollSamples;
		#endif
	
		// correct DC level of the integral
		LRIntKorr = 0;
		#ifdef OPT_ADXRS
		if( Rp > 10 ) LRIntKorr =  1;
		else
			if( Rp < 10 ) LRIntKorr = -1;
		#endif
		#ifdef OPT_IDG
		if( Rp > 10 ) LRIntKorr = -1;
		else
			if( Rp < 10 ) LRIntKorr =  1;
		#endif
	
		Rp = 0;
	
		// =====================================
		// Pitch-Achse
		// =====================================
		// Static compensation due to Gravity
	
		#ifdef OPT_ADXRS
		Pp -= SRS16(PitchSum * 11 + 16, 5);	// Pp um RollSum* 11/32 korrigieren
		#endif
		#ifdef OPT_IDG
		Pp -= SRS16(PitchSum * (-15) + 16, 5);	// Pp um RollSum* -14/32 korrigieren
		#endif
	
		// no dynamic correction of moved mass necessary
	
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
	
		Pp = 0;
	}	
	else
	{
		#ifdef DEBUG_SENSORS
		SendComValH16(0);
		SendComChar(';');
		SendComValH16(0);
		SendComChar(';');
		SendComValH16(0);
		SendComChar(';');
		#endif
	}	
} // CheckLISL

