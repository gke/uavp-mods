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

	ReadLISL(LISL_STATUS + LISL_READ);
	// the LISL registers are in order here!!
	Rp  = ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
	Rp |= ReadLISLNext()<<8;
	Yp  = ReadLISLNext();
	Yp |= ReadLISLNext()<<8;
	Pp  = ReadLISLNext();
	Pp |= ReadLISLNext()<<8;
	LISL_CS = 1;	// end transmission
	
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

		if( Vud >  20 ) 
			Vud =  20;
		else
			if( Vud < -20 ) 
				Vud = -20;
	}

	UDSum = DecayBand(UDSum, -10, 10, 10);

	#endif // ACCEL_VUD

	// =====================================
	// Roll-Achse
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
	Yl = PitchSum * 11;	// Pp um RollSum* 11/32 korrigieren
	#endif
	#ifdef OPT_IDG
	Yl = PitchSum * -15;	// Pp um RollSum* -14/32 korrigieren
	#endif
	Yl += 16;
	Yl >>= 5;

	Pp -= Yl;
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
} // CheckLISL

