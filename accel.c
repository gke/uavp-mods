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

// this routine is called ONLY ONCE while booting
// read 16 time all 3 axis of linear sensor.
// Puts values in Neutralxxx registers.
void GetEvenValues(void)
{	// get the even values
	uint8 i;

	Delay100mSWithOutput(2);	// wait 1/10 sec until LISL is ready to talk
	// already done in caller program
	Rp = 0;
	Pp = 0;
	Yp = 0;
	for( i=0; i < 16; i++)
	{
		ReadAccelerations();
		
		Rp += Ax;
		Pp += Az;
		Yp += Ay;
	}
	Rp = SRS16(Rp + 8, 4);
	Pp = SRS16(Pp + 8, 4);
	Yp = SRS16(Yp + 8, 4);

	NeutralLR = Limit(Rp, -99, 99);
	NeutralFB = Limit(Pp, -99, 99);
	NeutralUD = Limit(Yp-1024, -99, 99); // -1g
} // GetEvenValues

// read all acceleration values from LISL sensor
// and compute correction adders (Rp, Pp, Vud)
void CheckLISL(void)
{
	if( _UseLISL )
	{
		ReadAccelerations();

		Rp = Ax;
		Yp = Ay;
		Pp = Az;;
		
		#ifdef DEBUG_SENSORS
		if( IntegralCount == 0 )
		{
			TxValH16(Rp);
			TxChar(';');
			TxValH16(Pp);
			TxChar(';');
			TxValH16(Yp);
			TxChar(';');
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
	
			Vud = Limit(Vud, -10, 10); // was 20
		}
	
		UDSum = DecayBand(UDSum, -10, 10, 10);
	
		#endif // ACCEL_VUD
	
		// Roll

		// Static compensation due to Gravity
		#ifdef OPT_ADXRS
		Rp -= SRS16(RollSum * 11 + 16, 5);
		#else // OPT_IDG
		Rp -= SRS16(RollSum * (-15) + 16, 5); 
		#endif
	
		// dynamic correction of moved mass - turn coordination
		#ifdef OPT_ADXRS
		Rp += (int16)RollSamples << 1;
		#else // OPT_IDG
		Rp -= (int16)RollSamples;
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

		// Static compensation due to Gravity
		#ifdef OPT_ADXRS
		Pp -= SRS16(PitchSum * 11 + 16, 5);	
		#else // OPT_IDG
		Pp -= SRS16(PitchSum * (-15) + 16, 5);
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
	}	
	else
	{
		Vud = 0;

		#ifdef DEBUG_SENSORS
		TxValH16(-1);
		TxChar(';');
		TxValH16(-1);
		TxChar(';');
		TxValH16(-1);
		TxChar(';');
		#endif
	}	
} // CheckLISL

