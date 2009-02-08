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
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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

// Utilities and subroutines

#pragma codepage=1
#include "c-ufo.h"
#include "bits.h"

// Math Library
#include "mymath16.h"


// Limit integral sum of Roll gyros
// it must be limited to avoid numeric overflow
// which would cause a serious flip -> crash
void LimitRollSum(void)
{

	RollSum += (long)RollSamples;

	if( IntegralCount == 0 )
	{
		NegFact = -RollIntLimit;

		if( (int)RollSum.high8 >= RollIntLimit )
		{
			RollSum.high8 = RollIntLimit;
			RollSum.low8 = 0;
		}
		else
		if( (int)RollSum.high8 < NegFact )
		{
			RollSum.high8 = NegFact;
			RollSum.low8 = 0;
		}
		RollSum += LRIntKorr;
		if( RollSum > 0 ) RollSum--;
		if( RollSum < 0 ) RollSum++;

	}

}

// Limit integral sum of Nick gyros
// it must be limited to avoid numeric overflow
// which would cause a serious flip -> crash
void LimitNickSum(void)
{

	NickSum += (long)NickSamples;

	if( IntegralCount == 0 )
	{
		NegFact = -NickIntLimit;
		if( (int)NickSum.high8 >= NickIntLimit )
		{
			NickSum.high8 = NickIntLimit;
			NickSum.low8 = 0;
		}
		else
		if( (int)NickSum.high8 < NegFact )
		{
			NickSum.high8 = NegFact;
			NickSum.low8 = 0;
		}
		NickSum += FBIntKorr;
		if( NickSum > 0 ) NickSum--;
		if( NickSum < 0 ) NickSum++;
#ifdef NADA
SendComValH(NickSamples.high8);
SendComValH(NickSamples.low8);
SendComValH(NickSum.high8);
SendComValH(NickSum.low8);
SendComValH(MidNick.high8);
SendComValH(MidNick.low8);
SendComChar(0x0d);SendComChar(0x0a);
#endif
	}
}

// Limit integral sum of Yaw gyros
// it must be limited to avoid numeric overflow
// which would cause a uncontrolled yawing -> crash
void LimitYawSum(void)
{

// add the yaw stick value
	TE += ITurn;

	if ( _UseCompass )
	{
		// add compass heading correction
		// CurDeviation is negative if Ufo has yawed to the right (go back left)

		// this double "if" is necessary because of dumb CC5X compiler
		NegFact = YawNeutral + COMPASS_MIDDLE;
		if ( ITurn > NegFact )
			// yaw stick is not in neutral zone, learn new desired heading
			AbsDirection = COMPASS_INVAL;
		else		
		{
			NegFact = YawNeutral - COMPASS_MIDDLE;
			if ( ITurn < NegFact )
				// yaw stick is not in neutral zone, learn new desired heading
				AbsDirection = COMPASS_INVAL;
			else
			{
				// yaw stick is in neutral zone, hold heading
				if( CurDeviation > COMPASS_MAXDEV )
					TE -= COMPASS_MAXDEV;
				else
				if( CurDeviation < -COMPASS_MAXDEV )
					TE += COMPASS_MAXDEV;
				else
					TE -= CurDeviation;
			}
		}
	}

	YawSum += (long)TE;
	NegFact = -YawIntLimit;
	if( (int)YawSum.high8 >= YawIntLimit )
	{
		YawSum.high8 = YawIntLimit;
		YawSum.low8 = 0;
	}
 	if( (int)YawSum.high8 < NegFact )
	{
		YawSum.high8 = NegFact;
		YawSum.low8 = 0;
	}
}

// to avoid stopping motors in the air, the
// motor values are limited to a minimum and
// a maximum
// the eventually corrected value is returned
int SaturInt(long l)
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
	return((int)l);
}

// mix the PID-results (Rl, Nl and Tl) and the throttle
// on the motors and check for numerical overrun
void MixAndLimit(void)
{
	uns8 CurrGas;

	CurrGas = IGas;	// to protect against IGas being changed in interrupt
 
#ifndef TRICOPTER
	if( FlyCrossMode )
	{	// "Cross" Mode
		Ml = CurrGas + Nl; Ml -= Rl;
		Mr = CurrGas - Nl; Mr += Rl;
		Mv = CurrGas - Nl; Mv -= Rl;
		Mh = CurrGas + Nl; Mh += Rl;
	}
	else
	{	// "Plus" Mode
#ifdef MOUNT_45
		Ml = CurrGas - Rl; Ml -= Nl;	// K2 -> Front right
		Mr = CurrGas + Rl; Mr += Nl;	// K3 -> Rear left
		Mv = CurrGas + Rl; Mv -= Nl;	// K1 -> Front left
		Mh = IGas - Rl; Mh += Nl;	// K4 -> Rear rigt
#else
		Ml = CurrGas - Rl;	// K2 -> Front right
		Mr = CurrGas + Rl;	// K3 -> Rear left
		Mv = CurrGas - Nl;	// K1 -> Front left
		Mh = CurrGas + Nl;	// K4 -> Rear rigt
#endif
	}

	Mv += Tl;
	Mh += Tl;
	Ml -= Tl;
	Mr -= Tl;

	// Altitude stabilization factor
	Mv += Vud;
	Mh += Vud;
	Ml += Vud;
	Mr += Vud;

	Mv += VBaroComp;
	Mh += VBaroComp;
	Ml += VBaroComp;
	Mr += VBaroComp;

// if low-throttle limiting occurs, must limit other motor too
// to prevent flips!

	if( CurrGas > MotorLowRun )
	{
		if( (Mv > Mh) && (Mh < MotorLowRun) )
		{
			NegFact = Mh - MotorLowRun;
			Mv += NegFact;
			Ml += NegFact;
			Mr += NegFact;
		}
		if( (Mh > Mv) && (Mv < MotorLowRun) )
		{
			NegFact = Mv - MotorLowRun;
			Mh += NegFact;
			Ml += NegFact;
			Mr += NegFact;
		}
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			NegFact = Mr - MotorLowRun;
			Ml += NegFact;
			Mv += NegFact;
			Mh += NegFact;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{	
			NegFact = Ml - MotorLowRun;
			Mr += NegFact;
			Mv += NegFact;
			Mh += NegFact;
		}
	}
#else	// TRICOPTER
	Mv = CurrGas + Nl;	// front motor
	Ml = CurrGas + Rl;
	Mr = CurrGas - Rl;
	Rl >>= 1;
	Ml -= Rl;	// rear left
    	Mr -= Nl;	// rear right
	Mh = Tl + _Neutral;	// yaw servo

	if( CurrGas > MotorLowRun )
	{
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			// Mv += Mh - MotorLowRun
			Ml += Mr;
			Ml -= MotorLowRun;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{
			// Mh += Mv - MotorLowRun
			Mr += Ml;
			Mr -= MotorLowRun;
		}
	}
#endif


// Ergebnisse auf Überlauf testen und korrigieren

	MVorne = SaturInt(Mv);
	MLinks = SaturInt(Ml);
	MRechts = SaturInt(Mr);
	MHinten = SaturInt(Mh);
}

