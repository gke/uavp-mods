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

// Gyros & Accelerometers

#include "c-ufo.h"
#include "bits.h"
#include <adc.h>

// IDG300
// 3.3V Reference +-500 Deg/Sec
// 0.4882815 Deg/Sec/LSB 

// ADRSX300 Yaw
// 5V Reference +-300 Deg/Sec
// 0.2926875 Deg/Sec/LSB 

// ADXRS150
// 5V Reference +-150 Deg/Sec
// 0.146484375 Deg/Sec/LSB

int16 GetRollRate(void)
{	// + roll right
	return(GYROSIGN_ROLL*ADC(ADCRollChan, ADCEXTVREF_PITCHROLL));
} // GetRollRate

int16 GetPitchRate(void)
{	// + pitch up
	return(GYROSIGN_PITCH*ADC(ADCPitchChan, ADCEXTVREF_PITCHROLL));
}  // GetPitchRate

int16 GetYawRate(void)
{	// + yaw CW
	return(GYROSIGN_YAW * ADC(ADCYawChan, ADCEXTVREF_YAW));		
} // GetYawRate

void InitAttitude(void)
{
	int16 i;

    RollAngle = PitchAngle = YawAngle = 0;
	for ( i = 256; i ; i-- )
		{		
		RollAngle += GetRollRate();
		PitchAngle += GetPitchRate();
		YawAngle += GetYawRate();
		}

	if ( !_UseLISL )
	{
		RollAngle += MiddleLR;
		PitchAngle += MiddleFB;
	}
	#ifdef OPT_ADXRS150
	MidRoll = SRS32(RollAngle + 256, 9);	
	MidPitch = SRS32(PitchAngle + 256, 9);
	#else // IDG300
	MidRoll = SRS32(RollAngle + 128, 8);								
	MidPitch = SRS32(PitchAngle + 128, 8);		
	#endif

	MidYaw = SRS32(YawAngle + 128, 8);
	PrevYawRate = MidYaw;

	RollAngle = PitchAngle = YawAngle = 0;
	REp = PEp = YEp = 0;

} // InitAttitude

void InitAccelerometers(void)
{	// get initial values with quadrocopter level and 
	// stationary on the ground
	uint8 s;
	int32 Tx, Ty, Tz;

	IsLISLactive();

	Tx = Ty = Tz = 0;
	for( s=256; s ; s--)
	{
		ReadLISLXYZ();
		Tx += ACCSIGN_X * Ax;
		Ty += ACCSIGN_Y * Ay;
		Tz += ACCSIGN_Z * Az;
		// delay in here ???
	}
	NeutralLR = SRS32(Tx + 128, 8);
	NeutralUD = SRS32(Ty + 128, 8);
	NeutralFB = SRS32(Tz + 128, 8);
	UDVelocity = 0;
} // InitAcelerometers

void CompensateGyros(void)
{	// + Ax right, + Ay up, + Az back
	int32	Rp, Pp, Up, Temp;

	ReadLISLXYZ();		
	Rp = ACCSIGN_X * Ax - NeutralLR;
	Up = ACCSIGN_Y * Ay - NeutralUD;	
	Pp = ACCSIGN_Z * Az - NeutralFB;
							
	// Roll
	#ifdef OPT_ADXRS	
	Rp -= SRS32(RollAngle * 11 + 16, 5);		// empirical ???
	#else // OPT_IDG	
	Rp -= SRS32(RollAngle * 15 + 16, 5);
	#endif

	// dynamic correction of moved mass  - for coordinated turns ???
	#ifdef OPT_ADXRS
//	Rp += (int32)RollRate * 2;					// ~500/300 deg/sec ???
	#else // OPT_IDG
//	Rp += (int32)RollRate;
	#endif

	// correct DC level of the integral (Roll Angle)
	LRIntKorr = 0;
	if( Rp > 10 ) 
		LRIntKorr =  -1;
	else
		if( Rp < -10 ) 
			LRIntKorr = 1;

	// Nick									
	Pp = -Pp;									// accelerometer opp sense.

	#ifdef OPT_ADXRS	
	Pp += SRS32(PitchAngle * 11 + 16, 5);	
	#else // OPT_IDG	
	Pp += SRS32(PitchAngle * 15 + 16, 5);
	#endif

	// no dynamic correction of moved mass necessary

	// correct DC level of the integral (Pitch angle)
	FBIntKorr = 0;
	if( Pp > 10 ) 
		FBIntKorr =  -1;
	else
		if( Pp < -10 ) 
			FBIntKorr = 1;

	// Vertical - reinstated

	// UDAngle increases as quadrocopter falls
	UDVelocity += Up;

	Temp = SRS32(SRS32(UDVelocity + 8, 4) * LinUDIntFactor + 128, 8);

	if( (BlinkCount & 0x03) == 0 )	
	{
		if( Temp > Vud )
			Vud++;
		else
			if( Temp < Vud )
				Vud--;
		Vud = Limit(Vud, -20, 20);
	}
	UDVelocity = DecayBand(UDVelocity, -10, 10, 10);

} // CompensateGyros

void DoHeadingLock(void)
{
	int16 Temp, CurrIYaw;

	CurrIYaw = IYaw;							// protect from change by irq
	YE += CurrIYaw;								// add the yaw stick value

	if ( _UseCompass )
		if ((CurrIYaw < (YawFailsafe - COMPASS_MIDDLE)) || (CurrIYaw > (YawFailsafe + COMPASS_MIDDLE)))
			AbsDirection = COMPASS_INVAL;		// new hold zone
		else
			YE += Limit(CurrDeviation, -COMPASS_MAXDEV, COMPASS_MAXDEV);

	if( CompassTest )
	{
		ALL_LEDS_OFF;
		if( CurrDeviation > 0 )
			LedGreen_ON;
		else
			if( CurrDeviation < 0 )
				LedRed_ON;
		if( AbsDirection > COMPASS_MAX )
			LedYellow_ON;
	}
} // DoHeadingLock

void DetermineAttitude(void)
{
	int16 Temp;

	RollRate = GetRollRate();			// first of two samples per cycle
	PitchRate = GetPitchRate();

	YawRate = GetYawRate();

	// second gyro sample delayed roughly by intervening routines!
	// no obvious reason for this except minor filtering by averaging.
	RollRate += GetRollRate();
	PitchRate += GetPitchRate();

	#ifdef OPT_ADXRS150
	RollRate = SRS16(RollRate + 2, 2)
	PitchRate =  SRS16(PitchRate + 2, 2);
	#else // IDG300 and ADXRS300
	RollRate = SRS16(RollRate + 1, 1);
	PitchRate =  SRS16(PitchRate + 1, 1);
	#endif

	RollRate -= MidRoll;
	PitchRate -= MidPitch;

	if( FlyCrossMode )
	{
		// Real Roll = 0.707 * (P + R), Pitch = 0.707 * (P - R)
		Temp = ((RollRate + PitchRate) * 7 + 5)/10 ;	
		PitchRate = ((PitchRate - RollRate) * 7 + 5)/10;
		RollRate = Temp;	
	}

	// Lots of ad hoc scaling - leave for now ???
	
	// Roll is + right
	#ifdef OPT_ADXRS
	RE = SRS16(RollRate + 2, 2);					// use 8 bit res. for PD control
	RollRate = SRS16(RollRate + 1, 1);				// use 9 bit res. for I control
	#else // OPT_IDG
	RE=SRS16(RollRate + 1, 1);
	#endif // OPT_ADXRS

	RollAngle += (int32)RollRate;
	RollAngle = Limit(RollAngle, -(RollIntLimit*256), RollIntLimit*256);							
	RollAngle = Decay(RollAngle + LRIntKorr);

	// Pitch is + up
	#ifdef OPT_ADXRS
	PE = SRS16(PitchRate + 2, 2);
	PitchRate = SRS16(PitchRate + 1, 1);
	#else // OPT_IDG
	PE = SRS16(PitchRate + 1, 1);
	#endif // OPT_ADXRS

	PitchAngle += (int32)PitchRate;
	PitchAngle = Limit(PitchAngle, -(PitchIntLimit*256), PitchIntLimit*256);
	PitchAngle = Decay(PitchAngle + FBIntKorr);		// ??? Decay(PitchAngle) + FBIntKorr;
 
	// Yaw + CW  sample once per cycle
	YawRate = MediumFilter(PrevYawRate, YawRate);
	PrevYawRate = YawRate;	
	
	Temp = YawRate - MidYaw;
	if ( Abs(Temp) <= 2 )							// needs further thought
		MidYaw = Limit(YawRate, -500, 500);

	YawRate -= MidYaw;

	YE = SRS16(YawRate + 2, 2);
	DoHeadingLock();
	YawAngle = Limit(YawAngle + YE, -(YawIntLimit*256), YawIntLimit*256);
	YawAngle = Decay(YawAngle);

	CompensateGyros();

} // DetermineAttitude

