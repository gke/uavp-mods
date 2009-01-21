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

// Control routines

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

void InitGyros(void)
{ // quadrocopter MUST be stationary and level
	uint16 s;

    RollAngle = PitchAngle = YawAngle = 0;
	for ( s = 256; s ; s-- )
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

	MidRoll = SRS32(RollAngle + 128, 8);								
	MidPitch = SRS32(PitchAngle + 128, 8);
	MidYaw = SRS32(YawAngle + 128, 8);		
	
	#ifdef OPT_ADXRS150
	MidRoll = SRS32(MidRoll + 1, 1);	
	MidPitch = SRS32(MidPitch + 1, 1);
	#endif

	RollGyroRate = MidRoll;
	PitchGyroRate = MidPitch;
	YawGyroRate = MidYaw;

	RollAngle = PitchAngle = YawAngle = 0;
	REp = PEp = YEp = 0;

} // InitGyros

void InitAccelerometers(void)
{	// quadrocopter MUST be stationary and level
	uint16 s;
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
	int16 Rc, Pc, Uc, Temp;

	ReadLISLXYZ();		
	Rc = ACCSIGN_X * Ax - NeutralLR;
	Uc = ACCSIGN_Y * Ay - NeutralUD;	
	Pc = ACCSIGN_Z * Az - NeutralFB;

	#ifdef DRIFT
	TxVal((int32)Pc, 0, ';');
	Temp = Rc;
	#endif
								
	// Roll
	Rc = -SRS32(Rc * 3 + 1, 1);					// empirical 1.5					
	Rc -= RollAngle;	
	Rc += RollRate;								// turn coordination ???

	if( Rc > 10 ) 
		RollAngle += 4;
	else
		if( Rc < -10 ) 
			RollAngle -= 4;
	
	RollAngle = Decay(RollAngle);

	// Pitch									
	Pc = SRS16(Pc * 3 + 1,1); 
	Pc -= PitchAngle;
								
	if( Pc > 10 ) 
		PitchAngle += 4;
	else
		if( Pc < -10 ) 
			PitchAngle -= 4;

	PitchAngle = Decay(PitchAngle);

	#ifdef DRIFT
	TxVal((int32)PitchAngle, 0, ';');
	TxVal((int32)PitchRate, 0, ';');

	TxVal((int32)Temp, 0, ';');
	TxVal((int32)RollAngle, 0, ';');
	TxVal((int32)RollRate, 0, ';');

	TxNextLine();
	#endif

	/*
	// "velocity" increases as quadrocopter falls
	UDVelocity += Uc;

	Temp = SRS32(SRS32(UDVelocity + 8, 4) * LinUDIntFactor + 128, 8);

	if( (CycleCount & 0x00000003) == 0 )		// ?????
	{
		if( Temp > Vud )
			Vud++;
		else
			if( Temp < Vud )
				Vud--;
		Vud = Limit(Vud, -20, 20);
	}
	UDVelocity = DecayBand(UDVelocity, -10, 10, 10);
	*/

} // CompensateGyros

void DetermineAttitude(void)
{
	int16 Temp;

	RollGyroRate = GyroFilter(RollGyroRate, GetRollRate());	
	PitchGyroRate = GyroFilter(PitchGyroRate, GetPitchRate());
	YawGyroRate = GyroFilter(YawGyroRate, GetYawRate());

	#ifdef OPT_ADXRS150
	// throw away 1 bit of precision ???
	RollGyroRate = SRS16(RollGyroRate + 1, 1);
	PitchGyroRate =  SRS16(PitchGyroRate + 1, 1);
	#endif

	RollRate = RollGyroRate - MidRoll;
	PitchRate = PitchGyroRate - MidPitch;
/*
	if( FlyCrossMode )								// does this work with compensation NO ??? 
	{
		// Real Roll = 0.707 * (P + R), Pitch = 0.707 * (P - R)
		Temp = ((RollRate + PitchRate) * 7 + 5)/10 ;	
		PitchRate = ((PitchRate - RollRate) * 7 + 5)/10;
		RollRate = Temp;	
	}
*/	
	// Roll is + right
	RE=SRS16(RollRate + 1, 1);						// use 9 bit res. for PD control
	RollAngle += (int32)RollRate;					// use 10 bit res. for I control							
	RollAngle = Limit(RollAngle, -(RollIntLimit*256), RollIntLimit*256);

	// Pitch is + up
	PE = SRS16(PitchRate + 1, 1);
	PitchAngle += (int32)PitchRate;
	PitchAngle = Limit(PitchAngle, -(PitchIntLimit*256), PitchIntLimit*256);
 
	// Yaw + CW 	
//	Temp = YawGyroRate - MidYaw;
//	if ( Abs(Temp) <= 2 )							// needs further thought ???
//		MidYaw = Limit(YawGyroRate, (YawGyroRate - 5), YawGyroRate + 5);

	YawRate = YawGyroRate - MidYaw;
	YE = SRS16(YawRate + 2, 2);

	DoHeadingLock();

	YawAngle = Limit(YawAngle + YE, -(YawIntLimit*256), YawIntLimit*256);
	YawAngle = Decay(YawAngle);

	CompensateGyros();

} // DetermineAttitude

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

void InitAttitude(void)
{
	// DON'T MOVE THE UFO!
	// ES KANN LOSGEHEN!
	LedRed_ON;
	Delay100mSec(100);							// ~10Sec. to get hands away after power up
	Beeper_ON;
	InitDirection();		
	InitAltimeter();
	InitAccelerometers();
	InitGyros();
	Beeper_OFF;					
	LedRed_OFF;
} // InitAttitude

void DoControl()
{
	if( _NewValues )
		CheckThrottleMoved();

	GetDirection();
	GetAltitude();				
	DetermineAttitude();

	PID();

} // DoControl

