// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "uavx.h"

void GetGyroValues(void);
boolean RazorReady(void);
void RazorTest(void);
void InitRazor(void);
void GetNeutralAccelerations(void);
void AttitudeTest(void);

void CompensateRollPitchGyros(void);

/*
typedef union {
	struct {
		uint8 Sentinel;
		int16 Roll, Pitch, Yaw;
		int16 Heading;
		uint8 CheckSum;
	};
	uint8 B[25];
} RazorRec;
*/

RazorRec Razor;

int16 LRAcc, FBAcc, DUAcc;
int8 NeutralLR, NeutralFB, NeutralDU;
int16 Roll, Pitch, Yaw;

int16 GyroMidRoll, GyroMidPitch, GyroMidYaw;
int16 RollRate, PitchRate, YawRate;
i32u YawRateF;
int16 RollRateADC, PitchRateADC, YawRateADC;

int16 DUVel, LRVel, FBVel, DUAcc, LRAcc, FBAcc, DUComp, LRComp, FBComp;
int8 LRIntCorr, FBIntCorr;

i32u YawRateF;

int16 MagneticDeviation;

boolean RazorReady(void)
{
	static boolean b;

	DisableInterrupts;
		b = RxQ.Entries >= sizeof(Razor.B);
	EnableInterrupts;
	
	return ( b );
} // RazorReady

void GetGyroValues(void)
{
	static uint8 i, b;

	if ( F.PacketReceived )
	{
		F.PacketReceived = false;
		mS[RazorUpdate] = mSClock() + 20;
		F.CompassValid = F.AccelerationsValid = true;
		F.GyroFailure = false;
	
		RollSum = -Razor.Roll;
		PitchSum = -Razor.Pitch;
		YawSum = Razor.Yaw;

		RollRate = -Razor.RollRate/48; // (5/3.3) * (0.01657*1000) * 2 (~48)
		PitchRate = -Razor.PitchRate/48;
		YawRate = Razor.YawRate/24;

		LRAcc = Razor.LRAcc; // MilliG - close enough
		FBAcc = -Razor.BFAcc;
		DUAcc = -Razor.UDAcc;

		Heading = Make2Pi(Razor.Heading + MagneticDeviation);
	}
	else
		if ( mSClock() > mS[RazorUpdate] )
		{
			mS[RazorUpdate] = mSClock() + 30;
			F.CompassValid = F.AccelerationsValid = false;
			F.GyroFailure = true;
			Stats[AccFailS]++;
			Stats[GyroFailS]++;
			Stats[CompassFailS]++; 
		}

} // GetRazor

#ifdef TESTING

void AttitudeTest(void)
{
	TxString("\r\nAttitude Test\r\n");
	
	TxString("Angles\r\n");
	TxString("\tRoll: "); TxVal32((Razor.Roll * 180)/MILLIPI, 0, 0); TxNextLine();
	TxString("\tPitch: "); TxVal32((Razor.Pitch * 180)/MILLIPI, 0, 0); TxNextLine();
	TxString("\tYaw: "); TxVal32((Razor.Yaw * 180)/MILLIPI, 0, 0); TxNextLine();

	TxString("Heading: "); TxVal32((Razor.Heading * 180)/MILLIPI, 0, 0); TxNextLine();

	if ( !( F.CompassValid && F.AccelerationsValid && !F.GyroFailure) )
		TxString("FAILED\r\n");

} // AttitudeTest

#endif // TESTING

void GetNeutralAccelerations(void)
{
	NeutralLR = NeutralFB = NeutralDU = 0;
	LRAcc = FBAcc = DUAcc = 0;
} // GetNeutralAccelerations

void InitGyros(void)
{
	static uint8 i;

	RxQ.Tail = 0;

	for ( i = 0; i < sizeof(Razor.B); i++ )
		Razor.B[i] = 0;

	F.CompassValid = F.AccelerationsValid = false;
	F.GyroFailure = true;

	NeutralLR = NeutralFB = NeutralDU = 0;
	RollRate = PitchRate = YawRate = Roll = Pitch = Yaw = RollSum = PitchSum = YawSum = 0;
} // InitRazor

// ----------------------------------------

void InitHeading(void)
{

} // InitHeading

void GetHeading(void)
{


} // GetHeading

void InitCompass(void)
{

} // InitCompass

void CompensateRollPitchGyros(void)
{

} // CompensateRollPitchGyros

void CalculateGyroRates(void)
{

} // CalculateGyroRates

void ErectGyros(void)
{

} // ErectGyros

void InitAccelerometers(void)
{
	NeutralLR = NeutralFB = NeutralDU = 0;
	LRAcc = FBAcc = DUAcc = 0;
} //InitAccelerometers

void ShowGyroType(uint8 g)
{
	TxString("SparkFun Razor 9DOF\r\n");
} // ShowGyroType

