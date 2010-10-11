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

void Razor(uint8);
boolean RazorReady(void);
void GetAttitude(int16);
void SetNeutralAccelerations(void);
void ErectGyros(void);
void InitAttitude(void); // set neutrals
void AttitudeTest(void);

/*
typedef union {
	struct {
		int16 RollAngle, PitchAngle, YawAngle;
		int16 RollRate, PitchRate, YawRate; 
		int16 DUAcc, LRAcc, FBAcc;
		int16 Heading;
		int8 NeutralLR, NeutralFB, NeutralDU;
		uint8 CheckSum;
	};
	uint8 B[24];
} AttitudeRxRec;
*/

AttitudeRxRec Attitude;

i32u YawRateF;
int16 YawFilterA;
int16 MagneticDeviation;

int16 NavSensitivity;

void Razor(uint8 c)
{
	TxChar(c);
} // RazorCommand

boolean RazorReady(void)
{
	return ( RxQ.Tail >= sizeof(Attitude.B) );
} // RazorReady

void GetAttitude(int16 dt)
{
	static uint8 i;
	static int24 Timeout;

	// attitude request occured after previous output PWM generation
	Timeout = mSClock() + dt;

	while ( ( !RazorReady() ) && ( Timeout > mSClock() ) ) Delay1mS(1); // wait until data has arrived
	if ( !RazorReady() ) goto RazorError;

	RxCheckSum = 0;
	for ( i = 0; i < sizeof(Attitude.B); i++ )
		RxCheckSum ^= RxQ.B[i+1];

	if ( RxCheckSum == (uint8)0 )
	{
		F.CompassValid = F.AccelerationsValid = true;
		F.GyroFailure = false;
		//DisableInterrupts;
		for ( i = 0; i < sizeof(Attitude.B); i++ )
			Attitude.B[i] = RxQ.B[i+1];
		RxQ.Tail = 0;
		//EnableInterrupts;
	
		Attitude.Heading += MagneticDeviation;
		LPFilter16(&Attitude.YawRate, &YawRateF, YawFilterA);
		
		// scale other attitude variables
	}
	else
		goto RazorError;

	return;

RazorError:

	Razor('!'); // worth a try
	F.CompassValid = F.AccelerationsValid = false;
	F.GyroFailure = true;
	Stats[AccFailS]++;
	Stats[GyroFailS]++;
	Stats[CompassFailS]++; 

} // GetAttitude

void SetNeutralAccelerations(void)
{
	static uint8 i;

	Attitude.NeutralLR = P[MiddleLR];
	Attitude.NeutralFB = P[MiddleFB];
	Attitude.NeutralDU = P[MiddleDU];

	Razor('N');
	TxValS(Attitude.NeutralLR);
	TxValS(Attitude.NeutralFB);
	TxValS(Attitude.NeutralDU);

} // SetNeutralAccelerations

void ErectGyros(void)
{
	static uint8 i;		
	
	Razor(Restart);
	for ( i = 0; i < (uint8)5; i++)
	{
		LEDRed_TOG;
		Delay100mSWithOutput(2);
	}
	LEDRed_OFF;

} // ErectGyros

void InitAttitude(void)
{
	static uint8 i;

	RxQ.Tail = 0;

	for ( i = 0; i < sizeof(Attitude.B); i++ )
		Attitude.B[i] = 0;

	F.CompassValid = F.AccelerationsValid = false;
	F.GyroFailure = true;

	Razor('!');
	Razor('?');

} // InitAttitude

#ifdef TESTING

void AttitudeTest(void)
{
	TxString("\r\nAttitude Test\r\n");
	
	TxString("Rates\r\n");
	TxString("\tRoll: "); TxVal32((Attitude.RollRate * 180)/MILLIPI, 0, 0); TxNextLine();
	TxString("\tPitch: "); TxVal32((Attitude.PitchRate * 180)/MILLIPI, 0, 0); TxNextLine();
	TxString("\tYaw: "); TxVal32((Attitude.YawRate * 180)/MILLIPI, 0, 0); TxNextLine();

	TxString("Angles\r\n");
	TxString("\tRoll: "); TxVal32((Attitude.RollAngle * 180)/MILLIPI, 0, 0); TxNextLine();
	TxString("\tPitch: "); TxVal32((Attitude.PitchAngle * 180)/MILLIPI, 0, 0); TxNextLine();
	TxString("\tYaw: "); TxVal32((Attitude.YawAngle * 180)/MILLIPI, 0, 0); TxNextLine();

	TxString("Acc\r\n");
	TxString("\tLR: "); TxVal32(Attitude.LRAcc, 0, 0); TxNextLine();
	TxString("\tFB: "); TxVal32(Attitude.FBAcc, 0, 0); TxNextLine();
	TxString("\tDU: "); TxVal32(Attitude.DUAcc, 0, 0); TxNextLine();

	TxString("Acc Neutrals\r\n");
	TxString("\tLR: "); TxVal32(Attitude.NeutralLR, 0, 0); TxNextLine();
	TxString("\tFB: "); TxVal32(Attitude.NeutralFB, 0, 0); TxNextLine();
	TxString("\tDU: "); TxVal32(Attitude.NeutralDU, 0, 0); TxNextLine();

	TxString("Heading: "); TxVal32((Attitude.Heading * 180)/MILLIPI, 0, 0); TxNextLine();

	if ( !( F.CompassValid && F.AccelerationsValid && !F.GyroFailure) )
		TxString("FAILED\r\n");

} // AttitudeTest

#endif // TESTING



