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

void GetAttitude(void);
void GetNeutralAccelerations(void);
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
		};
		struct {
			int8 NeutralLR, NeutralFB, NeutralDU;
		};
		uint8 B[40];
} AttitudeRxRec;


*/

uint8 RxHead, RxTail;

AttitudeRxRec Attitude, AttitudeP;

i32u YawRateF;
int16 YawFilterA;
int16 MagneticDeviation;

int16 NavSensitivity;

void GetAttitude(void)
{
	DisableInterrupts;
	TxString("#2");

// read it into , AttitudeP

	EnableInterrupts;
} // GetAttitude

void GetNeutralAccelerations(void)
{
	static uint8 i;



} // GetNeutralAccelerations

void SetNeutralAccelerations(void)
{
	static uint8 i;

	Attitude.NeutralLR = P[MiddleLR];
	Attitude.NeutralFB = P[MiddleFB];
	Attitude.NeutralDU = P[MiddleDU];

	TxString("#3");
	TxValS(Attitude.NeutralLR);
	TxValS(Attitude.NeutralFB);
	TxValS(Attitude.NeutralDU);

} // SetNeutralAccelerations

void ErectGyros(void)
{
	static uint8 i;		
	
	TxString("#1");
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

	for ( i = 0; i < (uint8)40; i++ )
		Attitude.B[i] = 0;

	TxString("#0");

// get neutrals
	Attitude.NeutralLR = 0;
	Attitude.NeutralFB = 0;
	Attitude.NeutralDU = 0;

	// write the Neutral Acc values

	F.CompassValid = true;
	F.AccelerationsValid = true;

} // InitAttitude

void AttitudeTest(void)
{


} // AttitudeTest





