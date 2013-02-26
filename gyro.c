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

// Gyros

#include "uavx.h"

void ShowGyroType(uint8);
void GetGyroValues(void);
void CalculateGyroRates(void);
void CheckGyroFault(uint8, uint8, uint8);
void ErectGyros(uint16);
void InitGyros(void);

uint8 GyroType;
int16 RawGyro[3];
int32 NoAccCorr;

#include "gyro_i2c.h"
#include "gyro_analog.h"

#pragma idata gyronames
const rom uint8 * GyroName[GyroUnknown+1] ={
		 "MPU6050", "MLX90609", "ADXRS613/150",
		"ST-AY530", "ADXRS610/300", "MPU6050", "MPU6050", "ITG3200", 
		"Unsupported", "Failed" };
#pragma idata

void ShowGyroType(uint8 G)
{
	TxString(GyroName[G]);
} // ShowGyroType

void CalculateGyroRates(void)
{
	static i24u RollT, PitchT, YawT;

	A[Roll].Rate = A[Roll].GyroADC - A[Roll].GyroBias;
	A[Pitch].Rate = A[Pitch].GyroADC - A[Pitch].GyroBias;
	A[Yaw].Rate = A[Yaw].GyroADC - A[Yaw].GyroBias;

	switch ( GyroType ) {
 	case LY530Gyro:// REFERENCE generically 300deg/S 3.3V
		RollT.i24 = (int24)A[Roll].Rate * 256;
		PitchT.i24 = (int24)A[Pitch].Rate * 256;
		YawT.i24 = (int24)A[Yaw].Rate * 128;
		break;
	case MLX90609Gyro:// generically 300deg/S 5V
		RollT.i24 = (int24)A[Roll].Rate * 127;
		PitchT.i24 = (int24)A[Pitch].Rate * 127;
		YawT.i24 = (int24)A[Yaw].Rate * 63;
		break;
	case ADXRS300Gyro:// ADXRS610/300 300deg/S 5V
		RollT.i24 = (int24)A[Roll].Rate * 169;
		PitchT.i24 = (int24)A[Pitch].Rate * 169;
		YawT.i24 = (int24)A[Yaw].Rate * 84;
		break;
	case ADXRS150Gyro:// ADXRS613/150 or generically 150deg/S 5V
		RollT.i24 = (int24)A[Roll].Rate * 68;
		PitchT.i24 = (int24)A[Pitch].Rate * 68;
		YawT.i24 = (int24)A[Yaw].Rate * 34;
		break;
	case FreeIMU:
	case GY86IMU:
	case DrotekIMU:
	case UAVXArm32IMU:
		RollT.i24 = (int24)A[Roll].Rate * 11; // 18
		PitchT.i24 = (int24)A[Pitch].Rate * 11;
		YawT.i24 = (int24)A[Yaw].Rate * 5;
		break;
	default:
		break;
	} // GyroType

	A[Roll].Rate = RollT.i2_1;
	A[Pitch].Rate = PitchT.i2_1;
	A[Yaw].Rate = YawT.i2_1;

} // CalculateGyroRates

void ErectGyros(uint16 s)
{
	static uint8 g;
	static uint16 i;
	static int32 Av[3];
	static AxisStruct *C;

	for ( g = Roll; g <=(uint8)Yaw; g++ )	
		Av[g] = 0;

    for ( i = 0; i < s; i++ ) {
		LEDRed_TOG;
		Delay1mS(20);
		OutSignals();

		GetGyroValues();

		for ( g = Roll; g <= (uint8)Yaw; g++ )
			Av[g] += A[g].GyroADC;
	}
	
	for ( g = Roll; g <= (uint8)Yaw; g++ ) {
		C = &A[g];
		C->GyroBias = Av[g]/s; // InvenSense is signed
		C->Rate = C->Ratep = C->Angle = C->RawAngle = 0;
	}

	LEDRed_OFF;

} // ErectGyros

void GetGyroValues(void)
{
	switch ( P[SensorHint] ) {
	case DrotekIMU:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[X];
		A[Pitch].GyroADC = -RawGyro[Y];
		A[Yaw].GyroADC = RawGyro[Z];
		break;
	#ifdef INC_MPU6050
	case FreeIMU:
	case UAVXArm32IMU:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[Y];
		A[Pitch].GyroADC = -RawGyro[X];
		A[Yaw].GyroADC = RawGyro[Z];
		break;
	case GY86IMU:
		BlockReadInvenSenseGyro();
		A[Roll].GyroADC = -RawGyro[X];
		A[Pitch].GyroADC = RawGyro[Y];
		A[Yaw].GyroADC = RawGyro[Z];
		break;
	#else
	case FreeIMU:
	case UAVXArm32IMU:
		A[Roll].GyroADC = A[Pitch].GyroADC = A[Yaw].GyroADC = 0;
		break;	
	#endif // INC_MPU6050	
	default:
		GetAnalogGyroValues();
		break;
	} // switch

} // GetGyroValues

void InitGyros(void)
{
	F.UsingAnalogGyros = false;

	switch ( P[SensorHint]) {
	case FreeIMU:
	case GY86IMU:
	case UAVXArm32IMU:
	#ifdef INC_MPU6050
		INVGyroAddress = MPU6050_GYRO_XOUT_H;
		if (InvenSenseGyroActive())	{
			GyroType = P[SensorHint];
			InitInvenSenseGyro();
		}
		else
			GyroType = GyroUnknown;
		break;
	#else
		GyroType = GyroUnsupported;
	#endif // INC_MPU6050
		break;	
	case DrotekIMU:
		INVGyroAddress = INV_GX_H;
		if (InvenSenseGyroActive()) {
			GyroType = DrotekIMU;
			InitInvenSenseGyro();
		}
		else
			GyroType = GyroUnknown;
		break;
	default:
		InitAnalogGyros();
		GyroType = P[SensorHint];
		F.UsingAnalogGyros = true;
		break;
	} // switch

} // InitGyros







