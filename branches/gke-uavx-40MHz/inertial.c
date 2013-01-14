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

void CompensateRollPitchGyros(void);
void GetAttitude(void);
void DoAttitudeAngles(void);
void GyrosAndAccsTest(void);

uint8 EffAccTrack;

void CompensateRollPitchGyros(void)
{
	// RESCALE_TO_ACC is dependent on cycle time and is defined in uavx.h
	#define ANGLE_COMP_STEP 6 //25

	static int16 AbsAngle, Grav, Dyn, NewAcc, NewCorr;
	static uint8 a;
	static AxisStruct *C;

	if ( F.AccelerationsValid ) 
	{
		ReadAccelerations();

		A[Yaw].Acc = A[Yaw].AccADC - A[Yaw].AccBias; 

		for ( a = Roll; a<(uint8)Yaw; a++ )
		{
			C = &A[a];
			AbsAngle = Abs(C->Angle);
			if ( AbsAngle < (30 * DEG_TO_ANGLE_UNITS) ) // ArcSin approximation holds
			{	
				C->Acc = C->AccADC - C->AccBias; 
				Grav = SRS32((int32)C->Angle * EffAccTrack, 8); 
				Dyn = 0; //A[a].Rate;
		
			    NewCorr = SRS32(C->Acc + Grav + Dyn, 3); 
	
				if ( (State == InFlight) && (AbsAngle > 10 * DEG_TO_ANGLE_UNITS) )
				{
					C->AccCorrAv += Abs(NewCorr);
					C->AccCorrMean += NewCorr;
					NoAccCorr++;
				}
				// smoothing filter here?
	
				C->DriftCorr = Limit1(NewCorr, ANGLE_COMP_STEP);
			}
			else
				C->DriftCorr = 0;			
		}
	}	
	else
	{
		A[Roll].Angle = Decay1(A[Roll].Angle);
		A[Pitch].Angle = Decay1(A[Pitch].Angle);
		A[Roll].DriftCorr = A[Roll].Acc = A[Pitch].DriftCorr = A[Pitch].Acc = 0;
		A[Yaw].Acc = GRAVITY;
	}

} // CompensateRollPitchGyros

void DoAttitudeAngles(void)
{	
	// Angles and rates are normal aircraft coordinate conventions
	// X/Forward axis reversed for Acc to simplify compensation
	
	static uint8 a;
	static int16 Temp;
	static AxisStruct *C;

	for ( a = Roll; a <= (uint8)Pitch; a++ )
	{
		C = &A[a];
	
		#ifdef INC_RAW_ANGLES
		C->RawAngle += C->Rate; // for Acc comp studies
		#endif
	
		Temp = C->Angle + C->Rate;	
		Temp = Limit1(Temp, ANGLE_LIMIT); // turn off comp above this angle?
		Temp -= C->DriftCorr;			// last for accelerometer compensation
		C->Angle = Temp;
	}

} // DoAttitudeAngles


void GetAttitude(void)
{
	GetGyroValues();
	CalculateGyroRates();
	CompensateRollPitchGyros();
#ifdef OLD_YAW
	CompensateYawGyro();
#endif
	DoAttitudeAngles();
} // GetAttitude

#ifdef TESTING

void GyrosAndAccsTest(void)
{
	static int16 Mag;

	ReadAccelerations();
	GetGyroValues();

	TxString("\r\nGyros & Accelerometers test\r\n\r\nAccs: ");
	ShowAccType();
	if (F.AccelerationsValid) 
		TxString(" (OK)"); 
	else 
		TxString(" (FAIL)");
	TxString("\r\nGyros: ");
	ShowGyroType(GyroType);
	TxNextLine();

	#ifdef INC_MPU6050
	if ( AccType == MPU6050Acc ) 
	{
		TxString("\r\nDLPF: ");
		if ((MPU6050DLPF >= MPU_RA_DLPF_BW_256) && (MPU6050DLPF
						<= MPU_RA_DLPF_BW_5)) 
		{
			TxVal32(InertialLPFHz[MPU6050DLPF],0,0);
			TxString("Hz");
		} 
		else
			TxString("Unknown");
		
		TxString(" DHPF: ");
		if ((MPU6050DHPF >= MPU_RA_DHPF_RESET) && (MPU6050DLPF
						<= MPU_RA_DHPF_HOLD)) 
			TxString(DHPFName[MPU6050DHPF]);
		else
			TxString("Unknown\r\n");
		}
	#endif
	#ifdef INC_BMA180
	if ( AccType == BMA180Acc )
		ShowBMA180State();
	#endif // INC_BMA180
	
	TxString("\r\nAccMag:\t");
	Mag = int32sqrt(Sqr((int24)A[Roll].AccADC)+Sqr((int24)A[Pitch].AccADC)+Sqr((int24)A[Yaw].AccADC));
	TxVal32((int32)Mag, 0, 0);
	
	TxString("\r\n\r\n\tRoll: \t");
	TxVal32(A[Roll].GyroADC,0, HT);
	TxChar('(');
	TxVal32(A[Roll].GyroBias,0, ')');
	TxString("\tL->R: \t");
	TxVal32((int32)A[Roll].AccADC, 0, HT);
	TxChar('(');
	TxVal32((int32)A[Roll].AccBias, 0, ')');

	TxString("\r\n\tPitch:\t");
	TxVal32(A[Pitch].GyroADC,0, HT);
	TxChar('(');
	TxVal32(A[Pitch].GyroBias,0, ')');
	TxString("\tF->B: \t");	
	TxVal32((int32)A[Pitch].AccADC, 0, HT);
	TxChar('(');
	TxVal32((int32)A[Pitch].AccBias, 0, ')');	

	TxString("\r\n\tYaw:  \t");
	TxVal32(A[Yaw].GyroADC,0, HT);
	TxChar('(');
	TxVal32(A[Yaw].GyroBias,0, ')');
	TxString("\tD->U:    \t");
	
	TxVal32((int32)A[Yaw].AccADC, 0, HT);
	TxChar('(');
	TxVal32((int32)A[Yaw].AccBias, 0, ')');	

} // GyrosAndAccsTest

#endif // TESTING





