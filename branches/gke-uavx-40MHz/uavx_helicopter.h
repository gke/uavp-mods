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

	#pragma idata paramdefaults	
	const rom int8 DefaultParams[MAX_PARAMETERS][2] = {
			{20,0},				// RollKp, 			01
			{10,0},	 			// RollKi,			02
			{20, 0},			// RollKpAngle was RollKd			03
			{1,true},			// NeutralRadius was HorizDampKp,	04
			{30,0},	 			// RollIntLimit,	05

			{20,0},	 			// PitchKp,			06
			{10,0},	 			// PitchKi,			07
			{20,0},	 			// PitchKpAngle was PitchKd			08
			{5,true},	 		// AltKp,			09 // was 8
			{30,0},	 			// PitchIntLimit,	10

			{25,0},	 			// YawKpRate, 			11
			{0,0},	 			// Unused12 was YawKi,			12
			{0, true},			// IMU was AccTrack was YawKd,		13
			{20,0},	 			// YawLimit,		14 was 50
			{CompoundPPM, true},// RCType was YawIntLimit,		15

			{2,true}, 			// ConfigBits,		16c
			{1,true},			// RxThrottleCh was TimeSlots,	17
			{51,true}, 			// LowVoltThres,	18c // 51
			{20,true}, 			// CamRollKp,		19c
			{45,true}, 			// PercentCruiseThr,20c

			{8,true}, 			// BaroFilt,		21c // 7 is magic number
			{0,true}, 			// MiddleDU,		22c
			{20,true}, 			// PercentIdleThr,	23c
			{0,true}, 			// MiddleLR,		24c
			{0,true}, 			// MiddleFB,		25c
			{20,true}, 			// CamPitchKp,		26c
			{10,0}, 				// YawKpAngle (Compass),		27
			{0,0},				// Unused28 was AltKi,			28 // was 8
			{2,true}, 			// NavSlew was NavRadius,	29
			{2,true}, 			// ClosingRadius was NavKi,	30

			{0,0}, 				// GSThrottle,	    31
			{0,0},				// Acro,	    	32
			{10,true}, 		    // NavRTHAlt,		33
			{13,true},			// NavMagVar,		34c
			{LY530Gyro,true}, // SensorHint,     	35c
			{ESCPPM,true}, 		// ESCType,			36c
			{7,true}, 			// RxChannels was TxRxType		37c
			{2,true},			// RxRollCh was NeutralRadius	38
			{0,true},			// Unused39 was PercentNavSens6Ch	39c
			{1,true},			// CamRollTrim,		40c

			{2,true},			// NavMaxVelMpS was was NavKd	41
			{3,true},			// RxPitchCh was VertDampDecay    42
			{4,true},			// RxYawCh was HorizDampDecay	43
			{AF_TYPE,true},		// AFType was BaroScale 	    44c
			{UAVXTelemetry, true}, // TelemetryType	45c
			{10,true},		    // MaxDescentRateDmpS 	46
			{30,true},			// DescentDelayS	47
			{MPU_RA_DLPF_BW_98, true},	// GyroLPF was NavIntLimit		48
			{0,0},				// Unused49 was AltIntLimit		49 was 8
			{5,true},			// RxGearCh was GravComp 50c

			{6,true},			// RxAux1Ch was CompSteps	51
			{0,true},			// ServoSense		52c
			{2,true},			// AccConfSD was AccComp was CompassOffsetQtr 53c
			{49,true},			// BatteryCapacity	54c
			{7,true},			// RxAux2Ch was GyroYawType	55c
			{8,true},			// RxAux3Ch was AltKd		56
			#if (defined  TRICOPTER) | (defined VTCOPTER ) | (defined Y6COPTER )
			{24,true},			// Orient			57
			#else	
			{0,true},			// Orient			57
			#endif // TRICOPTER | VTCOPTER | Y6COPTER			

			{12,true},			// NavYawLimit		58
			{50,0},				// Balance			59
			{9,true},			// RxAux4Ch			60
			{-20,0},			// RollKpRate		61

			{23,0},				// PitchKpRate		62
			{0,true},			// HorizDampKp
			{0,true}			// VertDampKp
			};
	#pragma idata

