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
			{20,0},				// RollRateKp, 				01
			{10,0},	 			// RollRateKi,				02
			{20, 0},			// RollAngleKp,				03
			{0,true},			// Unused4,					04
			{30,0},	 			// RollIntLimit,			05

			{20,0},	 			// PitchRateKp,				06
			{10,0},	 			// PitchRateKi,				07
			{20,0},	 			// PitchAngleKp,			08
			{5,true},	 		// AltKp,					09
			{30,0},	 			// PitchIntLimit,			10

			{25,0},	 			// YawKpRate, 				11
			{50,0},	 			// RollRateKd was YawKi,	12
			{Wolferl, true},	// IMU,						13
			{20,0},	 			// YawLimit,				14
			{CompoundPPM, true},// RCType,					15

	
			{2,true}, 			// ConfigBits,				16c
			{1,true},			// RxThrottleCh,			17
			{51,true}, 			// LowVoltThres,			18c 
			{20,true}, 			// CamRollKp,				19c
			{45,true}, 			// PercentCruiseThr,		20c

			{0,0}, 				// Unused21,				21c // 7 is magic number
			{30,true}, 			// RollYawMix,				22c
			{5,true}, 			// PercentIdleThr,			23c
			{15,0}, 			// RollKiAngle,				24c
			{15,0}, 			// PitchKiAngle,			25c
			{20,true}, 			// CamPitchKp,				26c
			{10,0}, 			// YawKpAngle (Compass),	27
			{50,0},				// PitchRateKd,				28
			{10,true}, 			// NavRollPitchSlew,		29
			{0,true}, 			// Unused30,				30

			{0,0}, 				// GSThrottle,	    		31
			{0,0},				// Acro,	    			32
			{10,true}, 		    // NavRTHAlt,				33
			{13,true},			// NavMagVar,				34c
#ifdef INC_MPU6050
			{FreeIMU,true}, 	// SensorHint,     			35c
#else
			{LY530Gyro,true}, 	// SensorHint,     			35c
#endif
			{ESCPPM,true}, 		// ESCType,					36c
			{7,true}, 			// RxChannels,				37c
			{2,true},			// RxRollCh,				38
			{20,true},			// MadgwickKp,				39c
			{1,true},			// CamRollTrim,				40c

			{2,true},			// NavMaxVelMpS,			41
			{3,true},			// RxPitchCh,				42
			{4,true},			// RxYawCh,					43
			{AF_TYPE,true},		// AFType,					44c
			{NoTelemetry, true}, // TelemetryType,		45c
			{10,true},		    // MaxDescentRatedMpS, 		46
			{30,true},			// DescentDelayS			47
			{MPU_RA_DLPF_BW_98, true},	// GyroLPF,			48
			{0,true},			// NavCrossTrackKp,			49
			{5,true},			// RxGearCh,				50c

			{6,true},			// RxAux1Ch,				51
			{0,true},			// ServoSense				52c
			{2,true},			// AccConfSD,				53c
			{22,true},			// BatteryCapacity			54c
			{7,true},			// RxAux2Ch,				55c
			{8,true},			// RxAux3Ch,				56
			#if (defined  TRICOPTER) | (defined VTCOPTER ) | (defined Y6COPTER )
			{24,true},			// Orient					57
			#else	
			{0,true},			// Orient					57
			#endif // TRICOPTER | VTCOPTER | Y6COPTER			

			{12,true},			// NavYawSlew				58
			{50,0},				// Balance					59
			{9,true},			// RxAux4Ch					60
			{50,true},			// DriveFilt				61

			{NMEAGPS,true},		// GPSProtocol,				62
			{0,true},			// HorizDampKp,				63
			{0,true}			// VertDampKp,				64
			};
	#pragma idata


