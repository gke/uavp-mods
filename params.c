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

void ReadParametersEE(void);
void WriteParametersEE(uint8);
void UseDefaultParameters(void);
void UpdateWhichParamSet(void);
boolean ParameterSanityCheck(void);
void InitParameters(void);

const rom uint8 ESCLimits [] = { OUT_MAXIMUM, OUT_HOLGER_MAXIMUM, OUT_X3D_MAXIMUM, OUT_YGEI2C_MAXIMUM, OUT_LRC_MAXIMUM };

#ifdef MULTICOPTER
	#include "uavx_multicopter.h"
#else
	#ifdef HELICOPTER
		#include "uavx_helicopter.h"	
	#else
		#ifdef ELEVONS
			#include "uavx_elevon.h"
		#else
			#include "uavx_aileron.h"
		#endif
	#endif
#endif 


uint8	ParamSet;
boolean ParametersChanged;
int8 Orientation;
int16 OrientationMRad;
uint8 UAVXAirframe;

#pragma udata params
int8 P[MAX_PARAMETERS];
#pragma udata

#pragma udata orient
int16 OSin[48], OCos[48];
#pragma udata

void ReadParametersEE(void)
{
	static uint8 i, b;
	static uint16 a;

	if ( ParametersChanged ) { // overkill if only a single parameter has changed but is not in flight loop

		a = (ParamSet - 1)* MAX_PARAMETERS;	
		for ( i = 0; i < (uint8)MAX_PARAMETERS; i++)
			P[i] = ReadEE(a + i);

		A[Roll].AngleKp = P[RollAngleKp];
		A[Roll].AngleKi = P[RollAngleKi];

		A[Roll].RateKp = P[RollRateKp];
		A[Roll].RateKi = P[RollRateKi];
		A[Roll].RateKd = P[RollRateKd];

		A[Roll].IntLimit = P[RollIntLimit];
		
		A[Pitch].AngleKp = P[PitchAngleKp];
		A[Pitch].AngleKi = P[PitchAngleKi];

		A[Pitch].RateKp = P[PitchRateKp];
		A[Pitch].RateKi = P[PitchRateKi];
		A[Pitch].RateKd = P[PitchRateKd];

		A[Pitch].IntLimit = P[PitchIntLimit];
		
		A[Yaw].RateKp = P[YawRateKp];
		A[Yaw].Limiter = P[YawLimit];

		// Navigation
		#if (defined  TRICOPTER) | (defined  Y6COPTER) | (defined HELICOPTER) | (defined VTCOPTER)
			F.AllowTurnToWP = true;
		#else
			F.AllowTurnToWP = false;	
		#endif

		Nav.MaxVelocitydMpS = P[NavMaxVelMpS]*10;
		Nav.PosKp = 1;
		Nav.VelKp = 5;
		Nav.YawKp = 2;
		Nav.CrossTrackKp = P[NavCrossTrackKp];

		Nav.RollPitchSlewRate = (int16)P[NavRollPitchSlew];
		Nav.YawSlewRate = (int16)P[NavYawSlew];
		
		Nav.MaxVelocitydMpS = P[NavMaxVelMpS] * 10; 

		ReadAccCalEE();

		ESCMax = ESCLimits[P[ESCType]];
		if ( P[ESCType] == ESCPPM )
			TRISB = 0b00000000; // make outputs
		else
			for ( i = 0; i < (uint8)NO_OF_I2C_ESCS; i++ )
				ESCI2CFail[i] = 0;

		#ifdef INC_MPU6050
			MPU6050AccActive();
			InitMPU6050Acc();
		#endif

		InitCompass();
		InitAccelerometers();
		InitGyros();
		ErectGyros(16);

		if ( P[ESCType] == ESCPPM )
			PIDCycleShift = PID_40MHZ_SHIFT; 
		else
			PIDCycleShift = PID_40MHZ_I2CESC_SHIFT; 

		PIDCyclemS = PID_BASE_CYCLE_MS * ((int8)1 << PIDCycleShift);
		ServoInterval = ((SERVO_UPDATE_INTERVAL+PIDCyclemS/2)/PIDCyclemS);
		EffAccTrack = RESCALE_TO_ACC >> (2-PIDCycleShift);

		b = P[ServoSense];
		for ( i = 0; i < (uint8)6; i++ ) {
			PWSense[i]  = ((b & 1) ? -1 : 1);
			b >>=1;
		}

		F.UsingAltControl = ( (P[ConfigBits] & UseAltControlMask ) != 0);
		RollYawMixFrac = FromPercent(P[RollYawMix], 256);

		#ifdef SIMULATE
			CruiseThrottle = NewCruiseThrottle = FromPercent(45, RC_MAXIMUM);
		#else
			CruiseThrottle = NewCruiseThrottle = FromPercent((int16)P[PercentCruiseThr], RC_MAXIMUM);
		#endif

		#ifdef MULTICOPTER
			IdleThrottle = Limit((int16)P[PercentIdleThr], 10, 20);
		#else
			IdleThrottle = Limit((int16)P[PercentIdleThr],2, 5);
		#endif // MULTICOPTER

		IdleThrottle = FromPercent(IdleThrottle, RC_MAXIMUM);

		MinROCCmpS = -(int16)P[MaxDescentRateDmpS] * 10;

		RTHAltitude = (int24)P[NavRTHAlt]*100;

		//zzz AltLPFHz = (int16)P[BaroFilt];

		CompassOffset = - (int16)P[NavMagVar]; // changed sign of MagVar AGAIN!
		if ( CompassType == HMC6352Compass )
			CompassOffset += (int16)COMPASS_OFFSET_QTR * 90L; 
		CompassOffset = ((int32)CompassOffset *MILLIPI)/180;

		#ifdef MULTICOPTER
			Orientation = P[Orient];
			if (Orientation == -1 ) // uninitialised
				Orientation = 0;
			else
				if (Orientation < 0 )
					Orientation += 48;
			OrientationMRad = ((int24)Orientation * MILLIPI)/48;
		#else
			Orientation = OrientationMRad = 0;
		#endif // MULTICOPTER

		NoOfControls = Limit(P[RCChannels], 4, CONTROLS);
		if ( (( NoOfControls&1 ) != (uint8)1 ) && !F.UsingCompoundPPM )
			NoOfControls--;

		InitRC();

		F.RFInInches = ((P[ConfigBits] & RFInchesMask) != 0);

		F.FailsafesEnabled = ((P[ConfigBits] & UseFailsafeMask) == 0);

		F.UsingTxMode2 = ((P[ConfigBits] & TxMode2Mask) != 0);
		F.UsingRTHAutoDescend = ((P[ConfigBits] & UseRTHDescendMask) != 0);
		NavRTHTimeoutmS = (uint24)P[DescentDelayS]*1000L;

		BatteryVoltsLimitADC = BatteryVoltsADC = ((int24)P[LowVoltThres] * 1024 + 70L) / 139L; // UAVPSet 0.2V units
		BatteryCurrentADC = 0;
		
		F.ParametersValid = ParameterSanityCheck();

		ParametersChanged = false;
	}
	
} // ReadParametersEE

void WriteParametersEE(uint8 s)
{
	static uint8 p;
	static uint16 addr;
	
	addr = (s - 1)* MAX_PARAMETERS;

	for ( p = 0; p < (uint8)MAX_PARAMETERS; p++)
		WriteEE( addr + p,  P[p]);
} // WriteParametersEE

void UseDefaultParameters(void)
{ // loads a representative set of initial parameters as a base for tuning
	static uint16 p;

//	for ( p = 0; p < (uint16)MAX_EEPROM; p++ )
//		WriteEE( p,  0xff);

	for ( p = 0; p < (uint16)MAX_PARAMETERS; p++ )
		P[p] = DefaultParams[p][0];

	WriteParametersEE(1);
	WriteParametersEE(2);
	ParamSet = 1;

	WriteEE(NAV_NO_WP, 0); // set NoOfWaypoints to zero

	TxString("\r\nUse READ CONFIG to refresh UAVPSet display\r\n");	
} // UseDefaultParameters

void UpdateParamSetChoice(void)
{
	#define STICK_WINDOW 30

	static uint8 NewParamSet, NewAllowNavAltitudeHold, NewAllowTurnToWP;
	static int16 Selector;

	NewParamSet = ParamSet;
	NewAllowNavAltitudeHold = F.AllowNavAltitudeHold;
	NewAllowTurnToWP = F.AllowTurnToWP;

	if ( F.UsingTxMode2 )
		Selector = A[Roll].Desired;
	else
		Selector = -A[Yaw].Desired;

	if ( (Abs(A[Pitch].Desired) > STICK_WINDOW) && (Abs(Selector) > STICK_WINDOW) ) {
		if ( A[Pitch].Desired > STICK_WINDOW ) { // bottom
			if ( Selector < -STICK_WINDOW ) // left
			{ // bottom left
				NewParamSet = 1;
				NewAllowNavAltitudeHold = true;
			} else
				if ( Selector > STICK_WINDOW ) { // right
 					// bottom right
					NewParamSet = 2;
					NewAllowNavAltitudeHold = true;
				}
		}		
		else
			if ( A[Pitch].Desired < -STICK_WINDOW ) { // top		
				if ( Selector < -STICK_WINDOW ) { // left
					NewAllowNavAltitudeHold = false;
					NewParamSet = 1;
				} else 
					if ( Selector > STICK_WINDOW ) { // right
						NewAllowNavAltitudeHold = false;
						NewParamSet = 2;
					}
			}

		if ( ( NewParamSet != ParamSet ) || ( NewAllowNavAltitudeHold != F.AllowNavAltitudeHold ) ) {
			ParamSet = NewParamSet;
			F.AllowNavAltitudeHold = NewAllowNavAltitudeHold;
			LEDBlue_ON;
			DoBeep100mSWithOutput(2, 2);
			if ( ParamSet == (uint8)2 )
				DoBeep100mSWithOutput(2, 2);
			if ( F.AllowNavAltitudeHold )
				DoBeep100mSWithOutput(4, 4);
			ParametersChanged |= true;
			Beeper_OFF;
			LEDBlue_OFF;
		}
	}

	if ( F.UsingTxMode2 )
		Selector = -A[Yaw].Desired;
	else
		Selector = A[Roll].Desired;

	if ( (Abs(RC[ThrottleRC]) < STICK_WINDOW) && (Abs(Selector) > STICK_WINDOW ) ) {
		if ( Selector < -STICK_WINDOW ) // left
			NewAllowTurnToWP = false;
		else
			if ( Selector > STICK_WINDOW ) // left
				NewAllowTurnToWP = true; // right
			
		if ( NewAllowTurnToWP != F.AllowTurnToWP ) {		
			F.AllowTurnToWP = NewAllowTurnToWP;
			LEDBlue_ON;
		//	if ( F.AllowTurnToWP )
				DoBeep100mSWithOutput(4, 2);

			LEDBlue_OFF;
		}
	}

} // UpdateParamSetChoice

boolean ParameterSanityCheck(void)
{
	return ((P[RollRateKp] != 0) &&
			(P[PitchRateKp]!= 0) &&
			(P[YawRateKp] != 0) );
} // ParameterSanityCheck

void InitParameters(void) {
	static uint8 i;
	static int16 A;

	UAVXAirframe = AF_TYPE;

	for (i = 0; i < (uint8)48; i++) {
		A = (int16)(((int32)i * MILLIPI)/24L);
		OSin[i] = int16sin(A);
		OCos[i] = int16cos(A);
	}
	Orientation = 0;

	ALL_LEDS_ON;
	ParamSet = 1;

	if ( ( ReadEE((uint16)RCChannels) == -1 ) || (ReadEE(MAX_PARAMETERS + (uint16)RCChannels) == -1 ) )
		UseDefaultParameters();

	ParametersChanged = true;
	ParamSet = 1;
	ReadParametersEE();

	ALL_LEDS_OFF;  
} // InitParameters


