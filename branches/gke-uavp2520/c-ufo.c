// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =      Rewritten and ported to 18F2520 2008 by Prof. Greg Egan        =
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

#include "c-ufo.h"
#include "bits.h"

// Prototypes
void main(void);
void CheckThrottleClosed(void);
void InitMisc(void);

// Variables

#pragma udata assembly_language=0x080

// motor output pulse widths 
uint8 SHADOWB, CAMTOGGLE, MF, MB, ML, MR, MT, ME; // motor/servo outputs
// Bootloader

#pragma udata

#pragma udata globals
// Globals 
uint8	State;									
int32	ClockMilliSec, TimerMilliSec, LostTimer0Clicks;
int32	FailsafeTimeoutMilliSec, AutonomousTimeoutMilliSec, ThrottleClosedMilliSec;
int32	CycleCount;
int8	TimeSlot;
#pragma udata
// RC

#pragma idata rcmaps
#ifdef RX_DSM
const uint8 MapCh1[8] = {0, 6, 1, 4, 7, 3, 5, 2 }; // DX7 and AR7000
const uint8 MapCh3[8] = {0, 5, 3, 2, 1, 6, 4, 7 }; // Futaba 9C with Spektrum DM8 / JR 9XII with DM9 module
#else
const uint8 MapCh1[8] = {0, 1, 2, 3, 4, 5, 6, 7 };
const uint8 MapCh3[8] = {0, 3, 1, 2, 4, 5, 6, 7 };
#endif 
#pragma idata

#pragma udata globals2
uint8	Map[8];
int16	PPM[8], RC[8];
int32	BadRCFrames;
uint8	GoodRCFrames;

// Gyros
int32	RollAngle, PitchAngle, YawAngle;		// PID integral (angle)
int16	RollGyroRate, PitchGyroRate, YawGyroRate;// PID rate (raw gyro values)
int16	RollRate, PitchRate, YawRate;			// PID rate (scaled gyro values)
int16	MidRoll, MidPitch, MidYaw;				// PID gyro neutrals

// Acceleration Corrections
int32	UDVelocity;
int16	Ax, Ay, Az;								// LISL sensor accelerations							
int16	NeutralLR, NeutralFB, NeutralUD;		// LISL scaled neutral values					

// PID 
int32	Rl,Pl,Yl;								// PID output values
int16	RE, PE, YE;								// PID error
int16	REp, PEp, YEp;							// PID previous error
int16	DesiredThrottle, DesiredRoll, DesiredPitch, DesiredYaw, DesiredYawRate;
int16	DesiredCamRoll, DesiredCamPitch;
			
// Altitude
uint16	OriginBaroTemp;
int16	OriginBaroAltitude, DesiredBaroAltitude, CurrBaroAltitude, CurrBaroTemp;
int16	BE, BEp, BESum;	
int16	VBaroComp;
int32	Vud;

// Compass
int16 	Compass;								// raw compass value
int16	AbsDirection;							// desired heading (240 = 360 deg)
int16	CurrDeviation;							// deviation from correct heading

// Motors
uint8	MFront, MBack, MLeft, MRight, MCamPitch, MCamRoll;

// GPS
boolean GPSSentenceReceived; 
real32 GPSLatitude, GPSLongitude, GPSOriginLatitude, GPSOriginLongitude;
int16 GPSHeading, GPSAltitude, GPSOriginAltitude, GPSGroundSpeed;
real32 LongitudeCorrection;

// Misc
uint8	Flags[8];
uint8	Flags2[8];

uint8	LedShadow;								// shadow register
int16	BatteryVolts; 	
#pragma udata

#pragma idata params
// Die Reihenfolge dieser Variablen MUSS gewahrt bleiben!!!!
int8	RollPropFactor		=18;
int8	RollIntFactor		=4;
int8	RollDiffFactor		=0;
int8	BaroTempCoeff		=18;
int8	RollIntLimit		=4;
int8	PitchPropFactor		=18;
int8	PitchIntFactor		=4;
int8	PitchDiffFactor		=0;
int8	BaroThrottleProp	=1;
int8	PitchIntLimit		=4;
int8	YawPropFactor		=20;
int8	YawIntFactor		=40;
int8	YawDiffFactor		=6;
int8	YawLimit			=50;
int8	YawIntLimit			=6;
int8	ConfigParam			=0b00000000;
int8	NoOfTimeSlots		=11;
int8	LowVoltThres		=43;
int8	LinLRIntFactor		=0;	// unused
int8	LinFBIntFactor		=0;	// unused
int8	LinUDIntFactor		=8;
int8	MiddleUD			=0;
int8	MotorLowRun			=40;
int8	MiddleLR			=0;
int8	MiddleFB			=0;
int8	CamIntFactor		=0x44;
int8	CompassFactor		=5;
int8	BaroThrottleDiff	=4;
#pragma idata
// Ende Reihenfolgezwang

void InitMisc(void)
{
	uint8 i, c;

	LedShadow = 0;
    ALL_LEDS_OFF;

	for (i=8; i ; i--)
		Flags[i] = Flags2[i] = false;

	State = Initialising;

	// RC
	for (c = FirstC; c<=LastC; c++)
		PPM[c] = RC[c] = 0;
	BadRCFrames = 0;
	DesiredThrottle = DesiredRoll = DesiredPitch = DesiredYaw = 0;

	// Drives
	_MotorsEnabled = false;
	Rl = Pl = Yl = VBaroComp = Vud = UDVelocity = 0;
	MFront = MLeft = MRight = MBack = _Minimum;
	MCamPitch = MCamRoll = _Neutral;
					
} // InitMisc

void CheckThrottleClosed(void)
{
	if ( _Signal & _NewValues )
	{		
		_Armed = DesiredThrottle < _ThresStop;
		_NewValues = false;
		_UseCh7Trigger =  DesiredCamRoll < _Neutral;
		OutSignals();
	}
	else
		_Armed = false;
} // CheckThrottleClosed

void main(void)
{
	uint8 i;

	DisableInterrupts;
	InitMisc();
	InitPorts();
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);	
	InitADC();
						
#ifdef COMMISSIONING
	for (i=_EESet2*2; i ; i--)					// clear EEPROM parameter space
		WriteEE(i, -1);
	WriteParametersEE(1);						// copy RAM initial values to EE
	WriteParametersEE(2);
#endif
	ReadParametersEE();
	InitTimersAndInterrupts();
	InitAttitude();
	AcquireSatellites();
	ShowSetup(1);

	while(1)
	{
		_MotorsEnabled = false;
		DesiredThrottle = 0;

		ALL_LEDS_OFF;
		LedRed_ON;
		
		ProcessCommand();
	
//		CheckThrottleClosed();					// sets _Armed

		TimeSlot = Limit(NoOfTimeSlots, 22, 22);// 6 is possible
		ResetTimeOuts();
		UpdateControls();
		State = Landed;
_Armed = true;
		while( _Armed && 1 )//Switch )
		{
			while( TimeSlot > 0 ) { };			// user routine here if desired	
			TimeSlot = Limit(NoOfTimeSlots, 22, 22);
			CycleCount++;
			DoControl();
			OutSignals();
			UpdateGPS();
			
			DoDebugTraces();
			
			// housekeeping which must finish before TimeSlot = 0
			if ( State == Flying )
			{
				if ( _Signal )
				{
					ResetTimeOuts();	
					UpdateControls();
				}
				else
					if ( ClockMilliSec > FailsafeTimeoutMilliSec )
						{
							TimerMilliSec = ClockMilliSec + FAILSAFE_CANCEL;
							State = Failsafe;
						}
			}
			else
			if ( State == Failsafe )
			{
 				// hold all current settings including throttle
				if (ClockMilliSec > AutonomousTimeoutMilliSec )
				{
					TimerMilliSec = ClockMilliSec + AUTONOMOUS_CANCEL;
					State = Autonomous;
				}
				else
					if ( RCLinkRestored(FAILSAFE_CANCEL) )
						State = Flying;
			}
			else
			if ( State == Autonomous )
			{
				DoAutonomous();
				if ( RCLinkRestored(AUTONOMOUS_CANCEL) )
					State = Flying;
			}
			else
			if ( State == Landed )
			{
				if ( _Signal )
				{
					ALL_LEDS_OFF;
					LedGreen_ON;
					ResetTimeOuts();
					UpdateControls();
					
					if ( DesiredThrottle > _ThresStart )
					{
						ALL_LEDS_OFF;
						_MotorsEnabled = true;
						AbsDirection = COMPASS_INVAL;
						State = Flying;
					}
				}
				else
				{
					// do nothing - Red Led should be flashing
					DesiredThrottle = DesiredRoll = DesiredPitch = DesiredYaw = 0;
				}
			} // States
		
			CheckAlarms();
		} // while arming switch is on
	}
	// CPU should never arrive here	
} // Main
