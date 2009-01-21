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

#include "c-ufo.h"
#include "bits.h"

// The globals

#pragma udata assembly_language=0x080

// motor output pulse widths 
uint8 SHADOWB, CAMTOGGLE, MF, MB, ML, MR, MT, ME; // motor/servo outputs
// Bootloader

#pragma udata

#pragma udata globals
// Globals 
uint8	State;									
int32	ClockMilliSec, TimerMilliSec;
int32	FailsafeTimeoutMilliSec, AutonomousTimeoutMilliSec, ThrottleClosedMilliSec;
int32	CycleCount;
int8	TimeSlot;

// RC
uint8	IThrottle;
uint8 	PrevIThrottle;							// most recent past IThrottle					
uint8	DesiredThrottle;						// actual throttle
int16 	IRoll, IPitch, IYaw;
uint8	IK5,IK6,IK7;
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
			
// Altitude
uint16	BasePressure, BaseTemp, TempCorr;
int16	VBaroComp, BaroVal;
int32	Vud;
int32	BaroCompSum;

// Compass
int16 	Compass;								// raw compass value
int16	AbsDirection;							// desired heading (240 = 360 deg)
int16	CurrDeviation;							// deviation from correct heading

// Motors
uint8	MFront,MLeft,MRight,MBack;				// output channels
uint8	MCamRoll,MCamPitch;

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
int8	LinLRIntFactor		=0;
int8	LinFBIntFactor		=0;
int8	LinUDIntFactor		=8;
int8	MiddleUD			=0;
int8	MotorLowRun			=40;
int8	MiddleLR			=0;
int8	MiddleFB			=0;
int8	CamIntFactor		=0x00;
int8	CompassFactor		=5;
int8	BaroThrottleDiff	=4;
#pragma idata
// Ende Reihenfolgezwang

void InitMisc(void)
{
	uint8 i;

	LedShadow = 0;
    ALL_LEDS_OFF;

	for (i=8; i ; i--)
		Flags[i] = Flags2[i] = false;

	State = Initialising;

	// RC
	State = Initialising;
	IThrottle =	PrevIThrottle = DesiredThrottle = IK5 =	IRoll = IPitch = IYaw = 0;	
	IK6 = IK7 = _Neutral;
	BadRCFrames = 0;

	// Drives
	_MotorsEnabled = false;
	Rl = Pl = Yl = VBaroComp = Vud = UDVelocity = 0;
	MFront = MLeft = MRight = MBack = _Minimum;
	MCamRoll = MCamPitch = _Neutral;
					
} // InitMisc

uint8 RCLinkRestored(int32 d)
{
	// checks for good RC frames for a period d mS
	if ( !_Signal )
		TimerMilliSec = ClockMilliSec + d;
	return(ClockMilliSec > TimerMilliSec );
} // RCLinkRestored

void CheckThrottleClosed(void)
{
	if ( _Signal & _NewValues )
	{		
		_Armed = IThrottle < _ThresStop;
		_NewValues = false;
		_UseCh7Trigger =  IK7 < _Neutral;
		ReadParametersEE();
		OutSignals();
	}
	else
		_Armed = false;
} // CheckThrottleClosed

void ResetTimeOuts(void)
{
	FailsafeTimeoutMilliSec = ClockMilliSec + FAILSAFE_TIMEOUT;
	AutonomousTimeoutMilliSec = ClockMilliSec + AUTONOMOUS_TIMEOUT;
	ThrottleClosedMilliSec = ClockMilliSec + THROTTLE_TIMEOUT;
} // ResetTimeOuts


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
	ShowSetup(1);				

	while(1)
	{
		_MotorsEnabled = false;

		LedRed_ON;
		if(_UseLISL)
			LedYellow_ON;
		
		ProcessCommand();
//IThrottle = 0;		
		CheckThrottleClosed();					// sets _Armed
//IThrottle = 75;
		TimeSlot = Limit(NoOfTimeSlots, 22, 22);// 6 is possible
		ResetTimeOuts();
		State = Landed;

		while( _Armed && Switch )
		{
			while( TimeSlot > 0 ) { };			// user routine here if desired	
			TimeSlot = Limit(NoOfTimeSlots, 22, 22);
			CycleCount++;
			DoControl();
			OutSignals();
			DoDebugTraces();
			
			// housekeeping which must finish before TimeSlot = 0
			if ( State == Flying )
			{
				if ( _Signal )
				{
					ResetTimeOuts();	
					ReadParametersEE();
					DesiredThrottle = IThrottle;
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
 				// stop high-power runaways
				DesiredThrottle = Limit(DesiredThrottle, 0, THR_HOVER);
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
				// No GPS so no station holding or return home yet.
				DesiredThrottle = Descend(DesiredThrottle);
				IRoll = IPitch = IYaw = 0;
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
					ReadParametersEE();
					DesiredThrottle = IThrottle;
					
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
					DesiredThrottle = IRoll = IPitch = IYaw = 0;
				}
			} // States
		
			CheckAlarms();
		} // while arming switch is on
	}
	// CPU should never arrive here
	// do a processor reset ???	
} // Main
