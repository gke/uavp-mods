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
int32	ClockMilliSec, TimerMilliSec;
int32	RCTimeOutMilliSec, ThrottleClosedMilliSec;
int32	CycleCount;
int8	TimeSlot;

// RC
uint8	IThrottle;
int16 	IRoll,IPitch,IYaw;
uint8	IK5,IK6,IK7;

// Gyros
int32	RollAngle, PitchAngle, YawAngle;		// PID integral (angle)
int16	RollRate, PitchRate, YawRate;			// PID rate (scaled gyro values)
int16	PrevYawRate;							// PID for noisy Yaw gyro filtering
int16	MidRoll, MidPitch, MidYaw;				// PID gyro neutrals
// Acceleration Corrections
int32	UDVelocity;
int16	Ax, Ay, Az;								// LISL sensor accelerations							
int16	LRIntKorr, FBIntKorr;					// LISL gyro drift corrections
int16	NeutralLR, NeutralFB, NeutralUD;		// LISL scaled neutral values					

// PID 
int32	Rl,Pl,Yl;								// PID output values
int16	RE, PE, YE;								// PID error
int16	REp, PEp, YEp;							// PID previous error
int16	RollFailsafe, PitchFailsafe, YawFailsafe;
			
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
										
uint8	CurrThrottle;

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
int8	NoOfTimeSlots		=4;
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

// resets all important variables
// Do NOT call this while in flight!
void InitMisc(void)
{
	uint8 i;

	for (i=8; i ; i--)							// clear flags ???
	{
		Flags[i] = false;
		Flags2[i] = false;
	}    
	

	CurrThrottle = 0xff;
	RollFailsafe = PitchFailsafe = YawFailsafe = 0;

	LedShadow = 0;
    ALL_LEDS_OFF;
	IThrottle = IK5 = _Minimum;					// Kill throttle assume parameter set #1

	// RC
	_Flying = false;
	IThrottle = 0;
	CurrThrottle = 0xFF;
	IRoll = IPitch = IYaw = IK5 = IK6 = IK7 = 0;

	// Drives
	_MotorsEnabled = false;
//	Rl = Pl = Yl = VBaroComp =0;
	Vud = 0;
	MFront = _Minimum;	
	MLeft = _Minimum;
	MRight = _Minimum;
	MBack = _Minimum;

	MCamRoll = MCamPitch = _Neutral;
					
} // InitArrays

void CheckThrottleMoved(void)
{
	int16	Temp;

/*
	if( ThrDownCount > 0 )
	{
		if( LedCount & 1 )
			ThrDownCount--;
		if( ThrDownCount == 0 )
			CurrThrottle = IThrottle;			// remember current Throttle level
	}
	else
	{
		if( CurrThrottle < THR_MIDDLE ) 		// ??? tidy up
			Temp = 0;
		else
			Temp = CurrThrottle - THR_MIDDLE;
		if( IThrottle < THR_HOVER )
			ThrDownCount = THR_DOWNCOUNT;		// left dead area
		if( IThrottle < Temp )
			ThrDownCount = THR_DOWNCOUNT;		// left dead area
		if( IThrottle > CurrThrottle + THR_MIDDLE )
			ThrDownCount = THR_DOWNCOUNT;		// left dead area
	}
*/
} // CheckThrottleMoved

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
	RCTimeOutMilliSec = ClockMilliSec + 1500; 
//	ThrottleClosedMilliSec = 444444;
} // ResetTimeOuts

void InitAttitude(void)
{
	// DON'T MOVE THE UFO!
	// ES KANN LOSGEHEN!
	LedRed_ON;
//	Delay100mSec(100);							// ~10Sec. to get hands away after power up
	Beeper_ON;
	InitDirection();		
	InitAltimeter();
	InitAccelerometers();
	InitGyros();
	Beeper_OFF;					
	LedRed_OFF;
} // InitInertial

void DoControl()
{
	if( _NewValues )
		CheckThrottleMoved();

	GetDirection();
	GetAltitude();				
	DetermineAttitude();
	PID();

} // DoControl

void main(void)
{
	uint8 i;

	DisableInterrupts;							// disable all interrupts

	InitMisc();
	InitPorts();
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);	
	InitADC();
						
#ifdef COMMISSIONING
	for (i=_EESet2*2; i ; i--)						// clear EEPROM parameter space
		WriteEE(i, -1);
	WriteParametersEE(1);							// copy RAM initial values to EE
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
IThrottle = _Minimum;		
		CheckThrottleClosed();
IThrottle = 70;	

		TimeSlot = Limit(NoOfTimeSlots, 10, 22);	// 6 is possible

		while( _Armed && Switch )
		{
			while( TimeSlot > 0 ) { };				// user routine here if desired	
			TimeSlot = Limit(NoOfTimeSlots, 6, 22);
			CycleCount++;
			DoControl();
			OutSignals();
SwitchLedsOff(0x7f);
			// housekeeping which must finish before TimeSlot = 0
			if ( _Flying )
				if ( _Signal )
				{
					ResetTimeOuts();	
					ReadParametersEE();
				}
				else
				{
					ALL_LEDS_OFF;
					if ( ClockMilliSec > RCTimeOutMilliSec )
					{
						 // stop high-power runaways
						IThrottle = Limit(IThrottle, IThrottle, THR_HOVER);
						IRoll = RollFailsafe;
						IPitch = PitchFailsafe;
						IYaw = YawFailsafe;
					}
				}
			else
				if ( _Signal )
				{
					ALL_LEDS_OFF;
					LedGreen_ON;
					ResetTimeOuts();
					ReadParametersEE();
					
					if ( IThrottle > _ThresStart )
					{
						_Flying = true;
						_MotorsEnabled = true;
						AbsDirection = COMPASS_INVAL;
					}
				}
				else
				{
					// do nothing - Red Led should be flashing
					ALL_LEDS_OFF;
				}		
			CheckAlarms();
			DoDebugTraces();

		} // while arming switch is on
	}
	// CPU should never arrive here
	// do a processor reset ???	
} // Main
