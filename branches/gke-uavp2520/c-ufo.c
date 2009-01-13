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
uint8 SHADOWB, CAMTOGGLE, MF, MB, ML, MR, MT, ME; 			// motor/servo outputs
// Bootloader

#pragma udata

#pragma udata globals
// Globals 									
int32	ClockMilliSec, TimerMilliSec;
int8	TimeSlot;

// RC
uint8	IThrottle;
int16 	IRoll,IPitch,IYaw;
uint8	IK5,IK6,IK7;
int16	MidRoll, MidPitch, MidYaw;				// mid RC stick values

// Gyros
int32	RollAngle, PitchAngle, YawAngle;		// PID integral (angle)
int16	RollRate, PitchRate, YawRate;			// PID rate (scaled gyro values)

// Acceleration Corrections
int32	UDVelocity;
int16	Ax, Ay, Az;								// LISL sensor accelerations							
int16	LRIntKorr, FBIntKorr;
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
uint8	ThrDownCount;

uint8	BlinkCount;
uint8	LedShadow;								// shadow register
uint8	LedCount;
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
void InitArrays(void)
{
	// RC
	_Flying = false;
	IThrottle = 0;
	CurrThrottle = 0xFF;
	IRoll = IPitch = IYaw = IK5 = IK6 = IK7 = 0;

	// PID
	REp = PEp = YEp = 0;

	// Drives
	_MotorsEnabled = false;
	Rl = Pl = Yl = VBaroComp =0;
	Vud = 0;
	MFront = _Minimum;	
	MLeft = _Minimum;
	MRight = _Minimum;
	MBack = _Minimum;

	MCamRoll = MCamPitch = _Neutral;

	// Misc	
	BlinkCount = 0;
	ThrDownCount = THR_DOWNCOUNT;
					
} // InitArrays

void CheckThrottleMoved(void)
{
	int16	Temp;

	if( ThrDownCount > 0 )
	{
		if( LedCount & 1 )
			ThrDownCount--;
		if( ThrDownCount == 0 )
			CurrThrottle = IThrottle;						// remember current Throttle level
	}
	else
	{
		if( CurrThrottle < THR_MIDDLE ) 				// ??? tidy up
			Temp = 0;
		else
			Temp = CurrThrottle - THR_MIDDLE;
		if( IThrottle < THR_HOVER )
			ThrDownCount = THR_DOWNCOUNT;			// left dead area
		if( IThrottle < Temp )
			ThrDownCount = THR_DOWNCOUNT;			// left dead area
		if( IThrottle > CurrThrottle + THR_MIDDLE )
			ThrDownCount = THR_DOWNCOUNT;			// left dead area
	}
} // CheckThrottleMoved

void main(void)
{
	uint8	DropoutCount;
	uint8	LowThrottleCount;
	uint8	i, ch;

	DisableInterrupts;								// disable all interrupts

	InitPorts();

	LedShadow = 0;
    ALL_LEDS_OFF;

	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
	
	InitADC();
						
	for (i=8; i ; i--)								// clear flags ???
	{
		Flags[i] = false;
		Flags2[i] = false;
	}    

	_NoSignal = true;
	_NewValues = false;	

	InitArrays();
	CurrThrottle = 0xff;
	RollFailsafe = PitchFailsafe = YawFailsafe = 0;

#ifdef COMMISSIONING
	for (i=_EESet2*2; i ; i--)						// clear EEPROM parameter space
		WriteEE(i, -1);
	WriteParametersEE(1);							// Copy RAM initial values to EE
	WriteParametersEE(2);
#endif
	ReadParametersEE();

	InitTimersAndInterrupts();
								
#ifdef BOOTONLY
	ShowSetup(1);
	Delay100mSec(2);								// wait 2/10 sec until signal is there	
	while(1)ProcessCommand();
#else

	// DON'T MOVE THE UFO!
	// ES KANN LOSGEHEN!
	LedRed_ON;
	Delay100mSec(20);								// time to get hands away after power up
	Beeper_ON;
	InitDirection();		
	InitAltimeter();
	InitAccelerometers();
	InitAttitude();					
	Beeper_OFF;
	LedRed_OFF;

	// send "hello" text to USART
	ShowSetup(1);
	
Restart:
	IThrottle = IK5 = _Minimum;							// Kill throttle assume parameter set #1

	while(1)
	{
		Beeper_OFF;
		ALL_LEDS_OFF;

		LedRed_ON;
		if(_UseLISL)
			LedYellow_ON;							// to signal LISL sensor is active

		InitArrays();

		// Wait until a valid RC signal is received
		DropoutCount = MODELLOSTTIMER;
		do
		{
			Delay100mSec(2);						// wait 2/10 sec until signal is there
			ProcessCommand();
			if( _NoSignal )
			{
				if( Switch )
				{
					if( --DropoutCount == 0 )
					{
						Beeper_TOG;					// toggle beeper "model lost"
						DropoutCount = MODELLOSTTIMERINT;
					}
				}
				else
					Beeper_OFF;
			}
		}
		while( _NoSignal || !Switch );				// no signal or switch is off
		Beeper_OFF;
		LedRed_OFF;									// Rx Signal is OK
		
		ReadParametersEE();							// in case param set changed

		// Just for safety: don't let motors start if throttle is open!
		DropoutCount = 1;

		while( IThrottle >= _ThresStop )
		{
			if( _NoSignal )
				goto Restart;

			if( _NewValues )
			{
				_NewValues = false;
				// if Ch7 below midpoint assume use for camera trigger
				// else assume use for camera roll trim	
				_UseCh7Trigger =  IK7 < _Neutral;
				OutSignals();						// while waiting sync to Rx frame
				if( (--DropoutCount) <= 0 )
				{
					LedRed_TOG;						// toggle red LED 
					DropoutCount = 10;				// to signal: THROTTLE OPEN
				}
			}
			ProcessCommand();
		}

		// ## MAIN LOOP ##
		// Loop length is controlled by a programmable variable "TimeSlot". 
		// Standard PPM ESCs will need at least 9 or 10mS.
							
		DropoutCount = 0;
		
		while( Switch )								// as int32 as power switch is ON
		{
			_MotorsEnabled = true;

			RollRate = GetRollRate();				// first of two samples per cycle
			PitchRate = GetPitchRate();

			while( TimeSlot > 0 )
			{
			// Here is the place to insert your own routines
			// It should consume as little time as possible!
			// ATTENTION: Your routine must return BEFORE TimeSlot reaches 0
			// or non-optimal flight behavior might occur!!!
			} // user code

			GetDirection();
			GetAltitude();
	
			ReadParametersEE();						// re-sets TimeSlot

			// second gyro sample delayed roughly by intervening routines!
			// no obvious reason for this except minor filtering by averaging.
			RollRate += GetRollRate();
			PitchRate += GetPitchRate();
			YawRate = GetYawRate();
			DetermineAttitude();

			// check for signal dropout while in flight
			if ( _Flying && _NoSignal )
			{
				if( (BlinkCount & 0x07) == 0 )
					Beeper_TOG;
				else
					if( (BlinkCount & 0x0F) == 0 )
						DropoutCount++;
				if( DropoutCount < MAXDROPOUT )		// Failsafe - currently a minute or so!
				{
					ALL_LEDS_OFF;
					IThrottle = Limit(IThrottle, IThrottle, THR_HOVER); // stop high-power runaways
					IRoll = RollFailsafe;
					IPitch = PitchFailsafe;
					IYaw = YawFailsafe;
					goto DoPID;
				}
				else
					break;							
			}
	
			// allow motors to run on low throttle 
			// even if stick is at minimum for a short time (~2 Sec.)
			if( _Flying && (IThrottle <= _ThresStop) && ((--LowThrottleCount) > 0 ) )
				goto DoPID;

			if( _NoSignal||((_Flying&&(IThrottle<=_ThresStop))||(!_Flying&&(IThrottle<=_ThresStart))))
			{						
				// Quadrocopter has "landed", stop all motors									
				TimeSlot += 2; 						// ??? kludge to compensate PID() calc time!

				InitArrays();						// resets _Flying flag!
				
				if( _NoSignal && Switch )			// _NoSignal set, but Switch is on?
					break;							// then Rx signal was lost

				ALL_LEDS_OFF;				
				AUX_LEDS_OFF;

				LedGreen_ON;

				ProcessCommand();
			}
			else
			{	// Quadrocopter is flying!
				if( !_Flying )						// about to start
				{	// set current stick values as midpoints
					RollFailsafe = IRoll;			// ??? best place to do this
					PitchFailsafe = IPitch;
					YawFailsafe  = IYaw;

					AbsDirection = COMPASS_INVAL;

					ProcessCommand();
						
					LedCount = 1;
				}

				_Flying = true;
				if( DropoutCount )
					Beeper_OFF;						// turn off signal lost beeper

				DropoutCount = 0;
				LowThrottleCount = 100;

				LEDGame();					

DoPID:
				// do the control calculations

				// this block checks if throttle stick has moved
				if( _NewValues )
					CheckThrottleMoved();

				PID();
	
				REp = RE;							// remember old gyro values
				PEp = PE;
				YEp = YE;
			}			

			OutSignals();							// update motors and cam servos

			// do housekeeping after main control and motor/servo drive output
			CheckLowBattery();			
			DoDebugTraces();

			if( BlinkCount <= 0 )
				BlinkCount = BLINK_LIMIT;
			BlinkCount--;
		}	// while switch is on

		_MotorsEnabled = false;
		Beeper_OFF;
	}
	// CPU should never arrive here
	ALL_OUTPUTS_OFF;
	goto Restart; // worth a try!
#endif	
}
