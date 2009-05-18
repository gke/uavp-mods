// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

//#ifdef CLOCK_40MHZ
//#pragma	config OSC=HSPLL, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC
//#else
#pragma	config OSC=HS, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC  
//#endif

#include "uavx.h"

// Global Variables

uint8	IGas;						// actual input channel, can only be positive!
int8 	IRoll,IPitch,IYaw;			// actual input channels, 0 = neutral
uint8	IK5;						// actual channel 5 input
uint8	IK6;						// actual channel 6 input
uint8	IK7;						// actual channel 7 input

// PID Regler Variablen
 	
int16	RollRate, PitchRate, YawRate;
int16	PrevRollRate, PrevPitchRate, PrevYawRate;
int16	RollSum, PitchSum, YawSum;
int16	RollIntLimit256, PitchIntLimit256, YawIntLimit256;	
int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
int16	DesiredThrottle, DesiredRoll, DesiredPitch, DesiredYaw, Heading;
int16 	NavIntLimit256;
i16u	Ax, Ay, Az;
int8	LRIntKorr, FBIntKorr;
int8	NeutralLR, NeutralFB, NeutralUD;
int16 	UDSum, UDAcc, VUDComp;

// Failsafes
uint8	ThrNeutral;

// Variables for barometric sensor PD-controller
int24	BaroBasePressure, BaroBaseTemp;
int16   BaroRelPressure, BaroRelTempCorr;
i16u	BaroVal;
int16	VBaroComp;
uint8	BaroType, BaroTemp, BaroRestarts;

uint8	LEDShadow;		// shadow register
int16	AbsDirection;	// wanted heading (240 = 360 deg)
int16	CurDeviation;	// deviation from correct heading

int16	Rl,Pl,Yl;		// PID output values
uint8	MCamRoll,MCamPitch;
int16	Motor[NoOfMotors];

int16	Trace[LastTrace];

uint8	Flags[32];

uint24	Cycles, ThrCycles, DropoutCycles, GPSCycles, BaroCycles;
int16	FakeGPSCycles;
int16	LEDCount;
uint24	RCGlitches;
int8	BatteryVolts;

#pragma idata params
// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
int8	RollPropFactor		=18;
int8	RollIntFactor		=4;
int8	RollDiffFactor		=-40;
int8	BaroTempCoeff		=13;
int8	RollIntLimit		=16;
int8	PitchPropFactor		=18;
int8	PitchIntFactor		=4;	
int8	PitchDiffFactor		=-40;	 
int8	BaroThrottleProp	=2;
int8	PitchIntLimit		=16;
int8	YawPropFactor		=20;
int8	YawIntFactor		=45;
int8	YawDiffFactor		=6;
int8	YawLimit			=50;
int8	YawIntLimit			=3;
int8	ConfigParam			=0b00000000;
int8	TimeSlot			=2;	// control update interval + LEGACY_OFFSET
int8	LowVoltThres		=43;
int8	CamRollFactor		=4;	
int8	LinFBIntFactor		=0;	// unused
int8	LinUDIntFactor		=8; // unused
int8	MiddleUD			=0;
int8	MotorLowRun			=20;
int8	MiddleLR			=0;
int8	MiddleFB			=0;
int8	CamPitchFactor		=4;
int8	CompassFactor		=5;
int8	BaroThrottleDiff	=4;
#pragma idata

void main(void)
{
	uint8	i;
	uint8	LowGasCycles;

	DisableInterrupts;

	InitPorts();
	InitADC();

	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
	
	InitTimersAndInterrupts();

	for ( i = 0; i<32 ; i++ )
		Flags[i] = false; 
	
	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;
	Beeper_OFF;

	InitArrays();
	ThrNeutral = 0xFF;	
	IGas = DesiredThrottle = IK5 = IK6 = IK7 = _Minimum;

	InitParams();

	INTCONbits.PEIE = true;		// Enable peripheral interrupts
	EnableInterrupts;

	Delay100mSWithOutput(5);	// wait 0.5 sec until LISL is ready to talk
	InitLISL();

	InitDirection();
	InitBarometer();
	InitGPS();
	InitNavigation();

	ShowSetup(1);
	
	while( true )
	{
		ReceivingGPSOnly(false);

		if ( !_Signal )
		{
			IGas = DesiredThrottle = IK5 = _Minimum;	// Assume parameter set #1
			Beeper_OFF;
		}

		INTCONbits.TMR0IE = false;
		ALL_LEDS_OFF; 
		LEDRed_ON;	
		if( _AccelerationsValid )
			LEDYellow_ON;

		InitArrays();
		EnableInterrupts;	
		WaitForRxSignal(); 
		ReadParametersEE();
		WaitThrottleClosed();
		ThrCycles = THR_DOWNCOUNT;
		
		// if Ch7 below +20 (near minimum) assume use for camera trigger
		// else assume use for camera roll trim	
		_UseCh7Trigger = IK7 < 30;
	
		_Flying = false;
		DropoutCycles = MAXDROPOUT; // zzz
		Cycles = 0;

		while ( Armed )
		{	
			ReceivingGPSOnly(true); // no command processing while the Quadrocopter is armed

			Cycles++;

			WriteTimer0(0);
			INTCONbits.TMR0IF = false;
			INTCONbits.TMR0IE = true;

				RollRate = PitchRate = 0;
				GetGyroValues();
				GetDirection();
				ComputeBaroComp();
				CheckAutonomous(); // before timeslot delay to give maximum time	
				while( TimeSlot > 0 ) {}

			INTCONbits.TMR0IE = false;	// disable timer

			ReadParametersEE();	// re-sets TimeSlot
			GetGyroValues();

			if ( _Signal )
			{
				if( (_Flying && (DesiredThrottle <= _ThresStop)) ||
			      				(!_Flying && (DesiredThrottle <= _ThresStart))  )
				{ // Landed
					_Flying = false;

					ThrCycles = THR_DOWNCOUNT;
					InitArrays();

					ErectGyros();	// DON'T MOVE THE UFO! ES KANN LOSGEHEN!

					ALL_LEDS_OFF;				
					AUX_LEDS_OFF;
				}
				else
				{ // Flying
					if( !_Flying )	// about to start
					{	
						AbsDirection = COMPASS_INVAL;						
						LEDCount = 1;
					}
	
					DoControl();

					_Flying = true;
					_LostModel = false;
					LEDGame();
				}

				LEDGreen_ON;
				DropoutCycles = Cycles + MAXDROPOUT;
				LowGasCycles = Cycles + 123;
			}
			else
			{
				_LostModel = true;
				ALL_LEDS_OFF;
				DesiredRoll = DesiredPitch = DesiredYaw = 0;
				if( (Cycles >= DropoutCycles) )
					DesiredThrottle = _Minimum;
				else
					DesiredThrottle = _Minimum;		
				DoControl();
			}		
		
			MixAndLimitCam();
			OutSignals();

			CheckAlarms();
			DumpTrace();
		
		} // flight while armed
	
		Beeper_OFF;
	}
} // main

