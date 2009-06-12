// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
// =======================================================================

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
int16	RE, PE, YE;					// gyro rate error	
int16	REp, PEp, YEp;				// previous error for derivative
int16	RollSum, PitchSum, YawSum;	// integral 	
int16	RollRate, PitchRate, YawRate;
int16	RollIntLimit256, PitchIntLimit256, YawIntLimit256, NavIntLimit256;
int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
int16	HoverThrottle, DesiredThrottle, DesiredRoll, DesiredPitch, DesiredYaw, Heading;
i16u	Ax, Ay, Az;
int8	LRIntKorr, FBIntKorr;
int8	NeutralLR, NeutralFB, NeutralUD;
int16 	UDAcc, UDSum, VUDComp;

int16 	SqrNavClosingRadius, NavClosingRadius, CompassOffset;

int16	AltSum, AE;

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

uint8	MCamRoll,MCamPitch;
int16	Motor[NoOfMotors];

int16	Rl,Pl,Yl;		// PID output values
int16	Vud;

int16	Trace[LastTrace];

uint8	Flags[32];

int16	ThrDownCycles, DropoutCycles, GPSCycles, LEDCycles, BlinkCycle, BaroCycles;
int16	FakeGPSCycles;
uint32	Cycles;
int8	IntegralCount;
uint24	RCGlitches;
int8	BatteryVolts;
int8	Rw,Pw;

#pragma udata params
// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
int8	RollPropFactor;
int8	RollIntFactor;
int8	RollDiffFactor;
int8	BaroTempCoeff;
int8	RollIntLimit;
int8	PitchPropFactor;
int8	PitchIntFactor;	
int8	PitchDiffFactor;	 
int8	BaroThrottleProp;
int8	PitchIntLimit;
int8	YawPropFactor;
int8	YawIntFactor;
int8	YawDiffFactor;
int8	YawLimit;
int8	YawIntLimit;
int8	ConfigParam;
int8	TimeSlot;	// control update interval + LEGACY_OFFSET
int8	LowVoltThres;
int8	CamRollFactor;	
int8	LinFBIntFactor;	// unused
int8	LinUDIntFactor; // unused
int8	MiddleUD;
int8	MotorLowRun;
int8	MiddleLR;
int8	MiddleFB;
int8	CamPitchFactor;
int8	CompassFactor;
int8	BaroThrottleDiff;
int8	NavRadius;
int8	NavIntLimit;
int8	NavAltKp;
int8	NavAltKi;
int8	NavRTHAlt;
int8	NavMagVar;
#pragma udata

void main(void)
{
	uint8	i;
	uint8	LowGasCycles;

	DisableInterrupts;

	InitPorts();
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
	
	InitADC();
	
	InitTimersAndInterrupts();

	for ( i = 0; i<32 ; i++ )
		Flags[i] = false; 
	
	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;

	InitArrays();
	ReadParametersEE();

	INTCONbits.PEIE = true;		// Enable peripheral interrupts
	EnableInterrupts;

	Delay100mSWithOutput(5);	// wait 0.5 sec until LISL is ready to talk
	InitLISL();

	InitDirection();
	InitBarometer();
	InitGPS();
	InitNavigation();

	ShowSetup(1);

	ThrNeutral = 0xFF;
	IK6 = IK7 = _Minimum;
	Cycles = 0;

Restart:
	IGas = DesiredThrottle = IK5 = _Minimum;	// Assume parameter set #1
	Beeper_OFF;

	// DON'T MOVE THE UFO!
	// ES KANN LOSGEHEN!

	while( true )
	{
		INTCONbits.TMR0IE = false;		// Disable TMR0 interrupt

		// no command processing while the Quadrocopter is armed
		ReceivingGPSOnly(false);

		ALL_LEDS_OFF;
		LEDRed_ON;	
		if( _AccelerationsValid )
			LEDYellow_ON;

		InitArrays();
		ThrNeutral = 0xFF;

		EnableInterrupts;	
		WaitForRxSignal(); // Wait until a valid RX signal is received
		ReadParametersEE();
		WaitThrottleClosed();

		if ( !_Signal )
			goto Restart;

		// ######## MAIN LOOP ########

		// loop length is controlled by a programmable variable "TimeSlot"

		DropoutCycles = 0;
		IntegralCount = 16;	// do 16 cycles to find integral zero point
		ThrDownCycles = THR_DOWNCOUNT;

		// if Ch7 below +20 (near minimum) assume use for camera trigger
		// else assume use for camera roll trim	
		_UseCh7Trigger = IK7 < 30;
	
		while ( Armed )
		{
			Cycles++;

			ReceivingGPSOnly(true);

			// wait pulse pause delay time (TMR0 has 1024us for one loop)
			WriteTimer0(0);
			INTCONbits.TMR0IF = false;
			INTCONbits.TMR0IE = true;

			RollRate = PitchRate = 0;	// zero gyros sum-up memory
			// sample gyro data and add everything up while waiting for timing delay

			GetGyroValues();

			GetDirection();
			CheckAutonomous(); // before timeslot delay to give maximum time

			while( TimeSlot > 0 ) {}

			INTCONbits.TMR0IE = false;	// disable timer
			
			ComputeBaroComp();
			GetGyroValues();
			ReadParametersEE();	// re-sets TimeSlot
			CalcGyroValues();

			// check for signal dropout while in flight
			if( _Flying && !_Signal )
			{
				if( ( Cycles & 0x000f ) == 0 )
					DropoutCycles++;
				if( DropoutCycles < MAXDROPOUT )
				{	// FAILSAFE	- hold last throttle
					_LostModel = true;
					ALL_LEDS_OFF;
					DesiredRoll = DesiredPitch = DesiredYaw = 0;
					goto DoPID;
				}
				break;	// timeout, stop everything
			}

			// allow motors to run on low throttle 
			// even if stick is at minimum for a short time
			if( _Flying && ( DesiredThrottle <= _ThresStop ) )
				if( --LowGasCycles > 0 )
					goto DoPID;

			if( ( !_Signal ) || 
			    ( (_Flying && (DesiredThrottle <= _ThresStop)) ||
			      (!_Flying && (DesiredThrottle <= _ThresStart)) ) )
			{	// UFO is landed, stop all motors

				TimeSlot += 2; // to compensate PID() calc time!
				IntegralCount = 16;	// do 16 cycles to find integral zero point
				ThrDownCycles = THR_DOWNCOUNT;
				
				InitArrays();	// resets _Flying flag!
				GyroMidRoll = GyroMidPitch = GyroMidYaw = 0;
				if( Armed && !_Signal )	
					break;	// then RX signal was lost

				ALL_LEDS_OFF;				
				AUX_LEDS_OFF;
				LEDGreen_ON;
			}
			else
			{	// UFO is flying!
				if( !_Flying )	// about to start
				{	
					AbsDirection = COMPASS_INVAL;						
					LEDCycles = 1;
				}

				_Flying = true;
				_LostModel = false;
				DropoutCycles = 0;
				LowGasCycles = 100;		
				LEDGreen_ON;
				LEDGame();
DoPID:
				CheckThrottleMoved();

				if(	IntegralCount > 0 )
					IntegralCount--;
				else
				{
					PID();
					MixAndLimitMotors();
				}

				// remember old gyro values
				REp = RE;
				PEp = PE;
				YEp = YE;
			}
		
			MixAndLimitCam();
			OutSignals();

			CheckAlarms();

			if( IntegralCount == 0 )
				DumpTrace();		

		} // flight while armed

		Beeper_OFF;
	}
} // main

