// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.

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

uint8	State;

uint8	IGas;						// actual input channel, can only be positive!
int8 	IRoll,IPitch,IYaw;			// actual input channels, 0 = neutral
uint8	IK5;						// actual channel 5 input
uint8	IK6;						// actual channel 6 input
uint8	IK7;						// actual channel 7 input
 	
int16	RollRate, PitchRate, YawRate;
int16	REp, PEp, YEp, RE, PE, YE;
int16	RollSum, PitchSum, YawSum;
int16	RollIntLimit256, PitchIntLimit256, YawIntLimit256, NavIntLimit256;	
int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
int16	DesiredThrottle, HoverThrottle, AutonomousThrottle, DesiredRoll, DesiredPitch, DesiredYaw, Heading;
i16u	Ax, Ay, Az;
int8	LRIntKorr, FBIntKorr;
int8	NeutralLR, NeutralFB, NeutralUD;
int16 	UDSum, UDAcc, VUDComp;
int16	AE, AltSum;

uint8	ThrNeutral;

int24	BaroBasePressure, BaroBaseTemp;
int16   BaroRelPressure, BaroRelTempCorr;
i16u	BaroVal;
int16	VBaroComp;
uint8	BaroType, BaroTemp, BaroRestarts;

uint8	LEDShadow;		// shadow register
int16	AbsDirection;	// desired heading (240 = 360 deg)
int16	CurDeviation;	// deviation from correct heading

int16	Rl,Pl,Yl;		// PID output values
uint8	MCamRoll,MCamPitch;
int16	Motor[NoOfMotors];

int16	Trace[LastTrace];

int16	NavClosingRadius, SqrNavClosingRadius, CompassOffset;

uint8	Flags[32];

uint8	ThrDownCycles, DropoutCycles, GPSCycles, BaroCycles;
int16	FakeGPSCycles;
uint24	Cycles;
uint8	LEDCycles;
uint24	RCGlitches;
int8	BatteryVolts;

#pragma idata params
// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
#pragma udata parameters		
int8 P[64];
#pragma idata

void main(void)
{
	static	uint8	i;
	static	uint8	LowGasCycles;

	DisableInterrupts;

	InitPorts();
	InitADC();

	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
	
	InitTimersAndInterrupts();

	for ( i = 32; i ; i-- )
		Flags[i] = false; 
	
	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;
	Beeper_OFF;

	InitArrays();
	ThrNeutral = 255;	
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
		
		// if Ch7 below +20 (near minimum) assume use for camera trigger
		// else assume use for camera roll trim	
		_UseCh7Trigger = IK7 < 30;
	
		DropoutCycles = MAXDROPOUT;
		Cycles = 0;
		State = Starting;

		while ( Armed )
		{	
			ReceivingGPSOnly(true); // no Command processing while the Quadrocopter is armed

			Cycles++;

			if ( _Signal )
			{
				DesiredThrottle = IGas;
				DesiredRoll = IRoll;
				DesiredPitch = IPitch;
				DesiredYaw = IYaw;

				switch ( State ) {
				case Starting:
					ThrDownCycles = THR_DOWNCOUNT;
					InitArrays();
					ALL_LEDS_OFF;				
					AUX_LEDS_OFF;

					ErectGyros();	// DON'T MOVE THE UFO! ES KANN LOSGEHEN!
	
					LEDGreen_ON;
					State = Landed;
					break;
				case Landed:
					if ( DesiredThrottle >= P[MotorLowRun] )
					{
						AbsDirection = COMPASS_INVAL;						
						LEDCycles = 1;
						State = Flying;
					}
					break;
				case Flying:

					DoNavigation();

					LEDGame();
					LowGasCycles = LOWGASDELAY;
					if ( DesiredThrottle < P[MotorLowRun] )
					{
						DesiredThrottle = P[MotorLowRun];
						State = Landing;
					}
					break;
				case Landing:
					if ( DesiredThrottle >= P[MotorLowRun] )
						State = Flying;
					else
						if ( --LowGasCycles > 0 )
							DesiredThrottle = P[MotorLowRun];
						else
							State = Starting;
					break;
				} // Switch State
				_LostModel = false;
				DropoutCycles = MAXDROPOUT;
			}
			else
			{ // only relevant to PPM Rx
				_LostModel = true;
				ALL_LEDS_OFF;
				DesiredRoll = DesiredPitch = DesiredYaw = 0;
				if( --DropoutCycles > 0 )
					DesiredThrottle = P[MotorLowRun];
				else
					DesiredThrottle = 0;
			}		

			WriteTimer0(0);
			INTCONbits.TMR0IF = false;
			INTCONbits.TMR0IE = true;

				RollRate = PitchRate = 0;
				GetGyroValues();
				GetDirection();
				ComputeBaroComp();
	
				while( P[TimeSlot] > 0 ) {}
			INTCONbits.TMR0IE = false;	// disable timer

			ReadParametersEE();	// re-sets TimeSlot
			GetGyroValues();

			DoControl();
			MixAndLimitMotors();
			MixAndLimitCam();
			OutSignals();

			CheckAlarms();
			DumpTrace();
		
		} // flight while armed
	
		Beeper_OFF;
	}
} // main

