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

#pragma udata clocks
uint24	mS[CompassUpdate+1];
#pragma udata

// Interrupt related 
#pragma udata access isrvars
uint8 	SHADOWB, MF, MB, ML, MR, MT, ME; // motor/servo outputs
i16u 	PPM[CONTROLS];
int8 	PPM_Index;
int24 	PrevEdge, CurrEdge;
i16u 	Width;
int16 	PauseTime;
uint8 	GPSRxState;
boolean PosPPM, RCFrameOK, GPSSentenceReceived;
uint8 	ll, tt, gps_ch;
uint8 	RxCheckSum, GPSCheckSumChar, GPSTxCheckSum;
#pragma udata

int16 	RC[CONTROLS];
uint24	RCGlitches; 

#pragma udata gpsbuff
struct {
	uint8 s[GPSRXBUFFLENGTH];
	uint8 length;
	} NMEA;
#pragma udata

const rom uint8 NMEATag[6] = {"GPGGA"};

// Flight State
uint8	State;
uint8	CurrentParamSet;

// Control
int16	RE, PE, YE, HE;					// gyro rate error	
int16	REp, PEp, YEp, HEp;				// previous error for derivative
int16	RollSum, PitchSum, YawSum;		// integral 	
int16	RollRate, PitchRate, YawRate;
int16	RollTrim, PitchTrim, YawTrim;
int16	HoldRoll, HoldPitch, HoldYaw;
int16	RollIntLimit256, PitchIntLimit256, YawIntLimit256, NavIntLimit256;
int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
int16	HoverThrottle, DesiredThrottle, IdleThrottle;
int16	DesiredRoll, DesiredPitch, DesiredYaw, DesiredHeading, Heading;
i16u	Ax, Ay, Az;
int8	LRIntKorr, FBIntKorr;
int16	Rl,Pl,Yl;						// PID output values
int8	NeutralLR, NeutralFB, NeutralUD;
int16 	UDAcc, UDSum, VUDComp;

int16 	SqrNavClosingRadius, NavClosingRadius, CompassOffset;

uint8 	NavState;
uint8 	NavSensitivity;
int16	AltSum, AE;

int16	ThrLow, ThrHigh, ThrNeutral;

// Variables for barometric sensor PD-controller
int24	OriginBaroPressure;
int16	DesiredBaroPressure, CurrentBaroPressure;
int16	BE, BEp;
i16u	BaroVal;
int8	BaroSample;
int16	VBaroComp;
uint8	BaroType, BaroTemp;

uint8	LEDShadow;		// shadow register

uint8	MCamRoll,MCamPitch;
int16	Motor[NoOfMotors];

int16	Trace[TopTrace+1];
boolean	Flags[32];
uint8	LEDCycles;
int16	HoldResetCount;	
int8	BatteryVolts;

#pragma udata params
int8 P[LastParam+1];
#pragma udata

// mask giving common variables across parameter sets
const rom int8	ComParms[]={
	0,0,0,1,0,0,0,0,1,0,
	0,0,0,0,0,1,1,1,0,1,
	1,1,1,1,1,0,0,1,0,0,
	0,0,0,1,1,1,1
	};

// Reference Internal Quadrocopter Channel Order
// 1 Throttle
// 2 Aileron
// 3 Elevator
// 4 Rudder
// 5 Gear
// 6 Aux1
// 7 Aux2

const rom uint8 Map[CustomTxRx+1][CONTROLS] =
	{
		{ 3,1,2,4,5,6,7 }, 	// Futaba Ch3 Throttle
		{ 2,1,4,3,5,6,7 },	// Futaba Ch2 Throttle
		{ 5,3,2,1,6,4,7 },	// Futaba 9C Spektrum DM8/AR7000
		{ 1,2,3,4,5,6,7 },	// JR XP8103/PPM
		{ 7,1,4,6,3,5,2 },	// JR 9XII Spektrum DM9 ?

		{ 6,1,4,7,3,2,5 },	// JR DXS12 
		{ 6,1,4,7,3,2,5 },	// Spektrum DX7/AR7000
		{ 5,1,4,6,3,2,7 },	// Spektrum DX7/AR6200
		{ 6,1,4,7,3,2,5 } 	// custom Tx/Rx combination
	};

/*
// Rx signalling polarity - this is NOT the edge polarity as 
// seen by the PIC as it is inverted by the wired NOR
const rom boolean PPMPosPolarity[CustomTxRx+1] =
	{
		false, 	// Futaba Ch3 Throttle
		false,	// Futaba Ch2 Throttle
		true,	// Futaba 9C Spektrum DM8/AR7000
		true,	// JR XP8103/PPM
		true,	// JR 9XII Spektrum DM9 ?

		true,	// JR DXS12
		true,	// Spektrum DX7/AR7000
		true,	// Spektrum DX7/AR6200
		true	// custom Tx/Rx combination
	};

*/
void main(void)
{
	static int16	Temp;

	DisableInterrupts;

	InitMisc();
	InitPorts();

	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);

	InitADC();		
	InitParameters();
	InitTimersAndInterrupts();

	StopMotors();
	INTCONbits.PEIE = true;		// Enable peripheral interrupts
	INTCONbits.TMR0IE = true; 
	EnableInterrupts;

	InitLISL();

	InitCompass();
	InitBarometer();
	InitGPS();
	InitNavigation();

	ShowSetup(1);
	
	while( true )
	{
		StopMotors();

		ReceivingGPSOnly(false);

		Beeper_OFF;
		ALL_LEDS_OFF; 
		LEDRed_ON;	
		if( _AccelerationsValid ) LEDYellow_ON;

		EnableInterrupts;	
		WaitForRxSignalAndArmed();			// WAITS here until ARMED
		WaitThrottleClosedAndRTHOff();		

		_Failsafe = _LostModel = false;
		mS[FailsafeTimeout] = mS[Clock] + FAILSAFE_TIMEOUT;
		mS[AbortTimeout] = mS[Clock] + ABORT_TIMEOUT;
		mS[UpdateTimeout] = mS[Clock] + P[TimeSlots];

		State = Starting;

		while ( Armed && !_ParametersInvalid )
		{ // No Command processing while the Quadrocopter is armed
	
			ReceivingGPSOnly(true); 

			UpdateGPS();
			UpdateControls();

			if ( _Signal && !_Failsafe )
			{
				switch ( State  ) {
				case Starting:	// this state executed once only after arming
					InitControl();
					CaptureTrims();

					InitHeading();
					InitBarometer();
					InitNavigation();
					ResetGPSOrigin();
					ErectGyros();			// DO NOT MOVE QUADROCOPTER!
			
					ALL_LEDS_OFF;				
					AUX_LEDS_OFF;
					LEDGreen_ON;
					DesiredThrottle = 0;
					State = Landed;
					break;
				case Landed:
					if ( DesiredThrottle >= IdleThrottle  )
					{
						InitHeading();						
						LEDCycles = 1;
						mS[NavActiveTime] = mS[Clock] + NAV_ACTIVE_DELAY;
						State = InFlight;
					}
					break;
				case Landing:
					if ( DesiredThrottle > IdleThrottle )
						State = InFlight;
					else
						if ( mS[Clock] < mS[ThrottleIdleTimeout] )
							DesiredThrottle = IdleThrottle;
						else
						{
							State = Landed;
							Temp = ToPercent(HoverThrottle, OUT_MAXIMUM);
							WriteEE((CurrentParamSet-1) * MAX_PARAMETERS + PercentHoverThr, Temp);
						}
					break;
				case InFlight:
					DoNavigation();

					LEDGame();

					if ( DesiredThrottle < IdleThrottle )
					{
						mS[ThrottleIdleTimeout] = mS[Clock] + LOW_THROTTLE_DELAY;
						State = Landing;
					}
					break;

				} // Switch State
				_LostModel = false;
				mS[FailsafeTimeout] = mS[Clock] + FAILSAFE_TIMEOUT;
				mS[AbortTimeout] = mS[Clock] + ABORT_TIMEOUT;
			}
			else
				DoFailsafe();

			GetGyroValues();				// First gyro read
			GetHeading();
			GetBaroPressure();
	
			while ( mS[Clock] < mS[UpdateTimeout] ) {}; // cycle sync. point
			mS[UpdateTimeout] = mS[Clock] + P[TimeSlots];

			GetGyroValues();				// Second gyro read
			DoControl();
			MixAndLimitMotors();
			MixAndLimitCam();
			OutSignals();

			CheckAlarms();
			DumpTrace();
		
		} // flight while armed
	}
} // main

