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


uint16	PauseTime; 					// for tests

#pragma udata clocks
uint24	mS[CompassUpdate+1];
#pragma udata

#pragma udata isrvars
int16	PPM[CONTROLS];
int16 	RC[CONTROLS];
boolean	RCFrameOK;
int8	PPM_Index;
int24	PrevEdge, CurrEdge;
uint16 	Width, CurrCCPR1;
#pragma udata

uint8	State;
uint8	CurrentParamSet;

// PID Regler Variablen
int16	RE, PE, YE;					// gyro rate error	
int16	REp, PEp, YEp;				// previous error for derivative
int16	RollSum, PitchSum, YawSum;	// integral 	
int16	RollRate, PitchRate, YawRate;
int16	RollIntLimit256, PitchIntLimit256, YawIntLimit256, NavIntLimit256;
int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
int16	HoverThrottle, DesiredThrottle, IdleThrottle;
int16	DesiredRoll, DesiredPitch, DesiredYaw, Heading;
i16u	Ax, Ay, Az;
int8	LRIntKorr, FBIntKorr;
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
int16	AbsDirection;	// wanted heading (240 = 360 deg)
int16	CurDeviation;	// deviation from correct heading

uint8	MCamRoll,MCamPitch;
int16	Motor[NoOfMotors];

int16	Rl,Pl,Yl;		// PID output values
int16	Vud;

int16	Trace[LastTrace];

boolean	Flags[32];

uint8	LEDCycles;	
int8	IntegralCount;
uint24	RCGlitches;
int8	BatteryVolts;
int8	Rw,Pw;

#pragma udata params
// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
int8	RollKp;
int8	RollKi;
int8	RollKd;
int8	BaroTempCoeff;
int8	RollIntLimit;
int8	PitchKp;
int8	PitchKi;	
int8	PitchKd;	 
int8	BaroCompKp;
int8	PitchIntLimit;
int8	YawKp;
int8	YawKi;
int8	YawKd;
int8	YawLimit;
int8	YawIntLimit;
int8	ConfigParam;
int8	TimeSlots;	// control update interval + LEGACY_OFFSET
int8	LowVoltThres;
int8	CamRollKp;	
int8	PercentHoverThr;
int8	VertDampKp; 
int8	MiddleUD;
int8	PercentIdleThr;
int8	MiddleLR;
int8	MiddleFB;
int8	CamPitchKp;
int8	CompassKp;
int8	BaroCompKd;
int8	NavRadius;
int8	NavIntLimit;
int8	NavAltKp;
int8	NavAltKi;
int8	NavRTHAlt;
int8	NavMagVar;
int8 	GyroType;			
int8 	ESCType;
int8	TxRxType;		
#pragma udata

// mask giving common variables across parameter sets
const rom int8	ComParms[]={
	0,0,0,1,0,0,0,0,1,0,
	0,0,0,0,0,1,1,1,0,1,
	1,1,1,1,1,0,0,1,0,0,
	0,0,0,1,1,1,1
	};

// 1 Throttle
// 2 Aileron
// 3 Elevator
// 4 Rudder
// 5 Gear
// 6 Aux1
// 7 Aux2

const rom uint8 Map[DX7+1][CONTROLS]=
	{
		{ 3,1,2,4,5,6,7 }, 	// Futaba Traditional
		{ 5,3,2,1,6,4,7 },	// Futaba 9C Spektrum DM8
		{ 1,2,3,4,5,6,7 },	// JR Traditional
		{ 7,1,4,6,3,5,2 },	// JR 9XII Spektrum DM9 ?
		{ 6,1,4,7,3,5,2 },	// JR DXS12 ?
		{ 6,1,4,7,3,5,2 },	// Spektrum DX7
	};

void main(void)
{
	static uint8	i;
	static int16	Temp;

	DisableInterrupts;

	InitPorts();
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);

	InitADC();
	
	InitTimersAndInterrupts();

	CurrentParamSet = 1;
	ReadParametersEE();

	for ( i = 0; i<32 ; i++ )
		Flags[i] = false; 
	
	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;
	Beeper_OFF;

	InitArrays();
	ThrNeutral = ThrLow = ThrHigh = MAXINT16;	
	RC[ThrottleC] = DesiredThrottle = RC[RTHC] = RC[CamTiltC] = RC[NavGainC] = OUT_MINIMUM;

	INTCONbits.PEIE = true;		// Enable peripheral interrupts
	INTCONbits.TMR0IE = true; 
	EnableInterrupts;

	Delay100mSWithOutput(5);	// wait 0.5 sec until LISL is ready to talk
	InitLISL();

	InitDirection();
	InitBarometer();

	ShowSetup(1);
	
	while( true )
	{

		ReceivingGPSOnly(false);

		if ( !_Signal )
		{
			RC[ThrottleC] = RC[RTHC] = DesiredThrottle = OUT_MINIMUM;
			Beeper_OFF;
		}

		ALL_LEDS_OFF; 
		LEDRed_ON;	
		if( _AccelerationsValid )
			LEDYellow_ON;

		InitArrays();
		EnableInterrupts;	
		WaitForRxSignal();
		WaitThrottleClosed();		
		DesiredThrottle = 0;

		_Failsafe = _LostModel = false;
		mS[FailsafeTimeout] = mS[Clock] + FAILSAFE_TIMEOUT;
		mS[AbortTimeout] = mS[Clock] + ABORT_TIMEOUT;

		State = Starting;
		mS[UpdateTimeout] = mS[Clock] + TimeSlots;

		while ( Armed  )
		{	
			ReceivingGPSOnly(true); // no Command processing while the Quadrocopter is armed

			UpdateGPS();

			if ( _Signal && !_Failsafe )
			{
				UpdateControls();

				switch ( State  ) {
				case Starting:
					InitArrays();

					// reset origin position and altitude
					InitBarometer();
					InitGPS();
					InitNavigation();

					#ifdef NEW_ERECT_GYROS
					ErectGyros();
					IntegralCount = 0;
					#else
					IntegralCount = 16; // erect gyros - old style
					#endif

					ALL_LEDS_OFF;				
					AUX_LEDS_OFF;

					LEDGreen_ON;
					State = Landed;
					break;
				case Landed:
					if ( (DesiredThrottle >= IdleThrottle ) && (IntegralCount == 0) )
					{
						AbsDirection = COMPASS_INVAL;						
						LEDCycles = 1;
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
							State = Starting;
							Temp = ToPercent(HoverThrottle, OUT_MAXIMUM);
							WriteEE(_EESet1 + (&PercentHoverThr - &FirstProgReg), Temp);
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

			RollRate = PitchRate = 0;
			GetGyroValues();
			GetDirection();
			GetBaroPressure();
	
			while ( mS[Clock] < mS[UpdateTimeout] ) {};
			mS[UpdateTimeout] = mS[Clock] + TimeSlots;

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

