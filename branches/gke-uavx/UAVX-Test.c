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

#ifdef ICD2_DEBUG
#pragma	config OSC=HS, WDT=OFF, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC 
#else
#pragma	config OSC=HS, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC 
#endif

#include "uavx.h"

// Global Variables

#pragma udata mainvars
uint8	IGas;	
uint8	IK5, IK6, IK7;
int8 	IRoll,IPitch,IYaw;

// PID Regler Variablen
int16	RE, PE, YE;					// gyro rate error	
int16	REp, PEp, YEp;				// previous error for derivative
int16	RollSum, PitchSum, YawSum;	// integral 	
int16	RollSamples, PitchSamples;
int16	AverageYawRate, YawRate;
int16	MidRoll, MidPitch, MidYaw;
int16 	DesiredThrottle, DesiredRoll, DesiredPitch, DesiredYaw;
int16	Ax, Ay, Az;
int8	LRIntKorr, FBIntKorr;
int16 	UDSum;
int8	NeutralLR, NeutralFB, NeutralUD;

// Variables for barometric sensor PD-controller
int24	BaroBasePressure, BaroBaseTemp;
int16	BaroRelPressure, BaroRelTempCorr;
int16	VBaroComp;
uint16	BaroVal;
uint8	BaroType, BaroTemp, BaroRestarts;

uint8	LEDShadow;		// shadow register
int16	AbsDirection;	// wanted heading (240 = 360 deg)
int16	CurDeviation;	// deviation from correct heading

//int8	RCRollNeutral, RCPitchNeutral, RCYawNeutral;
int16 	CompassHeading;

int16	GPSCount;

uint8	MCamRoll,MCamPitch;
int16	Motor[NoOfMotors];

int16	Rl,Pl,Yl;		// PID output values
int16	Rp,Pp,Yp;
int16	Vud;

uint8	Flags[32];

#ifdef DEBUG_SENSORS
int16	Trace[LastTrace];
#endif // DEBUG_SENSORS

uint16	PauseTime;

int16	IntegralCount, ThrDownCount, DropoutCount, LEDCount, BaroCount;
int16	FakeGPSCount;
uint32 	BlinkCount;
uint24	RCGlitchCount;
int8	BatteryVolts;
#pragma udata

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
int8	YawIntFactor		=40;
int8	YawDiffFactor		=6;
int8	YawLimit			=50;
int8	YawIntLimit			=6;
int8	ConfigParam			=0b00000000;
int8	TimeSlot			=4;	// control update interval + LEGACY_OFFSET
int8	LowVoltThres		=43;
int8	CamRollFactor		=0;	// unused
int8	LinFBIntFactor		=0;	// unused
int8	LinUDIntFactor		=8;
int8	MiddleUD			=0;
int8	MotorLowRun			= 40;
int8	MiddleLR			=0;
int8	MiddleFB			=0;
int8	CamPitchFactor		=0x44;
int8	CompassFactor		=5;
int8	BaroThrottleDiff	=4;
#pragma idata
// End Parameters

void main(void)
{
	int8	i;

	DisableInterrupts;

	InitPorts();
	InitADC();
 
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400 );

	InitTimersAndInterrupts();

	INTCONbits.TMR0IE = false;

	// setup flags register
	for ( i = 0; i<32 ; i++ )
		Flags[i] = false;

	PauseTime = 0;

	InitArrays();

	#ifdef INIT_PARAMS
	for (i=_EESet2*2; i ; i--)					// clear EEPROM parameter space
		WriteEE(i, -1);
	WriteParametersEE(1);						// copy RAM initial values to EE
	WriteParametersEE(2);
	#endif // INIT_PARAMS
	IK5 = _Minimum;
	ReadParametersEE();

    ALL_LEDS_OFF;
	Beeper_OFF;
	LEDBlue_ON;
	AUX_LEDS_ON;

	InitGPS();
	InitNavigation();

	INTCONbits.PEIE = true;		
	EnableInterrupts;

	Delay1mS(1000);
	InitLISL();

	InitBarometer();
	Delay1mS(BARO_PRESS_TIME);

	InitDirection();
	Delay1mS(COMPASS_TIME);

	// send hello text to serial COM
	Delay1mS(100);
	ShowSetup(true);

	while(1)
	{
		// turn red LED on of signal missing or invalid, green if OK
		// Yellow led to indicate linear sensor functioning.
		if( !( _Signal && Armed ) )
		{
			LEDRed_ON;
			LEDGreen_OFF;
			if ( _UseLISL  )
				LEDYellow_ON;
		}
		else
		{
			LEDGreen_ON;
			LEDRed_OFF;
			LEDYellow_OFF;
		}

		ReadParametersEE();
		ProcessComCommand();

	}
} // main

