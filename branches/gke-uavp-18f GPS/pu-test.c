// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                           http://uavp.ch                            =
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

#include "c-ufo.h"
#include "bits.h"

// The globals

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

uint8	LedShadow;		// shadow register
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

int16	IntegralCount, ThrDownCount, DropoutCount, LedCount, BaroCount;
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


void LinearTest(void)
{
	TxString("\r\nAccelerometer test:\r\n");

	if( _UseLISL )
	{
		ReadAccelerations();
	
		TxString("Left->Right: \t");
		TxVal32(((int32)Ax*1000+512)/1024, 3, 'G');	
		if ( Abs(Ax) > 128 )
			TxString(" fault?");
		TxNextLine();

		TxString("Front->Back: \t");	
		TxVal32(((int32)Az*1000+512)/1024, 3, 'G');
		if ( Abs(Az) > 128 )
			TxString(" fault?");	
		TxNextLine();

		TxString("Down->Up:    \t");
	
		TxVal32(((int32)Ay*1000+512)/1024, 3, 'G');
		if ( ( Ay < 896 ) || ( Ay > 1152 ) )
			TxString(" fault?");	
		TxNextLine();
	}
	else
		TxString("\r\n(Acc. not present)\r\n");
} // LinearTest

uint8 ScanI2CBus(void)
{
	int16 s;
	uint8 d = 0;

	for(s=0x10; s<=0xf6; s+=2)
	{	// use all uneven addresses for read mode
		I2CStart();
		if( SendI2CByte(s) == I2C_ACK )
		{
			d++;
			TxString("\t0x");
			TxValH(s);
			TxNextLine();
		}
		I2CStop();

		Delay1mS(2);
	}
	return(d);
} // ScanI2CBus

void ReceiverTest(void)
{
	int8 s;
	int16 *p;
	uint16 v;

	if( NegativePPM )
		TxString("\r\nNeg. Rx PPM\r\n");
	else
		TxString("\r\nPos. Rx PPM\r\n");
	
	TxString("Rx vals:\r\n");
	
	// Be wary as new RC frames are being received as this
	// is being displayed so data may be from overlapping frames

	if( _NewValues )
	{
		_NewValues = false;

		p = &NewK1;
		for( s=1; s <= 7; s++ )
		{
			TxChar(s+'0');
			TxString(":\t 0x");
			v = *p++;
			TxValH16(v);
			TxChar(HT);	
			TxVal32(((int32)(v & 0x00ff)*100)/_Maximum, 0, '%');
			if( ( v & 0xff00) != (uint16)0x0100 ) 
				TxString(" FAIL");
			TxNextLine();
		}

		// show pause time
		TxString("Gap:\t");
		v = 2*PauseTime;
	 	v += (uint16)TMR2_5MS * 64;	// 78 * 16*16/4 us
		TxVal32( v, 3, 0);		
		TxString("mS\r\nGlitches:\t");
		TxVal32(RCGlitchCount,0,0);
		TxNextLine();
	}
	else
		TxString("(no new vals)\r\n");
} // ReceiverTest

void DoCompassTest()
{
	uint16 v;

	TxString("\r\nCompass test\r\n");

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) goto CTerror;
	v = ((uint16)RecvI2CByte(I2C_ACK)*256) | RecvI2CByte(I2C_NACK);
	I2CStop();

	TxVal32((int32)v, 1, 0);
	TxString(" deg (not corrected for orientation on airframe)\r\n");		
	return;
CTerror:
	I2CStop();
	TxString("FAIL\r\n");
} // DoCompassTest

void CalibrateCompass(void)
{	// calibrate the compass by rotating the ufo through 720 deg smoothly
	TxString("\r\nCalib. compass \r\nRotate horizontally 720 deg in ~30 sec.! \r\nPress any key to START.\r\n");
	while( PollRxChar() == NUL );

	// set Compass device to Calibration mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('C')  != I2C_ACK ) goto CCerror;
	I2CStop();

	TxString("\r\nPress any key to FINISH\r\n");
	while( PollRxChar() == NUL );

	// set Compass device to End-Calibration mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('E')  != I2C_ACK ) goto CCerror;
	I2CStop();

	Delay1mS(50);

	TxString("OK\r\n");
	return;
CCerror:
	I2CStop();
	TxString("FAIL\r\n");
} // CalibrateCompass

void BaroTest(void)
{
	uint8 r;
	uint16 P, T, C;

	TxString("\r\nBarometer test\r\n");
	if ( !_UseBaro ) goto BAerror;

	if ( BaroType == BARO_ID_BMP085 )
		TxString("Type:\tBMP085\r\n");
	else
		TxString("Type:\tSMD500\r\n");
	
	if( !StartBaroADC(BARO_PRESS) ) goto BAerror;
	Delay1mS(BARO_PRESS_TIME);
	r = ReadValueFromBaro();
	P = BaroVal;
	TxString("Press: \t");	
	TxVal32((int32)P, 0, 0);
		
	if( !StartBaroADC(BaroTemp) ) goto BAerror;
	Delay1mS(BARO_TEMP_TIME);
	r = ReadValueFromBaro();
	T = BaroVal;
	TxString("\tTemp: ");
	TxVal32((int32)T, 0, 0);	

	TxString("\tComp: ");
	C = P + SRS16((int16)T * (int16)BaroTempCoeff + 16, 5);
	TxVal32((int32)C, 0, 0);
	TxNextLine();

	TxNextLine();

	return;
BAerror:
	I2CStop();
	TxString("FAIL\r\n");
} // BaroTest

// flash output for a second, then return to its previous state
void PowerOutput(int8 d)
{
	int8 s;
	uint8 m;

	m = 1 << d;
	for( s=0; s < 10; s++ )	// 10 flashes (count MUST be even!)
	{
		LedShadow ^= m;
		SendLeds();
		Delay1mS(200);
	}		
} // PowerOutput

void GPSTest(void)
{
	uint8 ch; 

	TxString("\r\nGPS test\r\n");
	TxString("monitors GPS input until full power reconnect\r\n");
	TxString("units Metres and Degrees\r\n");

	DoCompassTest();
//CompassHeading = 0; // zzz
	TxString("CONNECT GPS\r\n");
	TxString("\r\nPress any key to cont.\r\n");

	while( !PollRxChar() );

	ReceivingGPSOnly(true);

	while ( true )
	{
		UpdateGPS();	
		if ( _GPSValid )
		{
			if ( Armed && _Signal )
			{
				DesiredRoll = IRoll;
				DesiredPitch = IPitch;
			}
			else
				DesiredRoll = DesiredPitch = 0;

			GetDirection();
//CompassHeading +=MILLIPI/32;
			Navigate(0, 0);

			TxVal32((int32)((int32)CompassHeading*180L)/(int32)MILLIPI, 0, ' ');
			if ( _CompassMisRead )
				TxChar('?');

			TxString("\t Fix=");
			TxVal32(GPSFix, 0, 0);

			TxString("\t Sats=");
			TxVal32(GPSNoOfSats, 0, 0);

			TxString("\t HDilute=");
			TxVal32(GPSHDilute, 1, 0);

			TxString("\t RelAlt=");
			TxVal32(GPSAltitude, 1, 0);

			TxString("\t ");
			TxVal32(Abs(ConvertGPSToM(GPSNorth)), 0, 0);
			if ( GPSNorth >=0 )
				TxChar('N');
			else
				TxChar('S');
			TxString("\t ");
			TxVal32(Abs(ConvertGPSToM(GPSEast)), 0, 0);
			if ( GPSEast >=0 )
				TxChar('E');
			else
				TxChar('W');

			TxString("\t -> R=");
			TxVal32(DesiredRoll, 0, ' ');
			TxString("\t P=");		
			TxVal32(DesiredPitch, 0, ' ');
			TxNextLine();
		}	
		_GPSValid = false;	
	}
} // GPSTest

void AnalogTest(void)
{
	int32 v;

	TxString("\r\nAnalog ch. test\r\n");

	// Roll
	v = ((int24)ADC(ADCRollChan, ADCVREF5V) * 50 + 5)/10; // resolution is 0,001 Volt
	//TxVal32(ADCRollChan, 0, ' ');
	TxString("Roll: \t"); 
	TxVal32(v, 3, 'V');
	#ifdef OPT_IDG
	if ( ( v < 750 ) || ( v > 1750 ) )
	#else
	if ( ( v < 2000 ) || ( v > 3000 ) )
	#endif
		TxString(" gyro NC or fault?"); 
	TxNextLine();

	// Pitch
	v = ((int24)ADC(ADCPitchChan, ADCVREF5V) * 50 + 5)/10; // resolution is 0,001 Volt
	//TxVal32(ADCPitchChan, 0, ' ');
	TxString("Pitch:\t");		
	TxVal32(v, 3, 'V');
	#ifdef OPT_IDG
	if ( ( v < 750 ) || ( v > 1750 ) )
	#else
	if ( ( v < 2000 ) || ( v > 3000 ) )
	#endif
		TxString(" gyro NC or fault?");	
	TxNextLine();

	// Yaw
	v = ((int24)ADC(ADCYawChan, ADCVREF5V) * 50 + 5)/10; // resolution is 0,001 Volt
	//TxVal32(ADCYawChan, 0, ' ');
	TxString("Yaw:  \t");
	TxVal32(v, 3, 'V');
	if ( ( v < 2000 ) || ( v > 3000 ) )
		TxString(" gyro NC or fault?");	
	TxNextLine();
	TxNextLine();

	// Battery
	v = ((int24)ADC(ADCBattVoltsChan, ADCVREF5V) * 46 + 9)/17; // resolution is 0,01 Volt
	//TxVal32(ADCBattVoltsChan, 0, ' ');
	TxString("Batt:\t");
	TxVal32(v, 2, 'V');
	if ( v < 900 )
		TxString(" ** LIPO ALARM < 9V ** ");
	else	
	if ( v < 950 )
		TxString(" ** LOW < 9.5V ** ");
	TxNextLine();

	// VRef
	v = ((int24)ADC(ADCVRefChan, ADCVREF5V) * 50 + 5)/10; // resolution is 0,001 Volt
	//TxVal32(ADCVRefChan, 0, ' ');	
	TxString("Ref:  \t");	
	TxVal32(v, 3, 'V');
	if ( ( v < 3000 ) || ( v > 4000 ) )
		TxString(" fault?");	
	TxNextLine();

	
} // AnalogTest

#ifdef ESC_YGEI2C

void Program_SLA(uint8 niaddr)
{
	uint8 nii;

	for(nii = 0x10; nii<0xF0; nii+=2)
	{
		EscI2CStart();
		if( SendEscI2CByte(nii) == 0 )
		{
			if( nii == niaddr )
			{	// controller is already programmed OK
				EscI2CStop();
				TxString("controller at SLA 0x");
				TxValH(nii);
				TxString(" is already programmed OK\r\n");
				return;
			}
			else
			{
				if( SendEscI2CByte(0x87) == 0 ) // select register 0x07
				{
					if( SendEscI2CByte(niaddr) == 0 ) // new slave address
					{
						EscI2CStop();
						TxString("controller at SLA 0x");
						TxValH(nii);
						TxString(" reprogrammed to SLA 0x");
						TxValH(niaddr);
						TxNextLine();
						return;
					}
				}
			}
		}
		EscI2CStop();
	}
	TxString("no controller found or reprogram failed\r\n");
} // Program_SLA

void ConfigureESCs(void)
{
	uint8 nic;

	for( nic=0; nic<4; nic++)
	{
		TxString("\r\nConnect ONLY ");
		switch(nic)
		{
			case 0 : TxString("front"); break;
			case 1 : TxString("back");  break;
			case 2 : TxString("right"); break;
			case 3 : TxString("left");  break;
		}
		TxString(" controller, then press any key\r\n");
		while( PollRxChar() == '\0' );
		TxString("\r\nprogramming the controller...\r\n");

		Program_SLA(0x62+nic+nic);
	}
} // ConfigureESCs

#endif // ESC_YGEI2C

void OutSignals(void)
{

	// no motor output

} // OutSignals

void main(void)
{
	int8	i;

	DisableInterrupts;

	InitPorts();
	InitADC();
 
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
			USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, 104 );//_B9600);

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
	LedBlue_ON;
	AUX_LEDS_ON;

	INTCONbits.PEIE = true;		
	EnableInterrupts;

	InitGPS();

	Delay1mS(100);
	IsLISLactive();
	NeutralLR = 0;
	NeutralFB = 0;
	NeutralUD = 0;
	if ( _UseLISL )
	{
		LedYellow_ON;
		GetNeutralAccelerations();
		LedYellow_OFF;
	}

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
			LedRed_ON;
			LedGreen_OFF;
			if ( _UseLISL  )
				LedYellow_ON;
		}
		else
		{
			LedGreen_ON;
			LedRed_OFF;
			LedYellow_OFF;
		}

		ReadParametersEE();
		ProcessComCommand();

	}
} // main

