// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =           http://code.google.com/p/uavp-mods/ http://uavp.ch        =
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

#include "uavx.h"

// Prototypes

void DoLEDs(void);
void LinearTest(void);
uint8 ScanI2CBus(void);
void ReceiverTest(void);
void GetCompassParameters(void);
void DoCompassTest(void);
void CompassRun(void);
void CalibrateCompass(void);
void BaroTest(void);
void PowerOutput(int8);
void GPSTest(void);
void AnalogTest(void);
void Program_SLA(uint8);
void ConfigureESCs(void);

void DoLEDs(void)
{
	if( !( _Signal && Armed ) )
	{
		LEDRed_ON;
		LEDGreen_OFF;
		if ( _AccelerationsValid  )
			LEDYellow_ON;
	}
	else
	{
		LEDGreen_ON;
		LEDRed_OFF;
		LEDYellow_OFF;
	}
} // DoLEDs

void LinearTest(void)
{
	TxString("\r\nAccelerometer test:\r\n");

	if( _AccelerationsValid )
	{
		ReadAccelerations();
	
		TxString("Left->Right: \t");
		TxVal32(((int32)Ax.i16*1000+512)/1024, 3, 'G');	
		if ( Abs((Ax.i16)) > 128 )
			TxString(" fault?");
		TxNextLine();

		TxString("Front->Back: \t");	
		TxVal32(((int32)Az.i16*1000+512)/1024, 3, 'G');
		if ( Abs((Az.i16)) > 128 )
			TxString(" fault?");	
		TxNextLine();

		TxString("Down->Up:    \t");
	
		TxVal32(((int32)Ay.i16*1000+512)/1024, 3, 'G');
		if ( ( Ay.i16 < 896 ) || ( Ay.i16 > 1152 ) )
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
	uint16 v;

	TxString("\r\nRx: ");
	ShowRxSetup();
	TxString("\r\n");
	
	TxString("Most recent Values:\r\n");
	
	// Be wary as new RC frames are being received as this
	// is being displayed so data may be from overlapping frames

	// don't check for NewValues as this is constantly toggling
	for( s=0; s < CONTROLS; s++ )
	{
		TxChar(s+'1');
		TxString(":\t 0x");
		v = PPM[s].i16;
		TxValH16(v);
		TxChar(HT);
		if ( ( s > 0 ) && ( s < 4 ))	
			TxVal32(((int32)(v & 0x00ff)*200)/RC_MAXIMUM - 100, 0, '%');
		else
			TxVal32(((int32)(v & 0x00ff)*100)/RC_MAXIMUM, 0, '%');

		if( ( v & 0xff00) != (uint16)0x0100 ) 
			TxString(" FAIL");
		TxNextLine();
	}

	// show pause time
	TxString("Gap:\t");
	v = TMR2_TICK*PauseTime;
	TxVal32( v, 3, 0);		
	TxString("mS\r\n");
	TxString("Glitches:\t");
	TxVal32(RCGlitches,0,0);
	TxNextLine();

} // ReceiverTest

static uint8 CP[9];

void GetCompassParameters(void)
{
	uint8 r;

	#define COMP_OPMODE 0b01110000	// standby mode to reliably read EEPROM

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('G')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto CTerror;
	if( SendI2CByte(COMP_OPMODE) != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(COMPASS_TIME_MS);

	for (r = 0; r <= 8; r++)
	{
		CP[r] = 0xff;

		Delay1mS(10);

		I2CStart();
		if ( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
		if ( SendI2CByte('r')  != I2C_ACK ) goto CTerror;
		if ( SendI2CByte(r)  != I2C_ACK ) goto CTerror;
		I2CStop();

		Delay1mS(10);

		I2CStart();
		if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) goto CTerror;
		CP[r] = RecvI2CByte(I2C_NACK);
		I2CStop();
	}

	Delay1mS(7);

	return;

CTerror:
	I2CStop();
	TxString("FAIL\r\n");

} // GetCompassParamters

void DoCompassTest(void)
{
	uint16 v, prev;
	int16 Temp;
	i16u Compass;

	TxString("\r\nCompass test\r\n");

	#define COMP_OPMODE 0b01110000	// standby mode to reliably read EEPROM

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('G')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto CTerror;
	if( SendI2CByte(COMP_OPMODE) != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(1);

	I2CStart(); // Do Set/Reset now		
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('O')  != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(7);

	GetCompassParameters();

	InitCompass();

	TxString("Registers\r\n");
	TxString("0:\tI2C"); 
	TxString("\t 0x"); TxValH(CP[0]); 
	if ( CP[0] != 0x42 ) 
		TxString("\t Error expected 0x42 for HMC6352");
	TxNextLine();

	Temp = (CP[1]*256)|CP[2];
	TxString("1:2:\tXOffset\t"); 
	TxVal32((int32)Temp, 0, 0); 
	TxNextLine(); 

	Temp = (CP[3]*256)|CP[4];
	TxString("3:4:\tYOffset\t"); 
	TxVal32((int32)Temp, 0, 0); 
	TxNextLine(); 

	TxString("5:\tDelay\t"); 
	TxVal32((int32)CP[5], 0, 0); 
	TxNextLine(); 

	TxString("6:\tNSum\t"); TxVal32((int32)CP[6], 0, 0);
	if ( CP[6] !=16 ) 
		TxString("\t Error expected 16");
	TxNextLine(); 

	TxString("7:\tSW Ver\t"); 
	TxString(" 0x"); TxValH(CP[7]); 
	TxNextLine(); 

	TxString("8:\tOpMode:");
	switch ( ( CP[8] >> 5 ) & 0x03 ) {
		case 0: TxString("  1Hz"); break;
		case 1: TxString("  5Hz"); break;
		case 2: TxString("  10Hz"); break;
		case 3: TxString("  20Hz"); break;
		}
 
	if ( CP[8] & 0x10 ) TxString(" S/R"); 

	switch ( CP[8] & 0x03 ) {
		case 0: TxString(" Standby"); break;
		case 1: TxString(" Query"); break;
		case 2: TxString(" Continuous"); break;
		case 3: TxString(" Not-allowed"); break;
		}
	TxNextLine(); 

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) goto CTerror;
	Compass.high8 = RecvI2CByte(I2C_ACK);
	Compass.low8 = RecvI2CByte(I2C_NACK);
	I2CStop();

	TxVal32((int32)Compass.u16, 1, 0);
	TxString(" deg \tCorrected ");

	Temp = Compass.u16 - (COMPASS_OFFSET_DEG - P[NavMagVar])*10;
	while (Temp < 0) Temp +=3600;
	while (Temp >= 3600) Temp -=3600;
	TxVal32((int32)Temp, 1, 0);
	TxString(" deg\r\n");
		
	return;
CTerror:
	I2CStop();
	TxString("FAIL\r\n");

} // DoCompassTest

void CompassRun(void)
{
	int16 i, r, Temp;
	static i16u Compass;	

	for (i = 0; i < 32000; i++)
	{
		I2CStart();
		_CompassMissRead |= SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK; 
		Compass.high8 = RecvI2CByte(I2C_ACK);
		Compass.low8 = RecvI2CByte(I2C_NACK);
		I2CStop();
		
		TxVal32((int32) mS[Clock],3,' ');

		TxVal32((int32) Compass.i16,1,' ');
		TxNextLine();
	}
} // CompassRun

void CalibrateCompass(void)
{	// calibrate the compass by rotating the ufo through 720 deg smoothly
	TxString("\r\nCalib. compass - Press any key (x) to continue\r\n");	
	while( PollRxChar() != 'x' ); // UAVPSet uses 'x' for any key button

	I2CStart(); // Do Set/Reset now		
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('O')  != I2C_ACK ) goto CCerror;
	I2CStop();

	Delay1mS(7);

	// set Compass device to Calibration mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('C')  != I2C_ACK ) goto CCerror;
	I2CStop();

	TxString("\r\nRotate horizontally 720 deg in ~30 sec. - Press any key (x) to FINISH\r\n");
	while( PollRxChar() != 'x' );

	// set Compass device to End-Calibration mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('E')  != I2C_ACK ) goto CCerror;
	I2CStop();

	TxString("\r\nCalibration complete\r\n");

	Delay1mS(COMPASS_TIME_MS);

	InitCompass();

	return;
CCerror:
	I2CStop();
	TxString("Calibration FAILED\r\n");

} // CalibrateCompass

void BaroTest(void)
{
	TxString("\r\nBarometer test\r\n");
	if ( !_BaroAltitudeValid ) goto BAerror;

	if ( BaroType == BARO_ID_BMP085 )
		TxString("Type:\tBMP085\r\n");
	else
		TxString("Type:\tSMD500\r\n");
	
	while ( mS[Clock] <= mS[BaroUpdate] );
	ReadBaro();
	CurrentBaroPressure = (int24)BaroVal.u16 - OriginBaroPressure;		
	TxString("Origin: \t");
	TxVal32((int32)OriginBaroPressure,0,0);
	TxString("\t Rel.: \t");	
	TxVal32((int32)CurrentBaroPressure, 0, 0);
		
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
		LEDShadow ^= m;
		SendLEDs();
		Delay1mS(200);
	}		
} // PowerOutput

void GPSTest(void)
{
	uint8 ch; 

	TxString("\r\nGPS test\r\n");
	TxString("Monitors GPS input at 9.6Kb - units metres and degrees\r\n");
	TxString("DISARM the quadrocopter\r\n");
	TxString("Press any key to continue, set Baud Rate to 9.6Kb and ARM \r\n");

	while( !PollRxChar() );

	ReceivingGPSOnly(true);
	_GPSTestActive = true;

	while ( true )
	{
		DoLEDs();

		UpdateGPS();	
		if ( _GPSValid )
		{
			if ( Armed && _Signal )
			{
				DesiredRoll = RC[RollC];
				DesiredPitch = RC[PitchC];
			}
			else
				DesiredRoll = DesiredPitch = 0;

			GetHeading();

			Navigate(0, 0);

			if ( _CompassValid)
				TxVal32((int32)ConvertMPiToDDeg(Heading), 1, 0);
			else
				TxString("___");
			if ( _CompassMissRead )
				TxChar('?');
			else
				TxChar(' ');
			
			TxString(" fx=");
			TxVal32(GPSFix, 0, ' ');
	
			TxString("s=");
			TxVal32(GPSNoOfSats, 0, ' ');
	
			TxString("hd=");
			TxVal32(GPSHDilute, 2, ' ');
	
			TxString("ra=");
			TxVal32(GPSRelAltitude, 1, ' ');

			TxVal32(Abs(ConvertGPSToM(GPSNorth)), 0, 0);
			if ( GPSNorth >=0 )
				TxChar('n');
			else
				TxChar('s');
			TxChar(' ');
			TxVal32(Abs(ConvertGPSToM(GPSEast)), 0, 0);
			if ( GPSEast >=0 )
				TxChar('e');
			else
				TxChar('w');

			TxString(" -> r=");
			TxVal32(DesiredRoll, 0, ' ');
			TxString(" p=");		
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
	if ( P[GyroType] == IDG300 )
		v = ((int24)ADC(IDGADCRollChan, ADCVREF5V) * 50 + 5)/10; // resolution is 0,001 Volt
	else
		v = ((int24)ADC(NonIDGADCRollChan, ADCVREF5V) * 50 + 5)/10; 
	//TxVal32(ADCRollChan, 0, ' ');
	TxString("Roll: \t"); 
	TxVal32(v, 3, 'V');
	if ( P[GyroType] == IDG300 )
	{
		if ( ( v < 750 ) || ( v > 1750 ) ) 
			TxString(" gyro NC or fault?");
	}
	else
	{
		if ( ( v < 2000 ) || ( v > 3000 ) )
			TxString(" gyro NC or fault?");
	}
	TxNextLine();

	// Pitch
	if ( P[GyroType] == IDG300 )
		v = ((int24)ADC(IDGADCPitchChan, ADCVREF5V) * 50 + 5)/10; 
	else
		v = ((int24)ADC(NonIDGADCPitchChan, ADCVREF5V) * 50 + 5)/10; 
	//TxVal32(ADCPitchChan, 0, ' ');
	TxString("Pitch:\t");		
	TxVal32(v, 3, 'V');
	if ( P[GyroType] == IDG300 )
	{
		if ( ( v < 750 ) || ( v > 1750 ) ) 
			TxString(" gyro NC or fault?");
	}
	else
	{
		if ( ( v < 2000 ) || ( v > 3000 ) )
			TxString(" gyro NC or fault?");
	}	
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

void Program_SLA(uint8 niaddr)
{
	uint8 nii;

	if ( P[ESCType] == ESCYGEI2C )
	{
/* broken
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
*/
TxString("Not implemented yet\r\n");
	}
	TxString("no controller found or reprogram failed\r\n");

} // Program_SLA

void ConfigureESCs(void)
{
	uint8 nic;

	if ( P[ESCType] == ESCYGEI2C )
	{
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
	}
} // ConfigureESCs


