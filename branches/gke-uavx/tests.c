// =======================================================================
// =                                 UAVX                                =
// =                         Quadrocopter Control                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
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

#include "bits.h"
#include "uavx.h"

// Prototypes

#ifdef TEST_SOFTWARE

extern uint8 BaroTemp;

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
	uint16 v, prev;
	uint8 r;

	TxString("\r\nCompass test\r\n");

	#define COMP_OPMODE 0b01110000	// standby mode to reliably read EEPROM

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('G')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto CTerror;
	if( SendI2CByte(COMP_OPMODE) != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(COMPASS_TIME);

	TxString("Registers\r\n");
	for (r = 0; r <= 8; r++)
	{
		I2CStart();
		if ( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
		if ( SendI2CByte('r')  != I2C_ACK ) goto CTerror;
		if ( SendI2CByte(r)  != I2C_ACK ) goto CTerror;

		Delay1mS(1);

		I2CStart();
		if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) goto CTerror;
		v = RecvI2CByte(I2C_NACK);
		I2CStop();

		TxVal32((int32)r,0,':');
		switch (r)
		{
		case 0: 
			TxString("\tI2C\t"); TxString(" 0x"); TxValH(v); 
			if ( v != 0x42 ) TxString("\t Error expected 0x42 for HMC6352");
			TxNextLine();
			break;
		case 1: break;
		case 2: TxString("\tXOffset\t"); TxVal32((int32)prev*256|(v & 0xff), 0, 0); TxNextLine(); break;
		case 3: break;
		case 4: TxString("\tYOffset\t"); TxVal32((int32)prev*256 |(v & 0xff), 0, 0); TxNextLine(); break;
		case 5: TxString("\tDelay\t"); TxVal32((int32)v, 0, 0); TxNextLine(); break;
		case 6: 
			TxString("\tNSum\t"); TxVal32((int32)v, 0, 0);
			if ( v !=16 ) TxString("\t Error expected 16");
			TxNextLine(); 
			break;
		case 7: TxString("\tSW Ver\t"); TxString(" 0x"); TxValH(v); TxNextLine(); break;
		case 8: 
			TxString("\tOpMode:");
			switch ( ( v >> 5 ) & 0x03 ) {
			case 0: TxString("  1Hz"); break;
			case 1: TxString("  5Hz"); break;
			case 2: TxString("  10Hz"); break;
			case 3: TxString("  20Hz"); break;
			}
 
			if ( v & 0x10 ) TxString(" S/R"); 

			switch ( v & 0x03 ) {
			case 0: TxString(" Standby"); break;
			case 1: TxString(" Query"); break;
			case 2: TxString(" Continuous"); break;
			case 3: TxString(" Not-allowed"); break;
			}
			TxNextLine(); 
			break;
		default: break;
		}
		prev = v;
		Delay1mS(COMPASS_TIME);
	}

	InitDirection(); 

	Delay1mS(COMPASS_TIME);

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) goto CTerror;
	v = ((uint16)RecvI2CByte(I2C_ACK)*256) | RecvI2CByte(I2C_NACK);
	I2CStop();

	TxString("Not corrected for orientation on airframe\r\n");
	TxVal32((int32)v, 1, 0);
	TxString(" deg\r\n");

		
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

	Delay1mS(COMPASS_TIME);

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
	TxString("monitors GPS input power is dosconnected and reconnected\r\n");
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
//Heading +=MILLIPI/32;
			Navigate(0, 0);

			TxVal32((int32)((int32)CompassHeading*180L)/(int32)MILLIPI, 0, 0);
			if ( _CompassMisRead )
				TxChar('?');
			else
				TxChar(' ');
			
			#ifdef GPS_USE_RMC
	
			TxString(" h=");
			TxVal32((int32)((int32)GPSHeading*180L)/(int32)MILLIPI, 0, 0);
			TxString(" mv=");
			TxVal32((int32)((int32)GPSMagVariation*180L)/(int32)MILLIPI, 0, 0);	

			TxString(" mode=");
			TxChar(GPSMode);
					
			#else
			TxString(" fx=");
			TxVal32(GPSFix, 0, 0);

			TxString(" s=");
			TxVal32(GPSNoOfSats, 0, ' ');

			TxString("hd=");
			TxVal32(GPSHDilute, 2, ' ');

			TxString("ra=");
			TxVal32(GPSAltitude, 1, 0);
			#endif // GPS_USE_RMC

			TxString("\t ");
			TxVal32(Abs(ConvertGPSToM(GPSNorth)), 0, 0);
			if ( GPSNorth >=0 )
				TxChar('n');
			else
				TxChar('s');
			TxString("\t ");
			TxVal32(Abs(ConvertGPSToM(GPSEast)), 0, 0);
			if ( GPSEast >=0 )
				TxChar('e');
			else
				TxChar('w');

			TxString("\t -> r=");
			TxVal32(DesiredRoll, 0, ' ');
			TxString("\t p=");		
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


#endif // TEST_SOFTWARE
