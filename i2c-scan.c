// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =           Extensively modified 2008-9 by Prof. Greg Egan            =
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


// I2C Scanner

// compass sensor

#include "pu-test.h"
#include "bits.h"

void I2CDelay(void)
{
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
}

// put SCL to high-z and wait until line is really hi
// returns != 0 if ok
uns8 I2CWaitClkHi(void)
{
	uns8 nii=1;

	I2CDelay();
	I2C_CIO=1;	// set SCL to input, output a high
	while( I2C_SCL == 0 )	// wait for line to come hi
	{
		nii++;
		if( nii == 0 )
			break;
	}		
	I2CDelay();
	return(nii);
}

// send a start condition
void I2CStart(void)
{
	I2C_DIO=1;	// set SDA to input, output a high
	I2CWaitClkHi();
	I2C_SDA = 0;	// start condition
	I2C_DIO = 0;	// output a low
	I2CDelay();
	I2C_SCL = 0;
	I2C_CIO = 0;	// set SCL to output, output a low
}

// send a stop condition
void I2CStop(void)
{
	I2C_DIO=0;	// set SDA to output
	I2C_SDA = 0;	// output a low
	I2CWaitClkHi();

	I2C_DIO=1;	// set SDA to input, output a high, STOP condition
	I2CDelay();		// leave clock high
}

static uns8 nii;	// mus be bank0 or shrBank

// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
uns8 SendI2CByte(uns8 nidata)
{

	for(nii=0; nii<8; nii++)
	{
		if( IsSet(nidata,7) )
		{
			I2C_DIO = 1;	// switch to input, line is high
		}
		else
		{
			I2C_SDA = 0;			
			I2C_DIO = 0;	// switch to output, line is low
		}
	
		if(!I2CWaitClkHi()) 
			return(I2C_NACK);
		I2C_SCL = 0;
		I2C_CIO = 0;	// set SCL to output, output a low
		nidata <<= 1;
	}
	I2C_DIO = 1;	// set SDA to input
	if(!I2CWaitClkHi())
		return(I2C_NACK);
	nii = I2C_SDA;	

	I2C_SCL = 0;
	I2C_CIO = 0;	// set SCL to output, output a low
	I2CDelay();
//	I2C_IO = 0;		// set SDA to output
//	I2C_SDA = 0;	// leave output low
	return(nii);
}


// read a byte from I2C slave and set ACK status
// 0 = ACK
// 1 = NACK
// returns read byte
uns8 RecvI2CByte(uns8 niack)
{
	uns8 nidata=0;

	I2C_DIO=1;	// set SDA to input, output a high

	for(nii=0; nii<8; nii++)
	{
		if( !I2CWaitClkHi() ) 
			return(0);
		nidata <<= 1;
		if( I2C_SDA )
			nidata |= 1;
		I2C_SCL = 0;
		I2C_CIO = 0;	// set SCL to output, output a low
	}
	I2C_SDA = niack;
	I2C_DIO = 0;	// set SDA to output
	if( !I2CWaitClkHi() )
		return(0);

	I2C_SCL = 0;
	I2C_CIO = 0;	// set SCL to output, output a low
	I2CDelay();
//	I2C_IO = 0;	// set SDA to output
	return(nidata);
}

// scan all slave bus addresses (read mode)
// and list found devices
uns8 ScanI2CBus(void)
{
	uns8 nij;
	uns8 nic = 0;

	for(nij=0x10; nij<=0xF6; nij+=2)
	{	// use all uneven addresses for read mode
		I2CStart();
		if( SendI2CByte(nij) == I2C_ACK )
		{
			nic++;
			SendComChar('0');
			SendComChar('x');
			SendComValH(nij);
			SendComCRLF();
		}
		I2CStop();

		WriteTimer0(0);
		while ( INTCONbits.T0IF == 0 ) {}; // 1mS wait
		INTCONbits.T0IF = 0;
	}
	return(nic);
}

void CompassTest(void)
{
	uns8 i;

// 20Hz standby mode - random read?
#define COMP_OPMODE 0b01100000
#define COMP_MULT	16

	// set Compass device to Compass mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('G')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto CTerror;

	// select operation mode, standby mode
	if( SendI2CByte(COMP_OPMODE) != I2C_ACK ) goto CTerror;
	I2CStop();

	// set EEPROM shadow register
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('w')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x08) != I2C_ACK ) goto CTerror;

	// select operation mode, standby mode
	if( SendI2CByte(COMP_OPMODE) != I2C_ACK ) goto CTerror;
	I2CStop();

	WriteTimer0(0);
	while ( INTCONbits.T0IF == 0 ) {}; // 1mS wait
	INTCONbits.T0IF = 0;

	// set output mode, cannot be shadowd in EEPROM :-(
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('G')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x4e) != I2C_ACK ) goto CTerror;

	// select heading mode (1/10th degrees)
	if( SendI2CByte(0x00) != I2C_ACK ) goto CTerror;
	I2CStop();

	// set multiple read option, can only be written to EEPROM
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('w')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x06) != I2C_ACK ) goto CTerror;
	if( SendI2CByte(COMP_MULT)   != I2C_ACK ) goto CTerror;
	I2CStop();

	WriteTimer0(0);
	while ( INTCONbits.T0IF == 0 ) {}; // 1mS wait
	INTCONbits.T0IF = 0;

	// read a direction
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('A')  != I2C_ACK ) goto CTerror;
	I2CStop();

	// wait 25ms for command to complete
	for (i = 30; i; i--)
	{
		WriteTimer0(0);
		while ( INTCONbits.T0IF == 0 ) {}; // 1mS wait
		INTCONbits.T0IF = 0;
	}

	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) goto CTerror;
	nilgval = (RecvI2CByte(I2C_ACK)<<8) | RecvI2CByte(I2C_NACK);
	I2CStop();

	// niltemp has 1/10th degrees
	nilgval /= 10;

	SendComValUL(NKS0+LEN3);
	SendComText(SerGrad);
	return;
CTerror:
	SendComCRLF();
	I2CStop();
	SendComText(SerI2CFail);
}

// calibrate the compass by rotating the ufo once smoothly
void CalibrateCompass(void)
{
	while( !RecvComChar() );
	
	// set Compass device to Calibration mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('C')  != I2C_ACK ) goto CCerror;
	I2CStop();

	SendComText(SerCCalib2);

	while( !RecvComChar() );

	// set Compass device to End-Calibration mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CCerror;
	if( SendI2CByte('E')  != I2C_ACK ) goto CCerror;
	I2CStop();

	SendComText(SerCCalib3);
	return;
CCerror:
	I2CStop();
	SendComText(SerCCalibE);
}

void BaroTest(void)
{
	uns8	BaroType, BaroTemp;
	uns16 	BaroTemperature, BaroPressure;
	
	uns8 i;
	int16 Temp;

	// SMD500 9.5mS (T) 34mS (P)  
	// BMP085 4.5mS (T) 25.5mS (P) OSRS=3, 7.5mS OSRS=1
	// Baro is assumed offline unless it responds - no retries!

	// Determine baro type
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_TYPE) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroType = RecvI2CByte(I2C_NACK);
	I2CStop();

	if ( BaroType == BARO_ID_BMP085 )
	{
		SendComText(SerBaroBMP085);
		BaroTemp = BaroTemp_BMP085;

	}
	else
	{
		SendComText(SerBaroSMD500);
		BaroTemp = BaroTemp_SMD500;
	}

	// set Baro device to start conversion
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;

	// access control register, start measurement
	if( SendI2CByte(BARO_CTL) != I2C_ACK ) goto BAerror;

	// select 32kHz input, measure pressure
	if( SendI2CByte(BARO_PRESS) != I2C_ACK ) goto BAerror;
	I2CStop();

	DELAY_MS(40);

	// Possible I2C protocol error - split read of ADC
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_ADC_MSB) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroPressure = RecvI2CByte(I2C_NACK) << 8;
	I2CStop();
		
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_ADC_LSB) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroPressure |= RecvI2CByte(I2C_NACK);
	I2CStop();

	nilgval = BaroPressure;
	SendComText(SerBaroOK);
	SendComValUL(NKS0+LEN5);

	// read temp
	// set baro device to start conversion
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	// access control register, start measurement
	if( SendI2CByte(BARO_CTL) != I2C_ACK ) goto BAerror;
	// select 32kHz input, measure temperature
	if( SendI2CByte(BaroTemp) != I2C_ACK ) goto BAerror;
	I2CStop();

	DELAY_MS(10);

	// Possible I2C protocol error - split read of ADC
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_ADC_MSB) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroTemperature = RecvI2CByte(I2C_NACK)<<8;
	I2CStop();
		
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_ADC_LSB) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroTemperature |= RecvI2CByte(I2C_NACK);
	I2CStop();

	nilgval = BaroTemperature;
	SendComText(SerBaroT);
	SendComValUL(NKS0+LEN5);

	SendComCRLF();

	return;
BAerror:
	SendComCRLF();
	I2CStop();

	SendComText(SerI2CFail);
}

