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

// i2c subroutines

#include "uavx.h"

void I2CStart(void);
void I2CStop(void);
uint8 I2CWaitClkHi(void);
void I2CDelay(void);
uint8 SendI2CByte(uint8);
uint8 RecvI2CByte(uint8);

// Wolfgang's I2C Routines - modified

// Prototypes

void I2CDelay(void);
void I2CStart(void);
void I2CStop(void);
uint8 SendI2CByte(uint8);
uint8 RecvI2CByte(uint8);

// Constants

#define I2C_SDA			PORTBbits.RB6
#define I2C_DIO			TRISBbits.TRISB6
#define I2C_SCL			PORTBbits.RB7
#define I2C_CIO			TRISBbits.TRISB7

void I2CDelay(void) 
{
	Delay10TCY();
} // I2CDelay

uint8 I2CWaitClkHi(void)
{
	uint8 s;

	I2CDelay();
	I2C_CIO=1;								// set SCL to input, output a high

	s = 1;
	while( !I2C_SCL )						// timeout wraparound through 255 1.25mS							
		if( ++s == 0 )			 
			break;	
	I2CDelay();
	return(s);
} // I2CWaitClkHi

void I2CStart(void)
{
	uint8 r;

	I2C_DIO=1;								// set SDA to input, output a high
	r = I2CWaitClkHi();
	I2C_SDA = 0;							// start condition
	I2C_DIO = 0;							// output a low
	I2CDelay();
	I2C_SCL = 0;
	I2C_CIO = 0;							// set SCL to output, output a low
} // I2CStart

void I2CStop(void)
{
	uint8 r;

	I2C_DIO=0;								// set SDA to output
	I2C_SDA = 0;							// output a low
	r = I2CWaitClkHi();

	I2C_DIO=1;								// set SDA to input, output a high, STOP condition
	I2CDelay();								// leave clock high
} // I2CStop 

uint8 SendI2CByte(uint8 d)
{
	uint8 s;

	for ( s = 8; s ; s-- )
	{
		if( d & 0x80 )
			I2C_DIO = 1;					// switch to input, line is high
		else
		{
			I2C_SDA = 0;			
			I2C_DIO = 0;					// switch to output, line is low
		}
	
		if( I2CWaitClkHi() )
		{ 	
			I2C_SCL = 0;
			I2C_CIO = 0;					// set SCL to output, output a low
			d <<= 1;
		}
		else
			return(I2C_NACK);	
	}

	I2C_DIO = 1;							// set SDA to input
	if( I2CWaitClkHi() )
		s = I2C_SDA;
	else
		return(I2C_NACK);	

	I2C_SCL = 0;
	I2C_CIO = 0;							// set SCL to output, output a low

	return(s);
} // SendI2CByte

uint8 RecvI2CByte(uint8 r)
{
	uint8 s, d;

	d = 0;
	I2C_DIO = 1;							// set SDA to input, output a high

	for ( s = 8; s ; s-- )
		if( I2CWaitClkHi() )
		{ 
			d <<= 1;
			if( I2C_SDA )
				d |= 1;
			I2C_SCL = 0;
			I2C_CIO = 0;
 		}
		else
			return(0);

	I2C_SDA = r;
	I2C_DIO = 0;
											// set SDA to output
	if( I2CWaitClkHi() )
	{
		I2C_SCL = 0;
		I2C_CIO = 0;						// set SCL to output, output a low
	//	I2CDelay();
	//	I2C_IO = 0;	// set SDA to output
		return(d);
	}
	else
		return(0);	
} // RecvI2CByte

// -----------------------------------------------------------
// Wolfgang's SW I2C Routines for ESCs - modified

// Prototypes

void ESCWaitClkHi(void);
void ESCI2CStart(void);
void ESCI2CStop(void);
uint8 SendESCI2CByte(uint8);

// Constants

#define	 ESC_SDA		PORTBbits.RB1
#define	 ESC_SCL		PORTBbits.RB2
#define	 ESC_DIO		TRISBbits.TRISB1
#define	 ESC_CIO		TRISBbits.TRISB2

void ESCWaitClkHi(void)
{
	static uint8 s;

	s = 1;

	I2CDelay();
	ESC_CIO=1;								// set SCL to input, output a high
	while( !ESC_SCL ) 
		if( ++s == 0 )			 
			break;;							// wait for line to come hi with timeout
	I2CDelay();
} // ESCWaitClkHi

void ESCI2CStart(void)
{
	ESC_DIO=1;								// set SDA to input, output a high
	ESCWaitClkHi();
	ESC_SDA = 0;							// start condition
	ESC_DIO = 0;							// output a low
	I2CDelay();
	ESC_SCL = 0;
	ESC_CIO = 0;							// set SCL to output, output a low
} // ESCI2CStart

void ESCI2CStop(void)
{
	ESC_DIO=0;								// set SDA to output
	ESC_SDA = 0;							// output a low
	ESCWaitClkHi();

	ESC_DIO=1;								// set SDA to input, output a high, STOP condition
	I2CDelay();								// leave clock high
} // ESCI2CStop

uint8 SendESCI2CByte(uint8 d)
{
	static uint8 s, t;

	for ( s = 8 ; s ; s-- )
	{
		if( d & 0x80 )
			ESC_DIO = 1;	
		else
		{
			ESC_SDA = 0;
			ESC_DIO = 0;		
		}
	
		I2CDelay();
		ESC_CIO=1;						

		t = 1;
		while( !ESC_SCL )					// timeout wraparound through 255 1.25mS							
			if ( ++t == 0 ) 
				return ( I2C_NACK );

		I2CDelay();
		ESC_SCL = 0;
		ESC_CIO = 0;		
		d <<= 1;
	}
	ESC_DIO = 1;					
	I2CDelay();
	ESC_CIO=1;			

	t = 1;
	while( !ESC_SCL )									
		if ( ++t == 0 ) 
			return ( I2C_NACK );

	I2CDelay();	
	ESC_SCL = 0;
	ESC_CIO = 0;						
	I2CDelay();

	return( I2C_ACK );

} // SendESCI2CByte
