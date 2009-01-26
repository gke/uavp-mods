// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =              Ported to 18F2520 2008 by Prof. Greg Egan              =
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

// i2c subroutines

#include "c-ufo.h"
#include "bits.h"

void I2CStart(void);
void I2CStop(void);
uint8 I2CWaitClkHi(void);
void I2CDelay(void);
uint8 SendI2CByte(uint8);
uint8 RecvI2CByte(uint8);

// -----------------------------------------------------------

// Wolfgang's I2C Routines

void I2CDelay(void) 
{
	Delay10TCY();
} // I2CDelay

// put SCL to high-z and wait until line is really hi
// returns != 0 if ok
uint8 I2CWaitClkHi(void)
{
	uint8 s;

	s = 1;

	I2CDelay();
	I2C_CIO=1;	// set SCL to input, output a high
	while( !I2C_SCL )						// timeout wraparound through 255 1.25mS							
		if( ++s == 0 )			 
			break;	
	I2CDelay();
	return(s);
} // I2CWaitClkHi

// send a start condition
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

// send a stop condition
void I2CStop(void)
{
	uint8 r;

	I2C_DIO=0;								// set SDA to output
	I2C_SDA = 0;							// output a low
	r = I2CWaitClkHi();

	I2C_DIO=1;								// set SDA to input, output a high, STOP condition
	I2CDelay();								// leave clock high
} // I2CStop 

// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
uint8 SendI2CByte(uint8 d)
{
	uint8 s;

	for(s=8; s ; s--)
	{
		if( d & 0x80 )
			I2C_DIO = 1;					// switch to input, line is high
		else
		{
			I2C_SDA = 0;			
			I2C_DIO = 0;					// switch to output, line is low
		}
	
		if(I2CWaitClkHi())
		{ 	
			I2C_SCL = 0;
			I2C_CIO = 0;					// set SCL to output, output a low
			d <<= 1;
		}
		else
			return(I2C_NACK);	
	}

	I2C_DIO = 1;							// set SDA to input
	if(I2CWaitClkHi())
		s = I2C_SDA;
	else
		return(I2C_NACK);	

	I2C_SCL = 0;
	I2C_CIO = 0;							// set SCL to output, output a low

	return(s);
} // SendI2CByte


// read a byte from I2C slave and set ACK status
// 0 = ACK
// 1 = NACK
// returns read byte
uint8 RecvI2CByte(uint8 r)
{
	uint8 s, d;

	d = 0;
	I2C_DIO = 1;							// set SDA to input, output a high

	for(s=8; s ; s--)
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
// Wolfgang's SW I2C Routines for ESCs

#if (defined ESC_X3D || defined ESC_HOLGER || defined ESC_YGEI2C) && !defined DEBUG_SENSORS

void EscI2CDelay(void)
{
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
} // EscI2CDelay

void EscWaitClkHi(void)
{
	uint8 s;

	s = 1;

	EscI2CDelay();
	ESC_CIO=1;								// set SCL to input, output a high
	while( !ESC_SCL ) 
		if( ++s == 0 )			 
			break;;							// wait for line to come hi with timeout
	EscI2CDelay();
} // EscWaitClkHi

// send a start condition
void EscI2CStart(void)
{
	ESC_DIO=1;								// set SDA to input, output a high
	EscWaitClkHi();
	ESC_SDA = 0;							// start condition
	ESC_DIO = 0;							// output a low
	EscI2CDelay();
	ESC_SCL = 0;
	ESC_CIO = 0;							// set SCL to output, output a low
} // EscI2CStart

// send a stop condition
void EscI2CStop(void)
{
	ESC_DIO=0;								// set SDA to output
	ESC_SDA = 0;							// output a low
	EscWaitClkHi();

	ESC_DIO=1;								// set SDA to input, output a high, STOP condition
	EscI2CDelay();							// leave clock high
} // EscI2CStop


// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
void SendEscI2CByte(uint8 d)
{
	uint8 s;

	for(s=8; s; s--)
	{
		if( d & 0x80 )
			ESC_DIO = 1;					// switch to input, line is high
		else
		{
			ESC_SDA = 0;			
			ESC_DIO = 0;					// switch to output, line is low
		}
	
		ESC_CIO=1;							// set SCL to input, output a high
		while( !ESC_SCL ) ;					// wait for line to come hi ??? hang
		EscI2CDelay();
		ESC_SCL = 0;
		ESC_CIO = 0;						// set SCL to output, output a low
		d <<= 1;
	}
	ESC_DIO = 1;							// set SDA to input
	EscI2CDelay();
	ESC_CIO=1;								// set SCL to input, output a high
	while( !ESC_SCL ) ;						// wait for line to come hi ??? hang

	EscI2CDelay();							// here is the ACK	

	ESC_SCL = 0;
	ESC_CIO = 0;							// set SCL to output, output a low
	EscI2CDelay();

} // SendEscI2CByte

#endif	// ESC_X3D || ESC_HOLGER || ESC_YGEI2C


