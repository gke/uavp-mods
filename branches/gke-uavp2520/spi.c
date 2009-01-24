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

// SPI routines for LEDs and LISL

#include "c-ufo.h"
#include "bits.h"

// LIS3LV02DQ Inertial Sensor (Accelerometer)

void WriteLISLByte(uint8 d)
{
	uint8	s;

	LISL_SCL = 0;						// to give a little more low time
	for( s = 8; s ; s-- )
	{
		LISL_SCL = 0;
		if (d & 0x80) 
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		d <<= 1;
		LISL_SCL = 1;
		//Delay10TCY();
	}
} // WriteLISLByte

uint8 ReadLISLNext(void)
{
	uint8	s, d;
	
	LISL_SCL = 0;					
	d = 0;
	for( s = 8; s ; s-- )
	{
		LISL_SCL = 0;
		d = (d << 1) & 0xfe;
		if ( LISL_SDA )
			d |= 1;	
		LISL_SCL = 1;
		//Delay10TCY();
	}	

	return(d);
} // ReadLISLNext

uint8 ReadLISL(uint8 addr)
{
	uint8	d;

	LISL_SDA = 1;						// very important!! really!! LIS3L likes it

	LISL_IO = 0;						// write
	LISL_CS = 0;
	WriteLISLByte(addr);

	LISL_IO = 1;						// read	
	d=ReadLISLNext();
	
	if( (addr & LISL_INCR_ADDR) == 0 )	// is this a single byte Read?
		LISL_CS = 1;
				
	return(d);
} // ReadLISL

void WriteLISL(uint8 d, uint8 addr)
{
	LISL_IO = 0;						// write
	LISL_CS = 0;

	WriteLISLByte(addr);
	WriteLISLByte(d);

	LISL_CS = 1;
	LISL_IO = 1;						// read
} // WriteLISL


void IsLISLactive(void)
{
	WriteLISL(0b01001010, LISL_CTRLREG_2); 			// enable 3-wire, BDU=1, +/-2g

	if( ReadLISL(LISL_WHOAMI + LISL_READ) == 0x3A )	// LIS03L sensor ident
	{
		WriteLISL(0b11000111, LISL_CTRLREG_1); 		// startup, enable all axis
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		WriteLISL(0b01001000, LISL_FF_CFG); 		// Y-axis is height
		WriteLISL(0b00000000, LISL_FF_THS_L);
		WriteLISL(0b11111100, LISL_FF_THS_H); 		// -0,5g threshold
		WriteLISL(255, LISL_FF_DUR);
		WriteLISL(0b00000000, LISL_DD_CFG);
		_UseLISL = true;
	}
	else
		_UseLISL = false;
} // IsLISLactive


void ReadLISLXYZ()
{ 
	// accelerations returned in Ax, Ay and Az
	if( _UseLISL )
	{
		while( (ReadLISL(LISL_STATUS+LISL_READ) & 0x08) == 0 ); //wait until ready
 		// 0.450mS
//		Ax = ReadLISL(LISL_OUTX_L+LISL_INCR_ADDR+LISL_READ)
//				|(ReadLISLNext()<<8);
//		Ay = ReadLISLNext()|(ReadLISLNext()<<8);
//		Az = ReadLISLNext()|(ReadLISLNext()<<8);
//		LISL_CS = 1;						// end of multiple read

		// 0.903mS
		Ax = (ReadLISL(LISL_OUTX_H|LISL_READ)*256)|ReadLISL(LISL_OUTX_L|LISL_READ);
		Ay = (ReadLISL(LISL_OUTY_H|LISL_READ)*256)|ReadLISL(LISL_OUTY_L|LISL_READ);
		Az = (ReadLISL(LISL_OUTZ_H|LISL_READ)*256)|ReadLISL(LISL_OUTZ_L|LISL_READ);
		LISL_IO = 1;						// read
	}
	else
		Ax = Ay = Az = 0;
} // ReadLISLXYZ

//-----------------------------------------------------------------------------

//send LedShadow byte to TPIC6B595N
void SendLeds(void)
{
	uint8	LEDs;
	uint8	s;

	LEDs = LedShadow;

    LISL_IO = 0;							// write
	LISL_CS = 1;							// select TPIC	
	for(s=8; s ; s--)
	{
		LISL_SCL = 0;
		if ( LEDs & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		LEDs <<= 1;
		LISL_SCL = 1;
	}

	PORTCbits.RC1=1;
	PORTCbits.RC1=0;						// latch into drivers
} // SendLeds
