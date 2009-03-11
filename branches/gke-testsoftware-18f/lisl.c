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

// The LISL controller routines


#include "pu-test.h"
#include "bits.h"

// send an address byte (niaddr) to linear sensor
// read the answer and return it
uint8 ReadLISL(uint8 d)
{
	uint8 s;

	LISL_SDA = 1;	// very important!! really!! LIS3L likes it
	LISL_IO = 0;	// SDA is output
	LISL_SCL = 0;
	LISL_CS = 0;	// CS to 0
	for( s = 8; s ; s-- )
	{
		LISL_SCL = 0;
		if( d & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		d <<= 1;
		LISL_SCL = 1;
	}

	LISL_IO = 1;	// SDA is input
	d = 0;
	for( s = 8; s ; s-- )
	{
		LISL_SCL = 0;
		d <<= 1;
		if( LISL_SDA == 1 )
			d |= 1;	// set LSB
		LISL_SCL = 1;
	}
	LISL_CS = 1;
	return(d);
}

// only needed for CC5X version 3.3 and earlier
void WriteLISL(uint8, uint8);

// send an address byte (niaddr) to lienar sensor
// and write data byte (nidata)
void WriteLISL(uint8 addr, uint8 d)
{
	uint8 s;

	LISL_IO = 0;	// SDA is output
	LISL_SCL = 0;
	LISL_CS = 0;	// CS to 0
	for( s = 8; s ; s-- )
	{
		LISL_SCL = 0;
		if( addr & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		addr <<= 1;
		LISL_SCL = 1;
	}

	for( s = 8; s ; s-- )
	{
		LISL_SCL = 0;
		if( d & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		d <<= 1;
		LISL_SCL = 1;
	}
	LISL_CS = 1;
	LISL_IO = 1;	// IO is input (to allow RS232 reception)
}

// put the base setup to linear sensor
// enable all axes, setup resolution
// setup parachute options
void IsLISLactive(void)
{
	uint8 r;

	LISL_CS = 1;
	WriteLISL(LISL_CTRLREG_2, 0b01001010); // enable 3-wire, BDU=1, +/-2g

	r = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( r == 0x3A )	// a LIS03L sensor is there!
	{
		WriteLISL(LISL_CTRLREG_1, 0b11010111); // startup, enable all axis
		WriteLISL(LISL_CTRLREG_3, 0b00000000);
		WriteLISL(LISL_FF_CFG,    0b01001000); // Y-axis is height
		WriteLISL(LISL_FF_THS_L,  0b00000000);
		WriteLISL(LISL_FF_THS_H,  0b11111100); // -0,5g threshold
		WriteLISL(LISL_FF_DUR,    255);
		WriteLISL(LISL_DD_CFG,    0b00000000);
		_UseLISL = 1;
	}
}


void LinearTest(void)
{
	uint8 r;

	r = ReadLISL(LISL_STATUS + LISL_READ);
	TxChar('S');
	TxChar(':');
	TxChar('0');
	TxChar('x');
	TxValH(r);
	TxNextLine();

	val = (ReadLISL(LISL_OUTX_H + LISL_READ)*256)
			| (ReadLISL(LISL_OUTX_L + LISL_READ));
	TxChar('X');
	TxChar(':');
	TxValUL(NKS3+LEN5+VZ);
	TxText(SerLinG);

	val = (ReadLISL(LISL_OUTZ_H + LISL_READ)*256) 
			| ( ReadLISL(LISL_OUTZ_L + LISL_READ));
	TxChar('Z');
	TxChar(':');
	TxValUL(NKS3+LEN5+VZ);
	TxText(SerLinG);

	val = (ReadLISL(LISL_OUTY_H + LISL_READ)*256)
			| (ReadLISL(LISL_OUTY_L + LISL_READ));
	TxChar('Y');
	TxChar(':');
	TxValUL(NKS3+LEN5+VZ);
	TxText(SerLinG);
	
}


