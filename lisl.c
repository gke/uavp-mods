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

#include "c-ufo.h"
#include "bits.h"

static uns8	nii;
static uns8 niaddr;

#define SSP_CLK PORTBbits.RB4;
#define SSP_SDA PORTBbits.RB5;

#ifdef DEBUGSSP
// SSP output for datalogger
// define DEBUG must be set to use it!
void OutSSP(uns8 nidata)
{
	for( nii=0; nii<8; nii++ )
	{
		if( nidata.7 )
			SSP_SDA = 1;
		else
			SSP_SDA = 0;
		nop();
		SSP_CLK = 1;
		nidata <<= 1;
		SSP_CLK = 0;
	}
} // OutSSP
#endif

// send a command byte to linear sensor, address = niaddr
void SendCommand(void)
{
	uns8 s;

	LISL_IO = 0;	// SDA is output
	LISL_SCL = 0;
	LISL_CS = 0;	// CS to 0
	for( s = 8; s != 0; s-- )
	{
		LISL_SCL = 0;
		if( niaddr & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		niaddr <<= 1;
		LISL_SCL = 1;
	}
} // SendCommand

// send an address byte (niaddr) to linear sensor
// read the answer and return it
uns8 ReadLISL(uns8 v)
{
	niaddr = v;
	nii = v;
	LISL_SDA = 1;	// very important!! really!! LIS3L likes it
	SendCommand();
	LISL_IO = 1;	// SDA is input
	v=ReadLISLNext();
	
	if( (nii & LISL_INCR_ADDR) == 0 )
		LISL_CS = 1;	// end transmission
	return(v);
} // ReadLISL

// read a data byte from linear sensor and return it
uns8 ReadLISLNext(void)
{
	uns8 s;
//	niaddr = 0;		// not really necessary
	for( s = 8; s != 0; s-- )
	{
		LISL_SCL = 0;
		niaddr <<= 1;
		if( LISL_SDA == 1 )
			niaddr |= 1;	// set LSB
		LISL_SCL = 1;
	}
	return(niaddr);
} // ReadLISLNext

// send an address byte (niaddr) to linear sensor
// and write data byte (nidata)
void WriteLISL(uns8 nidata, uns8 a)
{
	niaddr = a;
	SendCommand();

	for( nii = 0; nii < 8; nii++ )
	{
		LISL_SCL = 0;
		if( nidata & 0x80 )
			LISL_SDA = 1;
		else
			LISL_SDA = 0;
		nidata <<= 1;
		LISL_SCL = 1;
	}
	LISL_CS = 1;
	LISL_IO = 1;	// IO is input (to allow RS232 reception)
} // WriteLISL

// put the base setup to linear sensor
// enable all axes, setup resolution
// setup parachute options
void IsLISLactive(void)
{
	uns8 r;

	LISL_CS = 1;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	r = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( r == 0x3A )	// a LIS03L sensor is there!
	{
//		WriteLISL(0b.11010111, LISL_CTRLREG_1); // startup, enable all axis
// use 40Hz data rate, thanks to Tom Poub!
		WriteLISL(0b11000111, LISL_CTRLREG_1); // startup, enable all axis
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		WriteLISL(0b01001000, LISL_FF_CFG); // Y-axis is height
		WriteLISL(0b00000000, LISL_FF_THS_L);
		WriteLISL(0b11111100, LISL_FF_THS_H); // -0,5g threshold
		WriteLISL(255, LISL_FF_DUR);
		WriteLISL(0b00000000, LISL_DD_CFG);
		_UseLISL = 1;
	}
} // IsLISLactive

