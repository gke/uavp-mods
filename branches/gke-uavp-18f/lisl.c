// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://www.uavp.org                        =
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

// The LISL controller routines

#include "c-ufo.h"
#include "bits.h"

#define SSP_CLK PORTBbits.RB4;
#define SSP_SDA PORTBbits.RB5;

#ifdef DEBUGSSP
// SSP output for datalogger
// define DEBUG must be set to use it!
void OutSSP(uint8 d)
{
	uins8 s;

	for( s = 8; s ; s-- )
	{
		if( (d & 0x10) != 0 )
			SSP_SDA = 1;
		else
			SSP_SDA = 0;
		nop2();
		SSP_CLK = 1;
		d <<= 1;
		SSP_CLK = 0;
	}
} // OutSSP
#endif

#define SPI_BACKTRACK
#ifdef SPI_BACKTRACK

void SendCommand(int8 c)
{
	int8 s;

	SPI_IO = WR_SPI;	
	SPI_SCL = 0;
	SPI_CS = SEL_LISL;	
	for( s = 0; s < 8; s++ )
	{
		SPI_SCL = 0;
		if( c & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		c <<= 1;
		SPI_SCL = 1;
	}
} // SendCommand

uint8 ReadLISL(uint8 c)
{
	uint8 d;

	SPI_SDA = 1;	// very important!! really!! LIS3L likes it
	SendCommand(c);
	SPI_IO = RD_SPI;	// SDA is input
	d=ReadLISLNext();
	
	if( (c & LISL_INCR_ADDR) == 0 )
		SPI_CS = DSEL_LISL;
	return(d);
} // ReadLISL

uint8 ReadLISLNext(void)
{
	int8 s;
	uint8 d;

	for( s = 0; s<8; s++ )
	{
		SPI_SCL = 0;
		d <<= 1;
		if( SPI_SDA == 1 )
			d |= 1;	
		SPI_SCL = 1;
	}
	return(d);
} // ReadLISLNext

void WriteLISL(uint8 d, uint8 c)
{
	int8 s;

	SendCommand(c);

	for( s = 0; s<8; s++ )
	{
		SPI_SCL = 0;
		if( d & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_SCL = 1;
	}
	SPI_CS = DSEL_LISL;
	SPI_IO = RD_SPI;	// IO is input (to allow RS232 reception)
} // WriteLISL

void IsLISLactive(void)
{

	SPI_CS = DSEL_LISL;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	W = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( W == 0x3A )	// a LIS03L sensor is there!
	{
		WriteLISL(0b11000111, LISL_CTRLREG_1); // startup, enable all axis
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		WriteLISL(0b01001000, LISL_FF_CFG); // Y-axis is height
		WriteLISL(0b00000000, LISL_FF_THS_L);
		WriteLISL(0b11111100, LISL_FF_THS_H); // -0,5g threshold
		WriteLISL(255, LISL_FF_DUR);
		WriteLISL(0b00000000, LISL_DD_CFG);
		_UseLISL = true;
	}
} // IsLISLactive

void ReadAccelerations()
{
	ReadLISL(LISL_STATUS + LISL_READ);
	Ax  = (int16)ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
	Ax |= (int16)ReadLISLNext()*256;
	Ay  = (int16)ReadLISLNext();
	Ay |= (int16)ReadLISLNext()*256;
	Az  = (int16)ReadLISLNext();
	Az |= (int16)ReadLISLNext()*256;
	SPI_CS = DSEL_LISL;	// end transmission

} // ReadAccelerations

#else

void WriteLISLByte(uint8 d)
{
	uint8	s;

	SPI_IO = WR_SPI;
	SPI_SCL = 0;
	SPI_CS = SEL_LISL;
	for( s = 8; s ; s-- )
	{
		Delay10TCY();
		SPI_SCL = 0;
		if (d & 0x80) 
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_SCL = 1;
	}
} // WriteLISLByte

uint8 ReadLISLNext(void)
{
	uint8	s, d;
	
	SPI_IO = RD_SPI;
	SPI_SCL = 0;
	SPI_CS = SEL_LISL;					
	d = 0;
	for( s = 8; s ; s-- )
	{
		Delay10TCY();
		SPI_SCL = 0;
		d = (d << 1) & 0xfe;
		if ( SPI_SDA )
			d |= 1;	
		SPI_SCL = 1;
	}	

	return(d);
} // ReadLISLNext

uint8 ReadLISL(uint8 addr)
{
	uint8	d;

	SPI_SDA = 1;						// very important!! really!! LIS3L likes it
	WriteLISLByte(addr);
	d=ReadLISLNext();
	
	if( (addr & LISL_INCR_ADDR) == (uint8)0 )	// is this a single byte Read?
		SPI_CS = DSEL_LISL;
				
	return(d);
} // ReadLISL

void WriteLISL(uint8 addr, uint8 d)
{
	WriteLISLByte(addr);
	WriteLISLByte(d);

	SPI_CS = DSEL_LISL;
	SPI_IO = RD_SPI;
} // WriteLISL

void IsLISLactive(void)
{
	SPI_CS = DSEL_LISL;				// just in case

	WriteLISL(LISL_CTRLREG_2, 0b01001010); 			// enable 3-wire, BDU=1, +/-2g

	if( ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a )	// LIS03L sensor ident
	{
		WriteLISL(LISL_CTRLREG_1, 0b11010111); // startup, enable all axis
		WriteLISL(LISL_CTRLREG_3, 0b00000000);
		WriteLISL(LISL_FF_CFG,    0b01001000); // Y-axis is height
		WriteLISL(LISL_FF_THS_L,  0b00000000);
		WriteLISL(LISL_FF_THS_H,  0b11111100); // -0,5g threshold
		WriteLISL(LISL_FF_DUR,    255);
		WriteLISL(LISL_DD_CFG,    0b00000000);
		_UseLISL = true;
	}
	else
		_UseLISL = false;
} // IsLISLactive

void ReadAccelerations()
{
	uint8 r;

	SPI_CS = DSEL_LISL;				// just in case
	SPI_IO = RD_SPI;

	//while( (ReadLISL(LISL_STATUS+LISL_READ) & 0x08) == 0 ); //may hang!
	r = ReadLISL(LISL_STATUS+LISL_READ);

	Ax = ((int16)ReadLISL(LISL_OUTX_H|LISL_READ)*256)|(int16)ReadLISL(LISL_OUTX_L|LISL_READ);
	Ay = ((int16)ReadLISL(LISL_OUTY_H|LISL_READ)*256)|(int16)ReadLISL(LISL_OUTY_L|LISL_READ);
	Az = ((int16)ReadLISL(LISL_OUTZ_H|LISL_READ)*256)|(int16)ReadLISL(LISL_OUTZ_L|LISL_READ);
	SPI_IO = RD_SPI;

} // ReadAccelerations


#endif // SPI_BACKTRACK
