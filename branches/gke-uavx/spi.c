// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =              Ported to 18F2xxx 2008 by Prof. Greg Egan              =
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

#include "UAVX.h"

#define SPI_CS		PORTCbits.RC5
#define SPI_SDA		PORTCbits.RC4
#define SPI_SCL		PORTCbits.RC3
#define SPI_IO		TRISCbits.TRISC4

#define	RD_SPI	1
#define WR_SPI	0

#define SEL_LEDS  1
#define SEL_LISL  0

// LISL-Register mapping
#define	LISL_WHOAMI		(0x0f)
#define	LISL_OFFSET_X	(0x16)
#define	LISL_OFFSET_Y	(0x17)
#define	LISL_OFFSET_Z	(0x18)
#define	LISL_GAIN_X		(0x19)
#define	LISL_GAIN_Y		(0x1A)
#define	LISL_GAIN_Z		(0x1B)
#define	LISL_CTRLREG_1	(0x20)
#define	LISL_CTRLREG_2	(0x21)
#define	LISL_CTRLREG_3	(0x22)
#define	LISL_STATUS		(0x27)
#define LISL_OUTX_L		(0x28)
#define LISL_OUTX_H		(0x29)
#define LISL_OUTY_L		(0x2A)
#define LISL_OUTY_H		(0x2B)
#define LISL_OUTZ_L		(0x2C)
#define LISL_OUTZ_H		(0x2D)
#define LISL_FF_CFG		(0x30)
#define LISL_FF_SRC		(0x31)
#define LISL_FF_ACK		(0x32)
#define LISL_FF_THS_L	(0x34)
#define LISL_FF_THS_H	(0x35)
#define LISL_FF_DUR		(0x36)
#define LISL_DD_CFG		(0x38)
#define LISL_INCR_ADDR	(0x40)
#define LISL_READ		(0x80)

void IsLISLactive(void);
void WriteLISLByte(uint8);
void WriteLISL(uint8, uint8);
uint8 ReadLISL(uint8);
uint8 ReadLISLNext(void);
void ReadLISLXYZ(void);

// LIS3LV02DQ Inertial Sensor (Accelerometer)

#ifdef TEST_LISL

uint8 ReadLISL(uint8 d)
{
	uint8 s;

	SPI_SDA = 1;	// very important!! really!! LIS3L likes it
	SPI_IO = WR_SPI;
	SPI_SCL = 0;
	SPI_CS = SEL_LISL;	
	for( s = 8; s ; s-- )
	{
		SPI_SCL = 0;
		if( d & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_SCL = 1;
	}

	SPI_IO = RD_SPI;
	d = 0;
	for( s = 8; s ; s-- )
	{
		SPI_SCL = 0;
		d <<= 1;
		if( SPI_SDA == 1 )
			d |= 1;	// set LSB
		SPI_SCL = 1;
	}
	SPI_CS = SEL_LEDS;
	return(d);
} // ReadLISL

void WriteLISL(uint8 addr, uint8 d)
{
	uint8 s;

	SPI_IO = WR_SPI;
	SPI_SCL = 0;
	SPI_CS = SEL_LISL;	
	for( s = 8; s ; s-- )
	{
		SPI_SCL = 0;
		if( addr & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		addr <<= 1;
		SPI_SCL = 1;
	}

	for( s = 8; s ; s-- )
	{
		SPI_SCL = 0;
		if( d & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_SCL = 1;
	}

	SPI_CS = SEL_LEDS;
	SPI_IO = RD_SPI;

} // WriteLISL

void IsLISLactive(void)
{
	uint8 r;

	SPI_CS = SEL_LEDS;
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
		_UseLISL = true;
	}
} // IsLISLactive

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
	
	if( (addr & LISL_INCR_ADDR) == 0 )	// is this a single byte Read?
		SPI_CS = SEL_LEDS;
				
	return(d);
} // ReadLISL

void WriteLISL(uint8 addr, uint8 d)
{
	WriteLISLByte(addr);
	WriteLISLByte(d);

	SPI_CS = SEL_LEDS;
	SPI_IO = RD_SPI;
} // WriteLISL

void IsLISLactive(void)
{
	SPI_CS = SEL_LEDS;				// just in case

	WriteLISL(LISL_CTRLREG_2, 0b01001010); 			// enable 3-wire, BDU=1, +/-2g

	if( ReadLISL(LISL_WHOAMI + LISL_READ) == 0x3a )	// LIS03L sensor ident
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

#endif // TEST_LISL

void ReadLISLXYZ()
{
	uint8 r;

	SPI_CS = SEL_LEDS;				// just in case
	SPI_IO = RD_SPI;

	// while( (ReadLISL(LISL_STATUS+LISL_READ) & 0x08) == 0 ); //may hang!
	r = ReadLISL(LISL_STATUS+LISL_READ);

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
	SPI_IO = RD_SPI;

} // ReadLISLXYZ

//-----------------------------------------------------------------------------

//send LedShadow byte to TPIC6B595N
void SendLeds(void)
{
	uint8	LEDs;
	uint8	s;

	LEDs = LedShadow;

    SPI_IO = WR_SPI;
	SPI_CS = SEL_LEDS;							// select TPIC	
	for(s=8; s ; s--)
	{
		SPI_SCL = 0;
		if ( LEDs & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		LEDs <<= 1;
		SPI_SCL = 1;
	}

	PORTCbits.RC1=1;
	PORTCbits.RC1=0;						// latch into drivers
} // SendLeds
