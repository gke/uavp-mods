// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
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

// Accelerator sensor routine

#include "uavx.h"

#define SPI_HI_DELAY Delay10TCY()
#define SPI_LO_DELAY Delay10TCY()

void SendCommand(int8 c)
{
	int8 s;

	SPI_IO = WR_SPI;	
	SPI_CS = SEL_LISL;	
	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		if( c & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		c <<= 1;
		SPI_LO_DELAY;
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
} // SendCommand

uint8 ReadLISL(uint8 c)
{
	uint8 d;

//	SPI_SDA = 1;	// very important!! really!! LIS3L likes it
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

	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		SPI_LO_DELAY;
		d <<= 1;
		if( SPI_SDA == 1 )
			d |= 1;	
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	return(d);
} // ReadLISLNext

void WriteLISL(uint8 d, uint8 c)
{
	int8 s;

	SendCommand(c);

	for( s = 8; s; s-- )
	{
		SPI_SCL = 0;
		if( d & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_LO_DELAY;
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	SPI_CS = DSEL_LISL;
	SPI_IO = RD_SPI;	// IO is input (to allow RS232 reception)
} // WriteLISL

void IsLISLactive(void)
{
	int8 r;

	_UseLISL = false;
	SPI_CS = DSEL_LISL;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	r = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( r == 0x3A )	// a LIS03L sensor is there!
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

void InitLISL(void)
{
	NeutralLR = 0;
	NeutralFB = 0;
	NeutralUD = 0;
	#ifdef USE_ACCELEROMETER
	IsLISLactive();	
	if( _UseLISL )
		GetNeutralAccelerations();
	#endif  // USE_ACCELEROMETER
} // InitLISL

void ReadAccelerations()
{
	uint8 r;

	r = ReadLISL(LISL_STATUS + LISL_READ); // no check for 0x3a
	Ax  = (int16)ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
	Ax |= (int16)ReadLISLNext()*256;
	Ay  = (int16)ReadLISLNext();
	Ay |= (int16)ReadLISLNext()*256;
	Az  = (int16)ReadLISLNext();
	Az |= (int16)ReadLISLNext()*256;
	SPI_CS = DSEL_LISL;	// end transmission

} // ReadAccelerations

void GetNeutralAccelerations(void)
{
	// this routine is called ONLY ONCE while booting
	// read 16 time all 3 axis of linear sensor.
	// Puts values in Neutralxxx registers.
	uint8 i;
	int16 Rp, Pp, Yp;

	Delay100mSWithOutput(2);	// wait 1/10 sec until LISL is ready to talk
	// already done in caller program
	Rp = 0;
	Pp = 0;
	Yp = 0;
	for( i=0; i < 16; i++)
	{
		while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == 0 );
		ReadAccelerations();
		
		Rp += Ax;
		Pp += Az;
		Yp += Ay;
	}
	Rp = SRS16(Rp, 4);
	Pp = SRS16(Pp, 4);
	Yp = SRS16(Yp, 4);

	NeutralLR = Limit(Rp, -99, 99);
	NeutralFB = Limit(Pp, -99, 99);
	NeutralUD = Limit(Yp-1024, -99, 99); // -1g
} // GetNeutralAccelerations

void AccelerationCompensation(void)
{
	int16 AbsRollSum, AbsPitchSum, Temp;
	int16 Rp, Pp, Yp;

	if( _UseLISL )
	{
		ReadAccelerations();

		Rp = Ax;
		Yp = Ay;
		Pp = Az;;
		
		// NeutralLR ,NeutralFB, NeutralUD pass through UAVPSet 
		// and come back as MiddleLR etc.
	
		// 1 unit is 1/4096 of 2g = 1/2048g
		Rp -= MiddleLR;
		Pp -= MiddleFB;
		Yp -= MiddleUD;
	
		Yp -= 1024;	// subtract 1g
	
		#ifdef ENABLE_VERTICAL_VELOCITY_DAMPING
		// UDSum rises if ufo climbs
		// Empirical - vertical acceleration decreases at ~approx Angle/8

		AbsRollSum = Abs(RollSum);
		AbsPitchSum = Abs(PitchSum);

		if ( (AbsRollSum < 200) && ( AbsPitchSum < 200) ) // ~ 10deg
			UDSum += Yp + SRS16( AbsRollSum + AbsPitchSum, 3);

		UDSum = Limit(UDSum , -16384, 16384); 
		UDSum = DecayBand(UDSum, -10, 10, 10);
	
		Temp = SRS16(SRS16(UDSum, 4) * (int16) LinUDIntFactor, 8);
		if( (BlinkCount & 0x0003) == 0 )	
			if( Temp > Vud ) 
				Vud++;
			else
				if( Temp < Vud )
					Vud--;
	
		Vud = Limit(Vud, -20, 20);
	
		#endif // ENABLE_VERTICAL_VELOCITY_DAMPING

		#ifdef DEBUG_SENSORS
		if( IntegralCount == 0 )
		{
			Trace[TAx]= Rp;
			Trace[TAz] = Pp;
			Trace[TAy] = Yp;

			Trace[TUDSum] = UDSum;
			Trace[TVud] = Vud;
		}
		#endif
	
		// Roll

		// Static compensation due to Gravity
		#ifdef OPT_ADXRS
		Rp -= SRS16(RollSum * 11, 5);
		#else // OPT_IDG
		Rp -= SRS16(RollSum * (-15), 5); 
		#endif
	
		#ifndef ENABLE_DYNAMIC_MASS_COMP_ROLL
		// dynamic correction of moved mass
		#ifdef OPT_ADXRS
		Rp += (int16)RollRate << 1;
		#else // OPT_IDG
		Rp -= (int16)RollRate;
		#endif	
		#else
		// no dynamic correction of moved mass
		#endif

		// correct DC level of the integral
		LRIntKorr = 0;
		#ifdef OPT_ADXRS
		if( Rp > 10 ) LRIntKorr =  1;
		else
			if( Rp < 10 ) LRIntKorr = -1;
		#else // OPT_IDG
		if( Rp > 10 ) LRIntKorr = -1;
		else
			if( Rp < 10 ) LRIntKorr =  1;
		#endif
	
		// Pitch

		Pp = -Pp;				// long standing BUG!

		// Static compensation due to Gravity
		#ifdef OPT_ADXRS
		Pp -= SRS16(PitchSum * 11, 5);	
		#else // OPT_IDG
		Pp -= SRS16(PitchSum * (-15), 5);
		#endif
		
		#ifndef ENABLE_DYNAMIC_MASS_COMP_PITCH
		// dynamic correction of moved mass
		#ifdef OPT_ADXRS
		Pp += (int16)PitchRate << 1;
		#else // OPT_IDG
		Pp -= (int16)PitchRate;
		#endif	
		#else
		// no dynamic correction of moved mass
		#endif
	
		// correct DC level of the integral
		FBIntKorr = 0;
		#ifdef OPT_ADXRS
		if( Pp > 10 ) FBIntKorr =  1; 
		else 
			if( Pp < 10 ) FBIntKorr = -1;
		#endif
		#ifdef OPT_IDG
		if( Pp > 10 ) FBIntKorr = -1;
		else
			if( Pp < 10 ) FBIntKorr =  1;
		#endif		
	}	
	else
	{
		Vud = 0;
		#ifdef DEBUG_SENSORS
		Trace[TAx] = 0;
		Trace[TAz] = 0;
		Trace[TAy] = 0;

		Trace[TUDSum] = 0;
		Trace[TVud] = 0;
		#endif
	}
} // AccelerationCompensation

