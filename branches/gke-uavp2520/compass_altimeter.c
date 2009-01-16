// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =             Ported 2008 to 18F2520 by Prof. Greg Egan               =
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

#include "c-ufo.h"
#include "bits.h"

// initialise compass sensor
void InitDirection(void)
{
	_UseCompass = false;
	Compass = 0;
	CurrDeviation = 0;

	I2CStart();
	if( SendI2CByte(COMPASS_ADDR) != I2C_ACK ) goto IDerror;

	#ifdef CFG_COMPASS
	// apparently can be set up by TestSoftware - make free standing
	if( SendI2CByte('G')  != I2C_ACK ) goto IDerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto IDerror;
	// select operation mode, continuous mode, 20 Hz
	if( SendI2CByte(0b01110010) != I2C_ACK ) goto IDerror;
	I2CStop();

	I2CStart();
	if( SendI2CByte(COMPASS_ADDR) != I2C_ACK ) goto IDerror;
	if( SendI2CByte('r')  != I2C_ACK ) goto IDerror;
	if( SendI2CByte(0x06) != I2C_ACK ) goto IDerror;
	I2CStop();

	// read multiple read count to avoid wear-and-tear on EEPROM
	I2CStart();
	if( SendI2CByte(COMPASS_ADDR+1) != I2C_ACK ) goto IDerror;
	if( RecvI2CByte(!I2C_ACK) != 16 )
	{	// not correctly set, set it up.
		I2CStop();

		I2CStart();
		if( SendI2CByte(COMPASS_ADDR) != I2C_ACK ) goto IDerror;
		// 16 sets of magnetometer readings
		if( SendI2CByte('w')  != I2C_ACK ) goto IDerror;
		if( SendI2CByte(0x06) != I2C_ACK ) goto IDerror;
		if( SendI2CByte(16)   != I2C_ACK ) goto IDerror;
	}
	Delay100mSec(1);											// should be 6mS
	#endif // CFG_COMPASS
							
	_UseCompass = true;
IDerror:
	I2CStop();
} // InitDirection

void GetDirection(void)
{
	int16 DirVal, temp;

	if( _UseCompass && ((CycleCount & 0x00000003) == 0) )
	{
		// compass is in continous mode 
		I2CStart();
		if( SendI2CByte(COMPASS_ADDR+1) == I2C_ACK ) 
		{
			Compass = (RecvI2CByte(I2C_ACK)<<8)| RecvI2CByte(!I2C_ACK);
			I2CStop();

			// DirVal has 1/10th degrees - convert to set 360.0 deg = 240 units ??? why 240
			DirVal = Compass / 15;

			// must use pre-decrement, because of dumb compiler
			if( AbsDirection > COMPASS_MAX )
			{
				CurrDeviation = 0;
				AbsDirection--;
			}
			else
			{
				// setup desired heading (AbsDirection)
				if( AbsDirection == COMPASS_MAX )	// no heading stored yet
				{
					AbsDirection = DirVal;	// store current heading
					CurrDeviation = 0;
				}
				// calc deviation and direction of deviation
				DirVal = AbsDirection - DirVal;
				// handle wraparound
				if( DirVal <= -240/2 )
					DirVal +=  240;
				if( DirVal >   240/2 )
					DirVal -=  240;

				// positive means quadrocopter is left off-heading
				// negative means quadrocopter is right off-heading

				DirVal = Limit(DirVal, -20, 20);

				// New_CurDev = ((3*Old_CurDev)+DirVal) / 4
				DirVal = Limit(DirVal, -20, 20);			// empirical slew limit
				CurrDeviation = SRS32(((int32)CurrDeviation*3 + DirVal) * CompassFactor, 6);
			}
		}
		else
		{
			I2CStop();
			CurrDeviation = 0;
		}
	}
} // GetDirection

void DoHeadingLock(void)
{
	int16 Temp, CurrIYaw;

//	CurrIYaw = IYaw;							// protect from change by irq
//	YE += CurrIYaw;								// add the yaw stick value

	if ( _UseCompass )
		if ((CurrIYaw < (YawFailsafe - COMPASS_MIDDLE)) || (CurrIYaw > (YawFailsafe + COMPASS_MIDDLE)))
			AbsDirection = COMPASS_INVAL;		// new hold zone
		else
			YE -= Limit(CurrDeviation, -COMPASS_MAXDEV, COMPASS_MAXDEV);

	if( CompassTest )
	{
		ALL_LEDS_OFF;
		if( CurrDeviation > 0 )
			LedGreen_ON;
		else
			if( CurrDeviation < 0 )
				LedRed_ON;
		if( AbsDirection > COMPASS_MAX )
			LedYellow_ON;
	}
} // DoHeadingLock


// read temp & pressure values from baro sensor
// value in BaroVal;
// returns 1 if value is available
uint8 ReadBaro(void)
{
	uint8 OK, Conversion;
	uint8 Temp;

	_UseBaro = Conversion = false;
	// test if conversion is ready
	I2CStart();
	if( SendI2CByte(BARO_ADDR) == I2C_ACK ) 
		if( SendI2CByte(BARO_CTL) == I2C_ACK ) 
		{
			I2CStart();											// restart
			if( SendI2CByte(BARO_ADDR+1) == I2C_ACK ) 
			{
				_UseBaro = true;
				Temp = RecvI2CByte(I2C_NACK);
				I2CStop();

				if( (Temp & 0b00100000) == 0 )
				{	// conversion is ready, read it!
					I2CStart();
					if( SendI2CByte(BARO_ADDR) == I2C_ACK ) 	// access A/D registers
						if( SendI2CByte(BARO_ADC) == I2C_ACK )
						{
							I2CStart();							// restart
							if( SendI2CByte(BARO_ADDR+1) == I2C_ACK ) 
							{
								BaroVal = (RecvI2CByte(I2C_ACK)<<8) | RecvI2CByte(!I2C_NACK);
								I2CStop();
								Conversion = true;
							}
						}
				}
			}
		}
	if ( !_UseBaro )
	{
		I2CStop();
		BaroVal = 0;
	}

	return(Conversion);
} // ReadBaro

// start A/D conversion on altimeter sensor
// niaddr = 0xee to convert Temperature
//          0xf4 to convert pressure
// returns 1 if successful, else 0
uint8 StartBaroADC(uint8 addr)
{
	uint8 OK;

	OK = false;
	I2CStart();
	if( SendI2CByte(BARO_ADDR) == I2C_ACK )	
		if( SendI2CByte(BARO_CTL) == I2C_ACK ) 		// access control register, start measurement
			if( SendI2CByte(addr) == I2C_ACK ) 		// select 32kHz input, measure Temperature
				{
					I2CStop();
					_BaroTempRun = (addr == BARO_ADDR);
					OK = true;
				}
	if ( !OK )
	{
		I2CStop();
		_BaroTempRun = false;
	}

	return(OK);
} // StartBaroADC

// initialize altitude/pressure sensor
void InitAltimeter(void)
{
	uint8 c;

	VBaroComp = 0;
	BaroCompSum = 0;

	// read Temperature once to get base value
	// set SMD500 device to start Temperature conversion
	_UseBaro = StartBaroADC(BARO_ADDR);
	if( _UseBaro  ) 
	{
		// wait 40ms
		TimerMilliSec = ClockMilliSec + 40;
		while (ClockMilliSec < TimerMilliSec) ;
		
		_UseBaro = ReadBaro();

		if ( _UseBaro )
			BaseTemp = BaroVal;	// save start value
		else
			BaseTemp = 0;

		// read pressure once to get base value
		// set SMD500 device to start pressure conversion
		_UseBaro = StartBaroADC(BARO_CTL);
		if( _UseBaro  ) 
		{
			// wait 40ms
			for( c=40; c ; c--)
			{
				INTCONbits.T0IF=0;
				while(!INTCONbits.T0IF);
			}
			_UseBaro = ReadBaro();
		}
 	
		if ( _UseBaro )
			BasePressure = BaroVal;
		else
			BasePressure = 0;
	}
	else
	{
		BasePressure = 0;
		_UseBaro = false;
	}

	I2CStop();
} // InitAltimeter

void GetAltitude(void)
{
	int16 Temp;
	int32 Temp2;

	if( _UseBaro )
	{
		if( ReadBaro() )	
		{	// successful
			if( !_BaroTempRun )
			{	// current measurement was "pressure"
				if( _ThrChanging )				// while moving throttle stick
				{
					BasePressure = BaroVal;		// current read value is the new level
					BaroCompSum = 0;
				}
				else
				{	// while holding altitude
					BaroVal -= BasePressure;
					//  -400..+400 
					Temp2 = SRS32((int32)TempCorr * (int32)BaroTempCoeff + 16, 5);
					BaroVal += (int16)Temp2;	// compensating Temp
												// the corrected relative height, the higher 
												// altitude, the lesser value

					Temp2 = BaroVal;			// because of bank bits
					BaroVal = BaroCompSum;		// remember old value for delta
					BaroCompSum = ((BaroCompSum * 3) + BaroCompSum + 2) >> 2;
					Temp2 = BaroCompSum - BaroVal;	// subtract new height to get delta					

					BaroCompSum=Limit(BaroCompSum, -3, 8);
					// weiche Regelung (proportional)
					// Temp kann nicht überlaufen (-3..+8 * PropFact)
					BaroVal = (int16)(Lower8(BaroCompSum)) * BaroThrottleProp;
				
					VBaroComp = DecayBand(VBaroComp, -BaroVal, BaroVal, 2);

					// Differentialanteil		
					BaroVal = (int16)(Limit(Temp2, -8, 8) * BaroThrottleDiff);
					VBaroComp = Limit(VBaroComp+Temp, -5, 15);		
				}
				_UseBaro = StartBaroADC(BARO_ADDR);	// next is Temp
			}
			else
			{
				if( _ThrChanging )
					BaseTemp = BaroVal; 		// current read value
				else 							// TempCorr: The warmer, the higher
				{
					TempCorr = BaroVal - BaseTemp;
	//				TempCorr += 4;				// compensate rounding error later /8
				}
				_UseBaro = StartBaroADC(BARO_CTL);// next is pressure
			}
		}
	}
	else
		VBaroComp = 0;  						// no compensation
} // GetAltitude	

