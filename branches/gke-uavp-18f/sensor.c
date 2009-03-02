
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

// The "sensor bus" routines (barometric and compass)

#include "c-ufo.h"
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
} // I2CDelay

// put SCL to high-z and wait until line is really hi
// returns != 0 if ok
uns8 I2CWaitClkHi(void)
{
	uns8 c=1;

	I2CDelay();
	I2C_CIO=1;	// set SCL to input, output a high
	while( I2C_SCL == 0 )	// wait for line to come hi
	{
//		nop2();
		c++;
		if( c == 0 )
			break;
	}	
	I2CDelay();
	return(c);
} // I2CWaitClkHi

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
} // I2CStart

// send a stop condition
void I2CStop(void)
{
	I2C_DIO=0;	// set SDA to output
	I2C_SDA = 0;	// output a low
	I2CWaitClkHi();

	I2C_DIO=1;	// set SDA to input, output a high, STOP condition
	I2CDelay();		// leave clock high
} // I2CStop

static uns8 nii;	// mus be bank0 or shrBank

// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
uns8 SendI2CByte(uns8 nidata)
{

	for(nii=0; nii<8; nii++)
	{
		if( (nidata & 0x10) !=0 )
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
//	I2CDelay();
//	I2C_IO = 0;		// set SDA to output
//	I2C_SDA = 0;	// leave output low
	return(nii);
} // SendI2CByte


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
//	I2CDelay();
//	I2C_IO = 0;	// set SDA to output
	return(nidata);
} // RecvI2CByte


// initialize compass sensor
void InitDirection(void)
{

// set Compass device to Compass mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto IDerror;

	// CAUTION: compass calibration must be done using TestSoftware!

	_UseCompass = 1;
IDerror:
	I2CStop();
} // InitDirection

// Read direction, convert it to 2 degrees unit
// and store result in variable AbsDirection.
// The current heading correction is stored in CurDeviation
void GetDirection(void)
{

	int16 DirVal, temp;

	if( _UseCompass && ((BlinkCount & 0x03) == 0) )	// enter every 4th scan
	{
		// set Compass device to Compass mode 
		I2CStart();
		if( SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK ) 
		{
			I2CStop();
			CurDeviation = 0;	// no sensor present, deviation = 0
		}
		else
		{		
			DirVal = (RecvI2CByte(I2C_ACK)<<8) | RecvI2CByte(I2C_NACK);
			I2CStop();
	
			// DirVal has 1/10th degrees
			// convert to set 360.0 deg = 240 units
			DirVal /= 15;
	
			// must use pre-decrement, because of dumb compiler
			if( AbsDirection > COMPASS_MAX )
			{
				CurDeviation = 0;
				AbsDirection--;
			}
			else
			{
				// setup desired heading (AbsDirection)
				if( AbsDirection == COMPASS_MAX )	// no heading stored yet
				{
					AbsDirection = DirVal;	// store current heading
					CurDeviation = 0;
				}
				// calc deviation and direction of deviation
				DirVal = AbsDirection - DirVal;
				// handle wraparound
				if( DirVal <= -240/2 ) 
					DirVal +=  240;
				else
					if( DirVal >   240/2 ) 
						DirVal -=  240;
	
				// positive means ufo is left off-heading
				// negative means ufo is right off-heading
	
				DirVal = Limit(DirVal, -20, 20); // limit to give soft reaction
	
				// Empirical found :-)
				// New_CurDev = ((3*Old_CurDev)+DirVal) / 4
				CurDeviation = SRS16((((int16)CurDeviation *3 + DirVal) << 2) 
								* (int16)CompassFactor, 8);
			}
		}
		#ifdef DEBUG_SENSORS
		if( IntegralCount == 0 )
		{
			SendComValH(AbsDirection);
			SendComChar(';');
		}
		#endif				
	}
	#ifdef DEBUG_SENSORS
	else	// no new value received
		if( IntegralCount == 0 )
			SendComChar(';');
	#endif

} // GetDirection

static uns16 BaroVal;

// read temp & pressure values from baro sensor
// return value= niltemp;
// returns 1 if value is available
uns8 ReadValueFromBaro(void)
{
	// Possible I2C protocol error - split read of ADC
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto RVerror;
	if( SendI2CByte(BARO_ADC_MSB) != I2C_ACK ) goto RVerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto RVerror;
	BaroVal = RecvI2CByte(I2C_NACK) << 8;
	I2CStop();
		
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto RVerror;
	if( SendI2CByte(BARO_ADC_LSB) != I2C_ACK ) goto RVerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto RVerror;
	BaroVal |= RecvI2CByte(I2C_NACK);
	I2CStop();

	return(I2C_NACK);

RVerror:
	I2CStop();
	_UseBaro = 0; // read error, disable baro
	#ifdef BARO_RETRY
	_Hovering = 0;	
	if ( BaroRestarts < 100 )
	{
		InitAltimeter();
		_BaroRestart = 1;
		BaroRestarts++;
	}
	#endif
	return(I2C_ACK);
} // ReadValueFromBaro

// start A/D conversion on altimeter sensor
// TempOrPress = BARO_TEMP to convert temperature
//               BARO_PRESS to convert pressure
// returns 1 if successful, else 0
uns8 StartBaroADC(uns8 TempOrPress)
{
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto SBerror;

	// access control register, start measurement
	if( SendI2CByte(BARO_CTL) != I2C_ACK ) goto SBerror;

	// select 32kHz input, measure temperature
	if( SendI2CByte(TempOrPress) != I2C_ACK ) goto SBerror;
	I2CStop();

	// set or clear bit to signal what A/D is currently running
	_BaroTempRun = (TempOrPress == BaroTemp);
	
	return(I2C_NACK);
SBerror:
	I2CStop();
	return(I2C_ACK);
} // StartBaroADC

// initialize compass sensor
void InitAltimeter(void)
{
	uns8 Temp;
	// SMD500 9.5mS (T) 34mS (P)  
	// BMP085 4.5mS (T) 25.5mS (P) OSRS=3, 7.5mS OSRS=1
	// Baro is assumed offline unless it responds - no retries!

	VBaroComp = BaroCount = 0;

	// Determine baro type
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_TYPE) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroType = RecvI2CByte(I2C_NACK);
	I2CStop();

	if ( BaroType == BARO_ID_BMP085 )
		BaroTemp = BARO_TEMP_BMP085;
	else
		BaroTemp = BARO_TEMP_SMD500;

	// read temperature once to get base value
	if( !StartBaroADC(BaroTemp) ) goto BAerror;
	DELAY_MS(10);
	ReadValueFromBaro();
	BaroBaseTemp = BaroVal;	// save start value
		
	// read pressure once to get base value
	if( !StartBaroADC(BARO_PRESS) ) goto BAerror;
	DELAY_MS(30);
	ReadValueFromBaro();	
	BaroBasePressure = BaroVal;
	
	// set baro device to start temperature conversion
	// before first call to ComputeBaroComp
	if( !StartBaroADC(BaroTemp) ) goto BAerror;	

	_UseBaro = 1;

	return;

BAerror:
	_UseBaro = _Hovering = 0;
	I2CStop();
} // InitAltimeter

void ComputeBaroComp(void)
{
	uns8 r;
	uns16 OldBaroRelPressure, Delta, Temp;

	BaroCount++;

	if( _UseBaro )
		// ~10ms for Temperature and 40ms for Pressure at TimeStep = 2 - UGLY
#ifdef SLOW_BARO
		if (((BaroCount >= 8) && _BaroTempRun) || ((BaroCount >= 32 ) && !_BaroTempRun))
#else
		if (((BaroCount >= 2) && _BaroTempRun) || ((BaroCount >= 8 ) && !_BaroTempRun))
#endif // SLOW_BARO	
		{
			BaroCount = 0;
			if ( ReadValueFromBaro() == I2C_NACK) 	// returns niltemp as value		
				if( _BaroTempRun )
				{
					StartBaroADC(BARO_PRESS); // next is pressure - overlap
					if( ThrDownCount )
						BaroBaseTemp = BaroVal; // current read value
					else 
						BaroRelTempCorr = BaroVal - BaroBaseTemp;
	
				}
				else
				{	// current measurement was "pressure"
					StartBaroADC(BaroTemp);	// next is temp - overlap

					if( ThrDownCount )	// while moving throttle stick
					{
						BaroBasePressure = BaroVal;	
						_Hovering = VBaroComp = 0;	
					}
					else
					{
						#ifdef BARO_SCRATCHY_BEEPER
						Beeper_TOG;
						#endif

						if ( !_Hovering )
						{
							BaroRelPressure = 0;
							_Hovering = 1;
						}
	
						// while holding altitude
						BaroVal -= BaroBasePressure;
						// BaroVal has -400..+400 approx
						BaroVal += SRS16((int16)BaroRelTempCorr * (int16)BaroTempCoeff + 16, 5);

						OldBaroRelPressure = BaroRelPressure;	// remember old value for delta
	
						#ifdef BARO_HARD_FILTER
						BaroRelPressure = HardFilter(BaroRelPressure, BaroVal);
						#else
						BaroRelPressure = MediumFilter(BaroRelPressure, BaroVal);				
						#endif // BARO_HARD_FILTER
	
						Delta = BaroRelPressure - OldBaroRelPressure;	// subtract new height to get delta

						BaroRelPressure = Limit(BaroRelPressure, -2, 8); // was: +10 and -5
		
						// strictly this is acting more like an integrator 
						// bumping VBaroComp up and down proportional to the error?
		
						Temp = BaroRelPressure * (int16)BaroThrottleProp;
						if( VBaroComp > Temp )
							VBaroComp--;
						else
							if( VBaroComp < Temp )
								VBaroComp++; // climb
#ifndef SLOW_BARO
						if( VBaroComp > Temp )
							VBaroComp--;
						else
#endif // SLOW_BARO
							if( VBaroComp < Temp )
								VBaroComp++;
		
						// Differentialanteil		
						VBaroComp += Limit(Delta, -8, 8) * (int16)BaroThrottleDiff;
	
						VBaroComp = Limit(VBaroComp, -5, 15);
							
						#ifdef BARO_SCRATCHY_BEEPER
						Beeper_TOG;
						#endif
					}
				}
		}

	#ifdef DEBUG_SENSORS	
	if( IntegralCount == 0 )
		if ( _UseBaro )
		{
			SendComValH(VBaroComp);
			SendComChar(';');
			SendComValH16(BaroRelPressure);
			SendComChar(';');
		}
		else	// no baro sensor active
		{
			SendComChar(';');
			SendComChar(';');
		}
	#endif
} // ComputeBaroComp	

