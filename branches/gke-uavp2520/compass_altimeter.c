// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =      Rewritten and ported to 18F2520 2008 by Prof. Greg Egan        =
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

void InitDirection(void);
void GetDirection(void);
void DoHeadingLock(void);
void InitBarometer(void);
int16 AltitudeCompensation(uint16, uint16);
void GetBaroAltitude(void);
uint16 ReadBaro(void);
void StartBaroAcq(uint8);

void InitDirection(void)
{ 	// ugly goto hurdles but clearer perhaps
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
			Compass = (RecvI2CByte(I2C_ACK)<<8) | RecvI2CByte(!I2C_ACK);
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

				// positive means quadrocopter is left of heading
				// negative means quadrocopter is right of heading

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

// --------------------------------------------------------------------------------

uint16 ReadBaro(void)
{ 	// ugly goto hurdles but clearer perhaps
	uint16 BaroVal;
	uint8 BaroStatus;

	BaroVal = 0;
	_BaroReady = false;

	I2CStart();
	if( SendI2CByte(BARO_ADDR) != I2C_ACK ) goto RBerror; 
	if( SendI2CByte(BARO_CTL) != I2C_ACK ) goto RBerror;

	I2CStart();											// restart
	if( SendI2CByte(BARO_ADDR+1) != I2C_ACK ) goto RBerror;
		
	BaroStatus = RecvI2CByte(I2C_NACK);
	I2CStop();

	if( (BaroStatus & 0b00100000) == 0 )
    {
		I2CStart();						// conversion is ready, read it!
		if( SendI2CByte(BARO_ADDR) != I2C_ACK ) goto RBerror;
		if( SendI2CByte(BARO_ADC) != I2C_ACK ) goto RBerror;

		I2CStart();							// restart
		if( SendI2CByte(BARO_ADDR + 1) != I2C_ACK ) goto RBerror;
		BaroVal = (RecvI2CByte(I2C_ACK)<<8) | RecvI2CByte(!I2C_NACK);
		I2CStop();
		_BaroReady = true;
	}

RBerror:
	if ( !_BaroReady )
		I2CStop();
	return(BaroVal);
} // ReadBaro

void StartBaroAcq(uint8 addr)
{ // ugly goto hurdles but clearer perhaps

	_BaroReady = false;
	_BaroAcqTemp = (addr == BARO_TEMP);

	I2CStart();
	if( SendI2CByte(BARO_ADDR) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_CTL) != I2C_ACK )  goto BAerror; 	
	if( SendI2CByte(addr) != I2C_ACK )  goto BAerror;
	 		
BAerror:
	I2CStop();
} // StartBaroAcq

int16 AltitudeCompensation(uint16 A, uint16 T)
{	// altitudes are relative to Origin altitude
	return (OriginBaroAltitude - ((int32)A + SRS32( (int32)T * (int32)BaroTempCoeff + 16, 5)));
} // AltitudeCompensation

void InitBarometer(void)
{
	uint8 s;
	uint16 BaroVal;
	int32 TimeoutMilliSec, AvTemperature, AvPressure;

	OriginBaroAltitude = DesiredBaroAltitude = CurrBaroAltitude = 0;
	BE = BEp = BESum = 0;

	AvTemperature = AvPressure = 0;
	s = 0;
	do
	{
		TimeoutMilliSec = ClockMilliSec + 200;			// in case Baro is not present

		StartBaroAcq(BARO_TEMP);
		do
		{
			BaroVal = ReadBaro();
			AvTemperature += BaroVal;
		}
		while ( (ClockMilliSec < TimeoutMilliSec ) && !_BaroReady );
	
		StartBaroAcq(BARO_PRESS);
		do
		{
			BaroVal = ReadBaro();
			AvPressure += BaroVal;
		}	
		while ( (ClockMilliSec < TimeoutMilliSec ) && !_BaroReady );
	}
	while ( (++s < 32) && (ClockMilliSec < TimeoutMilliSec) );

	_UseBaro = ClockMilliSec < TimeoutMilliSec;
	
	if ( _UseBaro )
	{
		BaroVal = AvPressure >> 5;
		CurrBaroTemp = AvTemperature >> 5;
		OriginBaroAltitude = - AltitudeCompensation(BaroVal, CurrBaroTemp);
		OriginBaroTemp = CurrBaroTemp;
		StartBaroAcq(BARO_TEMP);					// overlap baro
	}
} // InitBarometer

void GetBaroAltitude(void)
{

// no integral in Param Set ???
#define BaroThrottleInt 2

	uint16 Temp, BaroVal;

	if ( _UseBaro )
	{
		BaroVal = ReadBaro();
		if ( _BaroReady )
			if( _BaroAcqTemp )
			{ 	// temperature
				CurrBaroTemp = BaroVal;
				StartBaroAcq(BARO_PRESS);				// next is pressure
			}
			else
			{	// pressure
				CurrBaroAltitude = SoftFilter(CurrBaroAltitude, AltitudeCompensation(BaroVal, CurrBaroTemp));
				if( _ThrChanging ) 						// while moving throttle stick
				{
					DesiredBaroAltitude = CurrBaroAltitude;
					BESum = 0; 							// zero integrator
					BEp = 0;
				}
			
				BE =  -(DesiredBaroAltitude - CurrBaroAltitude);
				BE = Limit(BE, -3, 8); 					// "soften" rate

				VBaroComp = BE * BaroThrottleProp;
			
				// added integral ???
				BESum = Limit(BESum + BE, -16, 16);
				VBaroComp += Limit(BESum * BaroThrottleInt, -4, 4);
		
				VBaroComp += Limit((BEp - BE) * BaroThrottleDiff, -8, 8);
				
				VBaroComp = Limit(-VBaroComp, -5, 15);  // faster up than down

				BEp = BE;
	
				StartBaroAcq(BARO_TEMP);				// next is temperature
			}
	}
} // GetBaroAltitude	

