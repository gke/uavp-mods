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

int16 ReadBaro(void)
{ 	// ugly goto hurdles but clearer perhaps
	int16 BaroVal;
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
	if( SendI2CByte(BARO_CTL) != I2C_ACK ) goto BAerror; 	
	if( SendI2CByte(addr) != I2C_ACK ) goto BAerror; 		
BAerror:
	I2CStop();
} // StartBaroAcq

void InitBarometer(void)
{
	int32 TimeoutMilliSec;

	TimeoutMilliSec = ClockMilliSec + 200;			// in case Baro is not present

	StartBaroAcq(BARO_TEMP);
	do
		OriginBaroTemp = ReadBaro();
	while ( (ClockMilliSec < TimeoutMilliSec ) && !_BaroReady );
	
	StartBaroAcq(BARO_PRESS);
	do
		OriginBaroAltitude = ReadBaro();
	while ( (ClockMilliSec < TimeoutMilliSec ) && !_BaroReady );

	_UseBaro = ClockMilliSec < TimeoutMilliSec;
	
	if ( _UseBaro )
	{
		DesiredBaroAltitude = CurrBaroAltitude = OriginBaroAltitude;
		BaroDescentRate = 0;
		CurrBaroTemp = OriginBaroTemp;
		VBaroComp = 0;
		StartBaroAcq(BARO_TEMP);						// overlap baro
	}
	else
		DesiredBaroAltitude = CurrBaroAltitude = BaroDescentRate = 
		CurrBaroTemp = 0;
} // InitBarometer

int32 FakeBaroVal=200;

void GetBaroAltitude(void)
{
	int16 Temp, BaroVal, PrevDescentRate, PrevBaroAltitude;
_UseBaro = true;
	if ( _UseBaro )
	{
		BaroVal = ReadBaro();
_BaroReady = true;
_BaroAcqTemp = false;
		if ( _BaroReady )
			if( _BaroAcqTemp )
			{ 	// temperature
				if ( _ThrChanging )
					CurrBaroTemp = BaroVal;
				BaroTempCorr = BaroVal - CurrBaroTemp;
				StartBaroAcq(BARO_PRESS);				// next is pressure
			}
			else
			{	// pressure
FakeBaroVal+=VBaroComp;
BaroVal = -FakeBaroVal;
BaroTempCorr = 0;
DesiredBaroAltitude = -256;
_ThrChanging = false;
				if( _ThrChanging ) // while moving throttle stick
				{
					Temp = BaroVal - OriginBaroAltitude;
					Temp += SRS32((int32)(OriginBaroTemp - CurrBaroTemp) * (int32)BaroTempCoeff + 16, 5);
					DesiredBaroAltitude = SoftFilter(DesiredBaroAltitude, Temp);
				}
				else
				{
					Temp = BaroVal - OriginBaroAltitude;
					Temp += SRS32((int32)(OriginBaroTemp - CurrBaroTemp) * (int32)BaroTempCoeff + 16, 5);
					CurrBaroAltitude = SoftFilter(CurrBaroAltitude, Temp);

					BPE = MediumFilter(BPEp, -(DesiredBaroAltitude - CurrBaroAltitude));
					BPE = Limit(BPE, -3, 8); // "soften" rate

					Temp = BPE * BaroThrottleProp;
					if ( Temp != 0)
						VBaroComp = DecayBand(VBaroComp, -Temp, Temp, 2);
					else
						VBaroComp = 0;

					// need integral term ???
		
					VBaroComp += Limit(BPE - BPEp, -8, 8) * BaroThrottleDiff;
					VBaroComp = Limit(VBaroComp, -5, 15);

					BPEp = BPE;
				}	
				StartBaroAcq(BARO_TEMP);				// next is temperature
			}
	}
} // GetBaroAltitude	

