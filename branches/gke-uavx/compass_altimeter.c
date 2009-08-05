// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =           http://code.google.com/p/uavp-mods/ http://uavp.ch        =
// =======================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "uavx.h"

// Prototypes

void InitCompass(void);
void InitHeading(void);
void GetHeading(void);

void StartBaroADC(void);
void ReadBaro(void);
void GetBaroPressure(void);
void InitBarometer(void);
void CheckForHover(void);
void BaroAltitudeHold(int16);

//_____________________________________________________________________________________

// Compass

void InitCompass(void)
{
	// 20Hz continuous read with periodic reset.
	#ifdef SUPPRESS_COMPASS_SR
		#define COMP_OPMODE 0b01100010
	#else
		#define COMP_OPMODE 0b01110010
	#endif // SUPPRESS_COMPASS_SR

	// Set device to Compass mode 
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('G')  != I2C_ACK ) goto CTerror;
	if( SendI2CByte(0x74) != I2C_ACK ) goto CTerror;
	if( SendI2CByte(COMP_OPMODE) != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(1);

	I2CStart(); // save operation mode in EEPROM
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('L')  != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(1);

	I2CStart(); // Do Bridge Offset Set/Reset now
	if( SendI2CByte(COMPASS_I2C_ID) != I2C_ACK ) goto CTerror;
	if( SendI2CByte('O')  != I2C_ACK ) goto CTerror;
	I2CStop();

	Delay1mS(50);

	// use default heading mode (1/10th degrees)

	_CompassValid = true;
	return;
CTerror:
	_CompassValid = false;

	I2CStop();
} // InitCompass


void InitHeading(void)
{
	GetHeading();
	DesiredHeading = Heading;
	HEp = 0;
} // InitHeading

void GetHeading(void)
{
	static i16u Compass;
	static int32 Temp;

	if( _CompassValid  ) // continuous mode but Compass only updates avery 50mS
	{
		I2CStart();
		_CompassMissRead |= SendI2CByte(COMPASS_I2C_ID+1) != I2C_ACK; 
		Compass.high8 = RecvI2CByte(I2C_ACK);
		Compass.low8 = RecvI2CByte(I2C_NACK);
		I2CStop();

		//Temp = (int32)((int32)Compass * MILLIPI)/1800L - COMPASS_OFFSET;
		Temp = ConvertDDegToMPi(Compass.i16) - CompassOffset;
		Heading = Make2Pi((int16) Temp);	
	}
	else
		Heading = 0;

} // GetHeading

//_____________________________________________________________________________________

// Barometer

#ifdef BARO_HARD_FILTER
	#define BaroFilter HardFilter
#else
	#define BaroFilter MediumFilter
#endif // BARO_HARD_FILTER

void StartBaroADC(void)
{
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto SBerror;

	// access control register, start measurement
	if( SendI2CByte(BARO_CTL) != I2C_ACK ) goto SBerror;

	// select 32kHz input, measure temperature
	if( SendI2CByte(BARO_PRESS) != I2C_ACK ) goto SBerror;
	I2CStop();

	if ( BaroType == BARO_ID_BMP085 )
		mS[BaroUpdate] = mS[Clock] + BMP085_PRESS_TIME_MS;
	else
		mS[BaroUpdate] = mS[Clock] + SMD500_PRESS_TIME_MS;

	_BaroAltitudeValid = true;
	return;
SBerror:
	I2CStop();
	_BaroAltitudeValid = false; 
	return;
} // StartBaroADC

void ReadBaro(void)
{
	// Possible I2C protocol error - split read of ADC
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto RVerror;
	if( SendI2CByte(BARO_ADC_MSB) != I2C_ACK ) goto RVerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto RVerror;
	BaroVal.high8 = RecvI2CByte(I2C_NACK);
	I2CStop();
			
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto RVerror;
	if( SendI2CByte(BARO_ADC_LSB) != I2C_ACK ) goto RVerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto RVerror;
	BaroVal.low8 = RecvI2CByte(I2C_NACK);
		I2CStop();

	StartBaroADC(); // overlap next acquisition

	return;

RVerror:
	I2CStop();

	_BaroAltitudeValid = false; 
	return;
} // ReadBaro

static int16 BaroAverage;

void GetBaroPressure(void)
{	// SMD500 9.5mS (T) 34mS (P)  
	// BMP085 4.5mS (T) 25.5mS (P) OSRS=3, 7.5mS OSRS=1
	// Baro is assumed offline unless it responds - no retries!

	if ( mS[Clock] > mS[BaroUpdate] )
	{
		ReadBaro();
		BaroAverage += ( (int24)BaroVal.u16 - OriginBaroPressure );
		BaroSample++;
		if ( BaroSample == 8 )
		{
			CurrentBaroPressure = BaroFilter(CurrentBaroPressure, SRS16( BaroAverage, 3) );
			BaroSample = BaroAverage = 0;
			_NewBaroValue = true;
		}

		#ifdef DEBUG_SENSORS	
		Trace[TCurrentBaroPressure] = CurrentBaroPressure;
		#endif
	}
}// GetBaroPressure

// initialize compass sensor
void InitBarometer(void)
{	
	const int8 Samples = 16; 
	int24 BaroAv;
	int8 s;
	uint8 r;

	VBaroComp = BaroAverage = 0;

	// Determine baro type
	I2CStart();

	if( SendI2CByte(BARO_I2C_ID) != I2C_ACK ) goto BAerror;
	if( SendI2CByte(BARO_TYPE) != I2C_ACK ) goto BAerror;
	I2CStart();	// restart
	if( SendI2CByte(BARO_I2C_ID+1) != I2C_ACK ) goto BAerror;
	BaroType = RecvI2CByte(I2C_NACK);
	I2CStop();

	// read pressure once to get base value
	StartBaroADC();
	if ( !_BaroAltitudeValid ) goto BAerror;

	BaroAv = 0;
	for (s = 0; s < Samples; s++)
	{
		while ( mS[Clock] < mS[BaroUpdate] );
		ReadBaro();
		BaroAv += (int24)BaroVal.u16;	
	}
	
	OriginBaroPressure = BaroAv / (int24)Samples;
	CurrentBaroPressure = 0;
	_NewBaroValue = false;
	BaroSample = 0;

	_BaroAltitudeValid = true;

	return;

BAerror:
	_BaroAltitudeValid = _Hovering = false;
	I2CStop();
} // InitBarometer

void CheckForHover(void)
{
	CheckThrottleMoved();
	
	if( _ThrottleMoving )	// while moving throttle stick
	{
		_Hovering = false;
		DesiredBaroPressure = CurrentBaroPressure;
		VBaroComp = BE = BEp = 0;	
	}
	else
		_Hovering = _BaroAltitudeValid;

	if ( _Hovering )
		BaroAltitudeHold(DesiredBaroPressure);

} // CheckForHover

void BaroAltitudeHold(int16 DesiredBaroPressure)
{	// decreasing pressure is increasing altitude
	static int16 Temp, Delta;

	if ( _NewBaroValue )
	{
		_NewBaroValue = false;

		#ifdef BARO_SCRATCHY_BEEPER
		Beeper_TOG;
		#endif
	
		BE = CurrentBaroPressure - DesiredBaroPressure;		
		BE = Limit(BE, -5, 10); 
			
		// strictly this is acting more like an integrator 
		// bumping VBaroComp up and down proportional to the error?	
		Temp = BE * (int16)P[BaroCompKp];
		if( VBaroComp > Temp )
			VBaroComp--;
		else
			if( VBaroComp < Temp )
				VBaroComp++; // climb
					
		// Differential	
		Delta = BE - BEp;	
		VBaroComp += Limit(Delta, -5, 8) * (int16)P[BaroCompKd];
		
		VBaroComp = Limit(VBaroComp, BARO_LOW_THR_COMP, BARO_HIGH_THR_COMP);
	
		if ( !(  _RTHAltitudeHold && ( NavState == ReturningHome ) ) )
		{
			Temp = DesiredThrottle + VBaroComp;
			HoverThrottle = HardFilter(HoverThrottle, Temp);
		}
	
		BEp = BE;
	
		#ifdef BARO_SCRATCHY_BEEPER
		Beeper_TOG;
		#endif
	}

} // ComputeBaroComp	

