// =======================================================================
// =                                 UAVX                                =
// =                        Quadrocopter Controller                      =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
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

// Autonomous flight routines

#include "uavx.h"

// Prototypes

void Descend(void);
void Navigate(int16, int16);
void CheckAutonomous(void);

// Defines

#define NAV_CLOSING_RADIUS ( (int32)CLOSING_RADIUS * 5L )
#define SQR_NAV_CLOSING_RADIUS ( NAV_CLOSING_RADIUS * NAV_CLOSING_RADIUS )
//#define NavKp (((int32)MAX_ANGLE * 256L ) / NAV_CLOSING_RADIUS )
#define NavKi 1

// Variables
extern int8 ValidGPSSentences;
extern boolean GPSSentenceReceived;

int16 NavRCorr, SumNavRCorr, NavPCorr, SumNavPCorr;

void GPSAltitudeHold(int16 DesiredAltitude)
{


} // GPSAltitudeHold

void Descend(void)
{	
	if( (BlinkCount & 0x000f) == 0 )
		DesiredThrottle = Limit(DesiredThrottle--, 0, _Maximum); // safest	
} // Descend

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer artihmetic

	int32 Temp;
	int16 NavKp, GPSGain;
	int16 Angle, Range, EastDiff, NorthDiff, Heading;

	if ( _NavComputed ) // use previous corrections
	{
		DesiredRoll = Limit(DesiredRoll + NavRCorr, -_Neutral, _Neutral);
		DesiredPitch = Limit(DesiredPitch + NavPCorr, -_Neutral, _Neutral);
	}
	else
	{
		#ifdef GPS_IK7_GAIN
		GPSGain = Limit(IK7-20, 0, 256); // compensate for EPA offset of up to 20
		NavKp = ((int32)GPSGain * MAX_ANGLE) / NAV_CLOSING_RADIUS; // /_Maximum) * 256L
		#else
		#define NavKp (((int32)MAX_ANGLE * 256L ) / NAV_CLOSING_RADIUS )
		#endif // GPS_IK7_GAIN

		EastDiff = GPSEastWay - GPSEast;
		NorthDiff = GPSNorthWay - GPSNorth;
		Heading = CompassHeading - ConvertDDegToMPi(MAGNETIC_VARIATION*10L);
	
		if ( (Abs(EastDiff) !=0 ) || (Abs(NorthDiff) != 0 ))
		{ 
			Range = Max(Abs(NorthDiff), Abs(EastDiff)); 
			if ( Range < NAV_CLOSING_RADIUS )
				Range = int16sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff); 
			else
				Range = NAV_CLOSING_RADIUS;
			
			Angle = Make2Pi(int16atan2(EastDiff, NorthDiff) - Heading);
		
			Temp = ((int32)Range * NavKp )>>8; // always +ve so can use >>
	
			NavRCorr = SRS32((int32)int16sin(Angle) * Temp, 8);			
			if ( Sign(SumNavRCorr) == Sign(NavRCorr) )
				SumNavRCorr = Limit (SumNavRCorr + Range, -(NavIntLimit)*256, NavIntLimit*256);
			else
				SumNavRCorr = 0;
			DesiredRoll += NavRCorr + (SumNavRCorr * NavKi) / 256;
			DesiredRoll = Limit(DesiredRoll , -_Neutral, _Neutral);
		
			NavPCorr = SRS32(-(int32)int16cos(Angle) * Temp, 8);
			if ( Sign(SumNavPCorr) == Sign(NavPCorr) )
				SumNavPCorr = Limit (SumNavPCorr + Range, -(NavIntLimit*256), NavIntLimit*256);
			else
				SumNavPCorr = 0;
			DesiredPitch += NavPCorr + (SumNavPCorr * NavKi) / 256;
			DesiredPitch = Limit(DesiredPitch , -_Neutral, _Neutral);

			_NavComputed = true;
		}
	}
} // Navigate

void CheckAutonomous(void)
{
	#ifdef FAKE_GPS
	int16 CosH, SinH, NorthD, EastD, A, Heading;

	_GPSValid = true;

	#endif // FAKE_GPS

	DesiredThrottle = IGas;
	DesiredRoll = IRoll;
	DesiredPitch = IPitch;
	DesiredYaw = IYaw;

	UpdateGPS();

	if ( _Flying && !_Signal )
	{ 
		// NO AUTONOMY ON LOST SIGNAL IN THIS VERSION
	  	// do nothing - use Wolfgang's failsafe
	}
	else
		if ( _GPSValid ) // && _UseCompass )
			if ( IK5 > _Neutral )
			{
				_HoldingStation = false;
				Navigate(0, 0);
			}
			else
			#ifdef OVERRIDE_HOVER_CONDITION
			{
				if ( (Abs(DesiredRoll) > MAX_CONTROL_CHANGE) || (Abs(DesiredPitch) > MAX_CONTROL_CHANGE) )
					{
						// acquire hold coordinates
						GPSNorthHold = GPSNorth;
						GPSEastHold = GPSEast;
						SumNavRCorr= SumNavPCorr = 0;
						_HoldingStation = true;
						_NavComputed = false;
					}		
					Navigate(GPSNorthHold, GPSEastHold);
			}
			#else
				if (_Hovering )
				{
					if ( (Abs(DesiredRoll) > MAX_CONTROL_CHANGE) || (Abs(DesiredPitch) > MAX_CONTROL_CHANGE) || !_HoldingStation )

					{
						// acquire hold coordinates
						GPSNorthHold = GPSNorth;
						GPSEastHold = GPSEast;
						SumNavRCorr= SumNavPCorr = 0;
						_HoldingStation = true;
						_NavComputed = false;
					}		
					Navigate(GPSNorthHold, GPSEastHold);
				}
				else
				{
					SumNavRCorr= SumNavPCorr = 0;
					_HoldingStation = false;
				}
			#endif // OVERRIDE_HOVER_CONDITION

	#ifdef FAKE_GPS

	#define FAKE_NORTH_WIND 	0
	#define FAKE_EAST_WIND 		0

	if(  BlinkCount >= FakeGPSCount )
	{

		Heading = CompassHeading - ConvertDDegToMPi(MAGNETIC_VARIATION*10L);

		FakeGPSCount = BlinkCount + 50;
		CosH = int16cos(Heading);
		SinH = int16sin(Heading);
		GPSEast += ((int32)(-DesiredPitch) * SinH + 128)/256;
		GPSNorth += ((int32)(-DesiredPitch) * CosH + 128)/256;
	
		A = Make2Pi(Heading + HALFMILLIPI);
		CosH = int16cos(A);
		SinH = int16sin(A);
		GPSEast += ((int32)DesiredRoll * SinH + 128) / 256L;
		GPSEast += FAKE_EAST_WIND; // wind	
		GPSNorth += ((int32)DesiredRoll * CosH + 128) / 256L;
		GPSNorth += FAKE_NORTH_WIND; // wind	

		// $GPRMC
		GPSGroundSpeed = 99;
		GPSHeading = 99;

		// $GPGGA
		GPSFix = 2;
		GPSHDilute = 0;
		GPSNoOfSats = 99;
		GPSRelAltitude = 100; // 100M

		_GPSValid = true;
		_NavComputed = false;

		TxVal32((int32)ConvertMPiToDDeg(Heading), 1, ' ');
		if ( _CompassMisRead )
			TxChar('?');
		else
			TxChar(' ');

		TxString("\t ");
		TxVal32(Abs(ConvertGPSToM(GPSNorth)), 0, 0);
		if ( GPSNorth >=0 )
			TxChar('n');
		else
			TxChar('s');
		TxString("\t ");
		TxVal32(Abs(ConvertGPSToM(GPSEast)), 0, 0);
		if ( GPSEast >=0 )
			TxChar('e');
		else
			TxChar('w');

		TxString("\t -> r=");
		TxVal32(DesiredRoll, 0, ' ');
		TxChar('{');
		TxVal32(NavRCorr, 0, ',');
		TxVal32((SumNavRCorr * NavKi) / 256, 0, '}');

		TxString("\t p=");		
		TxVal32(DesiredPitch, 0, ' ');
		TxChar('{');
		TxVal32(NavPCorr, 0, ',');
		TxVal32((SumNavPCorr * NavKi) / 256, 0, '}');

		if ( _Hovering )
			TxString(" hov");
		if ( IK5 > _Neutral )
			TxString(" rth");
		if( _HoldingStation )
			TxString(" hold");
		TxNextLine();
	}

	#endif // FAKE_GPS


} // CheckAutonomous

void InitNavigation(void)
{
	NavRCorr = SumNavRCorr = NavPCorr = SumNavPCorr = 0;
	_NavComputed = false;
} // InitNavigation

