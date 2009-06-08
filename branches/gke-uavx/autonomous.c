// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.
//
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

// Autonomous flight routines

#include "uavx.h"

// Prototypes

void Descend(void);
void Navigate(int16, int16);
void DoNavigation(void);

// Defines

#define NAV_CLOSING_RADIUS ( (int32)CLOSING_RADIUS * 5L )
#define SQR_NAV_CLOSING_RADIUS ( NAV_CLOSING_RADIUS * NAV_CLOSING_RADIUS )
//#define NavKp (((int32)MAX_ANGLE * 256L ) / NAV_CLOSING_RADIUS )
#define NavKi 1

// Variables
extern int16 ValidGPSSentences;
extern boolean GPSSentenceReceived;

int16 NavRCorr, SumNavRCorr, NavPCorr, SumNavPCorr, NavYCorr, SumNavYCorr;
enum NavStates { PIC, HoldingStation, ReturningHome, Navigating, Terminating };
uint8 NavState;

void GPSAltitudeHold(int16 DesiredAltitude)
{
	#ifdef ENABLE_GPS_ALT_HOLD

	AE = Limit(DesiredAltitude - GPSRelAltitude, -50, 50);
	AltSum += AE;
	AltSum = Limit(AltSum, -10, 10);	
	AutonomousThrottle = HoverThrottle + SRS16(AE*P[NavAltKp] + AltSum*P[NavAltKi], 4);

	DesiredThrottle = HoverThrottle + Limit(AutonomousThrottle, -20, 50);

	DesiredThrottle = Limit(AutonomousThrottle, 0, _Maximum);

	#endif // ENABLE_GPS_ALT_HOLD
} // GPSAltitudeHold

void Descend(void)
{


	
} // Descend

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// cos/sin/arctan lookup tables are used for speed.
	// BEWARE magic numbers for integer artihmetic

	static int32 Temp;
	static int16 NavKp, GPSGain;
	static int16 Range, EastDiff, NorthDiff, WayHeading, RelHeading, ScaledNavKp;

	if ( _NavComputed ) // use previous corrections
	{
		DesiredRoll = Limit(DesiredRoll + NavRCorr, -_Neutral, _Neutral);
		DesiredPitch = Limit(DesiredPitch + NavPCorr, -_Neutral, _Neutral);
		DesiredYaw = Limit(DesiredYaw + NavYCorr, -_Neutral, _Neutral);
	}
	else
	{
		EastDiff = GPSEastWay - GPSEast;
		NorthDiff = GPSNorthWay - GPSNorth;

		#ifdef FAKE_GPS
		Heading = GPSHeading;
		#endif // FAKE_GPS

		if ( (Abs(EastDiff) >= 1 ) || (Abs(NorthDiff) >=1 ))
		{ 
			Range = Max(Abs(NorthDiff), Abs(EastDiff)); 
			if ( Range < NavClosingRadius )
				Range = int16sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff); 
			else
				Range = NavClosingRadius;

			#ifdef GPS_IK7_GAIN
			GPSGain = Limit(IK7, 0, 256); // compensate for EPA offset of up to 20
			ScaledNavKp = ( GPSGain * P[NavKp] ) / NavClosingRadius; // /_Maximum) * 256L
			#else
			ScaledNavKp = (((int32)P[NavKp] * 256L ) / NavClosingRadius );
			#endif // GPS_IK7_GAIN
	
			Temp = ((int32)Range * ScaledNavKp )>>8; // always +ve so can use >>
			WayHeading = int16atan2(EastDiff, NorthDiff);

			RelHeading = Make2Pi(WayHeading - Heading);
			NavRCorr = SRS32((int32)int16sin(RelHeading) * Temp, 8);			
			NavPCorr = SRS32(-(int32)int16cos(RelHeading) * Temp, 8);

			#ifdef TURN_TO_HOME
			if ( Range >= NavClosingRadius )
			{
				RelHeading = MakePi(WayHeading - Heading); // make +/- MilliPi
				NavYCorr = -(RelHeading * P[NavYawLimit]) / HALFMILLIPI;
				NavYCorr = Limit(NavYCorr, -P[NavYawLimit], P[NavYawLimit]); // gently!
			}
			else
			#endif // TURN_TO_HOME
				NavYCorr = 0;
		
			SumNavRCorr = Limit (SumNavRCorr + Range, -NavIntLimit256, NavIntLimit256);
			DesiredRoll += NavRCorr + (SumNavRCorr * P[NavKi]) / 256L;
			DesiredRoll = Limit(DesiredRoll , -_Neutral, _Neutral);
	
			SumNavPCorr = Limit (SumNavPCorr + Range, -NavIntLimit256, NavIntLimit256);
			DesiredPitch += NavPCorr + (SumNavPCorr * P[NavKi]) / 256L;
			DesiredPitch = Limit(DesiredPitch , -_Neutral, _Neutral);

			DesiredYaw += NavYCorr;
		}
		else
			NavRCorr = NavPCorr = NavYCorr = 0;

		_NavComputed = true;
	}
} // Navigate

void DoNavigation(void)
{
	#ifdef FAKE_GPS
	static int16 CosH, SinH, NorthD, EastD, A;

	_GPSValid = true;

	#endif // FAKE_GPS

	UpdateGPS();
	
	if ( _GPSValid && ( IK7 > 20 ))
		if ( _CompassValid )
			switch ( NavState ) {
			case PIC:
			case HoldingStation:
				if ( (Abs(DesiredRoll) > MAX_CONTROL_CHANGE) || (Abs(DesiredPitch) > MAX_CONTROL_CHANGE) )
				{
					NavState = PIC;
					GPSNorthHold = GPSNorth;
					GPSEastHold = GPSEast;
					SumNavRCorr = SumNavPCorr = SumNavYCorr = 0;
					_NavComputed = false;
				}
				else
				{	
					NavState = HoldingStation;
					Navigate(GPSNorthHold, GPSEastHold);
				}
				
				if ( IK5 > _Neutral )
				{
					AutonomousThrottle = DesiredThrottle; 
					AltSum = 0; 
					NavState = ReturningHome;
				}
				break;
			case ReturningHome:
				GPSAltitudeHold(300);
				Navigate(0, 0);
				if ( IK5 <= _Neutral )
				{
					GPSNorthHold = GPSNorth;
					GPSEastHold = GPSEast;
					NavState = PIC;
				} 
				break;
			case Navigating:
				// not implemented yet
				break;
			} // switch NavState
		else
		{
			DesiredRoll = DesiredPitch = DesiredYaw = 0;
			GPSAltitudeHold(-50);
		}

	#ifndef DEBUG_SENSORS

	#ifdef FAKE_GPS

	#define FAKE_NORTH_WIND 	0
	#define FAKE_EAST_WIND 		0

	if(  Cycles >= FakeGPSCount )
	{

		// $GPRMC
		GPSGroundSpeed = (Abs(DesiredPitch)+Abs(DesiredRoll))*10; // decimetres/sec
		GPSHeading = Make2Pi(GPSHeading - DesiredYaw*10);
		_CompassValid = true;
		Heading = GPSHeading;

		// $GPGGA
		GPSFix = 2;
		GPSHDilute = 0;
		GPSNoOfSats = 99;
		GPSRelAltitude = 100; // 100M

		FakeGPSCount = BlinkCount + 50;
		CosH = int16cos(Heading);
		SinH = int16sin(Heading);
		GPSEast += ((int32)(-DesiredPitch) * SinH)/256;
		GPSNorth += ((int32)(-DesiredPitch) * CosH)/256;
	
		A = Make2Pi(Heading + HALFMILLIPI);
		CosH = int16cos(A);
		SinH = int16sin(A);
		GPSEast += ((int32)DesiredRoll * SinH) / 256L;
		GPSEast += FAKE_EAST_WIND; // wind	
		GPSNorth += ((int32)DesiredRoll * CosH) / 256L;
		GPSNorth += FAKE_NORTH_WIND; // wind	

		_NavComputed = false;

		TxVal32((int32)ConvertMPiToDDeg(Heading), 1, ' ');
		if ( _CompassMissRead )
			TxChar('?');
		else
			TxChar(' ');

		TxChar(' ');
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

		TxString(" -> \tr=");
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
		if ( NavState == ReturningHome )
			TxString(" rth");
		else
			if( NavState == HoldingStation )
				TxString(" hold");
			else
				TxString(" PIC");
		TxNextLine();
	}

	#endif // FAKE_GPS

	#endif // DEBUG_SENSORS

} // DoNavigation

void InitNavigation(void)
{
	GPSNorthHold = GPSEastHold = 0;

	NavRCorr = SumNavRCorr = NavPCorr = SumNavPCorr = NavYCorr = SumNavYCorr = 0;
	NavState = PIC;
	_NavComputed = false;
} // InitNavigation

