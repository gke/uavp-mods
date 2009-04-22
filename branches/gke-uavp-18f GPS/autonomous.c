// =======================================================================
// =                                 UAVX                                =
// =                         Quadrocopter Control                        =
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

#include "c-ufo.h"
#include "bits.h"

// Prototypes
void Descend(void);
void ParseGPSSentence(void);
void ResetTimeOuts(void);
void Navigate(int16, int16);

// Defines



// Variables
extern int8 ValidGPSSentences;
extern boolean GPSSentenceReceived;

int8  SumGPSRoll, SumGPSPitch;

void Descend(void)
{	
	if( (BlinkCount & 0x000f) == 0 )
		DesiredThrottle = Limit(DesiredThrottle--, 0, _Maximum); // safest	
} // Descend

#define GPSFilter SoftFilter
#define DESIRED_ANGLE (((int32)MAX_ANGLE * 256L ) / ( (int32)CLOSING_RADIUS * 5L))

void Navigate(int16 GPSNorthWay, int16 GPSEastWay)
{	// _GPSValid must be true immediately prior to entry	
	// This routine does not point the quadrocopter at the destination
	// waypoint. It simply rolls/pitches towards the destination
	// To avoid cos/sin/arctan calls a simple look up process is used.

	int16 Angle;
	int32 Temp;
	int16 RangeApprox;
	int16 EastDiff, NorthDiff;
	int16 RollCorrection, PitchCorrection;

	EastDiff = GPSEastWay - GPSEast;
	NorthDiff = GPSNorthWay - GPSNorth;

	if ( (Abs(EastDiff) !=0 ) || (Abs(NorthDiff) != 0 ))
	{ 
		RangeApprox = Max(Abs(NorthDiff), Abs(EastDiff)); 
	
		if ( RangeApprox < ( CLOSING_RADIUS * 5 ) )
			RangeApprox = int16sqrt( NorthDiff*NorthDiff + EastDiff*EastDiff); // 252uS @ 16MHz
		else
			RangeApprox = Limit(RangeApprox, 0, CLOSING_RADIUS * 5); // 5 ~M->GPS
		
		Angle = Make2Pi(int16atan2(EastDiff, NorthDiff) - CompassHeading);
	
		Temp = ((int32)RangeApprox * DESIRED_ANGLE)/256;

		RollCorrection = ((int32)int16sin(Angle) * Temp)/256;
		SumGPSRoll = Limit(SumGPSRoll + RollCorrection, -GPSIntLimit, GPSIntLimit);
		DesiredRoll = Limit(DesiredRoll + RollCorrection + SumGPSRoll, -_Maximum, _Maximum);
	
		PitchCorrection = (-(int32)int16cos(Angle) * Temp)/256;
		SumGPSPitch = Limit(SumGPSPitch + PitchCorrection, -GPSIntLimit, GPSIntLimit);
		DesiredPitch = Limit(DesiredPitch + PitchCorrection + SumGPSPitch, -_Maximum, _Maximum);
	}
} // Navigate

void CheckAutonomous(void)
{
	#ifdef FAKE_GPS
	int16 CosH, SinH, NorthD, EastD, A;

	_GPSValid = true;

	#endif // FAKE_GPS

	DesiredThrottle = IGas;
	DesiredRoll = IRoll;
	DesiredPitch = IPitch;
	DesiredYaw = IYaw;

	#ifdef ENABLE_AUTONOMOUS

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
				if (_Hovering )
				{
					if ( (Abs(DesiredRoll) > 5) || (Abs(DesiredPitch) > 5) || !_HoldingStation )
					{
						// acquire hold coordinates
						GPSNorthHold = GPSNorth;
						GPSEastHold = GPSEast;
						_HoldingStation = true;
					}		
					Navigate(GPSNorthHold, GPSEastHold);
				}
				else
					_HoldingStation = false;

	#endif // ENABLE_AUTONOMOUS

	#ifdef FAKE_GPS

	if(  BlinkCount >= FakeGPSCount )
	{
		FakeGPSCount = BlinkCount + 50;
		CosH = int16cos(CompassHeading);
		SinH = int16sin(CompassHeading);
		GPSEast += ((int32)(-DesiredPitch) * SinH + 128)/256;
		GPSNorth += ((int32)(-DesiredPitch) * CosH + 128)/256;
	
		A = Make2Pi(CompassHeading + HALFMILLIPI);
		CosH = int16cos(A);
		SinH = int16sin(A);
		GPSEast += ((int32)DesiredRoll * SinH + 128)/256L;
		GPSNorth += ((int32)DesiredRoll * CosH + 128)/256L;
	
		GPSAltitude = 1000; // 100M
	
		GPSFix = 2;
		GPSHDilute = 0.0;
		GPSNoOfSats = 99;

		TxVal32((int32)((int32)CompassHeading*180L)/(int32)MILLIPI, 0, ' ');
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
		TxString("\t p=");		
		TxVal32(DesiredPitch, 0, ' ');

		if ( _Hovering )
			TxString("hov ");
		if ( IK5 > _Neutral )
			TxString("rth ");
		if( _HoldingStation )
			TxString("hold");
		TxNextLine();

	}

	#endif // FAKE_GPS


} // CheckAutonomous

