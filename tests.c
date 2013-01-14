// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "uavx.h"

#ifdef TESTING

void DoLEDs(void);
void ReceiverTest(void);
void PowerOutput(int8);
void BatteryTest(void);

void DoLEDs(void)
{
	if( F.Signal )
	{
		LEDRed_OFF;
		LEDGreen_ON;
	}
	else
	{
		LEDGreen_OFF;
		LEDRed_ON;
	}
} // DoLEDs

void ReceiverTest(void)
{
	static uint8 s;
	static uint16 v;

	TxString("\r\nRx: ");
	ShowRxSetup();
	TxString("\r\n");
	
	TxString("\tRAW Rx frame values - neutrals NOT applied\r\n");

	TxString("\tChannel order is: ");
	for ( s = 0; s < NoOfControls; s++)
		TxChar(RxChMnem[RMap[s]]);

	if ( F.Signal )
		TxString("\r\nSignal OK ");
	else
		TxString("\r\nSignal FAIL ");	
	TxVal32(mSClock() - mS[LastValidRx], 0, 0);
	TxString(" mS ago\r\n");
	
	// Be wary as new RC frames are being received as this
	// is being displayed so data may be from overlapping frames

	for ( s = 0; s < NoOfControls; s++ )
	{
		TxChar(s+'1');
		TxString(": ");
		TxChar(RxChMnem[RMap[s]]);
		TxString(":\t");
 
		TxVal32(( PPM[s].i16 * 8L + 5L ) / 10L, 3, 0);
		TxChar(HT);
		TxVal32(((int32)PPM[s].i16*100L + 625L ) / 1250L, 0, '%');
		if ( ( PPM[s].i16 < 0 ) || ( PPM[s].i16 > 1250 ) ) 
			TxString(" FAIL");

		TxNextLine();
	}

	// show pause time
	TxString("Gap:\t");
	v = ( PauseTime * 8L + 5 )/10L;
	TxVal32( v, 3, 0);		
	TxString("mS\r\n");
	TxString("Glitches:\t");
	TxVal32(RCGlitches,0,0);
	TxNextLine();

} // ReceiverTest

#endif // TESTING

void BatteryTest(void)
{
	static int32 v;

	TxString("\r\nBattery test\r\n");

	// Battery
	v = ((int24)ADC(ADCBattVoltsChan) * 278L )/1024L; // resistive divider 
	TxString("Batt:\t");
	TxVal32(v, 1, 'V');
	TxString(" Limit > ");
	v = ((int24)BatteryVoltsLimitADC * 278L )/1024L; // resistive divider ADCBattVoltsChan
	TxVal32(v, 1, 'V');
	TxNextLine();
	
} // BatteryTest

#ifdef SIMULATE

#define EMU_DT_MS	100L
#define EMU_DTR		(1000L/EMU_DT_MS)

int32 HRBaroAltitude = 0;

void DoEmulation(void) {
	static uint24 UpdatemS = 0;
	
	if (Armed && (mSClock() > UpdatemS)) {
		UpdatemS = mSClock() + EMU_DT_MS;

		BaroROC = (DesiredThrottle - CruiseThrottle + AltComp) * 2;
		HRBaroAltitude += BaroROC; 
		BaroAltitude = HRBaroAltitude / EMU_DTR;
		if ((HRBaroAltitude) < 10)
			BaroROC = HRBaroAltitude = BaroAltitude = 0;

		GPSAltitude = BaroAltitude;
		F.NewBaroValue = true;

		A[Roll].Angle = ((int32)A[Roll].Control * DEG_TO_ANGLE_UNITS * 45) / RC_NEUTRAL;
		A[Pitch].Angle = ((int32)A[Pitch].Control * DEG_TO_ANGLE_UNITS * 45) / RC_NEUTRAL;

		Rl = -A[Roll].Control; 
		Pl = -A[Pitch].Control; 
		Yl = -A[Yaw].Control;

		#ifdef NAV_WING
			Heading += (A[Roll].Control - A[Yaw].Control);// * DegreesToRadians(30) ) / (EMU_DTR * RC_NEUTRAL); // say 180/sec
		#else
			Heading -= (A[Yaw].Control);// * DegreesToRadians(180) ) / (EMU_DTR * RC_NEUTRAL);
		#endif

		Heading = Make2Pi(Heading);
	}

} // DoEmulation

int32 ConvertdMToGPS(int32 c) {
	return ( ((int32)c * (int32)10000)/((int32)1855) );
} // ConvertdMToGPS

void GPSEmulation(void) {
	
	#define FAKE_NORTH_WIND 0
	#define FAKE_EAST_WIND 0
	#define SIM_CRUISE_DMPS 70L
	#define SIM_MAX_DMPS 100L

	int32 NorthDiff, EastDiff, PitchDiff, RollDiff, PitchDiffp, RollDiffp;

	#ifdef NAV_WING
		PitchDiff = SIM_CRUISE_DMPS;
		RollDiff = ((int32)A[Roll].Control * (SIM_MAX_DMPS/4)) / RC_NEUTRAL; 
	#else
		PitchDiff = -((int32)A[Pitch].Control * SIM_MAX_DMPS) / RC_NEUTRAL; 
		RollDiff = ((int32)A[Roll].Control * SIM_MAX_DMPS) / RC_NEUTRAL; 
	#endif

	Rotate(&NorthDiff, &EastDiff, PitchDiff, RollDiff, -Heading);

	GPSVel = int32sqrt(Sqr(PitchDiff) + Sqr(RollDiff));
	F.ValidGPSVel = true;

	NorthDiff += FAKE_NORTH_WIND;
	EastDiff += FAKE_EAST_WIND;

	GPSLongitudeRaw += ConvertdMToGPS(EastDiff) / GPS_UPDATE_HZ;
	GPSLatitudeRaw += ConvertdMToGPS(NorthDiff) / GPS_UPDATE_HZ;

	GPSHeading = Heading;

	GPSPacketTag = GPGGAPacketTag;
	GPSFix = 3;
	GPSNoOfSats = 10;
	GPSHDilute = 50;
	F.GPSValid = true;

} // EmulateGPS

#endif // SIMULATE

