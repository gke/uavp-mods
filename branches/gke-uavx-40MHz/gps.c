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

// 	GPS routines

#include "uavx.h"

void UpdateField(void);
int32 ConvertGPSToM(int32);
int32 ConvertGPSTodM(int32);
int32 ConvertMToGPS(int32);
int24 ConvertInt(uint8, uint8);
int32 ConvertLatLonM(uint8, uint8);
int32 ConvertUTime(uint8, uint8);
void ParseGPRMCSentence(void);
void ParseGPGGASentence(void);
void SetGPSOrigin(void);
void ParseGPSSentence(void);
void InitGPS(void);
void UpdateGPS(void);

const rom uint8 NMEATags[MAX_NMEA_SENTENCES][5]= {
    // NMEA
    {'G','P','G','G','A'}, // full positioning fix
    {'G','P','R','M','C'}, // main current position and heading
};

#pragma udata gpsbuff
NMEAStruct NMEA;
#pragma udata

#pragma udata gpsvars
uint8 	GPSPacketTag;
int32 	GPSMissionTime, GPSStartTime;
int24 	GPSAltitude, GPSAltitude, GPSOriginAltitude;
int32	GPSLatitudeRaw, GPSLongitudeRaw;
int32 	OriginLatitudeRaw, OriginLongitudeRaw;
int16 	GPSLongitudeCorrection;
int16	GPSHeading;
int16 	GPSVel;
uint8 	GPSNoOfSats;
uint8 	GPSFix;
int16 	GPSHDilute;
#pragma udata

#pragma udata gpsvars1
uint8 nll, cc, lo, hi;
boolean EmptyField;
int16 ValidGPSSentences;
#pragma udata

int32 ConvertGPSToM(int32 c)
{	// approximately 1.8553257183 cm per NMEA LSD with 5 decimal minutes at the Equator
	// 32bit is 0.933 cm less than a factor of 2
	// conversion max is 21Km
	return ( ((int32)c * (int32)1855)/((int32)100000) );
} // ConvertGPSToM

int32 ConvertGPSTodM(int32 c) {	
	// conversion max is 21Km
	return ( ((int32)c * (int32)1855)/((int32)10000) );
} // ConvertGPSTodM

int32 ConvertMToGPS(int32 c) {
	return ( ((int32)c * (int32)100000)/((int32)1855) );
} // ConvertMToGPS

int24 ConvertInt(uint8 lo, uint8 hi) {
	static uint8 i;
	static int24 r;

	r = 0;
	if ( !EmptyField )
		for (i = lo; i <= hi ; i++ )
			r = r * 10 + NMEA.s[i] - '0';

	return (r);
} // ConvertInt

int32 ConvertLatLonM(uint8 lo, uint8 hi) {
 	// NMEA coordinates normally assumed as DDDMM.MMMMM ie 5 decimal minute digits
	// but code can deal with 4 and 5 decimal minutes 
	// Positions are stored at 5 decimal minute NMEA resolution which is
	// approximately 1.855 cm per LSD at the Equator.
	static int32 dd, mm, dm, r;
	static uint8 dp;	
	
	r = 0;
	if ( !EmptyField )
	{
		dp = lo + 4; // could do this in initialisation for Lat and Lon?
		while ( NMEA.s[dp] != '.') dp++;

	    dd = ConvertInt(lo, dp - 3);
	    mm = ConvertInt(dp - 2 , dp - 1);
		if ( ( hi - dp ) > (uint8)4 )
			dm = ConvertInt(dp + 1, dp + 5);
		else
			dm = ConvertInt(dp + 1, dp + 4) * 10L;
			
	    r = dd * 6000000 + mm * 100000 + dm;
	}
	
	return(r);
} // ConvertLatLonM

int32 ConvertUTime(uint8 lo, uint8 hi) {
	static int32 ival;
	
	ival=0;
	if ( !EmptyField )
		ival=(int32)(ConvertInt(lo, lo+1))*3600+
				(int32)(ConvertInt(lo+2, lo+3)*60)+
				(int32)(ConvertInt(lo+4, hi));
	      
	return(ival);
} // ConvertUTime

void UpdateField(void)
{
	static uint8 ch;

	lo = cc;

	ch = NMEA.s[cc];
	while (( ch != ',' ) && ( ch != '*' ) && ( cc < nll )) 
		ch = NMEA.s[++cc];

	hi = cc - 1;
	cc++;
	EmptyField = hi < lo;
} // UpdateField

void ParseGPGGASentence(void) { // full position $GPGGA fix
	// 0.29mS @ 40MHz 

    UpdateField();
    
    UpdateField();   //UTime
	GPSMissionTime = ConvertUTime(lo,hi);

	UpdateField();   	//Lat
    GPSLatitudeRaw = ConvertLatLonM(lo, hi);
    UpdateField();   //LatH
    if (NMEA.s[lo] == 'S') GPSLatitudeRaw = -GPSLatitudeRaw;

    UpdateField();   	//Lon   
    GPSLongitudeRaw = ConvertLatLonM(lo, hi);
    UpdateField();   	//LonH
	if (NMEA.s[lo] == 'W') GPSLongitudeRaw = -GPSLongitudeRaw;
        
    UpdateField();   	//Fix 
    GPSFix = (uint8)(ConvertInt(lo,hi));

    UpdateField();   	//Sats
    GPSNoOfSats = (uint8)(ConvertInt(lo,hi));

    UpdateField();   	// HDilute
	GPSHDilute = ConvertInt(lo, hi-3) * 100 + ConvertInt(hi-1, hi); 

    UpdateField();   	// Alt

	GPSAltitude = ConvertInt(lo, hi-2) * 10L + ConvertInt(hi, hi) * 10; // Decimetres

    //UpdateField();   // AltUnit - assume Metres!

    //UpdateField();   // GHeight 
    //UpdateField();   // GHeightUnit 

   	F.GPSValid = (GPSFix >= (uint8)GPS_MIN_FIX) && ( GPSNoOfSats >= (uint8)GPS_MIN_SATELLITES );

	if ( State == InFlight )
	{
		StatsMinMax(GPSHDilute, MinHDiluteS, MaxHDiluteS);
		StatsMinMax(GPSNoOfSats, GPSMinSatsS, GPSMaxSatsS);

		F.GPSFailure = GPSHDilute > 150; 
	}
} // ParseGPGGASentence

void ParseGPRMCSentence() { 	
	// main current position and heading
	// 0.1mS @ 40MHz

	static i32u Temp32;

    UpdateField();

    UpdateField();   //UTime

    UpdateField();
    if ( NMEA.s[lo] == 'A' ) {
	
		F.GPSValid = true;

        UpdateField();   //Lat
        GPSLatitudeRaw = ConvertLatLonM(lo,hi);
        UpdateField();   //LatH
        if (NMEA.s[lo] == 'S')
        	GPSLatitudeRaw = -GPSLatitudeRaw;

        UpdateField();   //Lon
        GPSLongitudeRaw = ConvertLatLonM(lo,hi);

        UpdateField();   //LonH
        if ( NMEA.s[lo] == 'W' )
        	GPSLongitudeRaw = -GPSLongitudeRaw;

        UpdateField();   // Groundspeed (Knots)
		GPSVel = SRS32(((int32)ConvertInt(lo, hi-3) * 100L + ConvertInt(hi-1, hi)) * 13L, 8);//  5.144444 dMPS/Kt

        UpdateField();   // True course made good (Degrees)
		GPSHeading = SRS32(((int32)ConvertInt(lo, hi-3) * 100L + ConvertInt(hi-1, hi)) * 45L, 8); // MilliRadians 3142/18000; 

      	/*
        UpdateField();   //UDate

        UpdateField();   // Magnetic Deviation (Degrees)
        GPSMagDeviation = ConvertReal(lo, hi) * MILLIRAD;

        UpdateField();   // East/West
 
		if ( NMEA.s[lo] == 'W')
            GPSMagHeading = GPSHeading - GPSMagDeviation; // need to check sign????
        else
            GPSMagHeading = GPSHeading +  GPSMagDeviation;
		*/
        F.ValidGPSVel = true;
    } else
        F.ValidGPSVel = false;

} // ParseGPRMCSentence

void SetGPSOrigin(void) {
	static int32 Temp;

	if ( ( ValidGPSSentences == GPS_ORIGIN_SENTENCES ) && F.GPSValid )
	{
		mS[LastGPS] = mSClock();

		#ifdef SIMULATE
			GPSLatitudeRaw = (-3118333L * 60L);
			GPSLongitudeRaw = (13680833L * 60L);
		#endif

		GPSStartTime = GPSMissionTime;
		OriginLatitudeRaw = GPSLatitudeRaw;
		OriginLongitudeRaw = GPSLongitudeRaw;
		
		Temp = GPSLatitudeRaw/600000L; // to degrees * 10
		Temp = Abs(Temp);
		Temp = ConvertDDegToMPi(Temp);
		GPSLongitudeCorrection = int16cos(Temp);
	
		GPSOriginAltitude = GPSAltitude;

		GPSVel = 0;

		Write16EE(NAV_ORIGIN_ALT, (int16)(GPSAltitude/100));
		Write32EE(NAV_ORIGIN_LAT, GPSLatitudeRaw);
		Write32EE(NAV_ORIGIN_LON, GPSLongitudeRaw);

		if ( !F.NavValid )
		{
			DoBeep100mSWithOutput(2,0);
			Stats[NavValidS] = true;
			F.NavValid = true;
		}
		F.AcquireNewPosition = true;		
	}
} // SetGPSOrigin


void ParseGPSSentence(void) {
	static int24 NowmS;
	static int16 GPSdT;
	static int32 EastDiff, NorthDiff;

	cc = 0;
	nll = NMEA.length;

#ifdef SIMULATE

	GPSEmulation();

#else
	switch ( GPSPacketTag ) {
    case GPGGAPacketTag:
		ParseGPGGASentence();
		break;
	case GPRMCPacketTag:
		ParseGPRMCSentence();
		break;
	} 
#endif

	if ( F.GPSValid )
	{
		// all coordinates in 0.00001 Minutes or ~1.8553cm relative to Origin
		// There is a lot of jitter in position - could use Kalman Estimator?

		NowmS = mSClock();
		GPSdT = NowmS - mS[LastGPS];
		mS[LastGPS] = NowmS;

	    if ( ValidGPSSentences <  GPS_ORIGIN_SENTENCES )
		{   // repetition to ensure GPGGA altitude is captured
			F.GPSValid = false;

			if ( GPSHDilute <= GPS_MIN_HDILUTE )
				ValidGPSSentences++;
			else
				ValidGPSSentences = 0;
		}

		if (F.NavValid)
		{
			Nav.C[NorthC].Pos = ConvertGPSTodM(GPSLatitudeRaw - OriginLatitudeRaw);
			Nav.C[EastC].Pos = ConvertGPSTodM(GPSLongitudeRaw - OriginLongitudeRaw);
						//	* GPSLongitudeCorrection;

			if (!F.ValidGPSVel) {
				NorthDiff = Nav.C[NorthC].Pos - Nav.C[NorthC].PosP;
				EastDiff = Nav.C[EastC].Pos - Nav.C[EastC].PosP;

				GPSVel = int32sqrt(Sqr(NorthDiff) + Sqr(EastDiff)) * GPS_UPDATE_HZ;

				Nav.C[NorthC].PosP = Nav.C[NorthC].Pos;
				Nav.C[EastC].PosP = Nav.C[EastC].Pos;
			}
		}

		if ( State == InFlight )
		{
			StatsMax(GPSAltitude, GPSAltitudeS);
			StatsMax(GPSVel, GPSVelS);
		}
	}
	else
		if ( State == InFlight ) 
			Stats[GPSInvalidS]++;

} // ParseGPSSentence

void UpdateGPS(void) {

	#ifdef SIMULATE
	static uint24 NextGPS = 0;

	if ( mSClock( )> NextGPS ) {
		NextGPS += 1000/ GPS_UPDATE_HZ ;
		F.PacketReceived = true;
	}
	#endif

	if ( F.PacketReceived )
	{
		LEDBlue_ON;
		LEDRed_OFF;
		F.PacketReceived = false;  
		ParseGPSSentence(); // 3mS 18f2620 @ 40MHz
		if ( F.GPSValid )
		{
			LEDRed_OFF;
			F.NavComputed = false;
			mS[GPSTimeout] = mSClock() + GPS_TIMEOUT_MS;
		}
		else
			LEDRed_ON;
		LEDBlue_OFF;
	}
	else
	{
		if( mSClock() > mS[GPSTimeout] )
		{
			F.GPSValid = false;
			LEDRed_ON;
		}
		LEDBlue_OFF;
	}

} // UpdateGPS

void InitGPS(void) {
	cc = 0;

	GPSLongitudeCorrection = 256; // =>1.0
	GPSMissionTime = GPSFix = GPSNoOfSats = GPSHDilute = 0;
	GPSAltitude = GPSHeading = GPSVel = 0; 
	OriginLatitudeRaw = GPSLatitudeRaw = 0;
	OriginLongitudeRaw = GPSLongitudeRaw = 0;

	Write32EE(NAV_ORIGIN_LAT, 0);
	Write32EE(NAV_ORIGIN_LON, 0);
	Write16EE(NAV_ORIGIN_ALT, 0);

	ValidGPSSentences = 0;

	F.NavValid = F.GPSValid = F.PacketReceived = false;
  	RxState = WaitSentinel; 

} // InitGPS
