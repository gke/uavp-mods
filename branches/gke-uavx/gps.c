// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
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

// 	GPS routines

#include "uavx.h"

// Prototypes

void UpdateField(void);
int16 ConvertInt(uint8, uint8);
int32 ConvertLatLonM(uint8, uint8);
int32 ConvertUTime(uint8, uint8);
void ParseGPRMCSentence(void);
void ParseGPGGASentence(void);
void SetGPSOrigin(void);
void ParseGPSSentence(void);
void ResetGPSOrigin(void);
void InitGPS(void);
void UpdateGPS(void);

// Variables

#pragma udata gpsvars
uint8 GPSNoOfSats;
uint8 GPSFix;
int16 GPSHDilute;
#pragma udata

#pragma udata gpsvars2
int32 GPSMissionTime, GPSStartTime;
int32 GPSLatitude, GPSLongitude;
int32 GPSOriginLatitude, GPSOriginLongitude;
int16 GPSNorth, GPSEast, GPSNorthHold, GPSEastHold;
int16 GPSLongitudeCorrection;
int16 GPSAltitude, GPSRelAltitude, GPSOriginAltitude;
#pragma udata

// Global Variables

#pragma udata gpsvars3
int16 ValidGPSSentences;
uint8 nll, cc, lo, hi;
boolean EmptyField;
#pragma udata

int16 ConvertInt(uint8 lo, uint8 hi)
{
	static uint8 i;
	static int16 ival;

	ival = 0;
	if ( !EmptyField )
		for (i = lo; i <= hi ; i++ )
			ival = ival * 10 + NMEA.s[i] - '0';

	return (ival);
} // ConvertInt

int32 ConvertLatLonM(uint8 lo, uint8 hi)
{ 	// NMEA coordinates assumed as DDDMM.MMMM ie 4 decimal minute digits
	// Positions are stored at maximum transmitted NMEA resolution which is
	// approximately 0.1855 Metres per LSB at the Equator.
	static int32 dd, mm, dm;	
	static int32 ival;
	
	ival=0;
	if ( !EmptyField )
	{
	    dd = ConvertInt(lo, hi-7);
	    mm = ConvertInt(hi-6, hi-5);
		dm = ConvertInt(hi-3, hi);
	    ival = dd * 600000 + mm * 10000 + dm;
	}
	
	return(ival);
} // ConvertLatLonM

/*	
int32 ConvertUTime(uint8 lo, uint8 hi)
{
	int32 ival, hh;
	int16 mm, ss;
	
	ival=0;
	if ( !EmptyField )
		ival=(int32)(ConvertInt(lo, lo+1))*3600+
				(int32)(ConvertInt(lo+2, lo+3)*60)+
				(int32)(ConvertInt(lo+4, hi));
	      
	return(ival);
} // ConvertUTime
*/

void UpdateField(void)
{
	static uint8 ch;

	lo = cc;

	ch = NMEA.s[cc];
	while (( cc < nll ) && ( ch != ',' ) && ( ch != '*' ) ) 
		ch = NMEA.s[++cc];

	hi = cc - 1;
	cc++;
	EmptyField = hi < lo;
} // UpdateField

void ParseGPGGASentence(void)
{ 	// full position $GPGGA fix 

    UpdateField();
    
    UpdateField();   //UTime
	//GPSMissionTime=ConvertUTime(lo,hi);

	UpdateField();   //Lat
    GPSLatitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LatH
    if (NMEA.s[lo] == 'S')
      	GPSLatitude = -GPSLatitude;

    UpdateField();   //Lon
    // no latitude compensation on longitude - yet!    
    GPSLongitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LonH
	if (NMEA.s[lo] == 'W')
      	GPSLongitude = -GPSLongitude;
         
    UpdateField();   //Fix 
    GPSFix = (uint8)(ConvertInt(lo,hi));

    UpdateField();   //Sats
    GPSNoOfSats = (uint8)(ConvertInt(lo,hi));

    UpdateField();   // HDilute
	GPSHDilute = ConvertInt(lo, hi-3) * 100 + ConvertInt(hi-1, hi); 

    UpdateField();   // Alt
	GPSAltitude = ConvertInt(lo, hi-2) * 10 + ConvertInt(hi, hi); // Decimetres

    //UpdateField();   // AltUnit - assume Metres!

    //UpdateField();   // GHeight 
    //UpdateField();   // GHeightUnit 
 
    _GPSValid = (GPSFix >= GPS_MIN_FIX) && ( GPSNoOfSats >= GPS_MIN_SATELLITES );  

	if ( _GPSTestActive )
		TxString("$GPGGA ");
} // ParseGPGGASentence

void SetGPSOrigin(void)
{
	static int32 Temp;

	GPSStartTime = GPSMissionTime;
	GPSOriginLatitude = GPSLatitude;
	GPSOriginLongitude = GPSLongitude;
			
	Temp = GPSLatitude/60000L;
	Temp = Abs(Temp);
	Temp = ConvertDDegToMPi(Temp);
	GPSLongitudeCorrection = int16cos(Temp);

	GPSOriginAltitude = GPSAltitude;
} // SetGPSOrigin

void ParseGPSSentence(void)
{
	cc = 0;
	nll = NMEA.length;

	ParseGPGGASentence(); 

	if ( _GPSValid )
	{
	    if ( ValidGPSSentences <=  GPS_INITIAL_SENTENCES )
		{   // repetition to ensure GPGGA altitude is captured

			if ( _GPSTestActive )
			{
				TxVal32( GPS_INITIAL_SENTENCES - ValidGPSSentences, 0, 0);
				TxNextLine();
			}

			_GPSValid = false;

			if (GPSHDilute <= GPS_MIN_HDILUTE )
			{
				if ( ValidGPSSentences ==  GPS_INITIAL_SENTENCES )
				{
					SetGPSOrigin();
					DoBeep100mSWithOutput(2,0);
				}
				ValidGPSSentences++;
			}	
		}
		else
		{
			// all coordinates in 0.0001 Minutes or ~0.185M units relative to Origin
			GPSNorth = GPSLatitude - GPSOriginLatitude;
			GPSEast = GPSLongitude - GPSOriginLongitude;
			GPSEast = SRS32((int32)GPSEast * GPSLongitudeCorrection, 8); 

			GPSRelAltitude = GPSAltitude - GPSOriginAltitude;
			if ( ( State == InFlight ) && ( GPSRelAltitude > Stats[GPSAltitudeS].i16 )) 
				Stats[GPSAltitudeS].i16 = GPSRelAltitude;
		}
	}
	else
		if ( _GPSTestActive )
			TxString("invalid\r\n");
} // ParseGPSSentence

void ResetGPSOrigin(void)
{
	if ( ValidGPSSentences >  GPS_INITIAL_SENTENCES )
		SetGPSOrigin();	
	// otherwise continue with first acquisition
} // ResetGPSOrigin

void InitGPS(void)
{ 
	uint8 n;

	cc = 0;

	GPSMissionTime = GPSRelAltitude = GPSFix = GPSNoOfSats = GPSHDilute = 0;
	GPSEast = GPSNorth = 0;

	ValidGPSSentences = 0;

	_GPSValid = false; 
	GPSSentenceReceived = false;
  	GPSRxState = WaitGPSSentinel; 
  	
} // InitGPS

void UpdateGPS(void)
{
	if ( GPSSentenceReceived )
	{
		LEDBlue_ON;
		LEDRed_OFF;
		GPSSentenceReceived = false;  
		ParseGPSSentence(); // 7.5mS 18f2520 @ 16MHz
		if ( _GPSValid )
		{
			_NavComputed = false;
			mS[GPSTimeout] = mS[Clock] + GPS_TIMEOUT_MS;
		}
	}
	else
		if( mS[Clock] > mS[GPSTimeout] )
			_GPSValid = false;

	LEDBlue_OFF;
	if ( _GPSValid )
		LEDRed_OFF;
	else
		LEDRed_ON;	
} // UpdateGPS


