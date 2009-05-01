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

// 	GPS routines

#include "uavx.h"

// Prototypes

void UpdateField(void);
int16 ConvertInt(uint8, uint8);
int32 ConvertLatLonM(uint8, uint8);
int32 ConvertUTime(uint8, uint8);
void ParseGPRMCSentence(void);
void ParseGPGGASentence(void);
void ParseGPSSentence(void);
void PollGPS(uint8);
void InitGPS(void);
void UpdateGPS(void);

// Defines


// Variables

#pragma udata gpsvars
int16 GPSGroundSpeed, GPSHeading;
boolean GPSSentenceReceived;
uint8 GPSSentenceType;
uint8 GPSMode;

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

#pragma udata gpsbuff
// Max NMEA sentence length is 80 char
#define GPSRXBUFFLENGTH 80
struct {
	uint8 length;
	uint8 s[GPSRXBUFFLENGTH];
	} NMEA[2];
#pragma udata

// Global Variables
#pragma udata gpsbuff2
uint8 NHead, NTail, NEntries;
#pragma udata

#pragma udata gpsvars3
enum WaitGPSStates {WaitGPSSentinel, WaitGPSTag, WaitGPSBody, WaitGPSCheckSum};
int8 ValidGPSSentences;
uint8 GPSRxState;
uint8 ll, cc, tt, lo, hi;
boolean EmptyField;
uint8 GPSTxCheckSum, GPSRxCheckSum, GPSCheckSumChar;
#pragma udata

#pragma udata gpsvars4
#define MAXTAGINDEX 4
enum GPSSentences {GPGGA, GPRMC};
#define FirstGPSType GPGGA
#define LastGPSType GPRMC
// must be in sorted alpha order
const uint8 GPSTag[LastGPSType+1][6] = {{"GPGGA"},{"GPRMC"}};
#pragma udata

#ifndef FAKE_GPS

int16 ConvertInt(uint8 lo, uint8 hi)
{
	uint8 i;
	int16 ival;

	ival = 0;
	if (!EmptyField)
		for (i = lo; i <= hi ; i++)
		{
			ival *= 10;
			ival += (NMEA[NHead].s[i] - '0');
		}

	return (ival);
} // ConvertInt

int32 ConvertLatLonM(uint8 lo, uint8 hi)
{ 	// positions are stored at maximum transmitted GPS resolution which is
	// approximately 0.18553257183 Metres per LSB (without Longitude compensation)

	int32 dd, mm, ss;	
	int32 ival;
	
	ival=0;
	if ( !EmptyField )
	{
	    dd = ConvertInt(lo, hi-7);
	    mm = ConvertInt(hi-6, hi-5);
		ss = ConvertInt(hi-3, hi);
	    ival = dd * 600000 + mm * 10000 + ss;
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

void UpdateField()
{
	lo = cc;
	hi = lo - 1;
	if ((NMEA[NHead].s[cc] != ',')&&(NMEA[NHead].s[cc]!='*'))
    {
		while ((cc<NMEA[NHead].length)&&(NMEA[NHead].s[cc]!=',')&&(NMEA[NHead].s[cc]!='*'))
		cc++;
		hi = cc - 1;
	}
	cc++;
	EmptyField = hi < lo;
} // UpdateField

void ParseGPRMCSentence()
{ 	// $GPRMC Recommended minimum specific GNSS data 

    UpdateField();
    
    UpdateField();   //UTime
	//GPSMissionTime=ConvertUTime(lo,hi);

	UpdateField();	// Status
	_GPSValid = (NMEA[NHead].s[lo] == 'A');    

	UpdateField();   //Lat
    GPSLatitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LatH
    if (NMEA[NHead].s[lo] == 'S')
      	GPSLatitude = -GPSLatitude;

    UpdateField();   //Lon
    // no latitude compensation on longitude - yet!   
    GPSLongitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LonH
	if (NMEA[NHead].s[lo] == 'W')
      	GPSLongitude = -GPSLongitude;
         
    UpdateField();   //Speed over Ground in Knots
	GPSGroundSpeed = ConvertInt(lo, hi-2) * 10 + ConvertInt(hi, hi);
	//GPSGroundSpeed = ((int32)GPSGroundSpeed * 515) / 1000L; // Decimetres/Sec
	GPSGroundSpeed = ((int32)GPSGroundSpeed * 527L)>>10;

    UpdateField();   //Course over ground
	// drop one decimal place
    GPSHeading = ConvertInt(lo, hi-3) * 10 + ConvertInt(hi-1, hi-1);
	//GPSHeading = ((int32)GPSHeading * MILLIPI)/1800L;
	GPSHeading = ConvertDDegToMPi(GPSHeading);

    UpdateField();   // Date

	UpdateField();
	/* Not produced by most GPS units
	GPSMagVariation = ConvertInt(lo, hi-2) * 10 + ConvertInt(hi, hi);
	//GPSMagVariation = ((int32)GPSMagVariation * MILLIPI)/1800L;
	GPSMagVariation = ConvertDDegToMPi(GPSMagVariation);
	*/
    UpdateField();   // Mag Var Units
	/*
	if (NMEA[NHead].s[lo] == 'W')
		GPSMagVariation = -GPSMagVariation;
	*/
	UpdateField();   // Mode (A,D,E)
	GPSMode = NMEA[NHead].s[lo]; 
TxString("RMC\r\n");  
} // ParseGPRMCSentence

void ParseGPGGASentence()
{ 	// full position $GPGGA fix 

    UpdateField();
    
    UpdateField();   //UTime
	//GPSMissionTime=ConvertUTime(lo,hi);

	UpdateField();   //Lat
    GPSLatitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LatH
    if (NMEA[NHead].s[lo] == 'S')
      	GPSLatitude = -GPSLatitude;

    UpdateField();   //Lon
    // no latitude compensation on longitude - yet!    
    GPSLongitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LonH
	if (NMEA[NHead].s[lo] == 'W')
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
 
    _GPSValid = (GPSFix >= MIN_FIX) && ( GPSNoOfSats >= MIN_SATELLITES );  
TxString("GGA\r\n"); 
} // ParseGPGGASentence

#endif //  !FAKE_GPS

void ParseGPSSentence()
{
	int32 Temp;

	cc = 0;

	#ifndef  FAKE_GPS

	switch (GPSSentenceType) {
	case GPGGA: ParseGPGGASentence(); break;
	case GPRMC: ParseGPRMCSentence(); break;
	}

	NHead = ( NHead + 1 ) & 1;
	NEntries--;

	if ( _GPSValid )
	{
	    if ( ValidGPSSentences < INITIAL_GPS_SENTENCES )
		{
			_GPSValid = false;
			ValidGPSSentences++;
	
            GPSStartTime = GPSMissionTime;
	      	GPSOriginLatitude = GPSLatitude;
	      	GPSOriginLongitude = GPSLongitude;
	
			Temp = GPSLatitude/60000L;
			Temp = Abs(Temp);
			Temp = ConvertDDegToMPi(Temp);
			GPSLongitudeCorrection = int16cos(Temp);

	      	if ( GPSSentenceType = GPGGA )
				GPSOriginAltitude=GPSAltitude;
		}

		// all cordinates in 0.0001 Minutes relative to Origin
		GPSNorth = GPSLatitude - GPSOriginLatitude;
		GPSEast = GPSLongitude - GPSOriginLongitude;
		GPSEast = SRS32((int32)GPSEast * GPSLongitudeCorrection, 8); 

		if ( GPSSentenceType = GPGGA )
			GPSRelAltitude = GPSAltitude - GPSOriginAltitude;
	}

	#endif //  !FAKE_GPS
} // ParseGPSSentence

void PollGPS(uint8 ch)
{
	switch (GPSRxState) {
	case WaitGPSBody: 
		RxCheckSum ^= ch;
		NMEA[NTail].s[ll++] = ch;
		if ( ch == '*' )	      
		{
			GPSCheckSumChar = GPSTxCheckSum = 0;
			GPSRxState = WaitGPSCheckSum;
		}
		else         
			if (ch == '$') // abort partial Sentence 
			{
				ll = tt = RxCheckSum = 0;
				GPSSentenceType = FirstGPSType;
				GPSRxState = WaitGPSTag;
			}
			else
				if ( NMEA[NTail].length >= (GPSRXBUFFLENGTH-1) )
					GPSRxState = WaitGPSSentinel;
				else
					GPSRxCheckSum = RxCheckSum;
		break;
	case WaitGPSCheckSum:
		if (GPSCheckSumChar < 2)
		{
			GPSTxCheckSum = GPSTxCheckSum * 16 + (int16)(ch);
			if (ch>='A')
				GPSTxCheckSum += ( -(int16)'A' + 10);
			else
				GPSTxCheckSum -= (int16)'0';
			GPSCheckSumChar++;
		}
		else
		{
			GPSSentenceReceived = GPSRxCheckSum == GPSTxCheckSum;
			if ( GPSSentenceReceived )
			{
				NMEA[NTail].length = ll;
				NEntries++;
			}	
			GPSRxState = WaitGPSSentinel;
		}
		break;
	case WaitGPSTag:
		RxCheckSum ^= ch;
		while ( (ch != GPSTag[GPSSentenceType][tt]) && ( GPSSentenceType < LastGPSType ))
			GPSSentenceType++;
		if ( ch == GPSTag[GPSSentenceType][tt] ) 
			if ( tt == MAXTAGINDEX )
			{
TxChar(GPSSentenceType+'A');
				if ( NEntries > 1 )
				{
					Beeper_ON;
					GPSRxState = WaitGPSSentinel;
				}
				else
				{
					NTail = (NTail + 1) & 1;
					GPSRxState = WaitGPSBody;
				}
			}
	        else
				tt++;
		else
{
TxChar('x');
	        GPSRxState = WaitGPSSentinel;
}
		break;
	case WaitGPSSentinel:
		if (ch=='$')
		{
			ll = tt = RxCheckSum = 0;
			GPSSentenceType = FirstGPSType;
			GPSRxState = WaitGPSTag;
		}
		break;
    } 
 
} // PollGPS

void InitGPS()
{ 
	uint8 c;

	cc = 0;
	NEntries = NHead = NTail = 0;

	GPSMode = '_';
	GPSMissionTime = GPSRelAltitude = GPSHeading = GPSGroundSpeed = GPSFix = GPSNoOfSats = GPSHDilute = 0;
	GPSEast = GPSNorth = 0;

	ValidGPSSentences = 0;
	GPSCount = 0;
	FakeGPSCount = 100;

	_GPSValid = false; 
	GPSSentenceReceived=false;
  	GPSRxState=WaitGPSSentinel; 
  	
} // InitGPS

void UpdateGPS(void)
{
	#ifdef FAKE_GPS
	GPSSentenceReceived = true;
	#endif // FAKE_GPS

	if ( NEntries > 0 )
	{
		LEDBlue_ON;
		LEDRed_OFF;
	//	GPSSentenceReceived = false;  
		ParseGPSSentence(); // 7.5mS 18f2520 @ 16MHz
		if ( _GPSValid )
		{
			_NavComputed = false;
			GPSCount = 0;
		}
	}
	else
		if( (BlinkCount & 0x000f) == 0 )
			if( GPSCount > GPSDROPOUT )
				_GPSValid = false;
			else
				GPSCount++;

	LEDBlue_OFF;
	if ( _GPSValid )
		LEDRed_OFF;
	else
		LEDRed_ON;	

} // UpdateGPS


