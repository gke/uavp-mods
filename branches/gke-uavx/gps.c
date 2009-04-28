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
boolean GPSSentenceReceived;

uint8 GPSSentenceType;
uint8 GPSMode;
int16 GPSGroundSpeed, GPSHeading;
uint8 GPSNoOfSats;
uint8 GPSFix;
int16 GPSHDilute;

int32 GPSMissionTime, GPSStartTime;
int32 GPSLatitude, GPSLongitude;
int32 GPSOriginLatitude, GPSOriginLongitude;
int16 GPSNorth, GPSEast, GPSNorthHold, GPSEastHold;
int16 GPSLongitudeCorrection;
int16 GPSAltitude, GPSRelAltitude, GPSOriginAltitude;
#pragma udata

// Global Variables
#pragma udata gpsvars2
int8 ValidGPSSentences;
//int32 GPSStartTime, GPSMissionTime;
#pragma udata

#pragma udata gpsvars
enum WaitGPSStates {WaitGPSSentinel, WaitGPSTag, WaitGPSBody, WaitGPSCheckSum};
uint8 GPSRxState;

#define GPSRXBUFFLENGTH 80
uint8 ch, GPSRxBuffer[GPSRXBUFFLENGTH];
uint8 cc, ll, tt, lo, hi;

#define MAXTAGINDEX 4
enum GPSSentences {GPGGA , GPRMC};
#define FirstGPSType GPGGA
#define LastGPSType GPRMC
// must be in sorted alpha order
const uint8 GPSTag[2][6]= {{"GPGGA"},{"GPRMC"}};

boolean EmptyField;
uint8 GPSTxCheckSum, GPSRxCheckSum, GPSCheckSumChar;
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
			ival += (GPSRxBuffer[i] - '0');
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
	if ((GPSRxBuffer[cc] != ',')&&(GPSRxBuffer[cc]!='*'))
    {
		while ((cc<ll)&&(GPSRxBuffer[cc]!=',')&&(GPSRxBuffer[cc]!='*'))
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
	_GPSValid = (GPSRxBuffer[lo] == 'A');    

	UpdateField();   //Lat
    GPSLatitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LatH
    if (GPSRxBuffer[lo] == 'S')
      	GPSLatitude = -GPSLatitude;

    UpdateField();   //Lon
    // no latitude compensation on longitude - yet!   
    GPSLongitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LonH
	if (GPSRxBuffer[lo] == 'W')
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
	if (GPSRxBuffer[lo] == 'W')
		GPSMagVariation = -GPSMagVariation;
	*/
	UpdateField();   // Mode (A,D,E)
	GPSMode = GPSRxBuffer[lo];
    
} // ParseGPRMCSentence

void ParseGPGGASentence()
{ 	// full position $GPGGA fix 

    UpdateField();
    
    UpdateField();   //UTime
	//GPSMissionTime=ConvertUTime(lo,hi);

	UpdateField();   //Lat
    GPSLatitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LatH
    if (GPSRxBuffer[lo] == 'S')
      	GPSLatitude = -GPSLatitude;

    UpdateField();   //Lon
    // no latitude compensation on longitude - yet!    
    GPSLongitude = ConvertLatLonM(lo,hi);
    UpdateField();   //LonH
	if (GPSRxBuffer[lo] == 'W')
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
    
} // ParseGPGGASentence

#endif //  !FAKE_GPS

void ParseGPSSentence()
{
	int32 Temp;

	#ifndef  FAKE_GPS

	switch (GPSSentenceType) {
	case GPGGA: ParseGPGGASentence(); break;
	case GPRMC: ParseGPRMCSentence(); break;
	}

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
		GPSRxBuffer[ll] = ch;
		ll++;         
		if (( ch == '*' )||( ll == GPSRXBUFFLENGTH ))      
		{
			GPSCheckSumChar = GPSTxCheckSum = 0;
			GPSRxState = WaitGPSCheckSum;
		}
		else 
			if (ch == '$') // abort partial Sentence 
			{
				ll = cc = tt = RxCheckSum = 0;
				GPSSentenceType = FirstGPSType;
				GPSRxState = WaitGPSTag;
			}
			else
				GPSRxCheckSum = RxCheckSum;
		break;
	case WaitGPSCheckSum:
		if (GPSCheckSumChar < 2)
		{
			if (ch>='A')
				GPSTxCheckSum = GPSTxCheckSum * 16 + ((int16)(ch) + 10 - (int16)('A'));
			else
				GPSTxCheckSum=GPSTxCheckSum * 16 + ((int16)(ch) - (int16)('0'));
			GPSCheckSumChar++;
		}
		else
		{
			GPSSentenceReceived = GPSRxCheckSum == GPSTxCheckSum;
			GPSRxState = WaitGPSSentinel;
		}
		break;
	case WaitGPSTag:
		RxCheckSum ^= ch;
		while ( (ch != GPSTag[GPSSentenceType][tt]) && ( GPSSentenceType < LastGPSType ))
			GPSSentenceType++;
		if ( ch == GPSTag[GPSSentenceType][tt] ) 
			if ( tt == MAXTAGINDEX )
				GPSRxState = WaitGPSBody;
	        else
				tt++;
		else
	        GPSRxState = WaitGPSSentinel;
		break;
	case WaitGPSSentinel:
		if (ch=='$')
		{
			ll = cc = tt = RxCheckSum = 0;
			GPSSentenceType = FirstGPSType;
			GPSRxState = WaitGPSTag;
		}
		break;
    } 
 
} // PollGPS

void InitGPS()
{ 
	uint8 c;

	cc = ll = 0;
  	ch = ' ';

	GPSMode = '_';
	GPSRelAltitude = GPSHeading = GPSGroundSpeed = GPSFix = GPSNoOfSats = GPSHDilute = 0;
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

	if ( GPSSentenceReceived )
	{
		LEDBlue_ON;
		LEDRed_OFF;
		GPSSentenceReceived = false;  
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


