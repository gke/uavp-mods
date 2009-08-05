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
void PollGPS(uint8);
void InitGPS(void);
void UpdateGPS(void);

// Defines


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
	uint8 i;
	int16 ival;

	ival = 0;
	if ( !EmptyField )
		for (i = lo; i <= hi ; i++ )
			ival = ival * 10 + NMEA.s[i] - '0';

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

void UpdateField(void)
{
	uint8 ch;

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
 
    _GPSValid = (GPSFix >= MIN_FIX) && ( GPSNoOfSats >= MIN_SATELLITES );  

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
	static int32 Temp;
	static uint8 CurrNType;

	cc = 0;

	nll = NMEA.length;

	ParseGPGGASentence(); 

	if ( _GPSValid )
	{
	    if ( ValidGPSSentences <  INITIAL_GPS_SENTENCES )
		{   // repetition to ensure GPGGA altitude is captured

			if ( _GPSTestActive )
			{
				TxVal32( INITIAL_GPS_SENTENCES - ValidGPSSentences, 0, 0);
				TxNextLine();
			}

			_GPSValid = false;

			GPSOriginAltitude = GPSAltitude;
			if (GPSHDilute <= MIN_HDILUTE )
				ValidGPSSentences++;
		
			SetGPSOrigin();
			
		}
		else
		{
			// all cordinates in 0.0001 Minutes relative to Origin
			GPSNorth = GPSLatitude - GPSOriginLatitude;
			GPSEast = GPSLongitude - GPSOriginLongitude;
			GPSEast = SRS32((int32)GPSEast * GPSLongitudeCorrection, 8); 

			GPSRelAltitude = GPSAltitude - GPSOriginAltitude;
		}
	}
	else
		if ( _GPSTestActive )
			TxString("invalid\r\n");
} // ParseGPSSentence

/* inlined in irq.c
void PollGPS(uint8 ch)
{
	switch ( GPSRxState ) {
	case WaitGPSCheckSum:
		if (GPSCheckSumChar < 2)
		{
			GPSTxCheckSum *= 16;
			if (ch >= 'A')
				GPSTxCheckSum += ( ch - 'A' + 10);
			else
				GPSTxCheckSum += ( ch - '0' );

			GPSCheckSumChar++;
		}
		else
		{
			NMEA.length = ll;	
			GPSSentenceReceived = ( GPSTxCheckSum + RxCheckSum ) == 0;
			GPSRxState = WaitGPSSentinel;
		}
		break;
	case WaitGPSBody: 
		if ( ch == '*' )      
		{
			GPSCheckSumChar = GPSTxCheckSum = 0;
			GPSRxState = WaitGPSCheckSum;
		}
		else         
			if (ch == '$') // abort partial Sentence 
			{
				ll = tt = RxCheckSum = 0;
				GPSRxState = WaitNMEATag;
			}
			else
			{
				RxCheckSum ^= ch;
				NMEA.s[ll++] = ch; // no check for buffer overflow
				if ( ll > ( GPSRXBUFFLENGTH-1 ) )
					GPSRxState = WaitGPSSentinel;
			}
					
		break;
	case WaitNMEATag:
		RxCheckSum ^= ch;
		if ( ch == NMEATag[tt] ) 
			if ( tt == MAXTAGINDEX )
				GPSRxState = WaitGPSBody;
	        else
				tt++;
		else
	        GPSRxState = WaitGPSSentinel;
		break;
	case WaitGPSSentinel: // highest priority skipping unused sentence types
		if (ch == '$')
		{
			ll = tt = RxCheckSum = 0;
			GPSRxState = WaitNMEATag;
		}
		break;	
    } 
 
} // PollGPS
inlined in irq.c */

void ResetGPSOrigin(void)
{
	if ( ValidGPSSentences >=  INITIAL_GPS_SENTENCES )
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
			mS[GPSTimeout] = mS[Clock] + GPS_TIMEOUT_S*1000L;
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


