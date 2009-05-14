// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =                          http://uavp.ch                             =
// =======================================================================

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
void ParseGPSSentence(void);
void PollGPS(uint8);
void InitGPS(void);
void UpdateGPS(void);

// Defines


// Variables

#pragma udata gpsvars
int16 GPSGroundSpeed, GPSHeading;
boolean GPSSentenceReceived;
uint8 NType;
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
#define NBUFF_MASK	1
struct {
	uint8 s[GPSRXBUFFLENGTH];
	uint8 type, length;
	} NMEA[NBUFF_MASK+1];
#pragma udata

// Global Variables
#pragma udata gpsbuff2
uint8 NHead, NTail, NEntries;
#pragma udata

#pragma udata gpsvars3
enum WaitGPSStates {WaitGPSSentinel, WaitNMEATag, WaitGPSBody, WaitGPSCheckSum};
int16 ValidGPSSentences;
int8 NActiveTypes;
uint8 GPSRxState;
uint8 ll, nll, cc, tt, lo, hi;
boolean EmptyField;
uint8 GPSTxCheckSum, GPSRxCheckSum, GPSCheckSumChar;
#pragma udata

#pragma udata gpsvars4
#define MAXTAGINDEX 4
enum GPSSentences {GPGGA, GPRMC};
#define FirstNIndex GPGGA
#define LastNIndex GPRMC
// must be in sorted alpha order
const uint8 NMEATag[LastNIndex+1][6] = {{"GPGGA"},{"GPRMC"}};
uint8 NMEAActive[LastNIndex+1];
#pragma udata

#ifndef FAKE_GPS

int16 ConvertInt(uint8 lo, uint8 hi)
{
	uint8 i;
	int16 ival;

	ival = 0;
	if ( !EmptyField )
		for (i = lo; i <= hi ; i++ )
			ival = ival * 10 + NMEA[NHead].s[i] - '0';

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
	uint8 ch;

	lo = cc;

	ch = NMEA[NHead].s[cc];
	while (( cc < nll ) && ( ch != ',' ) && ( ch != '*' ) ) 
		ch = NMEA[NHead].s[++cc];

	hi = cc - 1;
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

	if ( _GPSTestActive )
		TxString("$GPRMC ");   
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

	if ( _GPSTestActive )
		TxString("$GPGGA ");
} // ParseGPGGASentence

#endif //  !FAKE_GPS

void ParseGPSSentence()
{
	static int32 Temp;
	static uint8 CurrNType;

	cc = 0;

	#ifndef  FAKE_GPS

	nll = NMEA[NHead].length;
	CurrNType = NMEA[NHead].type;

	switch ( CurrNType ) {
		case GPGGA: ParseGPGGASentence(); break;
		case GPRMC: ParseGPRMCSentence(); break;
		default: break;
	}

	NHead++; NHead &= NBUFF_MASK;
	NEntries--; // possibility of being clobbering by PollGPS in interrupt

	if ( _GPSValid )
	{
		if ( !NMEAActive[CurrNType] )
		{
			NMEAActive[CurrNType] = true;
			NActiveTypes++;
		}

	    if ( ValidGPSSentences <  INITIAL_GPS_SENTENCES )
		{   // repetition to ensure GPGGA altitude is captured

			if ( _GPSTestActive )
			{
				TxVal32( INITIAL_GPS_SENTENCES - ValidGPSSentences, 0, 0);
				TxNextLine();
			}

			_GPSValid = false;

			GPSStartTime = GPSMissionTime;
			GPSOriginLatitude = GPSLatitude;
			GPSOriginLongitude = GPSLongitude;
			
			Temp = GPSLatitude/60000L;
			Temp = Abs(Temp);
			Temp = ConvertDDegToMPi(Temp);
			GPSLongitudeCorrection = int16cos(Temp);
		
			if ( CurrNType == GPGGA )
			{
				GPSOriginAltitude = GPSAltitude;
				_GPSAltitudeValid = true;
				if (GPSHDilute <= MIN_HDILUTE )
					ValidGPSSentences++;
			}
			else
				if ( CurrNType == GPRMC )
					_GPSHeadingValid = true;
				
		}
		else
		{
			// all cordinates in 0.0001 Minutes relative to Origin
			GPSNorth = GPSLatitude - GPSOriginLatitude;
			GPSEast = GPSLongitude - GPSOriginLongitude;
			GPSEast = SRS32((int32)GPSEast * GPSLongitudeCorrection, 8); 

			if ( CurrNType == GPGGA )
				GPSRelAltitude = GPSAltitude - GPSOriginAltitude;
		}
	}
	else
		if ( _GPSTestActive )
			TxString("invalid\r\n");

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
				NType = FirstNIndex;
				GPSRxState = WaitNMEATag;
			}
			else
				if ( ll >= (GPSRXBUFFLENGTH-1) )
					GPSRxState = WaitGPSSentinel;
				else
					GPSRxCheckSum = RxCheckSum;
		break;
	case WaitGPSCheckSum:
		if (GPSCheckSumChar < 2)
		{
			GPSTxCheckSum = GPSTxCheckSum * 16 + ch;
			if (ch>='A')
				GPSTxCheckSum += 10 - 'A';
			else
				GPSTxCheckSum -= '0';

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
	case WaitNMEATag:
		RxCheckSum ^= ch;
		while ( (ch != NMEATag[NType][tt]) && ( NType < LastNIndex ))
			NType++;
		if ( ch == NMEATag[NType][tt] ) 
			if ( tt == MAXTAGINDEX )
				if ( NEntries > 1 )
				{
					Beeper_ON;
					GPSRxState = WaitGPSSentinel;
				}
				else
				{
					NTail++; NTail &= NBUFF_MASK;
					NMEA[NTail].type = NType;
					GPSRxState = WaitGPSBody;
				}
	        else
				tt++;
		else
	        GPSRxState = WaitGPSSentinel;
		break;
	case WaitGPSSentinel:
		if (ch=='$')
		{
			ll = tt = RxCheckSum = 0;
			NType = FirstNIndex;
			GPSRxState = WaitNMEATag;
		}
		break;
    } 
 
} // PollGPS

void InitGPS()
{ 
	uint8 n;

	cc = 0;
	NEntries = NHead = NTail = 0;
	for (n = FirstNIndex; n <=LastNIndex; n++)
		NMEAActive[n] = false;
	NActiveTypes = 0;

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
	_GPSHeadingValid = true;
	_GPSAltitudeValid = true;
	_GPSValid = true;
	#else

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
	#endif // FAKE_GPS
} // UpdateGPS


