// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007  Prof. Greg Egan                     =
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

#include "c-ufo.h"
#include "bits.h"

// Prototypes
void ParseGPSSentence(void);
void PollGPS(void);
void InitGPS(void);

#ifdef USE_GPS

// Global Variables

#pragma udata gpsextvars
boolean GPSSentenceReceived;
real32 GPSLatitude, GPSLongitude, GPSOriginLatitude, GPSOriginLongitude;
int16 GPSHeading, GPSAltitude, GPSOriginAltitude, GPSBearing, GPSGroundSpeed;
#pragma udata

#ifdef GPS_NMEA

// NMEA Prototypes

void UpdateField(void);
int16 ConvertInt(uint8, uint8);
real32 ConvertReal(uint8, uint8);
real32 ConvertLatLon(uint8, uint8);



#pragma udata gpsvars
enum WaitGPSStates {WaitGPSSentinel, WaitGPSTag, WaitGPSBody, WaitGPSCheckSum};
uint8 GPSRxState;

#define GPSRXBUFFLENGTH 80
uint8 ch, GPSRxBuffer[GPSRXBUFFLENGTH];
uint8 cc, ll, tt, lo, hi;

#define MAXTAGINDEX 4
const uint8 GPGGATag[]= {'GPGGA'}; 				// full positioning fix

boolean FirstGPSSentence;
boolean EmptyField;
uint8 GPSTxCheckSum, GPSRxCheckSum, GPSCheckSumChar;

#pragma udata

#pragma udata gpstags
  
#pragma udata



int16 ConvertInt(uint8 lo, uint8 hi)
{
  uint8 i;
  int16 ival;

  if (EmptyField)
    ival=0;
  else
    {
    ival=0;
    for (i=lo;i<=hi;i++)
      {
      ival*=10;
      ival+=(GPSRxBuffer[i]-'0');
      }
    }
  return (ival);
}

real32 ConvertReal(uint8 lo, uint8 hi)
{
  int16 i, n, dp;
  boolean Positive;
  int16 whole;
  real32 rval;

  if (EmptyField)
    rval=0.0;
  else
    {
    if (GPSRxBuffer[lo]=='-') 
      {
      Positive=false;
      lo++;
      } 
    else
      Positive=true;

    dp=lo;
    while ((GPSRxBuffer[dp] != '.')&&(dp<=hi))
      dp++;

    whole=ConvertInt(lo, dp-1);
    rval=ConvertInt(dp + 1, hi);

    n=hi-dp;
    for (i=1;i<=n;i++)
      rval/=10.0;
	  
    if (Positive)
      rval=(whole+rval);
    else
      rval=-(whole+rval);
  }

  return(rval);
}// ConvertReal

real32 ConvertLatLon(uint8 lo, uint8 hi)
{
  real32 dd, mm, rval;

  if (EmptyField)
    rval=0.0;
  else
    {
    dd=(real32)(ConvertInt(lo, hi-7));
    mm=ConvertReal(hi-6, hi);
    rval=dd + mm * 0.0166666667;
    }

  return(rval);
} // ConvertLatLon

void UpdateField()
{
  lo=cc;
  hi=lo-1;
  if ((GPSRxBuffer[cc] != ',')&&(GPSRxBuffer[cc]!='*'))
    {
    while ((cc<ll)&&(GPSRxBuffer[cc]!=',')&&(GPSRxBuffer[cc]!='*'))
      cc++;
    hi=cc-1;
    }
  cc++;
  EmptyField=(hi<lo);
} // UpdateField

void ParseGPSSentence()
{ // full position fix 
	uint8 GPSFix, GPSSats;
  	//int32 UTime;

    UpdateField();
    
    UpdateField();   //UTime
  	//UTime=ConvertUTime(lo,hi);      
  	//Nav.MissionTime=(UTime-GPSStartTime);
     
    UpdateField();   //Lat
    GPSLatitude=ConvertLatLon(lo,hi)*DEGTOM;
    UpdateField();   //LatH
    if (GPSRxBuffer[lo]=='S')
      GPSLatitude= -GPSLatitude;
    
    UpdateField();   //Lon
    // no latitude compensation on longditude    
    GPSLongitude=ConvertLatLon(lo,hi)*DEGTOM;
    UpdateField();   //LonH
    if (GPSRxBuffer[lo]=='W')
      GPSLongitude=-GPSLongitude;
           
    UpdateField();   //Fix 
    GPSFix=(uint8)(ConvertInt(lo,hi));

    UpdateField();   //Sats
    GPSSats=(uint8)(ConvertInt(lo,hi));

    UpdateField();   // HDilute

    UpdateField();   // Alt
    GPSAltitude=(int16) (ConvertReal(lo, hi)+0.5);

    //UpdateField();   // AltUnit

    //UpdateField();   // GHeight 
    //UpdateField();   // GHeightUnit 
 
    _GPSValid=(GPSFix>0);
    
    if (FirstGPSSentence&&_GPSValid)
      {
 //     GPSStartTime=Nav.MissionTime;
      GPSOriginLatitude=GPSLatitude;
      GPSOriginLongitude=GPSLongitude;
	//LongitudeCorrection=cos(GPSLatitude);
      GPSOriginAltitude=GPSAltitude;
      FirstGPSSentence=false;
      } 
} // ParseGPSSentence

void PollGPS(void)
{
  if (RxHead!=RxTail)
    switch (GPSRxState) {
    case WaitGPSBody:
      {
      ch=RxChar();
      GPSRxBuffer[ll]=ch;
      ll++;         
      if ((ch=='*')||(ll==GPSRXBUFFLENGTH))      
        {
        GPSCheckSumChar=0;
        GPSTxCheckSum=0;
        GPSRxState=WaitGPSCheckSum;
        }
      else 
        if (ch=='$') // abort partial Sentence 
          {
          ll=0;
          cc=0;
          tt=0;
          RxCheckSum=0;
          GPSRxState=WaitGPSTag;
          }
        else
          GPSRxCheckSum=RxCheckSum;
      break;
      }
    case WaitGPSCheckSum:
      {
      ch=RxChar();
      if (GPSCheckSumChar<2)
        {
        if (ch>='A')
          GPSTxCheckSum=GPSTxCheckSum*16+((int16)(ch)+10-(int16)('A'));
        else
          GPSTxCheckSum=GPSTxCheckSum*16+((int16)(ch)-(int16)('0'));
        GPSCheckSumChar++;
        }
      else
        {
        GPSSentenceReceived=(GPSRxCheckSum==GPSTxCheckSum);
        GPSRxState=WaitGPSSentinel;
        }
      break;
      }
    case WaitGPSTag:
      {
      ch=RxChar();
      if (ch==GPGGATag[tt])
        if (tt==MAXTAGINDEX)
          GPSRxState=WaitGPSBody;
        else
          tt++;
      else
        GPSRxState=WaitGPSSentinel;
      break;
      }
    case WaitGPSSentinel:
      {
      ch=RxChar();
      if (ch=='$')
        {
        ll=0;
        cc=0;
        tt=0;
        RxCheckSum=0;
        GPSRxState=WaitGPSTag;
        }
      break;
      }
    }  
} // PollGPS

void InitGPS()
{ 
	GPSSentenceReceived=false;
	FirstGPSSentence = true;
	_GPSValid = false; 
  	GPSRxState=WaitGPSSentinel; 
  	ll=0;
  	cc=ll;
  	ch=' '; 
} // InitGPS

#else // GPS_TRIMBLE

// TRIMBLE RAW CODE NOT COMMISSIONED FOR UAVP

void TSIPTime(void);
void TSIPHealth(void);
void TSIPPosition(void);
void TSIPStatus(void);
void TSIPVelocity(void);
void TSIPSatsInView(void);
void TSIPDifferentialFix(void);

enum TrimbleStatusValues {DoingPositionFix=0, NoGPSTime=1, NeedInitialisation=2, PDOPTooHigh=3, NoSatellites=8, OneSat=9, TwoSat=10, ThreeSat=11, BadSat=12};
enum DGPSModes {DGPSOff=0, DGPSManual=1, DGPSAutoOff=2, DGPSAutoOn=3};

typedef struct {
  uint8 BadAlmanac:1;
  uint8 NoRTC:1;
  uint8 NoAntenna:1;
  uint8 NoBattery:1;
  
  uint8 No3DFix:1;
  uint8 Mode2D:1;
  uint8 Mode3D:1;
  uint8 Fault:1; // general fault set if any other flags set
  } TrimbleFlags;


typedef struct {
  uint8 DGPSMode;
  uint8 Status;
  TrimbleFlags F;
  } TrimbleState;

#define TSIPRxBuffLength 55 // Trimble 

#pragma udata gpspacket
uint8 TSIPRxBuffer[TSIPRxBuffLength];
#pragma udata

uint8 cc, ll, tt;

int32 TrimbleEastVel, TrimbleNorthVel;
real32 LongitudeCorrection;
uint8 GPSNoOfSats;

boolean FirstGPSSentence, GPSFrameReceived; 

enum WaitTSIPStates {WaitTSIPSentinel, WaitTSIPBody, WaitTSIPDoubleDLE, WaitTSIPConfirmStart, WaitTSIPEnd};

uint8 TSIPTag;

TrimbleState Trimble;

uint8 TSIPRxState;

int16 ConvertInt16(uint8 lo, uint8 hi)
{
  int16 ival;

  ival=(TSIPRxBuffer[lo]<<8)|TSIPRxBuffer[hi];
  
  return (ival);
}

int32 ConvertInt32(uint8 lo, uint8 hi)
{
  int32 ival;
  uint8 b;

  ival=0;
  for (b=lo;b<=hi;b++)
    ival=(ival<<8)|TSIPRxBuffer[b];
    
  return(ival);
}

uint32 ConvertUInt32(uint8 lo, uint8 hi)
{
  uint32 ival;
  uint8 b;

  ival=0;
  for (b=lo;b<=hi;b++)
    ival=(ival<<8)|TSIPRxBuffer[b];
    
  return(ival);
}

typedef union {int32 i; real32 r;} ieee;

real32 ConvertReal32(uint8 lo, uint8 hi)
{
  ieee val;
  uint8 b;

  val.i=0;
  for (b=lo;b<=hi;b++)
    val.i=(val.i<<8)|TSIPRxBuffer[b];
  
 // val.r=ieeetomchp(val.r);
	
  return(val.r);
}

void TSIPTime() // 41 - 61.6uS
{
//  MissionTime=ConvertInt32(0,3);
}

void TSIPHealth()  // 46 - 4.9uS
{ // only useful at startup - remains fixed until Trimble reset

  Trimble.Status=TSIPRxBuffer[1];
  Trimble.F.NoBattery=IsSet(TSIPRxBuffer[1],0);
  Trimble.F.NoAntenna=IsSet(TSIPRxBuffer[1],4);
}

void TSIPPosition()  // 4a - 197.7-200.1uS
{
  GPSLatitude=ConvertReal32(0,3);
  GPSLongitude=ConvertReal32(4,7);
  GPSAltitude=(int16)(ConvertReal32(8,11));
  
  if (FirstGPSSentence)
    {
	GPSOriginLatitude=GPSLatitude;
	GPSOriginLongitude=GPSLongitude;
	GPSOriginAltitude=GPSAltitude;

   // LongitudeCorrection=cos(GPSLatitude); // 530uSec on 40MHz PIC18F processor !!!!
                                           // short range UAV so do it once
	FirstGPSSentence=false;
	}
  _GPSValid=true; // only emits position if it has a fix
 // Nav.F.No3DFix=GPSNoOfSats<4;
}

void TSIPStatus()  // 4b - 5.2uS
{
  Trimble.F.NoRTC=IsSet(TSIPRxBuffer[1],1);
  Trimble.F.BadAlmanac=IsSet(TSIPRxBuffer[1],3);
 // Trimble.F.NoSuperPackets=IsSet(TSIPRxBuffer[2],0);
}

void TSIPVelocity()  // 56 - 598uS -> 218.4uS 
{
  TrimbleEastVel=(int32)(ConvertReal32(0,3)*10.0);
  TrimbleNorthVel=(int32)(ConvertReal32(4,7)*10.0);  
//  GPSRateOfClimb=(int16)(ConvertReal32(8,11)+0.5);
  
  // moved to Differential Fx sentence to reduce time   
  //GPSHeading=int16atan2(EastVel, NorthVel);         
  //GPSGroundSpeed=int32sqrt((int32)(NorthVel)*(int32)(NorthVel)+(int32)(EastVel)*(int32)(EastVel)); 
}

void TSIPSatsInView()  // 6d - 78.4uS
{
  Trimble.F.Mode2D=false;
  Trimble.F.Mode3D=false;
  switch (TSIPRxBuffer[0]&0b00000111) {
    case 3: Trimble.F.Mode2D=true; break;
    case 4: Trimble.F.Mode3D=true; break;
    default: break;
    }
  GPSNoOfSats=(uint8)((TSIPRxBuffer[0]>>4)&0x0f);
 // Trimble.PDOP=ConvertReal32(1,4);
 // GPSEstHorizontalError=(int16)(ConvertReal32(5,8)*10.0);
 // Trimble.VDOP=ConvertReal32(9,12);
 // Trimble.TDOP=ConvertReal32(13,16);
} // TSIPSatsInView

void TSIPDifferentialFix()  // 82 - 1.2uS -> 374.7uS (a bit long but last sentence so OK) 
{ // This sentence is the last in the sequence of TSIP sentences 
  // following which there is a gap for processing
  GPSFrameReceived=true; // needs top be before next statement or it gets clobbered by banking optimisation in C18!!
/*
  // moved here to reduce computation time in Velocity sentence
  GPSHeading=int16atan2(TrimbleEastVel, TrimbleNorthVel);          
  GPSGroundSpeed=int32sqrt((int32)(TrimbleNorthVel)*(int32)(TrimbleNorthVel)+(int32)(TrimbleEastVel)*(int32)(TrimbleEastVel));

  Trimble.F.Fault=(
    Trimble.F.BadAlmanac||
    Trimble.F.NoRTC||
    Trimble.F.NoAntenna||
    Trimble.F.NoBattery||
    Trimble.F.No3DFix);
  Trimble.DGPSMode=(uint8)(TSIPRxBuffer[0]&3);  
*/
}

void ParseGPSSentence() // 2.4uS
{
  switch (TSIPTag) {
    case 0x41: TSIPTime(); break;
    case 0x46: TSIPHealth(); break;
    case 0x4a: TSIPPosition(); break;
    case 0x4b: TSIPStatus(); break;
    case 0x56: TSIPVelocity(); break;
    case 0x6d: TSIPSatsInView(); break;
    case 0x82: TSIPDifferentialFix(); break; 
    default: break;
    }
}

void PollGPS(void)
{
	uint8 ch;

	if ( RxTail != RxHead )
	{
	ch = RxChar();

    switch (TSIPRxState) {
    case WaitTSIPBody:
      {
      TSIPRxBuffer[ll]=ch;
      ll++;
      if (ch==DLE)
        TSIPRxState=WaitTSIPEnd;
      else
        if(ll>=TSIPRxBuffLength)
          TSIPRxState=WaitTSIPSentinel; 
      break;
      }
    case WaitTSIPEnd:
      {
      if (ch==DLE)  // double DLE
        TSIPRxState=WaitTSIPBody;
      else
        if (ch==ETX)
          {
          GPSSentenceReceived=true;
          TSIPRxState=WaitTSIPSentinel;
          }
        else
          {
          TSIPRxBuffer[ll]=ch;
          ll++;         
          TSIPRxState=WaitTSIPBody;
          }
      break;
      }
    case WaitTSIPSentinel:
      {
      if (ch==DLE)
        TSIPRxState=WaitTSIPConfirmStart;
      break;
      }
    case WaitTSIPConfirmStart:
      {
      if ((ch!=DLE)&&(ch!=ETX))
        {
        TSIPTag=ch;
        TSIPRxState=WaitTSIPBody;
        ll=0;
	    cc=0;
        tt=0;
        }
      else
        TSIPRxState=WaitTSIPSentinel;
      break;
      }
    } 
	} 
}

void InitGPS()
{   
  //GPSOriginLatitude=0.0;
  //GPSOriginLongitude=0.0;
  //GPSOriginAltitude=0;

  GPSSentenceReceived=false;
  GPSFrameReceived=false;
  _GPSValid=false; 
  FirstGPSSentence=true; 
   
 // do trimble startup stuff
  
  TSIPRxState=WaitTSIPSentinel; 
 // ch=' '; 
}

#endif // USE_GPS

