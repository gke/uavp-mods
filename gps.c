// ===============================================================================================
// =                              UAVXArm Quadrocopter Controller                                =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVXArm.

//    UAVXArm is free software: you can redistribute it and/or modify it under the terms of the GNU
//    General Public License as published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.

//    UAVXArm is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/

#include "UAVXArm.h"

void UpdateField(void);
int32 ConvertGPSToM(int32);
int32 ConvertMToGPS(int32);
int24 ConvertInt(uint8, uint8);
real32 ConvertReal(uint8, uint8);
int32 ConvertLatLonM(uint8, uint8);
void ConvertUTime(uint8, uint8);
void ConvertUDate(uint8, uint8);
void ParseGPGGASentence(void);
void ParseGPRMCSentence(void);
void SetGPSOrigin(void);
void ParseGPSSentence(void);
void RxGPSPacket(uint8);
void SetGPSOrigin(void);
void DoGPS(void);
void GPSTest(void);
void UpdateGPS(void);
void InitGPS(void);

const uint8 NMEATags[MAX_NMEA_SENTENCES][5]= {
    // NMEA
    {'G','P','G','G','A'}, // full positioning fix
    {'G','P','R','M','C'}, // main current position and heading
};

NMEAStruct NMEA;

uint8     GPSPacketTag;
struct    tm  GPSTime;
int32     GPSStartTime, GPSSeconds;
int32     GPSLatitude, GPSLongitude;
int32     OriginLatitude, OriginLongitude;
int32     DesiredLatitude, DesiredLongitude;
int32     LatitudeP, LongitudeP, HoldLatitude, HoldLongitude;
real32    GPSAltitude, GPSRelAltitude, GPSOriginAltitude;
real32    GPSLongitudeCorrection;
real32    GPSHeading, GPSMagHeading, GPSMagDeviation, GPSVel, GPSVelp, GPSROC;
int8      GPSNoOfSats;
int8      GPSFix;
int16     GPSHDilute;

int32     FakeGPSLongitude, FakeGPSLatitude;

uint8     ll, tt, ss, RxCh;
uint8     GPSCheckSumChar, GPSTxCheckSum;
uint8     nll, cc, lo, hi;
boolean   EmptyField;
int16     ValidGPSSentences;
real32    GPSdT, GPSdTR;
uint32    GPSuSp;

int32     SumGPSRelAltitude, SumBaroRelAltitude;

int32 ConvertGPSToM(int32 c) {   // approximately 1.8553257183 cm per LSB at the Equator
    // conversion max is 21Km
    return ( ((int32)c * (int32)1855)/((int32)100000) );
} // ConvertGPSToM

int32 ConvertMToGPS(int32 c) {
    // conversion max is 21Km
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

real32 ConvertReal(uint8 lo, uint8 hi) {
    int16 i, n, dp;
    boolean Positive;
    int16 whole;
    real32 rval;

    if (EmptyField)
        rval=0.0;
    else {
        if (NMEA.s[lo]=='-') {
            Positive=false;
            lo++;
        } else
            Positive=true;

        dp=lo;
        while ((NMEA.s[dp] != '.')&&(dp<=hi))
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
} // ConvertReal

int32 ConvertLatLonM(uint8 lo, uint8 hi) {    // NMEA coordinates normally assumed as DDDMM.MMMMM ie 5 decimal minute digits
    // but code can deal with 4 and 5 decimal minutes
    // Positions are stored at 5 decimal minute NMEA resolution which is
    // approximately 1.855 cm per LSB at the Equator.
    static int32 dd, mm, dm, r;
    static uint8 dp;

    r = 0;
    if ( !EmptyField ) {
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

void ConvertUTime(uint8 lo, uint8 hi) {

    if ( !EmptyField ) {
        GPSTime.tm_hour = ConvertInt(lo, lo+1);
        GPSTime.tm_min = ConvertInt(lo+2, lo+3);
        GPSTime.tm_sec = ConvertInt(lo+4, lo+5);
        GPSSeconds = (int32)GPSTime.tm_hour * 3600 + GPSTime.tm_min * 60 + GPSTime.tm_sec;
    }
} // ConvertUTime

void ConvertUDate(uint8 lo, uint8 hi) {

    static time_t seconds;

//   if ( !EmptyField && !F.RTCValid )
    {
        GPSTime.tm_mday = ConvertInt(lo, lo + 1);
        GPSTime.tm_mon = ConvertInt(lo + 2, lo + 3) - 1;
        GPSTime.tm_year = ConvertInt( lo + 4, hi ) + 100;
        seconds = mktime ( &GPSTime );
        set_time( seconds );

        F.RTCValid = true;
    }

} // ConvertUDate

void UpdateField(void) {
    static uint8 ch;

    lo = cc;

    ch = NMEA.s[cc];
    while (( ch != ',' ) && ( ch != '*' ) && ( cc < nll ))
        ch = NMEA.s[++cc];

    hi = cc - 1;
    cc++;
    EmptyField = hi < lo;
} // UpdateField

void ParseGPGGASentence(void) {    // full position $GPGGA fix

    UpdateField();

    UpdateField();   //UTime
    ConvertUTime(lo,hi);

    UpdateField();       //Lat
    GPSLatitude = ConvertLatLonM(lo, hi);
    UpdateField();   //LatH
    if (NMEA.s[lo] == 'S') GPSLatitude = -GPSLatitude;

    UpdateField();       //Lon
    GPSLongitude = ConvertLatLonM(lo, hi);
    UpdateField();       //LonH
    if (NMEA.s[lo] == 'W') GPSLongitude = -GPSLongitude;

    UpdateField();       //Fix
    GPSFix = (uint8)(ConvertInt(lo,hi));

    UpdateField();       //Sats
    GPSNoOfSats = (uint8)(ConvertInt(lo,hi));

    UpdateField();       // HDilute
    GPSHDilute = ConvertInt(lo, hi-3) * 100 + ConvertInt(hi-1, hi); // Cm

    UpdateField();       // Alt
    GPSAltitude = real32(ConvertInt(lo, hi-2) * 10L + ConvertInt(hi, hi)); // Metres

    //UpdateField();   // AltUnit - assume Metres!

    //UpdateField();   // GHeight
    //UpdateField();   // GHeightUnit

    F.GPSValid = (GPSFix >= GPS_MIN_FIX) && ( GPSNoOfSats >= GPS_MIN_SATELLITES );

    if ( State == InFlight ) {
        if ( GPSHDilute > Stats[MaxHDiluteS] ) {
            Stats[MaxHDiluteS] = GPSHDilute;
            F.GPSFailure = GPSHDilute > 150;
        } else
            if ( GPSHDilute < Stats[MinHDiluteS] )
                Stats[MinHDiluteS] = GPSHDilute;

        if ( GPSNoOfSats > Stats[GPSMaxSatsS] )
            Stats[GPSMaxSatsS] = GPSNoOfSats;
        else
            if ( GPSNoOfSats < Stats[GPSMinSatsS] )
                Stats[GPSMinSatsS] = GPSNoOfSats;
    }
} // ParseGPGGASentence

void ParseGPRMCSentence() { // main current position and heading

    // uint32 UTime;

    UpdateField();

    UpdateField();   //UTime
    //UTime = ConvertUTime(lo,hi);
    // GPSMissionTime =(int) (UTime-GPSStartTime);

    UpdateField();
    if ( NMEA.s[lo] == 'A' ) {
        UpdateField();   //Lat
        GPSLatitude = ConvertLatLonM(lo,hi);
        UpdateField();   //LatH
        if (NMEA.s[lo] == 'S')
            GPSLatitude= -GPSLatitude;

        UpdateField();   //Lon
        GPSLongitude = ConvertLatLonM(lo,hi);

        UpdateField();   //LonH
        if ( NMEA.s[lo] == 'W' )
           GPSLongitude = -GPSLongitude;

        UpdateField();   // Groundspeed (Knots)
        GPSVel = ConvertReal(lo, hi) * 0.514444444; // KTTOMPS

        UpdateField();   // True course made good (Degrees)
        GPSHeading = ConvertReal(lo, hi) * DEGRAD;

        UpdateField();   //UDate
        ConvertUDate(lo, hi);

        UpdateField();   // Magnetic Deviation (Degrees)
        GPSMagDeviation = ConvertReal(lo, hi) * DEGRAD;

        UpdateField();   // East/West
        if ( NMEA.s[lo] == 'W')
            GPSMagHeading = GPSHeading - GPSMagDeviation; // need to check sign????
        else
            GPSMagHeading = GPSHeading +  GPSMagDeviation;
        F.HaveGPRMC = true;
    } else
        F.HaveGPRMC = false;

} // ParseGPRMCSentence

void SetGPSOrigin(void) {
    if ( ( ValidGPSSentences == GPS_ORIGIN_SENTENCES ) && F.GPSValid ) {
        GPSStartTime = GPSSeconds;
        OriginLatitude = DesiredLatitude = HoldLatitude = LatitudeP = GPSLatitude;
        OriginLongitude = DesiredLongitude = HoldLongitude = LongitudeP = GPSLongitude;
        GPSVel = 0;

#ifdef SIMULATE
        FakeGPSLongitude = GPSLongitude;
        FakeGPSLatitude = GPSLatitude;
#endif // SIMULATE

        mS[LastGPS] = mSClock();

        GPSLongitudeCorrection = cos(fabs((real32)(GPSLatitude/600000L) * DEGRAD));

        GPSOriginAltitude = GPSAltitude;

        Write16PX(NAV_ORIGIN_ALT, (int16)(GPSAltitude/100));
        Write32PX(NAV_ORIGIN_LAT, GPSLatitude);
        Write32PX(NAV_ORIGIN_LON, GPSLongitude);

        if ( !F.NavValid ) {
            DoBeep100mS(2,0);
            Stats[NavValidS] = true;
            F.NavValid = true;
        }
        F.AcquireNewPosition = true;
    }
} // SetGPSOrigin

void ParseGPSSentence(void) {
    static uint32 Now;

#define FAKE_NORTH_WIND       0L
#define FAKE_EAST_WIND        0L
#define SCALE_VEL             64L

    cc = 0;
    nll = NMEA.length;

    switch ( GPSPacketTag ) {
        case GPGGAPacketTag:
            ParseGPGGASentence();
            break;
        case GPRMCPacketTag:
            ParseGPRMCSentence();
            break;
    }

    if ( F.GPSValid ) {
        // all coordinates in 0.00001 Minutes or ~1.8553cm relative to Origin
        // There is a lot of jitter in position - could use Kalman Estimator?

        Now = uSClock();
        GPSdT = ( Now - GPSuSp) * 0.0000001;
        GPSdTR = 1.0 / GPSdT;
        GPSuSp = Now;

        if ( ValidGPSSentences <  GPS_ORIGIN_SENTENCES ) {  // repetition to ensure GPGGA altitude is captured
            F.GPSValid = false;

            if ( ( GPSPacketTag == GPGGAPacketTag ) && ( GPSHDilute <= GPS_MIN_HDILUTE ))
                ValidGPSSentences++;
            else
                ValidGPSSentences = 0;
        }

#ifdef SIMULATE

        if ( State == InFlight ) { // don't bother with GPS longitude correction for now?
            CosH = int16cos(Heading);
            SinH = int16sin(Heading);
            GPSLongitude = FakeGPSLongitude + ((int32)(-FakeDesiredPitch) * SinH)/SCALE_VEL;
            GPSLatitude = FakeGPSLatitude + ((int32)(-FakeDesiredPitch) * CosH)/SCALE_VEL;

            A = Make2Pi(Heading + HALFMILLIPI);
            CosH = int16cos(A);
            SinH = int16sin(A);
            GPSLongitude += ((int32)FakeDesiredRoll * SinH) / SCALE_VEL;
            GPSLongitude += FAKE_EAST_WIND; // wind
            GPSLatitude += ((int32)FakeDesiredRoll * CosH) / SCALE_VEL;
            GPSLatitude += FAKE_NORTH_WIND; // wind

            FakeGPSLongitude = GPSLongitude;
            FakeGPSLatitude = GPSLatitude;

            GPSRelAltitude = BaroRelAltitude;
        }

#else
        if (F.NavValid )
            GPSRelAltitude = GPSAltitude - GPSOriginAltitude;

#endif // SIMULATE

        // possibly add GPS filtering here

        if ( State == InFlight ) {
            if ( GPSRelAltitude > Stats[GPSAltitudeS] )
                Stats[GPSAltitudeS] = GPSRelAltitude;

            if ( GPSVel * 10.0 > Stats[GPSVelS] )
                Stats[GPSVelS] = GPSVel * 10.0;

            if (( BaroRelAltitude > 5.0 ) && ( BaroRelAltitude < 15.0 )) { // 5-15M
                SumGPSRelAltitude += GPSRelAltitude;
                SumBaroRelAltitude += BaroRelAltitude;
            }
        }
    } else
        if ( State == InFlight )
            Stats[GPSInvalidS]++;

} // ParseGPSSentence

void RxGPSPacket(uint8 RxCh) {

    switch ( RxState ) {
        case WaitCheckSum:
            if (GPSCheckSumChar < (uint8)2) {
                GPSTxCheckSum *= 16;
                if ( RxCh >= 'A' )
                    GPSTxCheckSum += ( RxCh - ('A' - 10) );
                else
                    GPSTxCheckSum += ( RxCh - '0' );

                GPSCheckSumChar++;
            } else {
                NMEA.length = ll;
                F.GPSPacketReceived = GPSTxCheckSum == RxCheckSum;
                RxState = WaitSentinel;
            }
            break;
        case WaitBody:
            if ( RxCh == '*' ) {
                GPSCheckSumChar = GPSTxCheckSum = 0;
                RxState = WaitCheckSum;
            } else
                if ( RxCh == '$' ) { // abort partial Sentence
                    ll = tt = RxCheckSum = 0;
                    RxState = WaitTag;
                } else {
                    RxCheckSum ^= RxCh;
                    NMEA.s[ll++] = RxCh;
                    if ( ll > (uint8)( GPSRXBUFFLENGTH-1 ) )
                        RxState = WaitSentinel;
                }

            break;
        case WaitTag:
            RxCheckSum ^= RxCh;
            while ( ( RxCh != NMEATags[ss][tt] ) && ( ss < MAX_NMEA_SENTENCES ) ) ss++;
            if ( RxCh == NMEATags[ss][tt] )
                if ( tt == (uint8)NMEA_TAG_INDEX ) {
                    GPSPacketTag = ss;
                    RxState = WaitBody;
                } else
                    tt++;
            else
                RxState = WaitSentinel;
            break;
        case WaitSentinel: // highest priority skipping unused sentence types
            if ( RxCh == '$' ) {
                ll = tt = ss = RxCheckSum = 0;
                RxState = WaitTag;
            }
            break;
    }
} // RxGPSPacket

void UpdateGPS(void) {
    if ( F.GPSPacketReceived ) {
        LEDBlue_TOG;
        F.GPSPacketReceived = false;
        ParseGPSSentence();
        if ( F.GPSValid ) {
            F.NavComputed = false;
            mS[GPSTimeout] = mSClock() + GPS_TIMEOUT_MS;
        } else {
            NavCorr[Pitch] = DecayX(NavCorr[Pitch], 2);
            NavCorr[Roll] = DecayX(NavCorr[Roll], 2);
            NavCorr[Yaw] = 0;
        }
    } else
        if ( mSClock() > mS[GPSTimeout] )
            F.GPSValid = false;

    if ( F.GPSValid )
        LEDRed_OFF;
    else
        LEDRed_ON;

} // UpdateGPS

void GPSTest(void) {

    static uint8 i;

    TxString("\r\nGPS test\r\n\r\n");

    UpdateGPS();

    UpdateRTC();
    i = 0;
    while ( RTCString[i] != NULL )
        TxChar(RTCString[i++]);
    TxNextLine();
    TxString("Fix:     \t");
    TxVal32(GPSFix,0,0);
    TxNextLine();
    TxString("Sats:    \t");
    TxVal32(GPSNoOfSats,0,0);
    TxNextLine();
    TxString("HDilute: \t");
    TxVal32(GPSHDilute,2,0);
    TxNextLine();
    TxString("Alt:     \t");
    TxVal32(GPSAltitude,1,0);
    TxNextLine();
    TxString("Lat:     \t");
    TxVal32(GPSLatitude/600, 4, 0);
    TxNextLine();
    TxString("Lon:     \t");
    TxVal32(GPSLongitude/600, 4, 0);
    if ( F.HaveGPRMC ) {
        TxString("\r\nVel.   :\t");
        TxVal32(GPSVel * 10.0 , 1, 0);
        TxString("\r\nHeading:\t");
        TxVal32(GPSHeading * RADDEG * 10.0 , 1, 0);
        TxString("\r\nMDev.  :\t");
        TxVal32(GPSMagDeviation * RADDEG * 10.0 , 1, 0);
        TxNextLine();
    }
    TxNextLine();
} // GPSTest

void InitGPS(void) {
    cc = 0;

    GPSuSp = uSClock();

    GPSLongitudeCorrection = 1.0;
    GPSSeconds = GPSFix = GPSNoOfSats = GPSHDilute = 0;
    GPSRelAltitude =  GPSAltitude = GPSMagDeviation = GPSHeading = GPSVel = 0.0;
    GPSPacketTag = GPSUnknownPacketTag;

    GPSTime.tm_hour = GPSTime.tm_min = GPSTime.tm_sec = GPSTime.tm_mday = GPSTime.tm_mon = GPSTime.tm_year = 0;

    OriginLatitude = DesiredLatitude = HoldLatitude = LatitudeP = GPSLatitude = 0;
    OriginLongitude = DesiredLongitude = HoldLongitude = LongitudeP = GPSLongitude = 0;

    Write32PX(NAV_ORIGIN_LAT, 0);
    Write32PX(NAV_ORIGIN_LON, 0);
    Write16PX(NAV_ORIGIN_ALT, 0);

    ValidGPSSentences = 0;

    SumGPSRelAltitude = SumBaroRelAltitude = 0;

    F.NavValid = F.GPSValid = F.GPSPacketReceived = false;
    RxState = WaitSentinel;

} // InitGPS
