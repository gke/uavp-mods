// ===============================================================================================
// =                              UAVXArm Quadrocopter Controller                                =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                           http://code.google.com/p/uavp-mods/                               =
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

void RxTelemetryPacket(uint8);
void InitTelemetryPacket(void);
void BuildTelemetryPacket(uint8);

void SendPacketHeader(void);
void SendPacketTrailer(void);

void SendTelemetry(void);
void SendCycle(void);
void SendControl(void);
void SendFlightPacket(void);
void SendNavPacket(void);
void SendControlPacket(void);
void SendStatsPacket(void);
void SendParamPacket(uint8, uint8);
void SendParameters(uint8);
void SendMinPacket(void);
void SendArduStation(void);
void SendPIDTuning(void);
void SendCustom(void);
void SensorTrace(void);

uint8 UAVXCurrPacketTag;
uint8 RxPacketLength, RxPacketByteCount;
uint8 RxCheckSum;
uint8 RxPacketTag, ReceivedPacketTag;
uint8 PacketRxState;
boolean CheckSumError, TelemetryPacketReceived;

int16 RxLengthErrors, RxTypeErrors, RxCheckSumErrors;

uint8 UAVXPacket[256];

FILE *logfile = NULL;
boolean EchoToLogFile = false;
uint32 LogChars;
boolean LogfileIsOpen = false;

uint8 TxCheckSum;

void InitTelemetryPacket(void) {
    RxPacketByteCount = 0;
    RxCheckSum = 0;
    RxPacketTag = UnknownPacketTag;
    RxPacketLength = 2; // set as minimum
    PacketRxState = WaitRxSentinel;
} // InitTelemetryPacket

void BuildTelemetryPacket(uint8 ch) {
    static boolean RxPacketError;

    UAVXPacket[RxPacketByteCount++] = ch;

    if (RxPacketByteCount == 1) {
        RxPacketTag = ch;
        PacketRxState=WaitRxBody;
    } else
        if (RxPacketByteCount == 2) {
            RxPacketLength = ch; // ignore
            PacketRxState = WaitRxBody;
        } else
            if (RxPacketByteCount >= (RxPacketLength + 3)) {
                RxPacketError = CheckSumError = RxCheckSum != 0;

                if (CheckSumError)
                    RxCheckSumErrors++;

                if (!RxPacketError) {
                    TelemetryPacketReceived = true;
                    ReceivedPacketTag=RxPacketTag;
                }
                PacketRxState = WaitRxSentinel;
                //   InitPollPacket();
            } else
                PacketRxState = WaitRxBody;
} // BuildTelemetryPacket

void RxTelemetryPacket(uint8 ch) {

    RxCheckSum ^= ch;
    switch (PacketRxState) {
        case WaitRxSentinel:
            if (ch == SOH) {
                InitTelemetryPacket();
                CheckSumError = false;
                PacketRxState = WaitRxBody;
            }
            break;
        case WaitRxBody:
            if (ch == ESC)
                PacketRxState = WaitRxESC;
            else
                if (ch == SOH) { // unexpected start of packet
                    RxLengthErrors++;

                    InitTelemetryPacket();
                    PacketRxState = WaitRxBody;
                } else
                    if (ch == EOT) { // unexpected end of packet
                        RxLengthErrors++;
                        PacketRxState = WaitRxSentinel;
                    } else
                        BuildTelemetryPacket(ch);
            break;
        case WaitRxESC:
            BuildTelemetryPacket(ch);
            break;
        default:
            PacketRxState = WaitRxSentinel;
            break;
    }
} // ParseTelemetryPacket


#define NAV_STATS_INTERLEAVE    10
static int8 StatsNavAlternate = 0;

void CheckTelemetry(void) {

    // check incoming - switch on associated action

    // do routine telemetry Tx
    if ( mSClock() > mS[TelemetryUpdate] )
        switch ( P[TelemetryType] ) {
            case UAVXTelemetry:
                mS[TelemetryUpdate] = mSClock() + UAVX_TEL_INTERVAL_MS;
                SendCycle();
                break;
            case ArduStationTelemetry:
                mS[TelemetryUpdate] = mSClock() + UAVX_CONTROL_TEL_INTERVAL_MS;
                SendArduStation();
                break;
            case UAVXControlTelemetry:
                mS[TelemetryUpdate] = mSClock() + UAVX_CONTROL_TEL_INTERVAL_MS;
                SendControlPacket();
                break;
            case UAVXMinTelemetry:
                mS[TelemetryUpdate] = mSClock() + UAVX_MIN_TEL_INTERVAL_MS;
                SendMinPacket();
                break;
            case CustomTelemetry:
                mS[TelemetryUpdate] = mSClock() + CUSTOM_TEL_INTERVAL_MS;
                SendPIDTuning();//SendCustom();
                break;
            case GPSTelemetry:
                break;
        }

} // CheckTelemetry

void SendPacketHeader(void) {

    EchoToLogFile = true;

    TxChar(0xff); // synchronisation to "jolt" USART
    TxChar(SOH);
    TxCheckSum = 0;
} // SendPacketHeader

void SendPacketTrailer(void) {
    TxESCu8(TxCheckSum);
    TxChar(EOT);

    TxChar(CR);
    TxChar(LF);

    EchoToLogFile = false;
} // SendPacketTrailer

void ShowAttitude(void) {
    // Stick units
    TxESCi16(DesiredRoll);
    TxESCi16(DesiredPitch);
    TxESCi16(DesiredYaw);

    // mRadian and mG
    TxESCi16(Gyro[Roll] * 1000.0); // Rate
    TxESCi16(Gyro[Pitch] * 1000.0);
    TxESCi16(Gyro[Yaw] * 1000.0);

    TxESCi16(Angle[Roll] * 1000.0);
    TxESCi16(Angle[Pitch] * 1000.0);
    TxESCi16(Angle[Yaw] * 1000.0);

    TxESCi16(Acc[LR] * 1000.0);
    TxESCi16(Acc[BF] * 1000.0);
    TxESCi16(Acc[UD] * 1000.0);

} // ShowAttitude

void SendFlightPacket(void) {
    static int8 b;

    SendPacketHeader();

    TxESCu8(UAVXFlightPacketTag);
    TxESCu8(58 + TELEMETRY_FLAG_BYTES);
    for ( b = 0; b < TELEMETRY_FLAG_BYTES; b++ )
        TxESCu8(F.AllFlags[b]);

    TxESCu8(State);

    // dV, dA, mAH
    TxESCi16(BatteryVolts * 10.0); // to do scaling
    TxESCi16(BatteryCurrent * 10.0);
    TxESCi16(BatteryChargeUsedAH * 1000.0);

    TxESCi16(RCGlitches);
    TxESCi16(DesiredThrottle);

    ShowAttitude();

    TxESCi8((int8)Comp[LR]);
    TxESCi8((int8)Comp[BF]);
    TxESCi8((int8)Comp[UD]);
    TxESCi8((int8)Comp[Alt]);

    for ( b = 0; b < 8; b++ )
        TxESCi16((int16)PWM[b]);

    TxESCi24(mSClock() - mS[StartTime]);

    SendPacketTrailer();
} // SendFlightPacket

void SendControlPacket(void) {
    static int8 b;

    SendPacketHeader();

    TxESCu8(UAVXControlPacketTag);
    TxESCu8(52);

    TxESCi16(DesiredThrottle);

    ShowAttitude();

    TxESCi16(Heading * 1000.0);
    TxESCi16(DesiredHeading * 1000.0);

    TxESCu8(UAVXAirframe | 0x80);

    for ( b = 0; b < 8; b++ ) // motor/servo channels
        TxESCi16((int16)PWM[b]);

    TxESCi24( time(NULL) );

    SendPacketTrailer();

} // SendControlPacket

void SendNavPacket(void) {
    SendPacketHeader();

    TxESCu8(UAVXNavPacketTag);
    TxESCu8(59);

    TxESCu8(NavState);
    TxESCu8(FailState);
    TxESCu8(GPSNoOfSats);
    TxESCu8(GPSFix);

    TxESCu8(CurrWP);

    TxESCi16(ROC * 10.0 );                         // dm/S
    TxESCi24(BaroRelAltitude * 10.0);

    TxESCi16(GPSHeading * 1000.0);
    TxESCi16(RangefinderAltitude * 100.0);          // cm

    TxESCi16(GPSHDilute);
    TxESCi16(Heading * 1000.0);
    TxESCi16(WayHeading * 1000.0);

    TxESCi16(GPSVel * 10.0);
    TxESCi16(0);                                   // was GPSROC;

    TxESCi24(GPSRelAltitude * 10.0);               // dm
    TxESCi32(GPSLatitude);                         // 5 decimal minute units
    TxESCi32(GPSLongitude);

    TxESCi24(DesiredAltitude * 10.0 );
    TxESCi32(DesiredLatitude);
    TxESCi32(DesiredLongitude);

    TxESCi24(mS[NavStateTimeout] - mSClock());    // mS

    TxESCi16(AmbientTemperature.i16);             // 0.1C
    TxESCi32( time(NULL) );                       //GPSSeconds);

    TxESCu8(NavSensitivity);
    TxESCi8(NavCorr[Roll]);
    TxESCi8(NavCorr[Pitch]);
    TxESCi8(NavCorr[Yaw]);

    SendPacketTrailer();

} // SendNavPacket

void SendStickPacket(void) {
    static uint8 c;

    SendPacketHeader();

    TxESCu8(UAVXStickPacketTag);
    TxESCu8( 1 + RC_CONTROLS * 2 );
    TxESCu8(RC_CONTROLS);
    for ( c = 0; c < RC_CONTROLS; c++ )
        TxESCi16(RC[c]);

    SendPacketTrailer();
} // SendStickPacket

void SendStatsPacket(void) {

    static uint8 i;

    SendPacketHeader();

    TxESCu8(UAVXStatsPacketTag);
    TxESCu8(MAX_STATS * 2 + 2);

    for ( i = 0; i < MAX_STATS ; i++)
        TxESCi16(Stats[i]);

    TxESCu8(UAVXAirframe | 0x80);
    TxESCu8(Orientation);

    SendPacketTrailer();

} // SendStatsPacket

void SendMinPacket(void) {
    static int8 b;

    SendPacketHeader();

    TxESCu8(UAVXMinPacketTag);
    TxESCu8(33 + TELEMETRY_FLAG_BYTES);
    for ( b = 0; b < TELEMETRY_FLAG_BYTES; b++ )
        TxESCu8(F.AllFlags[b]);

    TxESCu8(State);
    TxESCu8(NavState);
    TxESCu8(FailState);

    TxESCi16(BatteryVolts * 10.0); // to do scaling
    TxESCi16(BatteryCurrent * 10.0);
    TxESCi16(BatteryChargeUsedAH * 1000.0);

    TxESCi16(Angle[Roll] * 1000.0);
    TxESCi16(Angle[Pitch] * 1000.0);

    TxESCi24(BaroRelAltitude * 10.0);
    TxESCi16(RangefinderAltitude * 100.0);                 // cm

    TxESCi16(Heading * 1000.0);

    TxESCi32(GPSLatitude);                         // 5 decimal minute units
    TxESCi32(GPSLongitude);

    TxESCu8(UAVXAirframe | 0x80 ); // ARM has top bit set
    TxESCu8(Orientation);

    TxESCi24(mSClock() - mS[StartTime]);

    SendPacketTrailer();

} // SendMinPacket

void SendParamPacket(uint8 s, uint8 p) {

    SendPacketHeader();

    TxESCu8(UAVXArmParamPacketTag);
    TxESCu8(6);
    TxESCu8(s);
    TxESCu8(p);
    TxESCi32(K[p] * 1000.0 );

    SendPacketTrailer();

} // SendParamPacket

void SendParameters(uint8 s) {
    static uint8 p;

    for ( p = 0; p < MAX_PARAMETERS; p++ )
        SendParamPacket(s, p);
    SendParamPacket(0, MAX_PARAMETERS);

} // SendParameters

void SendCycle(void) {

    switch ( UAVXCurrPacketTag ) {
        case UAVXFlightPacketTag:
            SendFlightPacket();

            UAVXCurrPacketTag = UAVXNavPacketTag;
            break;

        case UAVXNavPacketTag:
            if ( ++StatsNavAlternate < NAV_STATS_INTERLEAVE)
                SendNavPacket();
            else {
                SendStatsPacket();
                StatsNavAlternate = 0;
            }

            UAVXCurrPacketTag = UAVXFlightPacketTag;
            break;

        default:
            UAVXCurrPacketTag = UAVXFlightPacketTag;
            break;
    }

} // SendCycle

void SendArduStation(void) {

    static int8 Count = 0;
    /*
    Definitions of the low rate telemetry (1Hz):
    LAT: Latitude
    LON: Longitude
    SPD: Speed over ground from GPS
    CRT: Climb Rate in M/S
    ALT: Altitude in meters
    ALH: The altitude is trying to hold
    CRS: Course over ground in degrees.
    BER: Bearing is the heading you want to go
    WPN: Waypoint number, where WP0 is home.
    DST: Distance from Waypoint
    BTV: Battery Voltage.
    RSP: Roll setpoint used to debug, (not displayed here).

    Definitions of the high rate telemetry (~4Hz):
    ASP: Airspeed, right now is the raw data.
    TTH: Throttle in 100% the autopilot is applying.
    RLL: Roll in degrees + is right - is left
    PCH: Pitch in degrees
    SST: Switch Status, used for debugging, but is disabled in the current version.
    */

    if ( ++Count == 4 ) {
        TxString("!!!");
        TxString("LAT:");
        TxVal32(GPSLatitude / 6000, 3, 0);
        TxString(",LON:");
        TxVal32(GPSLongitude / 6000, 3, 0);
        TxString(",ALT:");
        TxVal32(Altitude / 10,0,0);
        TxString(",ALH:");
        TxVal32(DesiredAltitude / 10, 0, 0);
        TxString(",CRT:");
        TxVal32(ROC / 100, 0, 0);
        TxString(",CRS:");
        TxVal32(Heading * RADDEG, 0, 0); // scaling to degrees?
        TxString(",BER:");
        TxVal32(WayHeading * RADDEG, 0, 0);
        TxString(",SPD:");
        TxVal32(GPSVel, 0, 0);
        TxString(",WPN:");
        TxVal32(CurrWP,0,0);
        TxString(",DST:");
        TxVal32(0, 0, 0); // distance to WP
        TxString(",BTV:");
        TxVal32((BatteryVoltsADC * 61)/205, 1, 0);
        TxString(",RSP:");
        TxVal32(DesiredRoll, 0, 0);

        Count = 0;
    } else {
        TxString("+++");
        TxString("ASP:");
        TxVal32(GPSVel / 100, 0, 0);
        TxString(",RLL:");
        TxVal32(Angle[Roll] / 35, 0, 0); // scale to degrees?
        TxString(",PCH:");
        TxVal32(Angle[Pitch] / 35, 0, 0);
        TxString(",THH:");
        TxVal32( ((int24)DesiredThrottle * 100L) / RC_MAXIMUM, 0, 0);
    }

    TxString(",***\r\n");

} // SendArduStation

void SendPIDTuning(void) { // user defined telemetry human readable OK for small amounts of data < 1mS

    // Fixed to roll axis

    SendPacketHeader();

    TxESCu8(UAVXCustomPacketTag);
    TxESCu8(1 + 10);
    TxESCu8(5); // how many 16bit elements
/*
    TxESCi16(DesiredRoll);
    TxESCi16(PWM[RightC]);

    TxESCi16(Gyro[Roll] * 1000.0);
    TxESCi16(Acc[Roll] * 1000.0);
    TxESCi16(Angle[Roll] * 1000.0 );
    */
    
    TxESCi16(DesiredYaw);
    TxESCi16(PWM[RightC]);

    TxESCi16(Gyro[Yaw] * 1000.0);
    TxESCi16(HE * 1000.0);
    TxESCi16(Angle[Yaw] * 1000.0 );

    SendPacketTrailer();

} // SendPIDTuning


void SendCustom(void) { // user defined telemetry human readable OK for small amounts of data < 1mS

    static uint8 s;

    // always a vector of int16;
    SendPacketHeader();

    TxESCu8(UAVXCustomPacketTag);
    TxESCu8(1 + 8 + MaxAttitudeScheme * 2);
    TxESCu8(4 + MaxAttitudeScheme ); // how many 16bit elements
    TxESCi16(AttitudeMethod);

    TxESCi16(0); // spare

    TxESCi16(Gyro[Roll] * 1000.0);
    TxESCi16(Acc[LR] * 1000.0);

    for ( s = 0; s < MaxAttitudeScheme; s++ )
        TxESCi16( EstAngle[Roll][s] * 1000.0 );

    SendPacketTrailer();

} // SendCustom

void SensorTrace(void) {
#ifdef TESTING

    if ( DesiredThrottle > 20 ) {
        EchoToLogFile = false; // direct to USART

        TxValH16(((int24)Heading * 180)/MILLIPI);
        TxChar(';');

        TxValH16(BaroRelAltitude);
        TxChar(';');
        TxValH16(RangefinderAltitude);
        TxChar(';');
        TxValH16(0);
        TxChar(';');

        TxValH16(DesiredThrottle);
        TxChar(';');
        TxValH16(DesiredRoll);
        TxChar(';');
        TxValH16(DesiredPitch);
        TxChar(';');
        TxValH16(DesiredYaw);
        TxChar(';');

        TxValH16(Rate[Roll]);
        TxChar(';');
        TxValH16(Rate[Pitch]);
        TxChar(';');
        TxValH16(Rate[Yaw]);
        TxChar(';');

        TxValH16(Angle[Roll]);
        TxChar(';');
        TxValH16(Angle[Pitch]);
        TxChar(';');
        TxValH16(Angle[Yaw]);
        TxChar(';');

        TxValH16(Acc[LR]);
        TxChar(';');
        TxValH16(Acc[FB]);
        TxChar(';');
        TxValH16(Acc[DU]);
        TxChar(';');

        TxValH16(Comp[LR]);
        TxChar(';');
        TxValH16(Comp[FB]);
        TxChar(';');
        TxValH16(Comp[DU]);
        TxChar(';');
        TxValH16(Comp[Alt]);
        TxChar(';');
        TxNextLine();
    }
#endif // TESTING
} // SensorTrace
