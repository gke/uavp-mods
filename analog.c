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
//    If not, see http://www.gnu.org/licenses//

#include "UAVXArm.h"

void GetBattery(void);
void BatteryTest(void);
void InitBattery(void);

real32 BatteryVolts, BatteryCurrentADCEstimated, BatteryChargeUsedAH;
real32 BatteryCharge, BatteryCurrent;
real32 BatteryVoltsScale;

void GetBattery(void) {
    //  AttoPilot Voltage and Current Sense Breakout SparkFun Part: SEN-09028

    const real32 BatteryCurrentScale = 90.15296;    // Amps FS

    if ( F.HaveBatterySensor ) {
        BatteryCurrent = BatteryCurrentADC.read() * BatteryCurrentScale;
        BatteryChargeUsedAH  += BatteryCurrent * (real32)(mSClock() - mS[LastBattery]) * 2.777777e-7;
        mS[LastBattery] = mSClock();
    } else
        BatteryCurrent =  BatteryChargeUsedAH = 0;

    BatteryVolts = BatteryVoltsADC.read() *  BatteryVoltsScale;
    F.LowBatt = BatteryVolts < K[LowVoltThres];

} // GetBattery

void BatteryTest(void) {

    TxString("\r\nBattery test\r\n");

    GetBattery();

    // Battery
    TxString("Volts  :\t");
    TxVal32(BatteryVolts * 10.0, 1, 'V');
    TxString(" Limit > ");
    TxVal32( K[LowVoltThres] * 10.0, 1, 'V');

    if ( F.HaveBatterySensor ) {
        TxString("\r\nCurrent:\t");
        TxVal32(BatteryCurrent * 10.0, 1, 'A');
        TxString(" ( ");
        TxVal32(BatteryChargeUsedAH * 1000.0, 0, 0 );
        TxString(" mAH )\r\n");
    } else
        TxString("\r\nCurrent:\tnot available - no battery sensor\r\n");

} // BatteryTest

void InitBattery() {

    F.HaveBatterySensor = true;
    GetBattery();
    F.HaveBatterySensor =  BatteryCurrent < 2.0;
    if ( F.HaveBatterySensor )
        BatteryVoltsScale = 51.8144;       // Volts FS
    else
        BatteryVoltsScale = BATTERY_VOLTS_SCALE;

} // InitBattery

//_____________________________________________________________________

void GetRangefinderAltitude(void);
void InitRangefinder(void);

real32 RangefinderAltitude;

const real32 RangefinderScale = 10.24; // Metres FS

void GetRangefinderAltitude(void) {

    if ( F.RangefinderAltitudeValid ) {
        RangefinderAltitude = RangefinderADC.read() * RangefinderScale;
        if ( F.RFInInches )
            RangefinderAltitude *= 2.54;

        if (( RangefinderAltitude < ALT_RF_ENABLE_M ) && !F.UsingRangefinderAlt)
            F.UsingRangefinderAlt = true;
        else
            if (( RangefinderAltitude > ALT_RF_DISABLE_M ) && F.UsingRangefinderAlt)
                F.UsingRangefinderAlt = false;
    } else {
        RangefinderAltitude = 0.0;
        F.UsingRangefinderAlt = false;
    }
} // GetRangefinderAltitude

void InitRangefinder(void) {

    F.RangefinderAltitudeValid = true;
    GetRangefinderAltitude();
    F.RangefinderAltitudeValid = RangefinderAltitude < 1.0; // => supply not RF
    GetRangefinderAltitude();

} // InitRangefinder
