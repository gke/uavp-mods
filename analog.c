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

void DirectAnalog(void) {
// Framework Simon Ford

    static uint32 data;



    // Select channel and start conversion
    LPC_ADC->ADCR &= ~0xFF;
    LPC_ADC->ADCR |= 1 << 5; // ADC0[5]
    LPC_ADC->ADCR |= 1 << 24;

    // Repeatedly get the sample data until DONE bit
    do {
        data = LPC_ADC->ADGDR;
    } while ((data & ((uint32)1 << 31)) == 0);

    // Stop conversion
    LPC_ADC->ADCR &= ~(1 << 24);

} // DirectAnalog

void InitDirectAnalog(void) {

// power on, clk divider /4
    LPC_SC->PCONP |= (1 << 12);
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);

    // software-controlled ADC settings
    LPC_ADC->ADCR = (0 << 0) // SEL: 0 = no channels selected
                    | (25 << 8)    // CLKDIV: PCLK max ~= 25MHz, /25 to give safe 1MHz
                    | (0 << 16)    // BURST: 0 = software control
                    | (0 << 17)    // CLKS: not applicable
                    | (1 << 21)    // PDN: 1 = operational
                    | (0 << 24)    // START: 0 = no start
                    | (0 << 27);   // EDGE: not applicable

    // setup P1_31 as sel 3 (ADC), mode 2 (no pull)
    LPC_PINCON->PINSEL3 &= ~((uint32)0x3 << 30);
    LPC_PINCON->PINSEL3 |= (uint32)0x3 << 30;

    LPC_PINCON->PINMODE3 &= ~((uint32)0x3 << 30);
    LPC_PINCON->PINMODE3 |= (uint32)0x2 << 30;
    
} // InitDirectAnalog

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
