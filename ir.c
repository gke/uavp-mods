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

// IR Sensors ( Scheme from "Leveller" Autopilot ~2005 )

void GetIRAttitude(void);
void TrackIRMaxMin(real32);
void InitIRSensors(void);

// FMA Roll/Pitch connector Pin1 Gnd, Pin 2 3.3V, Pin3 -Pitch, Pin4 Roll.
// FMA Z-Axis     connector Pin1 Gnd, Pin 2 3.3V, Pin3 Z , Pin4 Unused.
// DIYDrones      connector Pin1 Gnd, Pin 2 3.3V, Pin3 Z, Pin4 -Pitch, Pin5 Roll.

real32 IR[3], IRMax, IRMin, IRSwing;


void TrackIRMaxMin(real32 m) {
    if ( m > IRMax ) IRMax = m;
    else
        if ( m < IRMin ) IRMin = m;
        
    IRSwing = Max( IRSwing, fabs(m) );
    IRSwing -= 0.00001; // zzz

} // TrackIRMaxMin

void GetIRAttitude() {
    #define IR_NEUTRAL 1.0
    static uint8 i;

    if ( GyroType == IRSensors ) {
    
    IR[Pitch] = -PitchADC.read() * 2.0 - IR_NEUTRAL;
    IR[Roll] = RollADC.read() * 2.0 - IR_NEUTRAL;
    IR[Yaw] = YawADC.read() * 2.0 - IR_NEUTRAL;

    for ( i = 0; i < (uint8)3; i++ )
        TrackIRMaxMin(IR[i]);

    for ( i = 0; i < (uint8)2; i++ )
        Angle[i] = asin ( Limit( IR[i] / IRSwing, -1.0, 1.0 ) );

    Angle[Yaw] = 0.0;
    Rate[Yaw] = 0.0;
    
    }

} // GetIRAttitude

void InitIRSensors() {
    static uint8 i;

    LEDYellow_ON;
    IRSwing = 0.5;

    IRMax = -1.0;
    IRMin= 1.0;

    for ( i = 0; i < 20; i++ ) {
        GetIRAttitude();
        Delay1mS(100);
    }

    LEDYellow_OFF;

} // InitIRSensors
