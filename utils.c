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

void InitMisc(void);
void Delay1mS(int16);
void Delay100mS(int16);
void DoBeep100mS(uint8, uint8);
void DoStartingBeeps(uint8);
void CheckAlarms(void);
real32 SlewLimit(real32, real32, real32);
real32 DecayX(real32, real32);
real32 LPFilter(real32, real32, real32, real32);
void Timing(uint8, uint32);

TimingRec Times[(UnknownT+1)];

void InitMisc(void) {
    int8 i;

    State = Starting;                // For trace preconditions

    for ( i = 0; i <= UnknownT; i++ )
        Times[i].T = Times[i].Count = 0;

    for ( i = 0; i < FLAG_BYTES ; i++ )
        F.AllFlags[i] = false;

    F.ParametersValid = F.AcquireNewPosition = F.AllowNavAltitudeHold = true;

#ifdef SIMULATE
    F.Simulation = true;
#endif // SIMULATE

    BatteryCharge = 0;

    IdleThrottle = ((10L*OUT_MAXIMUM)/100);
    InitialThrottle = RC_MAXIMUM;
    ESCMin = OUT_MINIMUM;
    ESCMax = OUT_MAXIMUM;

    ALL_LEDS_OFF;
    LEDRed_ON;
    Beeper_OFF;
} // InitMisc

void Delay1mS(int16 d) {
    static int32 Timeout;

    Timeout = timer.read_us() + ((int32)d * 1000 );
    while (  timer.read_us() < Timeout ) {};

} // Delay1mS

void Delay100mS(int16 d) {
    Delay1mS( 100 * d );
} // Delay100mS

void DoBeep100mS(uint8 t, uint8 d) {
    Beeper_ON;
    Delay100mS(t);
    Beeper_OFF;
    Delay100mS(d);
} // DoBeep100mS

void DoStartingBeeps(uint8 b) {
    uint8 i;

    for ( i = 0; i < b; i++ )
        DoBeep100mS(2, 8);

    DoBeep100mS(8,0);
} // DoStartingBeeps

void CheckAlarms(void) {

    static uint16 BeeperOnTime, BeeperOffTime;

    F.BeeperInUse = F.LowBatt || F.LostModel  || (State == Shutdown);

    if ( F.BeeperInUse ) {
        if ( F.LowBatt ) {
            BeeperOffTime = 750;
            BeeperOnTime = 250;
        } else
            if ( State == Shutdown ) {
                BeeperOffTime = 4750;
                BeeperOnTime = 250;
            } else
                if ( F.LostModel ) {
                    BeeperOffTime = 125;
                    BeeperOnTime = 125;
                }

        if ( (mSClock() > mS[BeeperUpdate]) && BEEPER_IS_ON ) {
            mS[BeeperUpdate] = mSClock() + BeeperOffTime;
            Beeper_OFF;
            LEDRed_OFF;
        } else
            if ( (mSClock() > mS[BeeperUpdate]) && BEEPER_IS_OFF ) {
                mS[BeeperUpdate] = mSClock() + BeeperOnTime;
                Beeper_ON;
                LEDRed_ON;
            }
    }
#ifdef NAV_ACQUIRE_BEEPER
    else
        if ( (State == InFlight) && (!F.AcquireNewPosition) && (mSClock() > mS[BeeperTimeout]) )
            Beeper_OFF;
#endif // NAV_ACQUIRE_BEEPER 

} // CheckAlarms

real32 DecayX(real32 i, real32 d) {
    if ( i < 0 ) {
        i += d;
        if ( i >0 )
            i = 0;
    } else
        if ( i > 0 ) {
            i -= d;
            if ( i < 0 )
                i = 0;
        }
    return (i);
} // DecayX

real32 LPFilter(real32 i, real32 ip, real32 FilterA, real32 dT) {
    return ( ip + (i - ip) * FilterA );
} // LPFilter

real32 SlewLimit(real32 Old, real32 New, real32 Slew) {
    real32 Low, High;

    Low = Old - Slew;
    High = Old + Slew;
    return(( New < Low ) ? Low : (( New > High ) ? High : New));
} // SlewLimit

void Timing(uint8 w, uint32 T) {

    Times[w].T += timer.read_us() - T;
    Times[w].Count++;

} // Timing

