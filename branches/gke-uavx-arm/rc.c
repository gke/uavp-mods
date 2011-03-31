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

void DoRxPolarity(void);
void InitRC(void);
void MapRC(void);
void CheckSticksHaveChanged(void);
void UpdateControls(void);
void CaptureTrims(void);
void CheckThrottleMoved(void);
void ReceiverTest(void);

const uint8 Map[CustomTxRx+1][CONTROLS] = {
    { 2,0,1,3,4,5,6 },     // Futaba Thr 3 Throttle
    { 1,0,3,2,4,5,6 },    // Futaba Thr 2 Throttle
    { 4,2,1,0,5,3,6 },    // Futaba 9C Spektrum DM8/AR7000
    { 0,1,2,3,4,5,6 },    // JR XP8103/PPM
    { 6,0,3,5,2,4,1 },    // JR 9XII Spektrum DM9 ?

    { 5,0,3,6,2,1,4 },    // JR DXS12 
    { 5,0,3,6,2,1,4 },    // Spektrum DX7/AR7000
    { 4,0,3,5,2,1,6 },    // Spektrum DX7/AR6200

    { 2,0,1,3,4,6,5 },     // Futaba Thr 3 Sw 6/7
    { 0,1,2,3,4,5,6 },    // Spektrum DX7/AR6000
    { 0,1,2,3,4,5,6 },    // Graupner MX16S
    { 4,0,2,3,1,5,6 },    // Spektrum DX6i/AR6200
    { 2,0,1,3,4,5,6 },    // Futaba Th 3/R617FS
    { 4,0,2,3,5,1,6 },    // Spektrum DX7a/AR7000
    { 2,0,1,3,4,6,5 },     // External decoder (Futaba Thr 3 6/7 swap)
    { 0,1,2,3,4,5,6 },    // FrSky DJT/D8R-SP
    { 5,0,3,6,2,1,4 },    // UNDEFINED Spektrum DX7/AR7000
    { 2,0,1,3,4,5,6 }    // Custom
//{ 4,0,2,1,3,5,6 }    // Custom
    };

// Rx signalling polarity used only for serial PPM frames usually
// by tapping internal Rx circuitry.
const boolean PPMPosPolarity[CustomTxRx+1] =
    {
        false,     // Futaba Ch3 Throttle
        false,    // Futaba Ch2 Throttle
        true,    // Futaba 9C Spektrum DM8/AR7000
        true,    // JR XP8103/PPM
        true,    // JR 9XII Spektrum DM9/AR7000

        true,    // JR DXS12
        true,    // Spektrum DX7/AR7000
        true,    // Spektrum DX7/AR6200
        false,    // Futaba Thr 3 Sw 6/7
        true,    // Spektrum DX7/AR6000
        true,    // Graupner MX16S
        true,    // Graupner DX6i/AR6200
        true,    // Futaba Thr 3/R617FS
        true,     // Spektrum DX7a/AR7000
        false,     // External decoder (Futaba Ch3 Throttle)
        true,    // FrSky DJT/D8R-SP
        true,    // UNKNOWN using Spektrum DX7/AR7000
        true    // custom Tx/Rx combination
    };

// Reference Internal Quadrocopter Channel Order
// 0 Throttle
// 1 Aileron
// 2 Elevator
// 3 Rudder
// 4 Gear
// 5 Aux1
// 6 Aux2

int8 RMap[CONTROLS];

int16x8x4Q PPMQ;
int16 PPMQSum[CONTROLS];
int16 RC[CONTROLS], RCp[CONTROLS];
int16 ThrLow, ThrNeutral, ThrHigh;

boolean RCPositiveEdge;

void DoRxPolarity(void) {

    RCInterrupt.rise(&RCNullISR);
    RCInterrupt.fall(&RCNullISR);

    if ( F.UsingSerialPPM  ) // serial PPM frame from within an Rx
        if  ( PPMPosPolarity[P[TxRxType]] )
            RCInterrupt.rise(&RCISR);
        else
            RCInterrupt.fall(&RCISR);
    else {
        RCInterrupt.rise(&RCISR);
        RCInterrupt.fall(&RCISR);
    }

}  // DoRxPolarity

void InitRC(void) {
    static int8 c, q;

    DoRxPolarity();

    SignalCount = -RC_GOOD_BUCKET_MAX;
    F.Signal = F.RCNewValues = false;

    for (c = 0; c < RC_CONTROLS; c++) {
        PPM[c] = 0;
        RC[c] = RCp[c] = RC_NEUTRAL;

        for (q = 0; q <= PPMQMASK; q++)
            PPMQ.B[q][c] = RC_NEUTRAL;
        PPMQSum[c] = RC_NEUTRAL * 4;
    }
    PPMQ.Head = 0;

    DesiredRoll = DesiredPitch = DesiredYaw = DesiredThrottle = StickThrottle = 0;
    Trim[Roll] = Trim[Pitch] = Trim[Yaw] = 0;

    PPM_Index = PrevEdge = RCGlitches = 0;
} // InitRC

void MapRC(void) { // re-maps captured PPM to Rx channel sequence
    static int8 c;
    static int16 LastThrottle, i;
    static uint16 Sum;

    LastThrottle = RC[ThrottleC];

    for ( c = 0 ; c < RC_CONTROLS ; c++ ) {
        Sum = PPM[c];
        PPMQSum[c] -= PPMQ.B[PPMQ.Head][c];
        PPMQ.B[PPMQ.Head][c] = Sum;
        PPMQSum[c] += Sum;
        PPMQ.Head = (PPMQ.Head + 1) & PPMQMASK;
    }

    for ( c = 0 ; c < RC_CONTROLS ; c++ ) {
        i = Map[P[TxRxType]][c];
        RC[c] = ( PPMQSum[i] * RC_MAXIMUM ) / 4000; // needs rethink - throwing away precision
    }

    if ( THROTTLE_SLEW_LIMIT > 0 )
        RC[ThrottleRC] = SlewLimit(LastThrottle, RC[ThrottleRC], THROTTLE_SLEW_LIMIT);

} // MapRC

void CheckSticksHaveChanged(void) {
    static boolean Change;
    static uint8 c;

    if ( F.ReturnHome || F.Navigate  ) {
        if ( mSClock() > mS[StickChangeUpdate] ) {
            mS[StickChangeUpdate] = mSClock() + 500;
            if ( !F.ForceFailsafe && ( State == InFlight )) {
                Stats[RCFailsafesS]++;
                mS[NavStateTimeout] = mSClock() + NAV_RTH_LAND_TIMEOUT_MS;
                mS[DescentUpdate]  = mSClock() + ALT_DESCENT_UPDATE_MS;
                DescentComp = 0; // for no Baro case
            }

            F.ForceFailsafe = State == InFlight; // abort if not navigating
        }
    } else {
        if ( mSClock() > mS[StickChangeUpdate] ) {
            mS[StickChangeUpdate] = mSClock() + 500;

            Change = false;
            for ( c = ThrottleC; c <= (uint8)RTHC; c++ ) {
                Change |= abs( RC[c] - RCp[c]) > RC_STICK_MOVEMENT;
                RCp[c] = RC[c];
            }
        }

        if ( Change ) {
            mS[RxFailsafeTimeout] = mSClock() + RC_NO_CHANGE_TIMEOUT_MS;
            mS[NavStateTimeout] = mSClock();
            F.ForceFailsafe = false;
            if ( FailState == MonitoringRx ) {
                if ( F.LostModel ) {
                    Beeper_OFF;
                    F.LostModel = false;
                    DescentComp = 0;
                }
            }
        } else
            if ( mSClock() > mS[RxFailsafeTimeout] ) {
                if ( !F.ForceFailsafe && ( ( State == InFlight ) || ( ( mSClock() - mS[RxFailsafeTimeout])  > 120000 ) ) ) {
                    Stats[RCFailsafesS]++;
                    mS[NavStateTimeout] = mSClock() + NAV_RTH_LAND_TIMEOUT_MS;
                    mS[DescentUpdate]  = mSClock() + ALT_DESCENT_UPDATE_MS;
                    DescentComp = 0; // for no Baro case
                    F.ForceFailsafe = true;
                }

                //    F.ForceFailsafe = State == InFlight; // abort if not navigating
            }
    }

} // CheckSticksHaveChanged

void UpdateControls(void) {
    static int16 HoldRoll, HoldPitch, RollPitchScale;
    static boolean NewCh5Active;

    F.RCNewValues = false;

    MapRC();                                // remap channel order for specific Tx/Rx

    StickThrottle = RC[ThrottleRC];

    //_________________________________________________________________________________________

    // Navigation

    F.ReturnHome = F.Navigate = F.UsingPolar = false;
    NewCh5Active = RC[RTHRC] > RC_NEUTRAL;

    if ( F.UsingPositionHoldLock )
        if ( NewCh5Active & !F.Ch5Active )
            F.AllowTurnToWP = true;
        else
            F.AllowTurnToWP = SaveAllowTurnToWP;
    else
        if ( RC[RTHRC] > ((3L*RC_MAXIMUM)/4) )
            F.ReturnHome = true;
        else
            if ( RC[RTHRC] > (RC_NEUTRAL/2) )
                F.Navigate = true;

    F.Ch5Active = NewCh5Active;

#ifdef RX6CH
    DesiredCamPitchTrim = RC_NEUTRAL;
    // NavSensitivity set in ReadParametersPX
#else
    DesiredCamPitchTrim = RC[CamPitchRC] - RC_NEUTRAL;
    NavSensitivity = RC[NavGainRC];
    NavSensitivity = Limit(NavSensitivity, 0, RC_MAXIMUM);
#endif // !RX6CH

    //_________________________________________________________________________________________

    // Altitude Hold

    F.AltHoldEnabled = NavSensitivity > NAV_SENS_ALTHOLD_THRESHOLD;

    if ( NavState == HoldingStation ) { // Manual
        if ( StickThrottle < RC_THRES_STOP )    // to deal with usual non-zero EPA
            StickThrottle = 0;
    } else // Autonomous
        if ( F.AllowNavAltitudeHold &&  F.AltHoldEnabled )
            StickThrottle = CruiseThrottle;

    if ( (! F.HoldingAlt) && (!(F.Navigate || F.ReturnHome )) ) // cancel any current altitude hold setting
        DesiredAltitude = Altitude;

    //_________________________________________________________________________________________

    // Attitude

#ifdef ATTITUDE_NO_LIMITS
    DesiredRoll = RC[RollRC] - RC_NEUTRAL;
    DesiredPitch = RC[PitchRC] - RC_NEUTRAL;
#else
    RollPitchScale = MAX_ROLL_PITCH - (NavSensitivity >> 2);
    DesiredRoll = SRS16((RC[RollRC] - RC_NEUTRAL) * RollPitchScale, 7);
    DesiredPitch = SRS16((RC[PitchRC] - RC_NEUTRAL) * RollPitchScale, 7);
#endif // ATTITUDE_NO_LIMITS
    DesiredYaw = RC[YawRC] - RC_NEUTRAL;
    
    AdaptiveYawLPFreq();

    HoldRoll = abs(DesiredRoll - Trim[Roll]);
    HoldPitch = abs(DesiredPitch - Trim[Pitch]);
    CurrMaxRollPitch = Max(HoldRoll, HoldPitch);

    if ( CurrMaxRollPitch > ATTITUDE_HOLD_LIMIT )
        if ( AttitudeHoldResetCount > ATTITUDE_HOLD_RESET_INTERVAL )
            F.AttitudeHold = false;
        else {
            AttitudeHoldResetCount++;
            F.AttitudeHold = true;
        }
    else {
        F.AttitudeHold = true;
        if ( AttitudeHoldResetCount > 1 )
            AttitudeHoldResetCount -= 2;        // Faster decay
    }

    //_________________________________________________________________________________________

    // Rx has gone to failsafe

    //zzz  CheckSticksHaveChanged();

    F.NewCommands = true;

} // UpdateControls

void CaptureTrims(void)
{     // only used in detecting movement from neutral in hold GPS position
    // Trims are invalidated if Nav sensitivity is changed - Answer do not use trims ?
    #ifndef TESTING
    Trim[Roll] = Limit1(DesiredRoll, NAV_MAX_TRIM);
    Trim[Yaw] = Limit1(DesiredPitch, NAV_MAX_TRIM);
    Trim[Yaw] = Limit1(DesiredYaw, NAV_MAX_TRIM);
    #endif // TESTING

    HoldYaw = 0;
} // CaptureTrims

void CheckThrottleMoved(void) {
    if ( mSClock() < mS[ThrottleUpdate] )
        ThrNeutral = DesiredThrottle;
    else {
        ThrLow = ThrNeutral - THROTTLE_MIDDLE;
        ThrLow = Max(ThrLow, THROTTLE_MIN_ALT_HOLD);
        ThrHigh = ThrNeutral + THROTTLE_MIDDLE;
        if ( ( DesiredThrottle <= ThrLow ) || ( DesiredThrottle >= ThrHigh ) ) {
            mS[ThrottleUpdate] = mSClock() + THROTTLE_UPDATE_MS;
            F.ThrottleMoving = true;
        } else
            F.ThrottleMoving = false;
    }
} // CheckThrottleMoved

void ReceiverTest(void) {
    static int8 s;

    TxString("\r\nRx Test \r\n");

    TxString("\r\nRx: ");
    ShowRxSetup();
    TxString("\r\n");

    TxString("RAW Rx frame values - neutrals NOT applied\r\n");
    TxString("Channel order is: ");
    for ( s = 0; s < RC_CONTROLS; s++)
        TxChar(RxChMnem[RMap[s]]);

    if ( F.Signal )
        TxString("\r\nSignal OK ");
    else
        TxString("\r\nSignal FAIL ");
    TxVal32(mSClock() - mS[LastValidRx], 0, 0);
    TxString(" mS ago\r\n");

    // Be wary as new RC frames are being received as this
    // is being displayed so data may be from overlapping frames

    for ( s = 0; s < RC_CONTROLS ; s++ ) {
        TxChar(s+'1');
        TxString(": ");
        TxChar(RxChMnem[RMap[s]]);
        TxString(":\t");

        TxVal32((int32)PPM[s] + 1000, 3, 0);
        TxChar(HT);
        TxVal32(PPM[s] / 10, 0, '%');
        if ( ( PPM[s] < -200 ) || ( PPM[s] > 1200 ) )
            TxString(" FAIL");

        TxNextLine();
    }

    TxString("Glitches:\t");
    TxVal32(RCGlitches,0,0);
    TxNextLine();

} // ReceiverTest



