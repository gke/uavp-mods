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

void DoAltitudeHold(void);
void UpdateAltitudeSource(void);
void AltitudeHold(void);
void InertialDamping(void);
void DoOrientationTransform(void);
void DoControl(void);

void CheckThrottleMoved(void);
void LightsAndSirens(void);
void InitControl(void);

real32 Angle[3], Anglep[3], Rate[3], Ratep[3]; // Milliradians
real32 Comp[4];
real32 DescentComp;

real32 GS;
real32 Rl, Pl, Yl, Ylp;
int16 HoldYaw;
int16 CruiseThrottle, MaxCruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
int16 DesiredRoll, DesiredPitch, DesiredYaw, DesiredCamPitchTrim;
real32 DesiredHeading;
real32 ControlRoll, ControlPitch;
real32 CameraRollAngle, CameraPitchAngle;
int16 CurrMaxRollPitch;
int16 Trim[3];

int16 AttitudeHoldResetCount;
real32 AltDiffSum, AltD, AltDSum;
real32 DesiredAltitude, Altitude, AltitudeP;
real32 ROC;

uint32 AltuSp;
int16 DescentLimiter;
uint32 ControlUpdateTimeuS;

real32 GRollKp, GRollKi, GRollKd, GPitchKp, GPitchKi, GPitchKd;

boolean    FirstPass;
int8 BeepTick = 0;

void DoAltitudeHold(void) { // Syncronised to baro intervals independant of active altitude source

    static int16 NewAltComp;
    static real32 AltP, AltI, AltD;
    static real32 LimAltE, AltE;
    static real32 AltdT, AltdTR;
    static uint32 Now;

    Now = uSClock();
    AltdT = ( Now - AltuSp ) * 0.000001;
    AltdT = Limit(AltdT, 0.01, 0.1); // limit range for restarts
    AltdTR = 1.0 / AltdT;
    AltuSp = Now;

    AltE = DesiredAltitude - Altitude;
    LimAltE = Limit1(AltE, ALT_BAND_M);

    AltP = LimAltE * K[AltKp];
    AltP = Limit1(AltP, ALT_MAX_THR_COMP);

    AltDiffSum += LimAltE;
    AltDiffSum = Limit1(AltDiffSum, ALT_INT_WINDUP_LIMIT);
    AltI = AltDiffSum * K[AltKi] * AltdT;
    AltI = Limit1(AltDiffSum, K[AltIntLimit]);

    ROC = ( Altitude - AltitudeP ) * AltdTR; // may neeed filtering - noisy
    AltitudeP = Altitude;

    AltD = ROC * K[AltKd];
    AltD = Limit1(AltD, ALT_MAX_THR_COMP);

    if ( ROC < ( -K[MaxDescentRateDmpS] * 10.0 ) ) {
        DescentLimiter += 1;
        DescentLimiter = Limit(DescentLimiter, 0, ALT_MAX_THR_COMP * 2.0);
    } else
        DescentLimiter = DecayX(DescentLimiter, 1);

    NewAltComp = AltP + AltI + AltD + AltDSum + DescentLimiter;
    NewAltComp = Limit1(NewAltComp, ALT_MAX_THR_COMP);
    Comp[Alt] = SlewLimit(Comp[Alt], NewAltComp, 1.0);

    if ( ROC > Stats[MaxROCS] )
        Stats[MaxROCS] = ROC;
    else
        if ( ROC < Stats[MinROCS] )
            Stats[MinROCS] = ROC;

} // DoAltitudeHold

void UpdateAltitudeSource(void) {
    if ( F.UsingRangefinderAlt )
        Altitude = RangefinderAltitude;
    else
        Altitude = BaroRelAltitude;

} // UpdateAltitudeSource

void AltitudeHold() {
    static int16 NewCruiseThrottle;

    GetBaroAltitude();
    GetRangefinderAltitude();
    CheckThrottleMoved();

    if ( F.AltHoldEnabled ) {
        if ( F.NewBaroValue  ) { // sync on Baro which MUST be working
            F.NewBaroValue = false;

            UpdateAltitudeSource();

            if ( ( NavState != HoldingStation ) && F.AllowNavAltitudeHold ) { // Navigating - using CruiseThrottle
                F.HoldingAlt = true;
                DoAltitudeHold();
            } else
                if ( F.ThrottleMoving ) {
                    F.HoldingAlt = false;
                    DesiredAltitude = Altitude;
                    Comp[Alt] = Decay1(Comp[Alt]);
                } else {
                    F.HoldingAlt = true;
                    if ( fabs(ROC) < ALT_HOLD_MAX_ROC_MPS  ) {
                        NewCruiseThrottle = DesiredThrottle + Comp[Alt];
                        CruiseThrottle = HardFilter(CruiseThrottle, NewCruiseThrottle);
                        CruiseThrottle = Limit( CruiseThrottle , IdleThrottle, MaxCruiseThrottle );
                    }
                    DoAltitudeHold();
                }
        }
    } else {
        Comp[Alt] = Decay1(Comp[Alt]);
        ROC = 0.0;
        F.HoldingAlt = false;
    }
} // AltitudeHold

void InertialDamping(void) { // Uses accelerometer to damp disturbances while holding altitude
    static uint8 i;

    if ( F.AccelerationsValid  && F.NearLevel ) {
        // Up - Down

        Vel[UD] += Acc[UD] * dT;
        Comp[UD] = Vel[UD] * K[VertDampKp];
        Comp[UD] = Limit(Comp[UD], DAMP_VERT_LIMIT_LOW, DAMP_VERT_LIMIT_HIGH);

        Vel[UD] = DecayX(Vel[UD], K[VertDampDecay]);

        // Lateral compensation only when holding altitude
        if ( F.HoldingAlt && F.AttitudeHold ) {
            if ( F.WayPointCentred ) {
                // Left - Right
                Vel[LR] += Acc[LR] * dT;
                Comp[LR] = Vel[LR] * K[HorizDampKp];
                Comp[LR] = Limit1(Comp[LR], DAMP_HORIZ_LIMIT);
                Vel[LR] = DecayX(Vel[LR], K[HorizDampDecay]);

                // Back - Front
                Vel[BF] += Acc[BF] * dT;
                Comp[BF] = Vel[BF] * K[HorizDampKp];
                Comp[BF] = Limit1(Comp[BF], DAMP_HORIZ_LIMIT);
                Vel[BF] = DecayX(Vel[BF], K[HorizDampDecay]);
            } else {
                Vel[LR] = Vel[BF] = 0;
                Comp[LR] = Decay1(Comp[LR]);
                Comp[BF] = Decay1(Comp[BF]);
            }
        } else {
            Vel[LR] = Vel[BF] = 0;

            Comp[LR] = Decay1(Comp[LR]);
            Comp[BF] = Decay1(Comp[BF]);
        }
    } else
        for ( i = 0; i < (uint8)3; i++ )
            Comp[i] = Vel[i] = 0.0;

} // InertialDamping

void DoOrientationTransform(void) {
    static real32 OSO, OCO;

    if ( F.UsingPolar ) {
        OSO = OSin[PolarOrientation];
        OCO = OCos[PolarOrientation];
    } else {
        OSO = OSin[Orientation];
        OCO = OCos[Orientation];
    }

    if ( !F.NavigationActive )
        NavCorr[Roll] = NavCorr[Pitch] = NavCorr[Yaw] = 0;

    // -PS+RC
    ControlRoll = (int16)( -DesiredPitch * OSO + DesiredRoll * OCO );

    // PC+RS
    ControlPitch = (int16)( DesiredPitch * OCO + DesiredRoll * OSO );

    CameraRollAngle = Angle[Pitch] * OSO + Angle[Roll] * OCO;
    CameraPitchAngle = Angle[Pitch] * OCO - Angle[Roll] * OSO;

} // DoOrientationTransform

void GainSchedule(void) {

    /*
    // rudimentary gain scheduling (linear)
    static int16 AttDiff, ThrDiff;

    if ( (!F.NavigationActive) || ( F.NavigationActive && (NavState == HoldingStation ) ) )
    {
        // also density altitude?

        if ( P[Acro] > 0) // due to Foliage 2009 and Alexinparis 2010
        {
             AttDiff = CurrMaxRollPitch  - ATTITUDE_HOLD_LIMIT;
            GS = GS * ( 1000.0 - (AttDiff * (int16)P[Acro]) );
            GS *= 0.001;
            GS = Limit(GS, 0, 1.0);
        }

        if ( P[GSThrottle] > 0 )
        {
             ThrDiff = DesiredThrottle - CruiseThrottle;
            GS = (int32)GS * ( 1000.0 + (ThrDiff * (int16)P[GSThrottle]) );
            GS *= 0.001;
        }
    }

    */

    GS = 1.0; // Temp



} // GainSchedule

const real32 RelayKcu = ( 4.0 * 10 )/ ( PI * 0.8235 ); // stimulus 10 => Kcu 15.5
const real32 RelayPd  = 2.0 * 1.285;
const real32 RelayStim = 3.0;

real32 RelayA = 0.0;
real32 RelayTau = 0.0;
uint32 RelayIteration = 0;
real32 RelayP, RelayW;

void DoRelayTuning(void) {

    static real32 Temp;

    Temp = fabs(Angle[Roll]);
    if ( Temp > RelayA ) RelayA = Temp;

    if ( ( RelayP < 0.0 ) && ( Angle[Roll] >= 0.0 ) ) {

        RelayTau = RelayIteration * dT;

        RelayP = - (PI * RelayA) / ( 4.0 * RelayStim );
        RelayW = (2.0 * PI) / RelayTau;

#ifndef PID_RAW
        SendPIDTuning();
#endif // PID_RAW

        RelayA = 0.0;

        RelayIteration = 0;
    }

    RelayIteration++;
    RelayP = Angle[Roll];

} // DoRelayTuning

void Relay(void) {

    if ( Angle[Roll] < 0.0 )
        Rl = -RelayStim;
    else
        Rl = +RelayStim;

    DesiredRoll = Rl;

} // Relay

void DoControl(void) {

    static real32 RateE;


    GetAttitude();
    AltitudeHold();
    InertialDamping();

#ifdef SIMULATE

    FakeDesiredRoll = DesiredRoll + NavRCorr;
    FakeDesiredPitch = DesiredPitch + NavPCorr;
    FakeDesiredYaw =  DesiredYaw + NavYCorr;
    Angle[Roll] = SlewLimit(Angle[Roll], FakeDesiredRoll * 16.0, 4.0);
    Angle[Pitch] = SlewLimit(Angle[Pitch], FakeDesiredPitch * 16.0, 4.0);
    Angle[Yaw] = SlewLimit(Angle[Yaw], FakeDesiredYaw, 4.0);
    Rl = FakeDesiredRoll;
    Pl = FakeDesiredPitch;
    Yl = DesiredYaw;

#else

    DoOrientationTransform();

    GainSchedule();

#ifdef DISABLE_EXTRAS
    // for commissioning
    Comp[BF] = Comp[LR] = Comp[UD] = Comp[Alt] = 0;
    NavCorr[Roll] = NavCorr[Pitch] = NavCorr[Yaw] = 0;
#endif // DISABLE_EXTRAS

    // Roll

#ifdef USE_ANGLE_DERIVED_RATE  // Gyro rate noisy so compute from angle
    RateE = ( Angle[Roll] - Anglep[Roll] ) * dTR;
#else
    RateE = Rate[Roll];
#endif // USE_ANGLE_DERIVED_RATE
    Rl  = RateE * GRollKp + Angle[Roll] * GRollKi + ( RateE - Ratep[Roll] ) * GRollKd * dTR;
    Rl -=  ( NavCorr[Roll] + Comp[LR] );

#ifdef TEST_RIG
/*
    if ( DesiredRoll > 5 )
        ControlRoll = 15;
    else
        ControlRoll = 0;
        */
#endif  // TEST_RIG

    Rl += ControlRoll;

#ifdef USE_ANGLE_DERIVED_RATE
    Ratep[Roll] = RateE;
    Anglep[Roll] = Angle[Roll];
#else
    Ratep[Roll] = Rate[Roll];
#endif // USE_ANGLE_DERIVED_RATE

    // Pitch

#ifdef USE_ANGLE_DERIVED_RATE // Gyro rate noisy so compute from angle
    RateE = ( Angle[Pitch] - Anglep[Pitch] ) * dTR;
#else
    RateE = Rate[Pitch];
#endif // USE_ANGLE_DERIVED_RATE
    Pl  = RateE * GPitchKp + Angle[Pitch] * GPitchKi + ( RateE - Ratep[Pitch] ) * GPitchKd * dTR;
    Pl -= ( NavCorr[Pitch] + Comp[BF] );
    Pl += ControlPitch;

#ifdef USE_ANGLE_DERIVED_RATE
    Ratep[Pitch] = RateE;
    Anglep[Pitch] = Angle[Pitch];
#else
    Ratep[Pitch] = Rate[Pitch];
#endif // USE_ANGLE_DERIVED_RATE    

    // Yaw

#define MAX_YAW_RATE  (HALFPI / RC_NEUTRAL)  // Radians/Sec e.g. HalfPI is 90deg/sec

    DoLegacyYawComp(AttitudeMethod); // returns Angle as heading error along with compensated rate

    Yl  = ( Rate[Yaw] + ( DesiredYaw + NavCorr[Yaw] ) * MAX_YAW_RATE ) * K[YawKp] +
          Angle[Yaw] * K[YawKi] + (Rate[Yaw]-Ratep[Yaw]) * K[YawKd] * dTR;

    Ratep[Yaw] = Rate[Yaw];

#ifdef TRICOPTER
    Yl = SlewLimit(Ylp, Yl, 2.0);
    Ylp = Yl;
    Yl = Limit1(Yl, K[YawLimit] * 4.0);
#else
    Yl = Limit1(Yl, K[YawLimit]); // currently 25 default
#endif // TRICOPTER

#endif // SIMULATE 

#ifdef TEST_RIG
    Pl = Yl = 0.0;
#endif // TEST_RIG

} // DoControl

static int8 RCStart = RC_INIT_FRAMES;

void LightsAndSirens(void) {
    static int24 Ch5Timeout;

    LEDYellow_TOG;
    if ( F.Signal ) LEDGreen_ON;
    else LEDGreen_OFF;

    Beeper_OFF;
    Ch5Timeout = mSClock() + 500;                     // mS.

    do {

        ProcessCommand();

        if ( F.Signal ) {
            LEDGreen_ON;
            if ( F.RCNewValues ) {
                UpdateControls();
                if ( --RCStart == 0 ) { // wait until RC filters etc. have settled
                    UpdateParamSetChoice();
                    MixAndLimitCam();
                    RCStart = 1;
                }

                InitialThrottle = StickThrottle;
                StickThrottle = 0;
                OutSignals();
                if ( mSClock() > Ch5Timeout ) {
                    if ( F.Navigate || F.ReturnHome || !F.ParametersValid ) {
                        Beeper_TOG;
                        LEDRed_TOG;
                    } else
                        if ( Armed )
                            LEDRed_TOG;

                    Ch5Timeout += 500;
                }
            }
        } else {
            LEDRed_ON;
            LEDGreen_OFF;
        }
        ReadParameters();
        GetIRAttitude(); // only active if IRSensors selected
    } while ((!F.Signal) || (Armed && FirstPass) || F.Ch5Active || F.GyroFailure || (!F.AccelerationsValid) ||
             ( InitialThrottle >= RC_THRES_START ) || (!F.ParametersValid) );

    FirstPass = false;

    Beeper_OFF;
    LEDRed_OFF;
    LEDGreen_ON;
    LEDYellow_ON;

    mS[LastBattery] = mSClock();
    mS[FailsafeTimeout] = mSClock() + FAILSAFE_TIMEOUT_MS;

    F.LostModel = false;
    FailState = MonitoringRx;

} // LightsAndSirens

void InitControl(void) {
    static uint8 i;

    AltuSp = DescentLimiter = 0;

    for ( i = 0; i < (uint8)3; i++ )
        Angle[i] = Anglep[i] = Rate[i] = Vel[i] = Comp[i] = 0.0;

    Comp[Alt] = AltSum = Ylp =  AltitudeP = 0.0;
    ControlUpdateTimeuS = 0;

} // InitControl

