// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU
//    General Public License as published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/

#include "UAVX.h"

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

real32 AngleE[3], AngleIntE[3];

real32 GS;
real32 Rl, Pl, Yl, Ylp;
int16 HoldYaw;
int16 CruiseThrottle, MaxCruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
int16 DesiredRoll, DesiredPitch, DesiredYaw, DesiredHeading, DesiredCamPitchTrim;
real32 ControlRoll, ControlPitch, ControlRollP, ControlPitchP;
int16 CurrMaxRollPitch;

int16 AttitudeHoldResetCount;
real32 AltDiffSum, AltD, AltDSum;
real32 DesiredAltitude, Altitude, AltitudeP;
real32 ROC;

uint32 AltuSp;
int16 DescentLimiter;

real32 GRollKp, GRollKi, GRollKd, GPitchKp, GPitchKi, GPitchKd;

boolean    FirstPass;
int8 BeepTick = 0;

void DoAltitudeHold(void) { // Syncronised to baro intervals independant of active altitude source

    static int16 NewAltComp;
    static real32 AltP, AltI, AltD;
    static real32 LimAltE, AltE;
    static real32 AltdT, AltdTR;
    static uint32 Now;

#ifdef ALT_SCRATCHY_BEEPER
    if ( (--BeepTick <= 0) && !F.BeeperInUse )
        Beeper_TOG;
#endif
    Now = uSClock();
    AltdT = ( Now - AltuSp ) * 0.000001;
    AltdT = Limit(AltdT, 0.01, 0.1); // limit range for restarts
    AltdTR = 1.0 / AltdT;
    AltuSp = Now;

    AltE = DesiredAltitude - Altitude;
    LimAltE = Limit(AltE, -ALT_BAND_M, ALT_BAND_M);

    AltP = LimAltE * K[AltKp];
    AltP = Limit(AltP, -ALT_MAX_THR_COMP, ALT_MAX_THR_COMP);

    AltDiffSum += LimAltE;
    AltDiffSum = Limit(AltDiffSum, -ALT_INT_WINDUP_LIMIT, ALT_INT_WINDUP_LIMIT);
    AltI = AltDiffSum * K[AltKi] * AltdT;
    AltI = Limit(AltDiffSum, -K[AltIntLimit], K[AltIntLimit]);

    ROC = ( Altitude - AltitudeP ) * AltdTR; // may neeed filtering - noisy
    AltitudeP = Altitude;

    AltD = ROC * K[AltKd];
    AltD = Limit(AltD, -ALT_MAX_THR_COMP, ALT_MAX_THR_COMP);

    if ( ROC < ( -K[MaxDescentRateDmpS] * 10.0 ) ) {
        DescentLimiter += 1;
        DescentLimiter = Limit(DescentLimiter, 0, ALT_MAX_THR_COMP * 2.0);

    } else
        DescentLimiter = DecayX(DescentLimiter, 1);

    NewAltComp = AltP + AltI + AltD + AltDSum + DescentLimiter;

    NewAltComp = Limit(NewAltComp, -ALT_MAX_THR_COMP, ALT_MAX_THR_COMP);

    Comp[Alt] = SlewLimit(Comp[Alt], NewAltComp, 1.0);


    if ( ROC > Stats[MaxROCS] )
        Stats[MaxROCS] = ROC;
    else
        if ( ROC < Stats[MinROCS] )
            Stats[MinROCS] = ROC;

#ifdef ALT_SCRATCHY_BEEPER
    if ( (BeepTick <= 0) && !F.BeeperInUse) {
        Beeper_TOG;
        BeepTick = 5;
    }
#endif

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
                Comp[LR] = Limit(Comp[LR], -DAMP_HORIZ_LIMIT, DAMP_HORIZ_LIMIT);
                Vel[LR] = DecayX(Vel[LR], K[HorizDampDecay]);

                // Back - Front
                Vel[BF] += Acc[BF] * dT;
                Comp[BF] = Vel[BF] * K[HorizDampKp];
                Comp[BF] = Limit(Comp[BF], -DAMP_HORIZ_LIMIT, DAMP_HORIZ_LIMIT);
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

} // DoOrientationTransform

void GainSchedule(boolean UseAngle) {

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

    GRollKp = K[RollKp];
    GRollKi = K[RollKi];
    GRollKd = K[RollKd];

    GPitchKp = K[PitchKp];
    GPitchKi = K[PitchKi];
    GPitchKd = K[PitchKd];

} // GainSchedule

void DoControl(void) {
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

    GainSchedule(F.UsingAngleControl);

#ifdef DISABLE_EXTRAS
    // for commissioning
    Comp[BF] = Comp[LR] = Comp[UD] = Comp[Alt] = 0;
    NavCorr[Roll] = NavCorr[Pitch] = NavCorr[Yaw] = 0;

    F.UsingAngleControl = false;

#endif // DISABLE_EXTRAS

    if ( F.UsingAngleControl ) {
        // Roll

        AngleE[Roll] = ( ControlRoll * ATTITUDE_SCALE ) - Angle[Roll];
        AngleIntE[Roll] += AngleE[Roll] * dT;
        AngleIntE[Roll] = Limit(AngleIntE[Roll], -K[RollIntLimit], K[RollIntLimit]);
        Rl  = -(AngleE[Roll] * GRollKp + AngleIntE[Roll] * GRollKi + Rate[Roll] * GRollKd * dTR);
        Rl -=  NavCorr[Roll] - Comp[LR];

        // Pitch

        AngleE[Pitch] = ( ControlPitch * ATTITUDE_SCALE ) - Angle[Pitch];
        AngleIntE[Pitch] += AngleE[Pitch] * dT;
        AngleIntE[Pitch] = Limit(AngleIntE[Pitch], -K[PitchIntLimit], K[PitchIntLimit]);
        Pl  = -(AngleE[Pitch] * GPitchKp + AngleIntE[Pitch] * GPitchKi + Rate[Pitch] * GPitchKd * dTR);
        Pl -= NavCorr[Pitch] - Comp[BF];

    } else {
        // Roll

        AngleE[Roll] = Limit(Angle[Roll],  -K[RollIntLimit], K[RollIntLimit]);
        Rl  = Rate[Roll] * GRollKp + AngleE[Roll] * GRollKi + (Rate[Roll]-Ratep[Roll]) * GRollKd * dTR;
        Rl -=  NavCorr[Roll] - Comp[LR];

        Rl *= GS;

        Rl -= ControlRoll;

        ControlRollP = ControlRoll;
        Ratep[Roll] = Rate[Roll];

        // Pitch

        AngleE[Pitch] = Limit(Angle[Pitch],  -K[PitchIntLimit], K[PitchIntLimit]);
        Pl  = Rate[Pitch] * GPitchKp + AngleE[Pitch] * GPitchKi + (Rate[Pitch]-Ratep[Pitch]) * GPitchKd * dTR;
        Pl -= NavCorr[Pitch] - Comp[BF];

        Pl *= GS;

        Pl -= ControlPitch;

        ControlPitchP = ControlPitch;
        Ratep[Pitch] = Rate[Pitch];
    }

    // Yaw

    Rate[Yaw] -= NavCorr[Yaw];
    if ( abs(DesiredYaw) > 5 )
        Rate[Yaw] -= DesiredYaw;

    Yl  = Rate[Yaw] * K[YawKp] + Angle[Yaw] * K[YawKi] + (Rate[Yaw]-Ratep[Yaw]) * K[YawKd] * dTR;
    Ratep[Yaw] = Rate[Yaw];

#ifdef TRICOPTER
    Yl = SlewLimit(Ylp, Yl, 2.0);
    Ylp = Yl;
    Yl = Limit(Yl, -K[YawLimit] * 4.0, K[YawLimit] * 4.0);
#else
    Yl = Limit(Yl, -K[YawLimit], K[YawLimit]);
#endif // TRICOPTER

#endif // SIMULATE        

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
    } while ((!F.Signal) || (Armed && FirstPass) || F.Ch5Active ||
             // F.GyroFailure ||
             // (!F.AccelerationsValid) ||
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
        AngleE[i] = AngleIntE[i] = Angle[i] = Anglep[i] = Rate[i] = Trim[i] = Vel[i] = Comp[i] = 0.0;

    Comp[Alt] = AltSum = Ylp = ControlRollP = ControlPitchP = AltitudeP = 0.0;

} // InitControl
