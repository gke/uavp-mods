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

const real32 PWMScale = 1000.0 / OUT_MAXIMUM;

void DoMulticopterMix(real32);
void CheckDemand(real32);
void MixAndLimitMotors(void);
void MixAndLimitCam(void);
void OutSignals(void);
void InitI2CESCs(void);
void StopMotors(void);
void ExercisePWM(void);
void InitMotors(void);

boolean OutToggle;
real32 PWM[8];
real32 PWMSense[8];
int16 ESCI2CFail[6];
int16 CurrThrottle;
int16 CamRollPulseWidth, CamPitchPulseWidth;

int16 ESCMin, ESCMax;

#define PWM_PERIOD_US         (1000000/PWM_UPDATE_HZ)

#ifdef MULTICOPTER

uint8 TC(int16 T) {
    return ( Limit(T, ESCMin, ESCMax) );
} // TC

void DoMulticopterMix(real32 CurrThrottle) {
    static real32 Temp;

#ifdef Y6COPTER
    PWM[FrontTC] = PWM[LeftTC] = PWM[RightTC] = CurrThrottle;
#else
    PWM[FrontC] = PWM[LeftC] = PWM[RightC] = PWM[BackC] = CurrThrottle;
#endif

#ifdef TRICOPTER // usually flown K1 motor to the rear - use orientation of 24
    Temp = Pl * 0.5;
    PWM[FrontC] -= Pl;            // front motor
    PWM[LeftC] += (Temp - Rl);    // right rear
    PWM[RightC] += (Temp + Rl);   // left rear

    PWM[BackC] = -PWMSense[RudderC] * Yl + OUT_NEUTRAL;    // yaw servo
    if ( fabs(K[Balance]) > 0.5 )
        PWM[FrontC] =  PWM[FrontC] * K[Balance];
#else
#ifdef VTCOPTER     // usually flown VTail (K1+K4) to the rear - use orientation of 24
    Temp = Pl * 0.5;

    PWM[LeftC] += (Temp - Rl);    // right rear
    PWM[RightC] += (Temp + Rl); // left rear

    PWM[FrontLeftC] -= Pl - PWMSense[RudderC] * Yl;
    PWM[FrontRightC] -= Pl + PWMSense[RudderC] * Yl;
    if ( fabs(K[Balance]) > 0.01 ) {
        PWM[FrontLeftC] = PWM[FrontLeftC] * K[Balance];
        PWM[FrontRightC] = PWM[FrontRightC] * K[Balance];
    }
#else
#ifdef Y6COPTER

    Temp = Pl * 0.5;
    PWM[FrontTC] -= Pl;             // front motor
    PWM[LeftTC] += (Temp - Rl);     // right rear
    PWM[RightTC] += (Temp + Rl); // left rear

    PWM[FrontBC] = PWM[FrontTC];
    PWM[LeftBC]  = PWM[LeftTC];
    PWM[RightBC] = PWM[RightTC];

    if ( fabs(K[Balance]) > 0.01 ) {
        PWM[FrontTC] =  PWM[FrontTC] * K[Balance];
        PWM[FrontBC] = PWM[FrontTC];
    }

    Temp = Yl * 0.5;
    PWM[FrontTC] -= Temp;
    PWM[LeftTC]  -= Temp;
    PWM[RightTC] -= Temp;

    PWM[FrontBC] += Temp;
    PWM[LeftBC]  += Temp;
    PWM[RightBC] += Temp;

#else
    PWM[LeftC]  += -Rl + Yl;
    PWM[RightC] +=  Rl + Yl;
    PWM[FrontC] += -Pl - Yl;
    PWM[BackC]  +=  Pl - Yl;
#endif
#endif
#endif

} // DoMulticopterMix

boolean     MotorDemandRescale;

void CheckDemand(real32 CurrThrottle) {
    static real32 Scale, ScaleHigh, ScaleLow, MaxMotor, DemandSwing;

#ifdef Y6COPTER
    MaxMotor = Max(PWM[FrontTC], PWM[LeftTC]);
    MaxMotor = Max(MaxMotor, PWM[RightTC]);
    MaxMotor = Max(MaxMotor, PWM[FrontBC]);
    MaxMotor = Max(MaxMotor, PWM[LeftBC]);
    MaxMotor = Max(MaxMotor, PWM[RightBC]);
#else
    MaxMotor = Max(PWM[FrontC], PWM[LeftC]);
    MaxMotor = Max(MaxMotor, PWM[RightC]);
#ifndef TRICOPTER
    MaxMotor = Max(MaxMotor, PWM[BackC]);
#endif // TRICOPTER
#endif // Y6COPTER

    DemandSwing = MaxMotor - CurrThrottle;

    if ( DemandSwing > 0.0 ) {
        ScaleHigh = (OUT_MAXIMUM - CurrThrottle) / DemandSwing;
        ScaleLow = (CurrThrottle - IdleThrottle) / DemandSwing;
        Scale = Min(ScaleHigh, ScaleLow); // just in case!
        if ( Scale < 0.0 ) Scale = 1.0 / OUT_MAXIMUM;
        if ( Scale < 1.0 ) {
            MotorDemandRescale = true;
            Rl *= Scale;  // could get rid of the divides
            Pl *= Scale;
#ifndef TRICOPTER
            Yl *= Scale;
#endif // TRICOPTER 
        } else
            MotorDemandRescale = false;
    } else
        MotorDemandRescale = false;

} // CheckDemand

#endif // MULTICOPTER

void MixAndLimitMotors(void) {
    static real32 Temp, TempElevon, TempElevator;
    static uint8 m;

    if ( DesiredThrottle < IdleThrottle )
        CurrThrottle = 0;
    else
        CurrThrottle = DesiredThrottle;

#ifdef MULTICOPTER
    if ( State == InFlight )
        CurrThrottle += (-Comp[UD] + Comp[Alt]); // vertical compensation not optional

    Temp = OUT_MAXIMUM * 0.9; // 10% headroom for control
    CurrThrottle = Limit(CurrThrottle, 0, Temp );

    if ( CurrThrottle > IdleThrottle ) {
        DoMulticopterMix(CurrThrottle);

        CheckDemand(CurrThrottle);

        if ( MotorDemandRescale )
            DoMulticopterMix(CurrThrottle);
    } else {
#ifdef Y6COPTER
        for ( m = 0; m < (uint8)6; m++ )
            PWM[m] = CurrThrottle;
#else
        PWM[FrontC] = PWM[LeftC] = PWM[RightC] = CurrThrottle;
#ifdef TRICOPTER
        PWM[BackC] = -PWMSense[RudderC] * Yl + OUT_NEUTRAL;    // yaw servo
#else
        PWM[BackC] = CurrThrottle;
#endif // !TRICOPTER
#endif // Y6COPTER
    }
#else
    CurrThrottle += Comp[Alt]; // simple - faster to climb with no elevator yet

    PWM[ThrottleC] = CurrThrottle;
    PWM[RudderC] = -PWMSense[RudderC] * Yl + OUT_NEUTRAL;

#if ( defined AILERON | defined HELICOPTER )
    PWM[AileronC] = PWMSense[AileronC] * Rl + OUT_NEUTRAL;
    PWM[ElevatorC] = PWMSense[ElevatorC] * Pl + OUT_NEUTRAL;
#else // ELEVON
    TempElevator = PWMSense[2] * Pl + OUT_NEUTRAL;
    PWM[RightElevonC] = PWMSense[RightElevonC] * (TempElevator + Rl);
    PWM[LeftElevonC] = PWMSense[LeftElevonC] * (TempElevator -  Rl);
#endif
#endif

} // MixAndLimitMotors

void MixAndLimitCam(void) {
    static real32 Temp;

    // use only roll/pitch angle estimates
    Temp = Angle[Roll] * K[CamRollKp];
    PWM[CamRollC] = Temp + K[CamRollTrim];
    PWM[CamRollC] = PWMSense[CamRollC] * PWM[CamRollC] + OUT_NEUTRAL;

    Temp = Angle[Pitch] * K[CamPitchKp];
    PWM[CamPitchC] = Temp + DesiredCamPitchTrim;
    PWM[CamPitchC] = PWMSense[CamPitchC] * PWM[CamPitchC] + OUT_NEUTRAL;

} // MixAndLimitCam

#if ( defined Y6COPTER )
#include "outputs_y6.h"
#else
#if ( defined TRICOPTER | defined MULTICOPTER | defined VTCOPTER )
#include "outputs_copter.h"
#else
#include "outputs_conventional.h"
#endif // Y6COPTER
#endif // TRICOPTER | MULTICOPTER

void InitI2CESCs(void) {
    static int8 m;
    static uint8 r;

#ifdef MULTICOPTER

    if ( P[ESCType] ==  ESCHolger )
        for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
            I2CESC.start();
            r = I2CESC.write(0x52 + ( m*2 ));        // one cmd, one data byte per motor
            r += I2CESC.write(0);
            ESCI2CFail[m] += r;
            I2CESC.stop();
        }
    else
        if ( P[ESCType] == ESCYGEI2C )
            for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
                I2CESC.start();
                r = I2CESC.write(0x62 + ( m*2 ));    // one cmd, one data byte per motor
                r += I2CESC.write(0);
                ESCI2CFail[m] += r;
                I2CESC.stop();
            }
        else
            if ( P[ESCType] == ESCX3D ) {
                I2CESC.start();
                r = I2CESC.write(0x10);            // one command, 4 data bytes
                r += I2CESC.write(0);
                r += I2CESC.write(0);
                r += I2CESC.write(0);
                r += I2CESC.write(0);
                ESCI2CFail[0] += r;
                I2CESC.stop();
            }
#endif // MULTICOPTER
} // InitI2CESCs

void StopMotors(void) {
#ifdef MULTICOPTER
#ifdef Y6COPTER
    PWM[FrontTC] = PWM[LeftTC] = PWM[RightTC] =
    PWM[FrontBC] = PWM[LeftBC] = PWM[RightBC] = ESCMin;
#else
    PWM[FrontC] = PWM[LeftC] = PWM[RightC] = ESCMin;
#ifndef TRICOPTER
    PWM[BackC] = ESCMin;
#endif // !TRICOPTER
#endif // Y6COPTER    
#else
    PWM[ThrottleC] = ESCMin;
#endif // MULTICOPTER

    Out0.pulsewidth_us(1000 + (int16)( PWM[FrontC] * PWMScale ) );
    Out1.pulsewidth_us(1000 + (int16)( PWM[LeftC] * PWMScale ) );
    Out2.pulsewidth_us(1000 + (int16)( PWM[RightC] * PWMScale ) );
    Out3.pulsewidth_us(1000 + (int16)( PWM[BackC] * PWMScale ) );

    CamRollPulseWidth = 1000 + (int16)( PWM[CamRollC] * PWMScale );
    CamPitchPulseWidth = 1000 + (int16)( PWM[CamPitchC] * PWMScale );
    
#ifndef SOFTWARE_CAM_PWM 
    Out4.pulsewidth_us(CamRollPulseWidth);
    Out5.pulsewidth_us(CamPitchPulseWidth);
#endif // !SOFTWARE_CAM_PWM

    F.MotorsArmed = false;
} // StopMotors

void InitMotors(void) {
    static uint8 m;

    Out0.period_us(PWM_PERIOD_US);

    StopMotors();

#ifndef Y6COPTER
#ifdef TRICOPTER
    PWM[BackC] = OUT_NEUTRAL;
#endif // !TRICOPTER    
    PWM[CamRollC] = OUT_NEUTRAL;
    PWM[CamPitchC] = OUT_NEUTRAL;
#endif // Y6COPTER

} // InitMotors

