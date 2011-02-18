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

void ReadParameters(void);
void UseDefaultParameters(void);
void UpdateWhichParamSet(void);
boolean ParameterSanityCheck(void);
void InitParameters(void);

const uint8 ESCLimits [] = { OUT_MAXIMUM, OUT_HOLGER_MAXIMUM, OUT_X3D_MAXIMUM, OUT_YGEI2C_MAXIMUM };

#ifdef MULTICOPTER
#include "uavx_multicopter.h"
#else
#ifdef HELICOPTER
#include "uavx_helicopter.h"
#else
#ifdef ELEVONS
#include "uavx_elevon.h"
#else
#include "uavx_aileron.h"
#endif
#endif
#endif

uint8    ParamSet;
boolean ParametersChanged, SaveAllowTurnToWP;

int8 P[MAX_PARAMETERS];
real32 K[MAX_PARAMETERS]; // Arm rescaled legacy parameters as appropriate

real32 OSin[48], OCos[48];

uint8 Orientation , PolarOrientation;
uint8 UAVXAirframe;

void Legacy(void) {
    static uint8 p;

    for ( p = 0; p <MAX_PARAMETERS; p++ ) // brute force
        K[p] = (float)P[p];

    // Rate Control
    K[RollKp] *= 2.6;
    K[RollKi] *= 20.7;
    K[RollKd]  = -K[RollKd] * 0.021;
    K[RollIntLimit] *= DEGRAD;

    K[PitchKp] *= 2.6;
    K[PitchKi] *= 20.7;
    K[PitchKd]  = -K[PitchKd] * 0.021;
    K[PitchIntLimit] *= DEGRAD;

    K[YawKp] *= 2.6;
    K[YawKi] *= 41.4;
    K[YawKd]  = -K[YawKd] * 0.0004;

    // Angle Control

    // not yet

    // Inertial Damping
    K[VertDampKp] *= 0.1; // one click/MPS
    K[HorizDampKp] *= 0.1;
    K[VertDampDecay] *= 0.01;
    K[HorizDampDecay] *= 0.01;

    // Altitude Hold
    K[AltKp] *= 0.625;
    K[AltKi] *= 25.0;
    K[AltKd] *= 0.125;

    // Navigation
    K[NavKi] *= 0.08;
    K[NavKd] *= 0.0008;
    
    K[Balance] = ( 128.0 + (float)P[Balance])/128.0;
    
    K[CompassKp] = P[CompassKp] / 4096.0;
    K[YawIntLimit] = P[YawIntLimit] * 256.0 /1000.0;

    // Camera
    K[CamRollKp] *= 0.05;
    K[CamPitchKp] *= 0.05;

    // Acceleration Neutrals
    K[MiddleBF] = P[MiddleBF] * 0.001; // mG
    K[MiddleLR] = P[MiddleLR] * 0.001;
    K[MiddleUD] = P[MiddleUD] * 0.001;

    K[LowVoltThres] *= 0.2;

} // Legacy

void ReadParameters(void) {
    static int8 i, b, a;

    if ( ParametersChanged ) {  // overkill if only a single parameter has changed but is not in flight loop
        a = (ParamSet - 1)* MAX_PARAMETERS;
        for ( i = 0; i < MAX_PARAMETERS; i++)
            P[i] = ReadPX(a + i);

        Legacy();

        ESCMax = ESCLimits[P[ESCType]];
        if ( P[ESCType] == ESCPPM )
            ESCMin = 1;
        else {
            ESCMin = 0;
            for ( i = 0; i < NoOfI2CESCOutputs; i++ )
                ESCI2CFail[i] = 0;
            InitI2CESCs();
        }

        b = P[ServoSense];
        for ( i = 0; i < 6; i++ ) {
            if ( b & 1 )
                PWMSense[i] = -1;
            else
                PWMSense[i] = 1;
            b >>=1;
        }

        F.UsingPositionHoldLock = ( (P[ConfigBits] & UsePositionHoldLockMask ) != 0);
        F.UsingPolarCoordinates = ( (P[ConfigBits] & UsePolarMask ) != 0);

        for ( i = 0; i < CONTROLS; i++) // make reverse map
            RMap[Map[P[TxRxType]][i]] = i;

        IdleThrottle = Limit((int16)P[PercentIdleThr], 10, 30); // 10-30%
        IdleThrottle = (IdleThrottle * OUT_MAXIMUM )/100L;
        CruiseThrottle = ((int16)P[PercentCruiseThr] * OUT_MAXIMUM )/100L;
        MaxCruiseThrottle = (RC_MAXIMUM * 60L * OUT_MAXIMUM)/100L; // 60%

        NavNeutralRadius = Limit((int16)P[NeutralRadius], 0, NAV_MAX_NEUTRAL_RADIUS);
        NavNeutralRadius = ConvertMToGPS(NavNeutralRadius);

        NavYCorrLimit = Limit((int16)P[NavYawLimit], 5, 50);

        MagDeviation = (real32)P[NavMagVar] * DEGRAD;
        CompassOffset = ((real32)((int16)P[CompassOffsetQtr] * 90L) * DEGRAD );
        InitCompass();

#ifdef MULTICOPTER
        Orientation = P[Orient];
        if (Orientation == 0xff ) // uninitialised
            Orientation = 0;
#else
        Orientation = 0;
#endif // MULTICOPTER

        F.UsingSerialPPM = ( P[TxRxType] == FrSkyDJT_D8R ) || ( P[TxRxType] == ExternalDecoder ) || ( (P[ConfigBits] & RxSerialPPMMask ) != 0);
        DoRxPolarity();
        PPM_Index = PrevEdge = 0;

        F.UsingPolar = ((P[ConfigBits] & UsePolarMask) != 0);
        F.RFInInches = ((P[ConfigBits] & RFInchesMask) != 0);

        F.UsingTxMode2 = ((P[ConfigBits] & TxMode2Mask) != 0);

        if ( P[GyroRollPitchType] == IRSensors )
            F.UsingAngleControl = true;
        else
            F.UsingAngleControl = ((P[ConfigBits] & UseAngleControlMask) != 0);

        F.UsingRTHAutoDescend = ((P[ConfigBits] & UseRTHDescendMask) != 0);
        NavRTHTimeoutmS = (uint24)P[DescentDelayS]*1000L;

        BatteryVolts = K[LowVoltThres];
        BatteryCurrent = 0;

        F.ParametersValid = ParameterSanityCheck();

        ParametersChanged = false;

    }
} // ReadParameters

void UseDefaultParameters(void) { // loads a representative set of initial parameters as a base for tuning
    int8 p, d;
    static int16 a;

    for ( a = 0; a < PX_LENGTH; a++ )
        PX[a] = 0xff;

    for ( p = 0; p < MAX_PARAMETERS; p++ ) {
        d = DefaultParams[p][0];
        WritePX(p, d);
        WritePX(p+MAX_PARAMETERS, d);
    }

    WritePX(NAV_NO_WP, 0); // set NoOfWaypoints to zero

    WritePXImagefile();

    TxString("\r\nDefault Parameters Loaded\r\n");
    TxString("Do a READ CONFIG to refresh the UAVPSet parameter display\r\n");
} // UseDefaultParameters

void UpdateParamSetChoice(void) {
#define STICK_WINDOW 30

    uint8 NewParamSet, NewAllowNavAltitudeHold, NewAllowTurnToWP;
    int8 Selector;

    NewParamSet = ParamSet;
    NewAllowNavAltitudeHold = F.AllowNavAltitudeHold;
    NewAllowTurnToWP = F.AllowTurnToWP;

    if ( F.UsingTxMode2 )
        Selector = DesiredRoll;
    else
        Selector = -DesiredYaw;

    if ( (abs(DesiredPitch) > STICK_WINDOW) && (abs(Selector) > STICK_WINDOW) ) {
        if ( DesiredPitch > STICK_WINDOW ) { // bottom
            if ( Selector < -STICK_WINDOW ) // left
            { // bottom left
                NewParamSet = 1;
                NewAllowNavAltitudeHold = true;
            } else
                if ( Selector > STICK_WINDOW ) // right
                { // bottom right
                    NewParamSet = 2;
                    NewAllowNavAltitudeHold = true;
                }
        } else
            if ( DesiredPitch < -STICK_WINDOW ) { // top
                if ( Selector < -STICK_WINDOW ) { // left
                    NewAllowNavAltitudeHold = false;
                    NewParamSet = 1;
                } else
                    if ( Selector > STICK_WINDOW ) { // right
                        NewAllowNavAltitudeHold = false;
                        NewParamSet = 2;
                    }
            }

        if ( ( NewParamSet != ParamSet ) || ( NewAllowNavAltitudeHold != F.AllowNavAltitudeHold ) ) {
            ParamSet = NewParamSet;
            F.AllowNavAltitudeHold = NewAllowNavAltitudeHold;
            LEDBlue_ON;
            DoBeep100mS(2, 2);
            if ( ParamSet == (uint8)2 )
                DoBeep100mS(2, 2);
            if ( F.AllowNavAltitudeHold )
                DoBeep100mS(4, 4);
            ParametersChanged |= true;
            Beeper_OFF;
            LEDBlue_OFF;
        }
    }

    if ( F.UsingTxMode2 )
        Selector = -DesiredYaw;
    else
        Selector = DesiredRoll;

    if ( (abs(RC[ThrottleC]) < STICK_WINDOW) && (abs(Selector) > STICK_WINDOW ) ) {
        if ( Selector < -STICK_WINDOW ) // left
            NewAllowTurnToWP = false;
        else
            if ( Selector > STICK_WINDOW ) // left
                NewAllowTurnToWP = true; // right

        if ( NewAllowTurnToWP != F.AllowTurnToWP ) {
            F.AllowTurnToWP = NewAllowTurnToWP;
            LEDBlue_ON;
            //    if ( F.AllowTurnToWP )
            DoBeep100mS(4, 2);

            LEDBlue_OFF;
        }
    }

    SaveAllowTurnToWP = F.AllowTurnToWP;

} // UpdateParamSetChoice

boolean ParameterSanityCheck(void) {
    static boolean Fail;

    Fail = (P[RollKp] == 0) ||
           (P[PitchKp] == 0) ||
           (P[YawKp] == 0);

    return ( !Fail );
} // ParameterSanityCheck

void InitParameters(void) {
    static int8 i;
    static real32 A;

    F.ParametersValid = false;
    if ( !ReadPXImagefile() )
        UseDefaultParameters();

    UAVXAirframe = AF_TYPE;

    for (i = 0; i < 48; i++) {
        A = ((real32)i * PI)/24.0;
        OSin[i] = sin(A);
        OCos[i] = cos(A);
    }
    Orientation = 0;

    ALL_LEDS_ON;

    ParametersChanged = true;
    ParamSet = 1;
    ReadParameters();

    ALL_LEDS_OFF;
} // InitParameters


