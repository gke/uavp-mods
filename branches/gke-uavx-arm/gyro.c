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
const real32 GyroToRadian[UnknownGyro] = {
    8.635062,       // MLX90609
    4.607669,       // ADXRS150
    28.797933,      // IDG300
    17.453293,      // LY530
    11.519173,      // ADXRS300
    0.000438704,    // ITG3200 16bit 2's complement
    1.0             // Infrared Sensors
    // add others as required
};

void ReadGyros(void);
void GetGyroRates(void);
void CheckGyroFault(uint8, uint8, uint8);
void ErectGyros(void);
void InitGyros(void);
void GyroTest(void);
void ShowGyroType(void);

real32 GyroADC[3], GyroNoise[3], GyroNeutral[3], Gyro[3], Gyrop[3]; // Radians
uint8 GyroType;

void GetGyroRates(void) {
    static uint8 g;
    static real32 d, GyroA;

    ReadGyros();

    for ( g = 0; g < (uint8)3; g++ ) {
        d = fabs(Gyro[g]-Gyrop[g]);
        if ( d > GyroNoise[g] ) GyroNoise[g] = d;
    }

#ifndef SUPPRESS_ROLL_PITCH_GYRO_FILTERS
    // dT is almost unchanged so this could be optimised
    GyroA = dT / ( 1.0 / ( TWOPI * ROLL_PITCH_FREQ ) + dT );
    Gyro[Roll] = LPFilter( Gyro[Roll] - GyroNeutral[Roll], Gyrop[Roll], GyroA );
    Gyro[Pitch] = LPFilter( Gyro[Pitch] - GyroNeutral[Pitch], Gyrop[Pitch], GyroA );
#endif // !SUPPRESS_ROLL_PITCH_GYRO_FILTERS 

#ifndef SUPPRESS_YAW_GYRO_FILTERS

#ifdef USE_FIXED_YAW_FILTER
    GyroA = dT / ( 1.0 / ( TWOPI * MAX_YAW_FREQ ) + dT );
#else
    GyroA = dT / ( 1.0 / ( TWOPI * YawFilterLPFreq ) + dT );
#endif // USE_FIXED_YAW_FILTER

    Gyro[Yaw] = LPFilter( Gyro[Yaw] - GyroNeutral[Yaw], Gyrop[Yaw], GyroA );
#endif // !SUPPRESS_GYRO_FILTERS

    for ( g = 0; g < (uint8)3; g++ )
        Gyrop[g] = Gyro[g];

} // GetGyroRates

void ReadGyros(void) {
    switch ( GyroType ) {
        case ITG3200Gyro:
            ReadITG3200Gyro();
            break;
        case IRSensors:
            GetIRAttitude();
            break;
        default :
            ReadAnalogGyros();
            break;
    } // switch
} // ReadGyros

void ErectGyros(void) {
    static uint8 s, i, g;

    LEDRed_ON;

    for ( g = 0; g <(uint8)3; g++ )
        GyroNeutral[g] = 0.0;

    for ( i = 0; i < 32 ; i++ ) {
        Delay1mS(10);

        ReadGyros();
        for ( g = 0; g <(uint8)3; g++ )
            GyroNeutral[g] += Gyro[g];
    }

    for ( g = 0; g <(uint8)3; g++ ) {
        GyroNeutral[g] *= 0.03125;
        Gyro[g] = Gyrop[g] = 0.0;
        for ( s = 0; s < MaxAttitudeScheme; s++ )
            EstAngle[g][s] = EstRate[g][s] = 0.0;
    }

    LEDRed_OFF;

} // ErectGyros

void GyroTest(void) {
    TxString("\r\nGyro Test - ");
    ShowGyroType();

    ReadGyros();

    TxString("\r\n\tRate and Max Delta(Deg./Sec.)\r\n");

    TxString("\r\n\tRoll:   \t");
    TxVal32(Gyro[Roll] * MILLIANGLE, 3, HT);
    TxVal32(GyroNoise[Roll] * MILLIANGLE, 3, 0);
    TxNextLine();
    TxString("\tPitch:  \t");
    TxVal32(Gyro[Pitch] * MILLIANGLE, 3, HT);
    TxVal32(GyroNoise[Pitch] * MILLIANGLE, 3, 0);
    TxNextLine();
    TxString("\tYaw:    \t");
    TxVal32(Gyro[Yaw] * MILLIANGLE, 3, HT);
    TxVal32(GyroNoise[Yaw] * MILLIANGLE, 3, 0);
    TxNextLine();

    switch ( GyroType ) {
        case ITG3200Gyro:
            ITG3200Test();
            break;
        default:
            break;
    } // switch

    if ( F.GyroFailure )
        TxString("\r\nFAILED\r\n");

} // GyroTest

void InitGyros(void) {

    if ( ITG3200GyroActive() )
        GyroType = ITG3200Gyro;
    else
        GyroType = P[GyroRollPitchType];

    switch ( GyroType ) {
        case ITG3200Gyro:
            InitITG3200Gyro();
            break;
        case IRSensors:
            InitIRSensors();
        default:
            InitAnalogGyros();
            break;
    } // switch

    Delay1mS(50);
    ErectGyros();

} // InitGyros

void ShowGyroType(void) {
    switch ( GyroType ) {
        case MLX90609Gyro:
            TxString("MLX90609");
            break;
        case ADXRS150Gyro:
            TxString("ADXRS613/150");
            break;
        case IDG300Gyro:
            TxString("IDG300");
            break;
        case LY530Gyro:
            TxString("ST-AY530");
            break;
        case ADXRS300Gyro:
            TxString("ADXRS610/300");
            break;
        case ITG3200Gyro:
            TxString("ITG3200");
            break;
        case IRSensors:
            TxString("IR Sensors");
            break;
        default:
            TxString("unknown");
            break;
    } // switch
} // ShowGyroType

//________________________________________________________________________________________

// Analog Gyros

void ReadAnalogGyros(void);
void InitAnalogGyros(void);
void CheckAnalogGyroFault(uint8, uint8, uint8);
void AnalogGyroTest(void);

void ReadAnalogGyros(void) {
    static uint8 g;

    GyroADC[Roll] = RollADC.read();
    GyroADC[Pitch] = PitchADC.read();
    GyroADC[Yaw] = YawADC.read();

    for ( g = 0; g < (uint8)3; g++ )
        Gyro[g] = GyroADC[g] * GyroToRadian[GyroType];

} // ReadAnalogGyros

void InitAnalogGyros(void) {
    // nothing to do
    F.GyroFailure = false;
} // InitAnalogGyros

//________________________________________________________________________________________

// ITG3200 3-axis I2C Gyro

void ReadITG3200(void);
uint8 ReadByteITG3200(uint8);
boolean WriteByteITG3200(uint8, uint8);
void InitITG3200(void);
void ITG3200Test(void);
boolean ITG3200Active(void);

real32 ITG3200Temperature;

void ReadITG3200Gyro(void) {
    static char G[6];
    static uint8 g;
    static i16u GX, GY, GZ;

    I2CGYRO.start();
    if ( I2CGYRO.write(ITG3200_WR) != I2C_ACK ) goto ITG3200Error;
    if ( I2CGYRO.write(ITG3200_GX_H) != I2C_ACK ) goto ITG3200Error;
    I2CGYRO.stop();

    if ( I2CGYRO.blockread(ITG3200_ID, G, 6) ) goto ITG3200Error;

    GX.b0 = G[1];
    GX.b1 = G[0];
    GY.b0 = G[3];
    GY.b1 = G[2];
    GZ.b0 = G[5];
    GZ.b1 = G[4];

    if ( F.Using9DOF ) { // SparkFun/QuadroUFO breakout pins forward components up
        GyroADC[Roll] = -(real32)GY.i16;
        GyroADC[Pitch] = -(real32)GX.i16;
        GyroADC[Yaw] = -(real32)GZ.i16;
    } else { // SparkFun 6DOF breakout pins forward components down
        GyroADC[Roll] = -(real32)GX.i16;
        GyroADC[Pitch] = -(real32)GY.i16;
        GyroADC[Yaw] = (real32)GZ.i16;
    }

    for ( g = 0; g < (uint8)3; g++ )
        Gyro[g] = GyroADC[g] * GyroToRadian[ITG3200Gyro];

    return;

ITG3200Error:
    I2CGYRO.stop();

    I2CError[ITG3200_ID]++;

    Stats[GyroFailS]++; // not in flight keep trying
    F.GyroFailure = true;

} // ReadITG3200Gyro

uint8 ReadByteITG3200(uint8 a) {
    static uint8 d;

    I2CGYRO.start();
    if ( I2CGYRO.write(ITG3200_WR) != I2C_ACK ) goto ITG3200Error;
    if ( I2CGYRO.write(a) != I2C_ACK ) goto ITG3200Error;
    I2CGYRO.start();
    if ( I2CGYRO.write(ITG3200_RD) != I2C_ACK ) goto ITG3200Error;
    d = I2CGYRO.read(I2C_NACK);
    I2CGYRO.stop();

    return ( d );

ITG3200Error:
    I2CGYRO.stop();

    I2CError[ITG3200_ID]++;
    // GYRO FAILURE - FATAL
    Stats[GyroFailS]++;

    //F.GyroFailure = true;

    return ( 0 );

} // ReadByteITG3200

boolean WriteByteITG3200(uint8 a, uint8 d) {

    I2CGYRO.start();    // restart
    if ( I2CGYRO.write(ITG3200_WR) != I2C_ACK ) goto ITG3200Error;
    if ( I2CGYRO.write(a) != I2C_ACK ) goto ITG3200Error;
    if ( I2CGYRO.write(d) != I2C_ACK ) goto ITG3200Error;
    I2CGYRO.stop();

    return(false);

ITG3200Error:
    I2CGYRO.stop();

    I2CError[ITG3200_ID]++;
    // GYRO FAILURE - FATAL
    Stats[GyroFailS]++;
    F.GyroFailure = true;

    return(true);

} // WriteByteITG3200

void InitITG3200Gyro(void) {

#define FS_SEL 3

//#define DLPF_CFG 1 // 188HZ
#define DLPF_CFG 2 // 98HZ
//#define DLPF_CFG 3 // 42HZ

    if ( WriteByteITG3200(ITG3200_PWR_M, 0x80) ) goto ITG3200Error; // Reset to defaults
    if ( WriteByteITG3200(ITG3200_SMPL, 0x00) ) goto ITG3200Error; // continuous update
    if ( WriteByteITG3200(ITG3200_DLPF, (FS_SEL << 3) | DLPF_CFG ) ) goto ITG3200Error; // 188Hz, 2000deg/S
    if ( WriteByteITG3200(ITG3200_INT_C, 0x00) ) goto ITG3200Error; // no interrupts
    if ( WriteByteITG3200(ITG3200_PWR_M, 0x01) ) goto ITG3200Error; // X Gyro as Clock Ref.

    Delay1mS(50);

    F.GyroFailure = false;

    ReadITG3200Gyro();

    return;

ITG3200Error:

    F.GyroFailure = true;

} // InitITG3200Gyro

void ITG3200Test(void) {
    static uint8 MyID, SMPLRT_DIV, DLPF_FS, PWR_MGM;
    static int16 TEMP,GYRO_X, GYRO_Y, GYRO_Z;

    MyID = ReadByteITG3200(ITG3200_WHO);

    TxString("\tWHO_AM_I  \t0x");
    TxValH(MyID);
    TxNextLine();
    Delay1mS(1);
    SMPLRT_DIV = ReadByteITG3200(ITG3200_SMPL);
    DLPF_FS = ReadByteITG3200(ITG3200_DLPF);
    TEMP = (int16)ReadByteITG3200(ITG3200_TMP_H)<<8 | ReadByteITG3200(ITG3200_TMP_L);
    GYRO_X = (int16)ReadByteITG3200(ITG3200_GX_H)<<8 | ReadByteITG3200(ITG3200_GX_L);
    GYRO_Y = (int16)ReadByteITG3200(ITG3200_GY_H)<<8 | ReadByteITG3200(ITG3200_GY_L);
    GYRO_Z = (int16)ReadByteITG3200(ITG3200_GZ_H)<<8 | ReadByteITG3200(ITG3200_GZ_L);
    PWR_MGM = ReadByteITG3200(ITG3200_PWR_M);

    ITG3200Temperature = 35.0 + ((TEMP + 13200.0 ) / 280.0);

    TxString("\tSMPLRT_DIV\t");
    TxVal32( SMPLRT_DIV,0,0);
    TxNextLine();
    TxString("\tDLPF      \t");
    TxVal32( DLPF_FS & 7,0,0 );
    TxString(" FS         \t");
    TxVal32( (DLPF_FS>>3)&3, 0, 0);
    TxNextLine();
    TxString("\tTEMP      \t");
    TxVal32( TEMP, 0, 0);
    TxNextLine();
    TxString("\tGYRO_X    \t");
    TxVal32( GYRO_X, 0, 0);
    TxNextLine();
    TxString("\tGYRO_Y    \t");
    TxVal32( GYRO_Y, 0, 0);
    TxNextLine();
    TxString("\tGYRO_Z    \t");
    TxVal32( GYRO_Z, 0, 0);
    TxNextLine();
    TxString("\tPWR_MGM   \t0x");
    TxValH( PWR_MGM );
    TxNextLine();

    TxNextLine();

} // ITG3200Test

boolean ITG3200GyroActive(void) {

    F.GyroFailure = !I2CGYROAddressResponds( ITG3200_ID );

    if ( !F.GyroFailure )
        TrackMinI2CRate(400000);

    return ( !F.GyroFailure );

} // ITG3200GyroActive
