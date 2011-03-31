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

void ShowAccType(void);
void ReadAccelerometers(void);
void GetAccelerations(void);
void GetNeutralAccelerations(void);
void AccelerometerTest(void);
void InitAccelerometers(void);

real32 Vel[3], AccADC[3], AccADCp[3], AccNoise[3], Acc[3], AccNeutral[3], Accp[3];
int16 NewAccNeutral[3];
uint8 AccelerometerType;
real32 GravityR;

void ShowAccType(void) {
    switch ( AccelerometerType ) {
        case LISLAcc:
            TxString(" LIS3L");
            break;
        case ADXL345Acc:
            TxString(" ADXL345");
            break;
        case AccUnknown:
            TxString(" unknown");
            break;
        default:
            ;
    } // switch
} // ShowAccType

void ReadAccelerometers(void) {
    switch ( AccelerometerType ) {
        case LISLAcc:
            ReadLISLAcc();
            break;
        case ADXL345Acc:
            ReadADXL345Acc();
            break;
            // other accelerometers
        default:
            Acc[BF] = Acc[LR] = 0.0;
            Acc[UD] = 1.0;
            break;
    } // switch

} // ReadAccelerometers

void GetNeutralAccelerations(void) {
    static int16 i;
    static uint8 a;
    static real32 Temp[3] = {0.0, 0.0, 0.0};
    const int16 Samples = 100;

    if ( F.AccelerationsValid ) {
        for ( i = Samples; i; i--) {
            ReadAccelerometers();
            for ( a = 0; a <(uint8)3; a++ )
                Temp[a] += AccADC[a];
        }

        for ( a = 0; a <(uint8)3; a++ )
            Temp[a] /= Samples;

        // removes other accelerations
        GravityR = 1.0/sqrt(Sqr(Temp[BF])+Sqr(Temp[LR])+Sqr(Temp[UD]));
        for ( a = 0; a <(uint8)3; a++ )
            Acc[a] = Temp[a] * GravityR;

        NewAccNeutral[BF] = Limit1((int16)(Acc[BF] * 1000.0 ), 99);
        NewAccNeutral[LR] = Limit1( (int16)(Acc[LR] * 1000.0 ), 99);
        NewAccNeutral[UD] = Limit1( (int16)(( Acc[UD] + 1.0 ) * 1000.0) , 99);

    } else
        for ( a = 0; a <(uint8)3; a++ )
            AccNeutral[a] = 0.0;

} // GetNeutralAccelerations

void GetAccelerations(void) {

    static uint8 a;
    static real32 AccA;

    if ( F.AccelerationsValid ) {
        ReadAccelerometers();

        // Neutral[ {LR, BF, UD} ] pass through UAVPSet
        // and come back as AccMiddle[LR] etc.

        Acc[BF] = AccADC[BF] * GravityR - K[MiddleBF];
        Acc[LR] = AccADC[LR] * GravityR - K[MiddleLR];
        Acc[UD] = AccADC[UD] * GravityR - K[MiddleUD];

#ifndef SUPPRESS_ACC_FILTERS
        AccA = dT / ( 1.0 / ( TWOPI * ACC_FREQ ) + dT );
        for ( a = 0; a < (uint8)3; a++ )
            Acc[a] = LPFilter( Acc[a], Accp[a], AccA );
#endif // !SUPPRESS_ACC_FILTERS
        for ( a = 0; a < (uint8)3; a++ )
            Accp[a] = Acc[a];

    } else {
        Acc[LR] = Acc[BF] = 0;
        Acc[UD] = 1.0;
    }
} // GetAccelerations

void AccelerometerTest(void) {
    TxString("\r\nAccelerometer test -");
    ShowAccType();
    TxString("\r\nRead once - no averaging\r\n");

    InitAccelerometers();

    if ( F.AccelerationsValid ) {
        GetAccelerations();

        TxString("Sensor & Max Delta\r\n");

        TxString("\tL->R: \t");
        TxVal32( AccADC[LR], 0, HT);
        TxVal32( AccNoise[LR], 0, 0);
        if ( fabs(Acc[LR]) > 0.2 )
            TxString(" fault?");
        TxNextLine();

        TxString("\tB->F: \t");
        TxVal32( AccADC[BF], 0, HT);
        TxVal32( AccNoise[BF], 0, 0);
        if ( fabs(Acc[BF]) > 0.2 )
            TxString(" fault?");
        TxNextLine();

        TxString("\tU->D:    \t");
        TxVal32( AccADC[UD], 0, HT);
        TxVal32( AccNoise[UD], 0, 0);
        if ( fabs(Acc[UD]) > 1.2 )
            TxString(" fault?");
        TxNextLine();
    }
} // AccelerometerTest

void InitAccelerometers(void) {
    static uint8 a;

    F.AccelerationsValid = true; // optimistic

    for ( a = 0; a < (uint8)3; a++ ) {
        NewAccNeutral[a] = AccADC[a] = AccADCp[a] = AccNoise[a] = Acc[a] = Accp[a] = Vel[a] = 0.0;
        Comp[a] = 0;
    }
    Acc[2] = Accp[2] = 1.0;
    if ( ADXL345AccActive() ) {
        InitADXL345Acc();
        AccelerometerType = ADXL345Acc;

    } else
        if ( LISLAccActive() ) {
            InitLISLAcc();
            AccelerometerType = LISLAcc;
        } else
            // check for other accs in preferred order
        {
            AccelerometerType = AccUnknown;
            F.AccelerationsValid = false;
        }

    if ( F.AccelerationsValid ) {
        LEDYellow_ON;
        GetNeutralAccelerations();
        LEDYellow_OFF;
    } else
        F.AccFailure = true;
} // InitAccelerometers

//________________________________________________________________________________________

// ADXL345 3 Axis Accelerometer

void ReadADXL345Acc(void);
void InitADXL345Acc(void);
boolean ADXL345AccActive(void);

void ReadADXL345Acc(void) {

    static uint8 a;
    static char b[6];
    static i16u X, Y, Z;
    static real32 d;

    I2CACC.start();
    if ( I2CACC.write(ADXL345_WR) != I2C_ACK ) goto ADXL345Error; // point to acc data
    if ( I2CACC.write(0x32) != I2C_ACK ) goto ADXL345Error; // point to acc data
    I2CACC.stop();

    if ( I2CACC.blockread(ADXL345_ID, b, 6) ) goto ADXL345Error;

    X.b1 = b[1];
    X.b0 = b[0];
    Y.b1 = b[3];
    Y.b0  =b[2];
    Z.b1 = b[5];
    Z.b0 = b[4];

    if ( F.Using9DOF ) { // SparkFun/QuadroUFO 9DOF breakouts pins forward components up
        AccADC[BF] = -Y.i16;
        AccADC[LR] = -X.i16;
        AccADC[UD] = -Z.i16;
    } else {// SparkFun 6DOF breakouts pins forward components down
        AccADC[BF] = -X.i16;
        AccADC[LR] = -Y.i16;
        AccADC[UD] = Z.i16;
    }

    for ( a = 0; a < (int8)3; a++ ) {
        d = fabs(AccADC[a]-AccADCp[a]);
        if ( d>AccNoise[a] ) AccNoise[a] = d;
    }

    return;

ADXL345Error:
    I2CACC.stop();

    I2CError[ADXL345_ID]++;
    if ( State == InFlight ) {
        Stats[AccFailS]++;    // data over run - acc out of range
        // use neutral values!!!!
        F.AccFailure = true;
    }

} // ReadADXL345Acc

void InitADXL345Acc() {

    I2CACC.start();
    if ( I2CACC.write(ADXL345_WR) != I2C_ACK ) goto ADXL345Error;;
    if ( I2CACC.write(0x2D) != I2C_ACK ) goto ADXL345Error;  // power register
    if ( I2CACC.write(0x08) != I2C_ACK ) goto ADXL345Error;  // measurement mode
    I2CACC.stop();

    Delay1mS(5);

    I2CACC.start();
    if ( I2CACC.write(ADXL345_WR) != I2C_ACK ) goto ADXL345Error;
    if ( I2CACC.write(0x31) != I2C_ACK ) goto ADXL345Error;  // format
    if ( I2CACC.write(0x08) != I2C_ACK ) goto ADXL345Error;  // full resolution, 2g
    I2CACC.stop();

    Delay1mS(5);

    I2CACC.start();
    if ( I2CACC.write(ADXL345_WR) != I2C_ACK ) goto ADXL345Error;
    if ( I2CACC.write(0x2C) != I2C_ACK ) goto ADXL345Error;  // Rate
    if ( I2CACC.write(0x09) != I2C_ACK ) goto ADXL345Error;  // 50Hz, 400Hz 0x0C
    I2CACC.stop();

    Delay1mS(5);

    return;

ADXL345Error:
    I2CACC.stop();

    I2CError[ADXL345_ID]++;

} // InitADXL345Acc

boolean ADXL345AccActive(void) {

    F.AccelerationsValid = I2CACCAddressResponds(ADXL345_ID);

    if ( F.AccelerationsValid)
        TrackMinI2CRate(400000);

    return( F.AccelerationsValid );

} // ADXL345AccActive

//________________________________________________________________________________________

// LIS3LV02DG Accelerometer 400KHz

boolean WriteLISL(uint8, uint8);
void ReadLISLAcc(void);
void InitLISLAcc(void);
boolean LISLAccActive(void);

void ReadLISLAcc(void) {

    static uint8 a;
    static real32 d;
    static char b[6];
    static i16u X, Y, Z;

    F.AccelerationsValid = I2CACCAddressResponds( LISL_ID ); // Acc still there?

    if ( !F.AccelerationsValid ) goto LISLError;

    I2CACC.start();
    if ( I2CACC.write(LISL_WR) != I2C_ACK ) goto LISLError;
    if ( I2CACC.write(LISL_OUTX_L | 0x80 ) != I2C_ACK ) goto LISLError; // auto increment address
    I2CACC.stop();

    if ( I2CACC.blockread(LISL_RD, b, 6) ) goto LISLError;

    X.b1 = b[1];
    X.b0 = b[0];
    Y.b1 = b[3];
    Y.b0 = b[2];
    Z.b1 = b[5];
    Z.b0 = b[4];

    // UAVP Breakout Board pins to the rear components up
    AccADC[BF] = Y.i16;
    AccADC[LR] = X.i16;
    AccADC[UD] = -Z.i16;

    for ( a = 0; a < (int8)3; a++ ) {
        d = fabs(AccADC[a]-AccADCp[a]);
        if ( d>AccNoise[a] ) AccNoise[a] = d;
    }

    return;

LISLError:
    I2CACC.stop();

    I2CError[LISL_ID]++;

    if ( State == InFlight ) {
        Stats[AccFailS]++;    // data over run - acc out of range
        // use neutral values!!!!
        F.AccFailure = true;
    }

} // ReadLISLAccelerometers

boolean WriteLISL(uint8 d, uint8 a) {

    I2CACC.start();
    if ( I2CACC.write(LISL_WR) != I2C_ACK ) goto LISLError;
    if ( I2CACC.write(a) != I2C_ACK ) goto LISLError;
    if ( I2CACC.write(d) != I2C_ACK ) goto LISLError;
    I2CACC.stop();

    return(false);

LISLError:
    I2CACC.stop();

    I2CError[LISL_ID]++;

    return(true);

} // WriteLISL

void InitLISLAcc(void) {

    if ( WriteLISL(0x4a, LISL_CTRLREG_2) ) goto LISLError; // enable 3-wire, BDU=1, +/-2g
    if ( WriteLISL(0xc7, LISL_CTRLREG_1) ) goto LISLError; // on always, 40Hz sampling rate,  10Hz LP cutoff, enable all axes
    if ( WriteLISL(0, LISL_CTRLREG_3) ) goto LISLError;
    if ( WriteLISL(0x40, LISL_FF_CFG) ) goto LISLError;    // latch, no interrupts;
    if ( WriteLISL(0, LISL_FF_THS_L) ) goto LISLError;
    if ( WriteLISL(0xFC, LISL_FF_THS_H) ) goto LISLError;  // -0,5g threshold
    if ( WriteLISL(255, LISL_FF_DUR) ) goto LISLError;
    if ( WriteLISL(0, LISL_DD_CFG) ) goto LISLError;

    TrackMinI2CRate(400000);
    F.AccelerationsValid = true;

    return;

LISLError:
    F.AccelerationsValid = false;

} // InitLISLAcc

boolean LISLAccActive(void) {

    F.AccelerationsValid = I2CACCAddressResponds( LISL_ID );

    return ( F.AccelerationsValid );

} // LISLAccActive



