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

// Local magnetic declination not included
// http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp

void ReadCompass(void);
void GetHeading(void);
real32 MinimumTurn(real32);
void CalibrateCompass(void);
void ShowCompassType(void);
void DoCompassTest(void);
void InitCompass(void);

MagStruct Mag[3] = {{ 0,0 },{ 0,0 },{ 0,0 }};
real32 MagDeviation, CompassOffset;
real32 MagHeading, Heading, HeadingP, FakeHeading;
real32 HeadingSin, HeadingCos;
real32 CompassMaxSlew;
uint8 CompassType;

enum MagCoords { MX, MY, MZ };

void ReadCompass(void) {
    switch ( CompassType ) {
        case HMC5843:
            ReadHMC5843();
            break;
        case HMC6352:
            ReadHMC6352();
            break;
        default:
            Heading = 0;
            break;
    } // switch

} // ReadCompass

void CalibrateCompass(void) {
    switch ( CompassType ) {
        case HMC5843:
            CalibrateHMC5843();
            break;
        case HMC6352:
            CalibrateHMC6352();
            break;
        default:
            break;
    } // switch
} // CalibrateCompass

void ShowCompassType(void) {
    switch ( CompassType ) {
        case HMC5843:
            TxString("HMC5843");
            break;
        case HMC6352:
            TxString("HMC6352");
            break;
        default:
            break;
    }
} // ShowCompassType

void DoCompassTest(void) {
    switch ( CompassType ) {
        case HMC5843:
            DoHMC5843Test();
            break;
        case HMC6352:
            DoHMC6352Test();
            break;
        default:
            TxString("\r\nCompass test\r\nCompass not detected?\r\n");
            break;
    } // switch
} // DoCompassTest

void GetHeading(void) {

    static real32 HeadingChange, CompassA;

    ReadCompass();

    Heading = Make2Pi( MagHeading + MagDeviation - CompassOffset );
    HeadingChange = fabs( Heading - HeadingP );
    if ( HeadingChange > ONEANDHALFPI ) // wrap 0 -> TwoPI
        HeadingP = Heading;
    else
        if ( HeadingChange > CompassMaxSlew ) { // Sanity check - discard reading
            Heading =SlewLimit(HeadingP, Heading, CompassMaxSlew ); 
            Stats[CompassFailS]++;
        }

#ifndef SUPPRESS_COMPASS_FILTER
    CompassA = dT / ( 1.0 / ( TWOPI * YawFilterLPFreq ) + dT );
    Heading = Make2Pi( LPFilter(Heading, HeadingP, CompassA) );
#endif // !SUPPRESS_COMPASS_FILTER
    HeadingP = Heading;

#ifdef SIMULATE
#if ( defined AILERON | defined ELEVON )
    if ( State == InFlight )
        FakeHeading -= FakeDesiredRoll/5 + FakeDesiredYaw/5;
#else
    if ( State == InFlight ) {
        if ( Abs(FakeDesiredYaw) > 5 )
            FakeHeading -= FakeDesiredYaw/5;
    }

    FakeHeading = Make2Pi((int16)FakeHeading);
    Heading = FakeHeading;
#endif // AILERON | ELEVON
#endif // SIMULATE 
} // GetHeading

//boolean DirectionSelected = false;
//real32 DirectionSense = 1.0;

real32 MinimumTurn(real32 A ) {

    static real32 AbsA;

    AbsA = fabs(A);
    if ( AbsA > PI )
        A = ( AbsA - TWOPI ) * Sign(A);

    //DirectionSelected = fabs(A) > THIRDPI; // avoid dithering around reciprocal heading
    //DirectionSense = Sign(A);

    return ( A );

} // MinimumTurn

void InitCompass(void) {
    if ( IsHMC5843Active() )
        CompassType = HMC5843;
    else
        if ( HMC6352Active() )
            CompassType = HMC6352;
        else {
            CompassType = NoCompass;
            F.CompassValid = false;
        }

    switch ( CompassType ) {
        case HMC5843:
            InitHMC5843();
            break;
        case HMC6352:
            InitHMC6352();
            break;
        default:
            MagHeading = 0;
    } // switch

    ReadCompass();
    mS[CompassUpdate] = mSClock();
    Heading = HeadingP = Make2Pi( MagHeading - MagDeviation - CompassOffset );

} // InitCompass

//________________________________________________________________________________________

// HMC5843 3 Axis Magnetometer

void ReadHMC5843(void);
void GetHMC5843Parameters(void);
void DoHMC5843Test(void);
void CalibrateHMC5843(void);
void InitHMC5843(void);
boolean HMC5843Active(void);

void ReadHMC5843(void) {
    static char b[6];
    static i16u X, Y, Z;
    static real32 FX,FY;
    static real32 CRoll, SRoll, CPitch, SPitch;

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC5843_WR) != I2C_ACK ) goto HMC5843Error;
    if ( I2CCOMPASS.write(0x03) != I2C_ACK ) goto HMC5843Error; // point to data
    I2CCOMPASS.stop();

    if ( I2CCOMPASS.blockread(HMC5843_RD, b, 6) ) goto HMC5843Error;

    X.b1 = b[0];
    X.b0 = b[1];
    Y.b1 = b[2];
    Y.b0  =b[3];
    Z.b1 = b[4];
    Z.b0 = b[5];

    if ( F.Using9DOF ) { // SparkFun/QuadroUFO 9DOF Breakout pins  front edge components up
        Mag[BF].V = X.i16;
        Mag[LR].V = -Y.i16;
        Mag[UD].V = -Z.i16;
    } else { // SparkFun Magnetometer Breakout pins  right edge components up
        Mag[BF].V = -X.i16;
        Mag[LR].V = Y.i16;
        Mag[UD].V = -Z.i16;
    }
    DebugPin = true;
    CRoll = cos(Angle[Roll]);
    SRoll = sin(Angle[Roll]);       // Acc[LR] - optimisation not worthwhile
    CPitch = cos(Angle[Pitch]);
    SPitch = sin(Angle[Pitch]);     // Acc[BF]

    FX = (Mag[BF].V-Mag[BF].Offset) * CPitch + (Mag[LR].V-Mag[LR].Offset) * SRoll * SPitch + (Mag[UD].V-Mag[UD].Offset) * CRoll * SPitch;
    FY = (Mag[LR].V-Mag[LR].Offset) * CRoll - (Mag[UD].V-Mag[UD].Offset) * SRoll;

    // Magnetic Heading
    MagHeading = MakePi(atan2( -FY, FX ));

    DebugPin = false;
    F.CompassValid = true;

    return;

HMC5843Error:
    I2CCOMPASS.stop();

    I2CError[HMC5843_ID]++;
    if ( State == InFlight ) Stats[CompassFailS]++;

    F.CompassMissRead = true;
    F.CompassValid = false;

} // ReadHMC5843

real32  magFieldEarth[3],  magFieldBody[3], dcmMatrix[9];
boolean firstPassMagOffset = true;

void CalibrateHMC5843(void) {

    /*
    void magOffsetCalc()
    {
      int16   i, j ;
      static real32 tempMatrix[3] ;
      static real32 offsetSum[3] ;

      // Compute magnetic field of the earth

      magFieldEarth[0] = vectorDotProduct(3, &dcmMatrix[0], &magFieldBody[0]);
      magFieldEarth[1] = vectorDotProduct(3, &dcmMatrix[3], &magFieldBody[0]);
      magFieldEarth[2] = vectorDotProduct(3, &dcmMatrix[6], &magFieldBody[0]);

      // First pass thru?

      if ( firstPassMagOffset )
      {
        setPastValues();                         // Yes, set initial values for previous values
        firstPassMagOffset = false;              // Clear first pass flag
      }

      // Compute the offsets in the magnetometer:
      vectorAdd(3, offsetSum , magFieldBody, magFieldBodyPrevious) ;

      matrixMultiply(1, 3, 3, tempMatrix, magFieldEarthPrevious, dcmMatrix);
      vectorSubtract(3, offsetSum, offsetSum, tempMatrix);
      matrixMultiply(1, 3, 3, tempMatrix, magFieldEarth, dcmMatrixPrevious);
      vectorSubtract(3, offsetSum, offsetSum, tempMatrix) ;

      for ( i = 0 ; i < 3 ; i++ )
        if ( abs(offsetSum[i] ) < 3 )
          offsetSum[i] = 0 ;

      vectorAdd (3, magOffset, magOffset, offsetSum);
      setPastValues();
    }

    void setPastValues()
    {
      static uint8 i;

      for ( i = 0; i < 3; i++)
      {
        magFieldEarthPrevious[i] = magFieldEarth[i];
        magFieldBodyPrevious[i]  = magFieldBody[i];
      }

      for ( i = 0; i < 9; i++)
        dcmMatrixPrevious[i] = dcmMatrix[i];

    }
    */
} // CalibrateHMC5843

void GetHMC5843Parameters(void) {

    static char CP[16];
    static uint8 i;

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC5843_WR) != I2C_ACK) goto HMC5843Error;
    if ( I2CCOMPASS.write(0x00) != I2C_ACK) goto HMC5843Error; // point to data
    I2CCOMPASS.stop();

    if ( I2CCOMPASS.blockread(HMC5843_RD, CP, 13) ) goto HMC5843Error;

    for ( i = 0; i < (uint8)13; i++ ) {
        TxVal32(i,0,0);
        TxString(":\t0x");
        TxValH(CP[i]);
        TxChar(HT);
        TxBin8(CP[i]);
        TxNextLine();
    }

    return;

HMC5843Error:
    I2CCOMPASS.stop();

    I2CError[HMC5843_ID]++;

} // GetHMC5843Parameters

void DoHMC5843Test(void) {

    TxString("\r\nCompass test (HMC5843)\r\n\r\n");

    ReadHMC5843();

    GetHMC5843Parameters();

    TxString("\r\nMag:\t");
    TxVal32(Mag[BF].V, 0, HT);
    TxVal32(Mag[LR].V, 0, HT);
    TxVal32(Mag[UD].V, 0, HT);
    TxNextLine();
    TxNextLine();

    TxVal32(MagHeading * RADDEG * 10.0, 1, ' ');
    TxString("deg (Magnetic)\r\n");

    Heading = HeadingP = Make2Pi( MagHeading + MagDeviation - CompassOffset );
    TxVal32(Heading * RADDEG * 10.0, 1, ' ');
    TxString("deg (True)\r\n");
} // DoHMC5843Test

boolean WriteByteHMC5843(uint8 a, uint8 d) {

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC5843_WR) != I2C_ACK ) goto HMC5843Error;
    if ( I2CCOMPASS.write(a) != I2C_ACK ) goto HMC5843Error;
    if ( I2CCOMPASS.write(d) != I2C_ACK ) goto HMC5843Error;
    I2CCOMPASS.stop();

    return( false );

HMC5843Error:
    I2CCOMPASS.stop();

    I2CError[HMC5843_ID]++;

    return(true);

} // WriteByteHMC5843

void InitHMC5843(void) {

    CompassMaxSlew = (COMPASS_SANITY_CHECK_RAD_S/HMC5843_UPDATE_S);

    if ( WriteByteHMC5843(0x00, HMC5843_DR  << 2) ) goto HMC5843Error; // rate, normal measurement mode
    if ( WriteByteHMC5843(0x02, 0x00) ) goto HMC5843Error; // mode continuous

    Delay1mS(50);

    return;

HMC5843Error:
    I2CCOMPASS.stop();

    I2CError[HMC5843_ID]++;

    F.CompassValid = false;

} // InitHMC5843Magnetometer

boolean IsHMC5843Active(void) {

    F.CompassValid = I2CCOMPASSAddressResponds( HMC5843_ID );

    if ( F.CompassValid )
        TrackMinI2CRate(400000);

    return ( F.CompassValid );

} // IsHMC5843Active

//________________________________________________________________________________________

// HMC6352 Compass

void ReadHMC6352(void);
uint8 WriteByteHMC6352(uint8);
void GetHMC6352Parameters(void);
void DoHMC6352Test(void);
void CalibrateHMC6352(void);
void InitHMC6352(void);
boolean IsHMC6352Active(void);

void ReadHMC6352(void) {
    static i16u v;

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC6352_RD) != I2C_ACK ) goto HMC6352Error;
    v.b1 = I2CCOMPASS.read(I2C_ACK);
    v.b0 = I2CCOMPASS.read(I2C_NACK);
    I2CCOMPASS.stop();

    MagHeading = Make2Pi( ((real32)v.i16 * PI) / 1800.0 - CompassOffset ); // Radians

    return;

HMC6352Error:
    I2CCOMPASS.stop();

    if ( State == InFlight ) Stats[CompassFailS]++;

    F.CompassMissRead = true;

} // ReadHMC6352

uint8 WriteByteHMC6352(uint8 d) {

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC6352_WR) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(d) != I2C_ACK ) goto HMC6352Error;
    I2CCOMPASS.stop();

    return( I2C_ACK );

HMC6352Error:
    I2CCOMPASS.stop();

    I2CError[HMC6352_ID]++;

    return ( I2C_NACK );
} // WriteByteHMC6352

char CP[9];

#define TEST_COMP_OPMODE 0x70    // standby mode to reliably read EEPROM

void GetHMC6352Parameters(void) {
    int16 Temp;

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC6352_WR) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write('G')  != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(0x74) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(TEST_COMP_OPMODE) != I2C_ACK ) goto HMC6352Error;
    I2CCOMPASS.stop();

    Delay1mS(20);

    for (r = 0; r <= (uint8)8; r++) { // must have this timing - not block read!

        I2CCOMPASS.start();
        if ( I2CCOMPASS.write(HMC6352_WR) != I2C_ACK ) goto HMC6352Error;
        if ( I2CCOMPASS.write('r')  != I2C_ACK ) goto HMC6352Error;
        if ( I2CCOMPASS.write(r)  != I2C_ACK ) goto HMC6352Error;
        I2CCOMPASS.stop();

        Delay1mS(10);

        I2CCOMPASS.start();
        if ( I2CCOMPASS.write(HMC6352_RD) != I2C_ACK ) goto HMC6352Error;
        CP[r] = I2CCOMPASS.read(I2C_NACK);
        I2CCOMPASS.stop();

        Delay1mS(10);
    }

    TxString("\r\nRegisters\r\n");
    TxString("\t0:\tI2C");
    TxString("\t 0x");
    TxValH(CP[0]);
    if ( CP[0] != (uint8)0x42 )
        TxString("\t Error expected 0x42 for HMC6352");
    TxNextLine();

    Temp = (CP[1]*256)|CP[2];
    TxString("\t1:2:\tXOffset\t");
    TxVal32((int32)Temp, 0, 0);
    TxNextLine();

    Temp = (CP[3]*256)|CP[4];
    TxString("\t3:4:\tYOffset\t");
    TxVal32((int32)Temp, 0, 0);
    TxNextLine();

    TxString("\t5:\tDelay\t");
    TxVal32((int32)CP[5], 0, 0);
    TxNextLine();

    TxString("\t6:\tNSum\t");
    TxVal32((int32)CP[6], 0, 0);
    TxNextLine();

    TxString("\t7:\tSW Ver\t");
    TxString(" 0x");
    TxValH(CP[7]);
    TxNextLine();

    TxString("\t8:\tOpMode:");
    switch ( ( CP[8] >> 5 ) & 0x03 ) {
        case 0:
            TxString("  1Hz");
            break;
        case 1:
            TxString("  5Hz");
            break;
        case 2:
            TxString("  10Hz");
            break;
        case 3:
            TxString("  20Hz");
            break;
    }

    if ( CP[8] & 0x10 ) TxString(" S/R");

    switch ( CP[8] & 0x03 ) {
        case 0:
            TxString(" Standby");
            break;
        case 1:
            TxString(" Query");
            break;
        case 2:
            TxString(" Continuous");
            break;
        case 3:
            TxString(" Not-allowed");
            break;
    }
    TxNextLine();

    return;

HMC6352Error:
    I2CCOMPASS.stop();

    I2CError[HMC6352_ID]++;

    TxString("FAIL\r\n");

} // GetHMC6352Parameters

void DoHMC6352Test(void) {

    TxString("\r\nCompass test (HMC6352)\r\n");

    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC6352_WR) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write('G')  != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(0x74) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(TEST_COMP_OPMODE) != I2C_ACK ) goto HMC6352Error;
    I2CCOMPASS.stop();

    Delay1mS(1);

    //  I2CCOMPASS.start(); // Do Set/Reset now
    if ( WriteByteHMC6352('O')  != I2C_ACK ) goto HMC6352Error;

    Delay1mS(7);

    GetHMC6352Parameters();

    InitCompass();
    if ( !F.CompassValid ) goto HMC6352Error;

    Delay1mS(50);

    ReadHMC6352();
    if ( F.CompassMissRead ) goto HMC6352Error;

    TxNextLine();
    TxVal32(MagHeading * RADDEG * 10.0, 1, 0);
    TxString(" deg (Magnetic)\r\n");
    Heading = HeadingP = Make2Pi( MagHeading - MagDeviation - CompassOffset );
    TxVal32(Heading * RADDEG * 10.0, 1, 0);
    TxString(" deg (True)\r\n");

    return;

HMC6352Error:
    I2CCOMPASS.stop();

    I2CError[HMC6352_ID]++;

    TxString("FAIL\r\n");
} // DoHMC6352Test

void CalibrateHMC6352(void) {   // calibrate the compass by rotating the ufo through 720 deg smoothly
    TxString("\r\nCalib. compass - Press CONTINUE button (x) to Start\r\n");
    while ( PollRxChar() != 'x' ); // UAVPSet uses 'x' for CONTINUE button

    // Do Set/Reset now
    if ( WriteByteHMC6352('O') != I2C_ACK ) goto HMC6352Error;

    Delay1mS(7);

    // set Compass device to Calibration mode
    if ( WriteByteHMC6352('C') != I2C_ACK ) goto HMC6352Error;

    TxString("\r\nRotate horizontally 720 deg in ~30 sec. - Press CONTINUE button (x) to Finish\r\n");
    while ( PollRxChar() != 'x' );

    // set Compass device to End-Calibration mode
    if ( WriteByteHMC6352('E') != I2C_ACK ) goto HMC6352Error;

    TxString("\r\nCalibration complete\r\n");

    Delay1mS(50);

    InitCompass();

    return;

HMC6352Error:
    I2CCOMPASS.stop();

    I2CError[HMC6352_ID]++;

    TxString("Calibration FAILED\r\n");
} // CalibrateHMC6352

void InitHMC6352(void) {

    // 20Hz continuous read with periodic reset.
#ifdef SUPPRESS_COMPASS_SR
#define COMP_OPMODE 0x62
#else
#define COMP_OPMODE 0x72
#endif // SUPPRESS_COMPASS_SR

    CompassMaxSlew =  (COMPASS_SANITY_CHECK_RAD_S/HMC6352_UPDATE_S);

    // Set device to Compass mode
    I2CCOMPASS.start();
    if ( I2CCOMPASS.write(HMC6352_WR) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write('G')  != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(0x74) != I2C_ACK ) goto HMC6352Error;
    if ( I2CCOMPASS.write(COMP_OPMODE) != I2C_ACK ) goto HMC6352Error;
    I2CCOMPASS.stop();

    Delay1mS(10);

    // save operation mode in Flash
    if ( WriteByteHMC6352('L') != I2C_ACK ) goto HMC6352Error;

    Delay1mS(10);

    // Do Bridge Offset Set/Reset now
    if ( WriteByteHMC6352('O') != I2C_ACK ) goto HMC6352Error;

    Delay1mS(50);

    F.CompassValid = true;

    return;

HMC6352Error:
    I2CCOMPASS.stop();

    I2CError[HMC6352_ID]++;

    F.CompassValid = false;

    F.CompassFailure = true;

} // InitHMC6352

boolean HMC6352Active(void) {

    F.CompassValid = I2CCOMPASSAddressResponds( HMC6352_ID );

    if ( F.CompassValid )
        TrackMinI2CRate(100000);

    return ( F.CompassValid );

} // HMC6352Active
