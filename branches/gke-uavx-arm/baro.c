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

// Barometers Freescale TI ADC and Bosch BMP085 3.8MHz, Bosch SMD500 400KHz

void GetBaroAltitude(void);
void InitBarometer(void);

void ShowBaroType(void);
void BaroTest(void);

#define BaroROCFilter HardFilter

uint16   BaroPressure, BaroTemperature;
boolean  AcquiringPressure;
int16    BaroOffsetDAC;

int32    OriginBaroPressure, CompBaroPressure;
real32   BaroRelAltitude, BaroRelAltitudeP;
i16u     BaroVal;
int8     BaroType;
int16    BaroClimbAvailable, BaroDescentAvailable;
int16    AltitudeUpdateRate;
int8     BaroRetries;

real32   FakeBaroRelAltitude;
int8     SimulateCycles = 0;

real32  AltCF, AltTauCF;
real32  AltF[3] = { 0.0, 0.0, 0.0};

void ShowBaroType(void) {
    switch ( BaroType ) {
        case BaroMPX4115:
            TxString("MPX4115\r\n");
            break;
        case BaroSMD500:
            TxString("SMD500\r\n");
            break;
        case BaroBMP085:
            TxString("BMP085\r\n");
            break;
        case BaroUnknown:
            TxString("None\r\n");
            break;
        default:
            break;
    }
} // ShowBaro

void BaroTest(void) {

    TxString("\r\nAltitude test\r\n");
    TxString("Initialising\r\n");

    InitBarometer();

    while ( F.BaroAltitudeValid && ! F.NewBaroValue )
        GetBaroAltitude();

    TxString("\r\nType:\t");
    ShowBaroType();

    TxString("BaroScale:\t");
    TxVal32(P[BaroScale],0,0);
    TxString("\r\nInit Retries:\t");
    TxVal32((int32)BaroRetries - 2, 0, ' '); // always minimum of 2
    if ( BaroRetries >= BARO_INIT_RETRIES )
        TxString(" FAILED Init.\r\n");
    else
        TxNextLine();

    if ( BaroType == BaroMPX4115 ) {
        TxString("\r\nAddress :\t0x");
        TxValH(MCP4725_ID_Actual);
        TxString("\r\nRange   :\t");
        TxVal32((int32) BaroDescentAvailable * 10.0, 1, ' ');
        TxString("-> ");
        TxVal32((int32) BaroClimbAvailable * 10.0, 1, 'M');
        TxString(" {Offset ");
        TxVal32((int32)BaroOffsetDAC, 0,'}');
        if (( BaroClimbAvailable < BARO_MIN_CLIMB ) || (BaroDescentAvailable > BARO_MIN_DESCENT))
            TxString(" Bad climb or descent range - offset adjustment?");
        TxNextLine();
    }

    if ( F.BaroAltitudeValid ) {

        while ( !F.NewBaroValue )
            GetBaroAltitude();

        F.NewBaroValue = false;

        TxString("Alt.:     \t");
        TxVal32(BaroRelAltitude * 10.0, 1, ' ');
        TxString("M\r\n");

    } else
        TxString("Barometer FAILED\r\n");

    TxString("\r\nR.Finder: \t");
    if ( F.RangefinderAltitudeValid ) {
        GetRangefinderAltitude();
        TxVal32(RangefinderAltitude * 100.0, 2, ' ');
        TxString("M\r\n");
    } else
        TxString("no rangefinder\r\n");

    TxString("\r\nUAVXArm Shield:\t");
    TxVal32((int32)AmbientTemperature.i16, 1, ' ');
    TxString("C\r\n");

} // BaroTest

void GetBaroAltitude(void) {

    if ( BaroType == BaroMPX4115 )
        GetFreescaleBaroAltitude();
    else
        GetBoschBaroAltitude();

#ifdef SIMULATE
    if (( F.NewBaroValue ) && ( State == InFlight )) {
        if ( ++SimulateCycles >= AltitudeUpdateRate ) {
            FakeBaroRelAltitude += ( DesiredThrottle - CruiseThrottle ) + Comp[Alt];
            if ( FakeBaroRelAltitude < -5.0 )
                FakeBaroRelAltitude = 0.0;

            SimulateCycles = 0;

            BaroRelAltitudeP = FakeBaroRelAltitude;
        }
        BaroRelAltitude = FakeBaroRelAltitude;
    }
}

#endif // SIMULATE

} // GetBaroAltitude

void InitBarometer(void) {
    BaroRelAltitude = BaroRelAltitudeP = CompBaroPressure = OriginBaroPressure = 0;
    AltCF = AltF[0] = AltF[1] = AltF[2] = 0.0;
    BaroType = BaroUnknown;

    AltComp = 0;
    F.BaroAltitudeValid= true; // optimistic

    if ( IsFreescaleBaroActive() )
        InitFreescaleBarometer();
    else
        if ( IsBoschBaroActive() )
            InitBoschBarometer();
        else
            F.BaroAltitudeValid = F.HoldingAlt = false;

} // InitBarometer

real32 AltitudeCF(real32 Alt) {   // Complementary Filter originally authored by RoyLB for attitude estimation
    // http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
    // adapted for baro compensation by G.K. Egan 2011 using acceleration as an alias
    // for diplacement to avoid integration windup of vertical displacement estimate

    if ( F.AccelerationsValid && F.NearLevel ) {

        AltF[0] = (Alt - AltCF) * Sqr(AltTauCF);
        AltF[2] = AltF[2] + AltF[0] * BARO_DT;
        AltF[1] =  AltF[2] + (Alt - AltCF)*2.0*AltTauCF; // + 0.5*(Acc[UD] - 1.0)*BARO_DT*BARO_DT;
        AltCF = AltCF + AltF[1] * BARO_DT;

    } else
        AltCF = Altitude;

    AccAltComp = AltCF;// - AltE; // for debugging

    return( Alt ); // AltCF );

} // AltitudeCF

// -----------------------------------------------------------

// Freescale ex Motorola MPX4115 Barometer with ADS7823 12bit ADC

void SetFreescaleMCP4725(int16);
boolean IdentifyMCP4725(void);
void SetFreescaleOffset(void);
void ReadFreescaleBaro(void);
real32 FreescaleToM(int24);
void GetFreescaleBaroAltitude(void);
boolean IsFreescaleBaroActive(void);
void InitFreescaleBarometer(void);

uint8 MCP4725_ID_Actual;

void SetFreescaleMCP4725(int16 d) {
    static i16u dd;

    dd.u16 = d << 4;                            // left align

    I2CBARO.start();
    if ( I2CBARO.write(MCP4725_ID_Actual) != I2C_ACK ) goto MCP4725Error;
    if ( I2CBARO.write(MCP4725_CMD) != I2C_ACK ) goto MCP4725Error;
    if ( I2CBARO.write(dd.b1) != I2C_ACK ) goto MCP4725Error;
    if ( I2CBARO.write(dd.b0) != I2C_ACK ) goto MCP4725Error;
    I2CBARO.stop();

    return;

MCP4725Error:
    I2CBARO.stop();
    I2CError[MCP4725_ID_Actual]++;

} // SetFreescaleMCP4725

boolean IdentifyMCP4725(void) {

    static boolean r;

    r = true;
    MCP4725_ID_Actual = MCP4725_ID_0xCC;
    if ( I2CBAROAddressResponds( MCP4725_ID_0xCC ) )
        MCP4725_ID_Actual = MCP4725_ID_0xCC;
    else
        if ( I2CBAROAddressResponds( MCP4725_ID_0xC8 ) )
            MCP4725_ID_Actual = MCP4725_ID_0xC8;
        else
            r = false;

    return(r);

//   MCP4725_ID_Actual = FORCE_BARO_ID;
//   return(true);

} // IdentifyMCP4725

void SetFreescaleOffset(void) {
    // Steve Westerfeld
    // 470 Ohm, 1uF RC 0.47mS use 2mS for settling?

    TxString("\r\nOffset \tPressure\r\n");

    BaroOffsetDAC = MCP4725_MAX;

    SetFreescaleMCP4725(BaroOffsetDAC);

    Delay1mS(20);                               // initial settling
    ReadFreescaleBaro();

    while ( (BaroVal.u16 < (uint16)(((uint24)ADS7823_MAX*4L*7L)/10L) )
            && (BaroOffsetDAC > 20) ) {         // first loop gets close
        BaroOffsetDAC -= 20;                    // approach at 20 steps out of 4095
        SetFreescaleMCP4725(BaroOffsetDAC);
        Delay1mS(20);
        ReadFreescaleBaro();
        TxVal32(BaroOffsetDAC,0,HT);
        TxVal32(BaroVal.u16,0,' ');
        TxNextLine();
        LEDYellow_TOG;
    }

    BaroOffsetDAC += 20;                        // move back up to come at it a little slower
    SetFreescaleMCP4725(BaroOffsetDAC);
    Delay1mS(100);
    ReadFreescaleBaro();

    while ( (BaroVal.u16 < (uint16)(((uint24)ADS7823_MAX*4L*3L)/4L) ) && (BaroOffsetDAC > 1) ) {
        BaroOffsetDAC -= 1;
        SetFreescaleMCP4725(BaroOffsetDAC);
        Delay1mS(10); // 10
        ReadFreescaleBaro();
        TxVal32(BaroOffsetDAC,0,HT);
        TxVal32(BaroVal.u16,0,HT);
        TxVal32((int32)FreescaleToM(BaroVal.u16), 1, 0);
        TxNextLine();
        LEDYellow_TOG;
    }

    Delay1mS(200); // wait for caps to settle
    F.BaroAltitudeValid = BaroOffsetDAC > 0;

} // SetFreescaleOffset

void ReadFreescaleBaro(void) {
    static char B[8];
    static i16u B0, B1, B2, B3;

    mS[BaroUpdate] += BARO_UPDATE_MS;

    I2CBARO.start();  // start conversion
    if ( I2CBARO.write(ADS7823_WR) != I2C_ACK ) goto ADS7823Error;
    if ( I2CBARO.write(ADS7823_CMD) != I2C_ACK ) goto ADS7823Error;
    I2CBARO.stop();

    if ( I2CBARO.blockread(ADS7823_RD, B, 8) ) goto ADS7823Error;

    // read block of 4 baro samples

    B0.b0 = B[1];
    B0.b1 = B[0];
    B1.b0 = B[3];
    B1.b1 = B[2];
    B2.b0 = B[5];
    B2.b1 = B[4];
    B3.b0 = B[7];
    B3.b1 = B[6];

    BaroVal.u16 = (uint16)16380 - ( B0.u16 + B1.u16 + B2.u16 + B3.u16 );

    F.BaroAltitudeValid = true;

    return;

ADS7823Error:
    I2CBARO.stop();

    I2CError[ADS7823_ID]++;

    // F.BaroAltitudeValid = F.HoldingAlt = false;
    if ( State == InFlight ) {
        Stats[BaroFailS]++;
        F.BaroFailure = true;
    }

} // ReadFreescaleBaro

real32 FreescaleToM(int24 p) { // decreasing pressure is increase in altitude negate and rescale to metre altitude
    return( -( p * 0.8 ) / K[BaroScale] );
}  // FreescaleToDM

void GetFreescaleBaroAltitude(void) {
    static int24 BaroPressure;

    if ( mSClock() >= mS[BaroUpdate] ) {
        ReadFreescaleBaro();
        if ( F.BaroAltitudeValid ) {
            BaroPressure = (int24)BaroVal.u16; // sum of 4 samples
            BaroRelAltitude = FreescaleToM(BaroPressure - OriginBaroPressure);

            if ( fabs( BaroRelAltitude - BaroRelAltitudeP ) > MPX4115_BARO_SANITY_CHECK_M )
                Stats[BaroFailS]++;

            BaroRelAltitude = SlewLimit(BaroRelAltitudeP, BaroRelAltitude, MPX4115_BARO_SANITY_CHECK_M);
            BaroRelAltitude = AltitudeCF(BaroRelAltitude);
            BaroRelAltitudeP = BaroRelAltitude;

            F.NewBaroValue = true;
        }
    }

} // GetFreescaleBaroAltitude

boolean IsFreescaleBaroActive(void) { // check for Freescale Barometer

    static boolean r;

    r = I2CBAROAddressResponds( ADS7823_ID );
    if ( r ) {
        BaroType = BaroMPX4115;
        r = IdentifyMCP4725();
        TrackMinI2CRate(400000);
    }
    return (r);

} // IsFreescaleBaroActive

void InitFreescaleBarometer(void) {
    static int16 BaroOriginAltitude, MinAltitude;
    static int24 BaroPressureP;

    AltitudeUpdateRate = 1000L/ADS7823_TIME_MS;

    BaroTemperature = 0;
    if ( P[BaroScale] <= 0 )
        P[BaroScale] = 56; // failsafe setting

    BaroPressure =  0;
    BaroRetries = 0;
    do {
        BaroPressureP = BaroPressure;

        SetFreescaleOffset();

        while ( mSClock() < mS[BaroUpdate] ) {};
        ReadFreescaleBaro();
        BaroPressure = (int24)BaroVal.u16;
    } while ( ( ++BaroRetries < BARO_INIT_RETRIES )
              && ( fabs( FreescaleToM(BaroPressure - BaroPressureP) ) > 5 ) );

    F.BaroAltitudeValid = BaroRetries < BARO_INIT_RETRIES;

    OriginBaroPressure = BaroPressure;

    BaroRelAltitudeP = BaroRelAltitude = 0.0;

    MinAltitude = FreescaleToM((int24)ADS7823_MAX*4);
    BaroOriginAltitude = FreescaleToM(OriginBaroPressure);
    BaroDescentAvailable = MinAltitude - BaroOriginAltitude;
    BaroClimbAvailable = -BaroOriginAltitude;

    //F.BaroAltitudeValid &= (( BaroClimbAvailable >= BARO_MIN_CLIMB )
    // && (BaroDescentAvailable <= BARO_MIN_DESCENT));

#ifdef SIMULATE
    FakeBaroRelAltitude = 0;
#endif // SIMULATE

} // InitFreescaleBarometer

// -----------------------------------------------------------

// Bosch SMD500 and BMP085 Barometers

void StartBoschBaroADC(boolean);
real32 BoschToM(real32);
int24 CompensatedBoschPressure(uint16, uint16);

void GetBoschBaroAltitude(void);
boolean IsBoschBaroActive(void);
void InitBoschBarometer(void);

// SMD500 9.5mS (T) 34mS (P)
// BMP085 4.5mS (T) 25.5mS (P) OSRS=3
#define BOSCH_TEMP_TIME_MS            11    // 10 increase to make P+T acq time ~50mS
//#define BMP085_PRESS_TIME_MS         26
//#define SMD500_PRESS_TIME_MS         34
#define BOSCH_PRESS_TIME_MS            38
#define BOSCH_PRESS_TEMP_TIME_MS       50    // pressure and temp time + overheads     

void StartBoschBaroADC(boolean ReadPressure) {
    static uint8 TempOrPress;

    if ( ReadPressure ) {
        TempOrPress = BOSCH_PRESS;
        mS[BaroUpdate] = mSClock() + BOSCH_PRESS_TIME_MS;
    } else {
        mS[BaroUpdate] = mSClock() + BOSCH_TEMP_TIME_MS;
        if ( BaroType == BaroBMP085 )
            TempOrPress = BOSCH_TEMP_BMP085;
        else
            TempOrPress = BOSCH_TEMP_SMD500;
    }

    I2CBARO.start();
    if ( I2CBARO.write(BOSCH_ID) != I2C_ACK ) goto BoschError;
    // access control register, start measurement
    if ( I2CBARO.write(BOSCH_CTL) != I2C_ACK ) goto BoschError;
    // select 32kHz input, measure temperature
    if ( I2CBARO.write(TempOrPress) != I2C_ACK ) goto BoschError;
    I2CBARO.stop();

    F.BaroAltitudeValid = true;
    return;

BoschError:
    I2CBARO.stop();

    I2CError[BOSCH_ID]++;
    F.BaroAltitudeValid = false;

} // StartBoschBaroADC

void ReadBoschBaro(void) {

    // Possible I2C protocol error - split read of ADC
    I2CBARO.start();
    if ( I2CBARO.write(BOSCH_WR) != I2C_ACK ) goto BoschError;
    if ( I2CBARO.write(BOSCH_ADC_MSB) != I2C_ACK ) goto BoschError;
    I2CBARO.start();    // restart
    if ( I2CBARO.write(BOSCH_RD) != I2C_ACK ) goto BoschError;
    BaroVal.b1 = I2CBARO.read(I2C_NACK);
    I2CBARO.stop();

    I2CBARO.start();
    if ( I2CBARO.write(BOSCH_WR) != I2C_ACK ) goto BoschError;
    if ( I2CBARO.write(BOSCH_ADC_LSB) != I2C_ACK ) goto BoschError;
    I2CBARO.start();    // restart
    if ( I2CBARO.write(BOSCH_RD) != I2C_ACK ) goto BoschError;
    BaroVal.b0 = I2CBARO.read(I2C_NACK);
    I2CBARO.stop();

    F.BaroAltitudeValid = true;
    return;

BoschError:
    I2CBARO.stop();

    I2CError[BOSCH_ID]++;

    F.BaroAltitudeValid = F.HoldingAlt = false;
    if ( State == InFlight ) {
        Stats[BaroFailS]++;
        F.BaroFailure = true;
    }

} // ReadBoschBaro

real32 BoschToM(real32 p) { // decreasing pressure is increase in altitude negate and rescale to metre altitude

    return(-((int24)p * 2.45) / K[BaroScale]);
}  // BoschToM

#define BOSCH_BMP085_TEMP_COEFF        62L
#define BOSCH_SMD500_TEMP_COEFF        50L

int24 CompensatedBoschPressure(uint16 BaroPress, uint16 BaroTemp) {
    static int24 BaroTempComp;

    if ( BaroType == BaroBMP085 )
        BaroTempComp = (BaroTemp * BOSCH_BMP085_TEMP_COEFF + 64L) >> 7;
    else
        BaroTempComp = (BaroTemp * BOSCH_SMD500_TEMP_COEFF + 8L) >> 4;

    return ((int24)BaroPress + BaroTempComp - OriginBaroPressure);

} // CompensatedBoschPressure

void GetBoschBaroAltitude(void) {

    if ( mSClock() >= mS[BaroUpdate] ) {
        ReadBoschBaro();
        if ( F.BaroAltitudeValid )
            if ( AcquiringPressure ) {
                BaroPressure = (int24)BaroVal.u16;
                AcquiringPressure = false;
            } else {
                BaroTemperature = (int24)BaroVal.u16;
                AcquiringPressure = true;

                CompBaroPressure = CompensatedBoschPressure(BaroPressure, BaroTemperature);
                BaroRelAltitude = BoschToM(CompBaroPressure);

                if ( fabs( BaroRelAltitude - BaroRelAltitudeP ) > BOSCH_BARO_SANITY_CHECK_M )
                    Stats[BaroFailS]++;

                BaroRelAltitude = SlewLimit(BaroRelAltitudeP, BaroRelAltitude, BOSCH_BARO_SANITY_CHECK_M);
                BaroRelAltitude = AltitudeCF(BaroRelAltitude);
                BaroRelAltitudeP = BaroRelAltitude;

                F.NewBaroValue = F.BaroAltitudeValid;
            }
        else
            AcquiringPressure = true;

        StartBoschBaroADC(AcquiringPressure);
    }
} // GetBoschBaroAltitude

boolean IsBoschBaroActive(void) { // check for Bosch Barometers
    static uint8 r;

    I2CBARO.start();
    if ( I2CBARO.write(BOSCH_WR) != I2C_ACK ) goto BoschInactive;
    if ( I2CBARO.write(BOSCH_TYPE) != I2C_ACK ) goto BoschInactive;
    I2CBARO.start();    // restart
    if ( I2CBARO.write(BOSCH_RD) != I2C_ACK ) goto BoschInactive;
    r = I2CBARO.read(I2C_NACK);
    I2CBARO.stop();

    if (r == BOSCH_ID_BMP085 )
        BaroType = BaroBMP085;
    else
        BaroType = BaroSMD500;

    TrackMinI2CRate(400000);

    return(true);

BoschInactive:

    return(false);

} // IsBoschBaroActive

void InitBoschBarometer(void) {
    int24 CompBaroPressureP;

    AltitudeUpdateRate = 1000L / BOSCH_PRESS_TEMP_TIME_MS;

    F.NewBaroValue = false;
    CompBaroPressure = 0;

    TxString("Temp. \tPressure\r\n");

    BaroRetries = 0;
    do { // occasional I2C misread of Temperature so keep doing it until the Origin is stable!!
        CompBaroPressureP = CompBaroPressure;
        AcquiringPressure = true;
        StartBoschBaroADC(AcquiringPressure); // Pressure

        while ( mSClock() < mS[BaroUpdate] );
        ReadBoschBaro(); // Pressure
        BaroPressure = BaroVal.u16;

        AcquiringPressure = !AcquiringPressure;
        StartBoschBaroADC(AcquiringPressure); // Temperature
        while ( mSClock() < mS[BaroUpdate] );
        ReadBoschBaro();
        BaroTemperature = BaroVal.u16;

        CompBaroPressure = CompensatedBoschPressure(BaroPressure, BaroTemperature);

        AcquiringPressure = !AcquiringPressure;
        StartBoschBaroADC(AcquiringPressure);

    } while ( ( ++BaroRetries < BARO_INIT_RETRIES ) && ( fabs(BoschToM(CompBaroPressure - CompBaroPressureP)) > 0.5 ) );

    OriginBaroPressure = CompBaroPressure;

    F.BaroAltitudeValid = BaroRetries < BARO_INIT_RETRIES;
    BaroRelAltitudeP = BaroRelAltitude = 0.0;

#ifdef SIMULATE
    FakeBaroRelAltitude = 0.0;
#endif // SIMULATE

} // InitBoschBarometer
