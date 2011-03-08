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

boolean I2C0AddressResponds(uint8);
#ifdef HAVE_I2C1
boolean I2C1AddressResponds(uint8);
#endif // HAVE_I2C1
void TrackMinI2CRate(uint32);
void ShowI2CDeviceName(uint8);
uint8 ScanI2CBus(void);
boolean ESCWaitClkHi(void);
void ProgramSlaveAddress(uint8);
void ConfigureESCs(void);

uint32 MinI2CRate = I2C_MAX_RATE_HZ;

//______________________________________________________________________________________________

// Software I2C

#ifdef SW_I2C

void SDelay(uint16 d) { // 1.25 + 0.0475 * n uS ~0.05uS per click

    volatile int16 v;
    for (v = 0; v < d ; v++ ) {};

}  // SDelay

//#define I2C400KHZ
#ifdef I2C400KHZ

#define SCLLowStartT  SDelay(10) // 82 for 100KHz 10 for 400KHz
#define I2CDelay2uS SDelay(10)
#define SCLLowPadT SDelay(6) // 82 for 100KHz 10 for 400KHz
#define SCLHighT SDelay(13) // 85 for 100KHz 13 for 400KHz

#else

#define SCLLowStartT  SDelay(10) // 82 for 100KHz 10 for 400KHz
#define I2CDelay2uS SDelay(10)
#define SCLLowPadT SDelay(82) // 78 for 100KHz 6 for 400KHz
#define SCLHighT SDelay(85) // 85 for 100KHz 13 for 400KHz

#endif //I2C400KHZ

#define I2CSDALow {I2C0SDA.write(0);I2C0SDA.output();SCLLowPadT;}
#define I2CSDAFloat {I2C0SDA.input();SCLLowPadT;}
#define I2CSCLLow {I2C0SCL.write(0);I2C0SCL.output();}
#define I2CSCLFloat {I2C0SCL.input();SCLHighT;}

void MyI2C::frequency(uint32 f) {
// delay depending on rate
} // frequency

void MyI2C::start(void) {

    I2CSDAFloat;
    r = waitclock();
    I2CSDALow;
    SCLLowStartT;
    I2CSCLLow;
} // start

void MyI2C::stop(void) {

    I2CSDALow;
    r = waitclock();
    I2CSDAFloat;
    SCLLowStartT;

} // stop

boolean MyI2C::waitclock(void) {
    static uint32 s;

    I2CSCLFloat;        // set SCL to input, output a high
    s = 0;
    while ( I2C0SCL.read() == 0 )
        if ( ++s > 16000 ) { // ~1mS
            Stats[I2CFailS]++;
            return (false);
        }
    return( true );
} // waitclock

uint8 MyI2C::read(uint8 ack) {
    static uint8 s, d;

    I2CSDAFloat;
    d = 0;
    s = 8;
    do {
        if ( waitclock() ) {
            d <<= 1;
            if ( I2C0SDA.read() ) d |= 1;
            I2CSCLLow;
            I2CDelay2uS;
        } else
            return( 0 );
    } while ( --s );

    if ( ack == I2C_NACK )
        I2C0SDA.write(0xffff); // Port write with mask selecting SDA - messy
    else
        I2C0SDA.write(0); 
    I2C0SDA.output();

    SCLLowPadT;

    if ( waitclock() ) {
        I2CSCLLow;
        return( d );
    } else
        return( 0 );

} // read

uint8 MyI2C::write(uint8 d) {
    static uint8 s, r;

    for ( s = 0; s < 8; s++) {
        if ( d & 0x80 ) {
            I2CSDAFloat;
        } else {
            I2CSDALow;
        }

        if ( waitclock() ) {
            I2CSCLLow;
            d <<= 1;
        } else
            return(I2C_NACK);
    }

    I2CSDAFloat;
    if ( waitclock() ) {
        r = I2C0SDA.read() != 0;
        I2CSDALow;// kill runt pulses
        I2CSCLLow;
        return ( r );
    } else {
//   I2CSCLLow;
        return(I2C_NACK);
    }

} // write

uint8 MyI2C::blockread(uint8 a, char* S, uint8 l) {
    static uint8 b;
    static boolean err;

    I2C0.start();
    err = I2C0.write(a|1) != I2C_ACK;
    for (b = 0; b < (l - 1); b++)
        S[b] = I2C0.read(I2C_ACK);
    S[l-1] = I2C0.read(I2C_NACK);
    I2C0.stop();

    return( err );
} // blockread

void MyI2C::blockwrite(uint8 a, const char* S, uint8 l) {
    static uint8 b;
    static boolean r;

    I2C0.start();
    r = I2C0.write(a) == I2C_ACK;  // use this?
    for ( b = 0; b < l; b++ )
        r |= I2C0.write(S[b]);
    I2C0.stop();

} // blockwrite

#endif // SW_I2C

//______________________________________________________________________________________________

void TrackMinI2CRate(uint32 r) {
    if ( r < MinI2CRate )
        MinI2CRate = r;
} // TrackMinI2CRate

void ShowI2CDeviceName(uint8 d) {
    TxChar(' ');
    switch ( d  ) {
        case ADXL345_ID:
            TxString("ADXL345 Acc");
            break;
        case ITG3200_ID:
            TxString("ITG3200 Gyro");
            break;
        case HMC5843_ID:
            TxString("HMC5843 Magnetometer");
            break;
        case HMC6352_ID:
            TxString("HMC6352 Compass");
            break;
        case ADS7823_ID:
            TxString("ADS7823 ADC");
            break;
        case MCP4725_ID_0xCC:
            TxString("MCP4725 DAC");
            break;
        case MCP4725_ID_0xC8:
            TxString("MCP4725 DAC");
            break;
        case BOSCH_ID:
            TxString("Bosch Baro");
            break;
        case TMP100_ID:
            TxString("TMP100 Temp");
            break;
        case PCA9551_ID:
            TxString("PCA9551 LED");
            break;
        case LISL_ID:
            TxString("LIS3L Acc");
            break;
        default:
            break;
    } // switch
    TxChar(' ');

} // ShowI2CDeviceName

boolean I2C0AddressResponds(uint8 s) {
    static boolean r;
    I2C0.start();
    r = I2C0.write(s) == I2C_ACK;
    I2C0.stop();
    return (r);
} // I2C0AddressResponds

#ifdef HAVE_IC1
boolean I2C1AddressResponds(uint8 s) {
    static boolean r;
    I2C1.start();
    r = I2C1.write(s) == I2C_ACK;
    I2C1.stop();
    return (r);
} // I2C1AddressResponds
#endif // HAVE_IC1

uint8 ScanI2CBus(void) {
    uint8 s;
    uint8 d;

    d = 0;
    TxString("Buss 0\r\n");
    for ( s = 0x10 ; s <= 0xf6 ; s += 2 ) {
        if (  I2C0AddressResponds(s) ) {
            d++;
            DebugPin = 1;
            TxString("\t0x");
            TxValH(s);
            ShowI2CDeviceName( s );
            TxNextLine();
            DebugPin = 0;
        }
        Delay1mS(2);
    }

#ifdef HAVE_I2C1
    TxString("Buss 1\r\n");
    for ( s = 0x10 ; s <= 0xf6 ; s += 2 ) {
        if (  I2C0AddressResponds(s) ) {
            d++;
            TxString("\t0x");
            TxValH(s);
            ShowI2CDeviceName( s );
            TxNextLine();
        }
        Delay1mS(2);
    }
#endif // HAVE_I2C1

    PCA9551Test();

    return(d);
} // ScanI2CBus

void ProgramSlaveAddress(uint8 a) {
    static uint8 s;

    for (s = 0x10 ; s < 0xf0 ; s += 2 ) {
        I2CESC.start();
        if ( I2CESC.read(s) == I2C_ACK )
            if ( s == a ) {   // ESC is already programmed OK
                I2CESC.stop();
                TxString("\tESC at SLA 0x");
                TxValH(a);
                TxString(" is already programmed OK\r\n");
                return;
            } else {
                if ( I2CESC.read(0x87) == I2C_ACK ) // select register 0x07
                    if ( I2CESC.write( a ) == I2C_ACK ) { // new slave address
                        I2CESC.stop();
                        TxString("\tESC at SLA 0x");
                        TxValH(s);
                        TxString(" reprogrammed to SLA 0x");
                        TxValH(a);
                        TxNextLine();
                        return;
                    }
            }
        I2CESC.stop();
    }
    TxString("\tESC at SLA 0x");
    TxValH(a);
    TxString(" no response - check cabling and pullup resistors!\r\n");
} // ProgramSlaveAddress

boolean CheckESCBus(void) {
    return ( true );
} // CheckESCBus

void ConfigureESCs(void) {
    int8 m;

    if ( (int8)P[ESCType] == ESCYGEI2C ) {
        TxString("\r\nProgram YGE ESCs\r\n");
        for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
            TxString("Connect ONLY M");
            TxChar('1' + m);
            TxString(" ESC, then press any key \r\n");
            while ( PollRxChar() != 'x' ); // UAVPSet uses 'x' for any key button
            //    TxString("\r\n");
            ProgramSlaveAddress( 0x62 + ( m*2 ));
        }
        TxString("\r\nConnect ALL ESCs and power-cycle the Quadrocopter\r\n");
    } else
        TxString("\r\nYGEI2C not selected as ESC?\r\n");
} // ConfigureESCs



