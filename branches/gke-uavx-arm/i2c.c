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

uint32 MinI2CRate = I2C_MAX_RATE_HZ;

void TrackMinI2CRate(uint32 r) {
 if ( r < MinI2CRate )
    MinI2CRate = r;
} // TrackMinI2CRate

void ShowI2CDeviceName(uint8 d)
{
    TxChar(' ');
    switch ( d  ) {
    case ADXL345_ID: TxString("ADXL345 Acc"); break;
    case ITG3200_ID: TxString("ITG3200 Gyro"); break;
    case HMC5843_ID: TxString("HMC5843 Magnetometer"); break;
    case HMC6352_ID: TxString("HMC6352 Compass"); break;
    case ADS7823_ID: TxString("ADS7823 ADC"); break;
    case MCP4725_ID: TxString("MCP4725 DAC"); break;
    case BOSCH_ID: TxString("Bosch Baro"); break;
    case TMP100_ID: TxString("TMP100 Temp"); break;
    case PCA9551_ID: TxString("PCA9551 LED");break;
    case LISL_ID: TxString("LIS3L Acc"); break;
    default: break;
    } // switch
    TxChar(' ');

} // ShowI2CDeviceName

uint8 ScanI2CBus(void) {
    uint8 s;
    uint8 d;

    d = 0;

    TxString("Buss 0\r\n");
    for ( s = 0x10 ; s <= 0xf6 ; s += 2 ) {
        I2C0.start();
        if ( I2C0.write(s) == I2C_ACK ) {
            d++;
            TxString("\t0x");
            TxValH(s);
            ShowI2CDeviceName( s );
            TxNextLine();
        }
        I2C0.stop();

        Delay1mS(2);
    }
    
    /* 
    TxString("Buss 1\r\n");
    for ( s = 0x10 ; s <= 0xf6 ; s += 2 ) {
        I2C1.start();
        if ( I2C1.write(s) == I2C_ACK ) {
            d++;
            TxString("\t0x");
            TxValH(s);
            TxNextLine();
        }
        I2C1.stop();

        Delay1mS(2);
    }
    */

    PCA9551Test();

    return(d);
} // ScanI2CBus

void ProgramSlaveAddress(uint8 addr) {
    static uint8 s;

    for (s = 0x10 ; s < 0xf0 ; s += 2 ) {
        I2CESC.start();
        if ( I2CESC.read(s) == I2C_ACK )
            if ( s == addr ) {   // ESC is already programmed OK
                I2CESC.stop();
                TxString("\tESC at SLA 0x");
                TxValH(addr);
                TxString(" is already programmed OK\r\n");
                return;
            } else {
                if ( I2CESC.read(0x87) == I2C_ACK ) // select register 0x07
                    if ( I2CESC.write( addr ) == I2C_ACK ) { // new slave address
                        I2CESC.stop();
                        TxString("\tESC at SLA 0x");
                        TxValH(s);
                        TxString(" reprogrammed to SLA 0x");
                        TxValH(addr);
                        TxNextLine();
                        return;
                    }
            }
        I2CESC.stop();
    }
    TxString("\tESC at SLA 0x");
    TxValH(addr);
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
            TxString("Connect ONLY ");
            switch ( m ) {
#ifdef HEXACOPTER
                    not yet!
#else
                case 0 :
                    TxString("Front");
                    break;
                case 1 :
                    TxString("Back");
                    break;
                case 2 :
                    TxString("Right");
                    break;
                case 3 :
                    TxString("Left");
                    break;
#endif // HEXACOPTER
            }
            TxString(" ESC, then press any key \r\n");
            while ( PollRxChar() != 'x' ); // UAVPSet uses 'x' for any key button
            //    TxString("\r\n");
            ProgramSlaveAddress( 0x62 + ( m*2 ));
        }
        TxString("\r\nConnect ALL ESCs and power-cycle the Quadrocopter\r\n");
    } else
        TxString("\r\nYGEI2C not selected as ESC?\r\n");
} // ConfigureESCs



