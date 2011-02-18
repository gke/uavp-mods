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
void GetTemperature(void);
void InitTemperature(void);

i16u AmbientTemperature;

void GetTemperature(void) {

    I2CTEMP.start();
    if ( I2CTEMP.write(TMP100_RD) != I2C_ACK ) goto Terror;
    AmbientTemperature.b1 = I2CTEMP.read(I2C_ACK);
    AmbientTemperature.b0 = I2CTEMP.read(I2C_NACK);
    I2CTEMP.stop();

    // Top 9 bits 0.5C res. scale to 0.1C
    AmbientTemperature.i16 = SRS16(    AmbientTemperature.i16, 7) * 5;
    if ( AmbientTemperature.i16 > Stats[MaxTempS])
        Stats[MaxTempS] = AmbientTemperature.i16;
    else
        if ( AmbientTemperature.i16 < Stats[MinTempS] )
            Stats[MinTempS] = AmbientTemperature.i16;
    return;

Terror:
    I2CTEMP.stop();
    AmbientTemperature.i16 = 0;

    return;
} // GetTemperature

void InitTemperature(void) {
    I2CTEMP.start();
    if(  I2CTEMP.write(TMP100_WR) != I2C_ACK ) goto Terror;
    if(  I2CTEMP.write(TMP100_CMD) != I2C_ACK ) goto Terror;
    if(  I2CTEMP.write(TMP100_CFG) != I2C_ACK ) goto Terror;
    I2CTEMP.stop();

    I2CTEMP.start();
    if( I2CTEMP.write(TMP100_WR) != I2C_ACK ) goto Terror;
    if(  I2CTEMP.write(TMP100_TMP) != I2C_ACK ) goto Terror; // Select temperature
    I2CTEMP.stop();

    GetTemperature();
    
    return;
    
 Terror:
    I2CTEMP.stop();
    AmbientTemperature.i16 = 0;

} // InitTemperature
