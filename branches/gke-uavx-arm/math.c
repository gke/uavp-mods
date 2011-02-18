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

int16 SRS16(int16, uint8);
int32 SRS32(int32, uint8);
real32 Make2Pi(real32);
real32 MakePi(real32);
int16 Table16(int16, const int16 *);

real32 VDot(real32 v1[3], real32 v2[3]);
void VCross(real32 VOut[3], real32 v1[3], real32 v2[3]);
void VScale(real32 VOut[3], real32 v[3], real32 s);
void VAdd(real32 VOut[3],real32 v1[3], real32 v2[3]);
void VSub(real32 VOut[3],real32 v1[3], real32 v2[3]);

int16 SRS16(int16 x, uint8 s) {
    static i16u Temp;

    if ( s == (uint8)8 ) {
        Temp.i16 = x;
        return( (int16) Temp.i1 );
    } else
        return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS16

int32 SRS32(int32 x, uint8 s) {
    static i32u Temp;

    if ( s == (uint8)8 ) {
        Temp.i32 = x;
        return( (int32)Temp.i3_1 );
    } else
        return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS32

real32 Make2Pi(real32 A) {
    while ( A < 0 ) A += TWOPI;
    while ( A >= TWOPI ) A -= TWOPI;
    return( A );
} // Make2Pi

real32 MakePi(real32 A) {
    while ( A < -PI ) A += TWOPI;
    while ( A >= PI ) A -= TWOPI;
    return( A );
} // MakePi

int16 Table16(int16 Val, const int16 *T) {
    static uint8 Index,Offset;
    static int16 Temp, Low, High;

    Index = (uint8) (Val >> 4);
    Offset = (uint8) (Val & 0x0f);
    Low = T[Index];
    High = T[++Index];
    Temp = (High-Low) * Offset;

    return( Low + SRS16(Temp, 4) );
} // Table16


real32 VDot(real32 v1[3], real32 v2[3]) {
    static real32 op;
    static uint8 i;

    op = 0.0;
    for ( i = 0; i < (uint8)3; i++ )
        op += v1[i] * v2[i];

    return op;
} // VDot

void VCross(real32 VOut[3], real32 v1[3], real32 v2[3]) {
    VOut[0]= (v1[1] * v2[2]) - (v1[2] * v2[1]);
    VOut[1]= (v1[2] * v2[0]) - (v1[0] * v2[2]);
    VOut[2]= (v1[0] * v2[1]) - (v1[1] * v2[0]);
} // VCross

void VScale(real32 VOut[3], real32 v[3], real32 s) {
    static uint8 i;

    for ( i = 0; i < (uint8)3; i++ )
        VOut[i] = v[i] * s;
} // VScale

void VAdd(real32 VOut[3],real32 v1[3], real32 v2[3]) {
    static uint8 i;

    for ( i = 0; i < (uint8)3; i++ )
        VOut[i] = v1[i] + v2[i];
} // VAdd

void VSub(real32 VOut[3],real32 v1[3], real32 v2[3]) {
    static uint8 i;

    for ( i = 0; i < (uint8)3; i++ )
        VOut[i] = v1[i] - v2[i];
} // VSub

