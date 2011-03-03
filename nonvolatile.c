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

// transfers to Flash seem to be ~35KByte/Sec

void CheckSDCardValid(void);

void CreateLogfile(void);
void CloseLogfile(void);
void TxLogChar(uint8);

void WritePXImagefile(void);
boolean ReadPXImagefile(void);

int8 ReadPX(uint16);
int16 Read16PX(uint16);
int32 Read32PX(uint16);
void WritePX(uint16, int8);
void Write16PX(uint16, int16);
void Write32PX(uint16, int32);

FILE *pxfile = NULL;
FILE *newpxfile = NULL;

const int PX_LENGTH = 2048;
int8 PX[PX_LENGTH], PXNew[PX_LENGTH];

void CheckSDCardValid(void) {

    F.SDCardValid =  SDCard.initialise_card() != 0;

} // CheckSDCardValid

void WritePXImagefile(void) {
    static uint16 a;
    static int32 CheckSum;
    static int8 v, i;

    UpdateRTC();

    if ( F.PXImageStale ) {
        if ( F.SDCardValid )
            newpxfile = fopen("/SDCard/Params.txt", "w");
        else
            newpxfile = fopen("/local/Params.txt", "w");

        if ( newpxfile != NULL ) {
            CheckSum = 0;
            for ( a = 0; a < PX_LENGTH; a++ ) {
                v = PX[a];
                CheckSum += (int32)v;
                fprintf(newpxfile, "%i \r\n",  v);
            }
            fprintf(newpxfile, "%li \r\n ",  -CheckSum );

            i=0;
            while ( RTCString[i] != 0 )
                fprintf(newpxfile, "%c", RTCString[i++]);
            fprintf(newpxfile, "\r\n");

            F.PXImageStale = false;
            fclose(newpxfile);
        }
    }

} // WritePXIMagefile

boolean ReadPXImagefile(void) {
    static uint16 a;
    static int32 v, CheckSum;
    static boolean OK;

    if ( F.SDCardValid )
        pxfile = fopen("/SDCard/Params.txt", "r");
    else
        pxfile = fopen("/local/Params.txt", "r");

    OK = false;
    if ( pxfile != NULL ) {
        CheckSum = 0;
        a = 0;
        for ( a = 0; a < PX_LENGTH ; a++ ) {
            r = fscanf(pxfile, "%i ", &v);
            CheckSum += v;
            PXNew[a] = (int8)v;
        }
        r = fscanf(pxfile, "%li ", &v);
        CheckSum += (int32)v;
        OK = CheckSum == 0;

        fclose(pxfile);

        if ( OK ) {
            for ( a = 0; a < PX_LENGTH; a++ )
                PX[a] = PXNew[a];
            F.PXImageStale = false;
        }
    }

    return( OK );

} // ReadPXImagefile

void CreateLogfile(void) {

#ifndef SUPPRESS_SDCARD

    static int16 i;

    UpdateRTC();

    if ( F.SDCardValid )
        strftime(RTCLogfile, 32, "/SDCard/L%H-%M.log", RTCTime );
    else
        strftime(RTCLogfile, 32, "/local/L%H-%M.log", RTCTime );

    logfile = fopen(RTCLogfile, "w");

    LogfileIsOpen = logfile != NULL;
    if ( LogfileIsOpen ) {
        i = 0;
        while ( RTCString[i] != 0 )
            TxLogChar(RTCString[i++]);
        TxLogChar(CR);
        TxLogChar(LF);
    }
    LogChars = 0;

#endif // !SUPPRESS_SDCARD

} // CreateLogfile

void CloseLogfile(void) {
    fclose(logfile);
} // CloseLog

void TxLogChar(uint8 ch) {

#ifndef SUPPRESS_SDCARD
    if ( LogfileIsOpen ) {
        LogfileIsOpen =  fprintf(logfile, "%c",  ch) > 0;
        if ( !LogfileIsOpen )
            CloseLogfile();
    }
#endif // !SUPPRESS_SDCARD
} // TxLogChar


int8 ReadPX(uint16 a) {
    static int8 b;
    b = PX[a];
    return(b);
} // ReadPX

int16 Read16PX(uint16 a) {
    static i16u Temp16;

    Temp16.b0 = ReadPX(a);
    Temp16.b1 = ReadPX(a + 1);

    return ( Temp16.i16 );
} // Read16P

int32 Read32PX(uint16 a) {
    static i32u Temp32;

    Temp32.b0 = ReadPX(a);
    Temp32.b1 = ReadPX(a + 1);
    Temp32.b2 = ReadPX(a + 2);
    Temp32.b3 = ReadPX(a + 3);

    return ( Temp32.i32 );
} // Read32P

void WritePX(uint16 a, int8 d) {
    if ( PX[a] != d ) {
        PX[a] = d;
        F.PXImageStale = true;
    }
} // WritePX

void Write16PX(uint16 a, int16 d) {
    static i16u Temp16;

    Temp16.i16 = d;
    WritePX(a, Temp16.b0);
    WritePX(a + 1, Temp16.b1);

} // Write16P

void Write32PX(uint16 a, int32 d) {
    static i32u Temp32;

    Temp32.i32 = d;
    WritePX(a, Temp32.b0);
    WritePX(a + 1, Temp32.b1);
    WritePX(a + 2, Temp32.b2);
    WritePX(a + 3, Temp32.b3);

} // Write16PX

