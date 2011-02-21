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

// Interrupt Routines


const int16 MIN_PPM_SYNC_PAUSE = 2400;      // uS

// no less than 1500

extern void InitTimersAndInterrupts(void);

void enableTxIrq0(void);
void disableTxIrq0(void);
void enableTxIrq1(void);
void disableTxIrq1(void);

void RCISR(void);
void RCNullISR(void);
void RCTimeoutISR(void);
void CamPWMOnISR(void);
void CamRollPWMOffISR(void);
void CamPitchPWMOffISR(void);
void TelemetryInISR(void);
void GPSInISR(void);

Timer timer;
Timeout CamRollTimeout, CamPitchTimeout;
Ticker CameraTicker;
Timeout RCTimeout;

uint32    mS[CompassUpdate + 1];

uint32    PrevEdge, CurrEdge;
uint8     Intersection, PrevPattern, CurrPattern;
uint32    Width;
int16     PPM[MAX_CONTROLS];
int8      PPM_Index;
int24     PauseTime;
uint8     RxState;
int8      State, FailState;
int8      SignalCount;
uint16    RCGlitches;
int16     PWMCycles = 0;

void enableTxIrq0(void) {
    LPC_UART0->IER |= 0x0002;
} // enableTxIrq0

void disableTxIrq0(void) {
    LPC_UART0->IER &= ~0x0002;
} // disableTxIrq0

void enableTxIrq1(void) {
    LPC_UART1->IER |= 0x0002;
} // enableTxIrq1

void disableTxIrq1(void) {
    LPC_UART1->IER &= ~0x0002;
}  // disableTxIrq1

void InitTimersAndInterrupts(void) {
    static int8 i;

    timer.start();

    GPSSerial.attach(GPSInISR, GPSSerial.RxIrq);

    InitTelemetryPacket();
    //TelemetrySerial.attach(TelemetryInISR, TelemetrySerial.RxIrq);

    RCPositiveEdge = true;

    RCTimeout.attach_us(& RCTimeoutISR, 50000);

    CameraTicker.attach_us(&CamPWMOnISR, 22500);

    for (i = Clock; i<= CompassUpdate; i++)
        mS[i] = 0;

} // InitTimersAndInterrupts

void RCISR(void) {
    CurrEdge = timer.read_us();

    Width = CurrEdge - PrevEdge;
    PrevEdge = CurrEdge;

    if ( Width > MIN_PPM_SYNC_PAUSE ) {  // A pause  > 5ms
        PPM_Index = 0;                        // Sync pulse detected - next CH is CH1
        F.RCFrameOK = true;
        F.RCNewValues = false;
        PauseTime = Width;
    } else
        if (PPM_Index < RC_CONTROLS) { // Width in 1us ticks.
            if ( ( Width >= RC_MIN_WIDTH_US ) && ( Width <= RC_MAX_WIDTH_US ) )
                PPM[PPM_Index] = Width - 1000;
            else {
                // preserve old value i.e. default hold
                RCGlitches++;
                F.RCFrameOK = false;
            }

            PPM_Index++;
            // MUST demand rock solid RC frames for autonomous functions not
            // to be cancelled by noise-generated partially correct frames
            if ( PPM_Index == RC_CONTROLS ) {
                if ( F.RCFrameOK ) {
                    F.RCNewValues = true;
                    SignalCount++;
                } else {
                    F.RCNewValues = false;
                    SignalCount -= RC_GOOD_RATIO;
                }

                SignalCount = Limit(SignalCount, -RC_GOOD_BUCKET_MAX, RC_GOOD_BUCKET_MAX);
                F.Signal = SignalCount > 0;

                if ( F.Signal )
                    mS[LastValidRx] = timer.read_ms();

                RCTimeout.attach_us(& RCTimeoutISR, RC_SIGNAL_TIMEOUT_MS*1000);
            }
        }

} // RCISR

void RCNullISR(void) {
// discard edge
} // RCNullISR

void CamPWMOnISR(void) {

    PWMCamRoll.write(true);
    CamRollTimeout.attach_us(&CamRollPWMOffISR, CamRollPulseWidth);
    PWMCamPitch.write(true);
    CamPitchTimeout.attach_us(&CamPitchPWMOffISR, CamPitchPulseWidth);

} // CamPWMOnISR

void CamRollPWMOffISR(void) {

    PWMCamRoll.write(false);
    CamRollTimeout.detach();

} // CamRollPWMOffISR

void CamPitchPWMOffISR(void) {

    PWMCamPitch.write(false);
    CamPitchTimeout.detach();

} // CamPitchPWMOffISR

void RCTimeoutISR(void) {
    if ( F.Signal ) {
        F.Signal = false;
        SignalCount = -RC_GOOD_BUCKET_MAX;
    }
} // RCTimeoutISR

void GPSInISR(void) {
    RxGPSPacket( GPSSerial.getc());
} // GPSInISR

void TelemetryInISR(void) {
    RxTelemetryPacket(TelemetrySerial.getc());
} // TelemetryInISR

