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

void SendLEDs(void);
void SaveLEDs(void);
void RestoreLEDs(void);
void LEDsOn(uint16);
void LEDsOff(uint16);
void LEDChaser(void);

void PCA9551Test(void);
void WritePCA9551(uint8);
boolean IsPCA9551Active(void);

void DoLEDs(void);
void PowerOutput(int8);
void LEDsAndBuzzer(void);
void InitLEDs(void);

uint16 LEDShadow, SavedLEDs, LEDPattern = 0;
uint16 PrevLEDShadow;
uint8 PrevPCA9551LEDShadow;

boolean PrevHolding = false;
const uint16 LEDChase[8] = {
    YellowM,
    RedM,
    GreenM,
    BlueM,
    DRV1M,
    DRV3M,
    DRV2M,
    DRV0M
};

void WritePCA9551(uint8 S) {
    const uint8 M = 0; // On
    //  const uint8 M = 2; // PWM0 rate
    //  const uint8 M = 3; // PWM1 rate

    const uint8 LOFF = 1; // High impedance
    static uint8 L03, L47, i;

    L03 = L47 = 0;
    for ( i = 0; i < 4; i++ ) {
        L03 <<= 2;
        if ( S & 0x80 )
            L03 |= M;
        else
            L03 |= LOFF;
        S  <<= 1;
    }

    for ( i = 0; i <4; i++ ) {
        L47 <<= 2;
        if ( S & 0x80 )
            L47 |= M;
        else
            L47 |= LOFF;
        S  <<= 1;
    }

    I2CLED.start();
    I2CLED.write(PCA9551_ID);
    I2CLED.write(0x15);
    I2CLED.write(L03);
    I2CLED.write(L47);
    I2CLED.stop();

} // WritePCA9551

void SendLEDs(void) { // 39.3 uS @ 40MHz
    static uint8 PCA9551LEDShadow;

    if ( LEDShadow != PrevLEDShadow ) {

        BlueLED = LEDShadow & BlueM;
        RedLED = LEDShadow & RedM;
        GreenLED = LEDShadow & GreenM;
        YellowLED = LEDShadow & YellowM;

        PCA9551LEDShadow = uint8(LEDShadow & 0x00ff);

        if ( F.UsingLEDDriver && ( PCA9551LEDShadow != PrevPCA9551LEDShadow ) ) {
            WritePCA9551(PCA9551LEDShadow);
            PrevPCA9551LEDShadow = PCA9551LEDShadow;
        }

        PrevLEDShadow = LEDShadow;
    }

} // SendLEDs


void SaveLEDs(void) { // one level only
    SavedLEDs = LEDShadow;
} // SaveLEDs

void RestoreLEDs(void) {
    LEDShadow = SavedLEDs;
    SendLEDs();
} // RestoreLEDs

void LEDsOn(uint16 l) {
    LEDShadow |= l;
    SendLEDs();
} // LEDsOn

void LEDsOff(uint16 l) {
    LEDShadow &= ~l;
    SendLEDs();
} // LEDsOff

void LEDChaser(void) {
#define LED_NO  (uint8)6    // skip buzzer
//#define LED_NO  (uint8)7    // all LEDs

    if ( mSClock() > mS[LEDChaserUpdate] ) {
        if ( F.HoldingAlt ) {
            LEDShadow ^= LEDChase[LEDPattern];
            if ( LEDPattern < LED_NO ) LEDPattern++;
            else LEDPattern = 0;
            LEDShadow |= LEDChase[LEDPattern];
            SendLEDs();
        } else {
            LEDShadow = SavedLEDs;
            SendLEDs();
        }

        mS[LEDChaserUpdate] = mSClock() + 50;
    }
} // LEDChaser

void DoLEDs(void) {
    if ( F.AccelerationsValid  ) LEDYellow_ON;
    else LEDYellow_OFF;

    if ( F.Signal ) {
        LEDRed_OFF;
        LEDGreen_ON;
    } else {
        LEDGreen_OFF;
        LEDRed_ON;
    }
} // DoLEDs

void PowerOutput(int8 d) {
    int8 s;
    uint16 m;

    m = 1 << d;
    for ( s=0; s < 10; s++ ) { // 10 flashes (count MUST be even!)
        LEDShadow ^= m;
        SendLEDs();
        Delay1mS(50);
    }
} // PowerOutput

void LEDsAndBuzzer(void) {
    int8 s, m;
    uint16 mask, LEDSave;

    LEDSave = LEDShadow;
    LEDShadow  = 0;
    SendLEDs();

    TxString("\r\nOutput test\r\n");
    TxString("Sequence Drv0/Buzz, Drv1, Drv2, Drv3, Aux0, Aux1, Aux2, Aux3, Y, R, G, B\r\n");
    mask = (uint8)1;
    for ( m = 1; m <= 12; m++ ) {
        for ( s = 0; s < 10; s++ ) { // 10 flashes (count MUST be even!)
            LEDShadow ^= mask;
            SendLEDs();
            Delay1mS(100);
        }
        mask <<= 1;
    }
    LEDShadow  = LEDSave;
    SendLEDs();
    TxString("Test Finished\r\n");
} // LEDsAndBuzzer

//______________________________________________________________________________________________

// LED Driver

void PCA9551Test(void) {
    static char b[8], i, r;

    TxString("\r\nPCA9551Test\r\n");

    I2CLED.start();
    I2CGYRO.write(PCA9551_ID);
    I2CGYRO.write(0x11);
    I2CLED.stop();

    if ( I2CLED.blockread(PCA9551_ID, b, 7) == 0 ) {

    TxString("0:\t0b");
    TxBin8(b[6]);
    TxNextLine();
    for (i = 0; i <6; i++ ) {
        TxVal32(i+1, 0,0);
        TxString(":\t0b");
        TxBin8(b[i]);
        TxNextLine();
    }
    }
    else
    TxString("FAILED\r\n");
    
    TxNextLine();
} // PCA9551Test

boolean IsPCA9551Active(void) {

    const char b[7] = {0x11,0x25,0x80,0x25,0x80,0x00,0x00}; // Period 1Sec., PWM 50%, ON
    boolean r;

    F.UsingLEDDriver = I2CGYROAddressResponds( PCA9551_ID );

    if ( F.UsingLEDDriver ) {
        I2CLED.blockwrite(PCA9551_ID, b, 7);
        TrackMinI2CRate(400000);
    }

    return ( F.UsingLEDDriver );
    
} //IsPCA9551Active

void InitLEDs(void) {

    boolean r;

    r = IsPCA9551Active();

    LEDShadow = SavedLEDs = LEDPattern = 0;
    PrevLEDShadow = 0x0fff;

    PrevPCA9551LEDShadow = 0;
    WritePCA9551( PrevPCA9551LEDShadow );

} // InitLEDs

