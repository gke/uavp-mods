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

void UpdateRTC(void);
void InitHarness(void);

LocalFileSystem Flash("local");

// connections to ARM
                                        // 1 GND
                                        // 2 4.5-9V
                                        // 3 VBat
                                        // 4 NReset
                                        
//SPI SPI0(p5, p6, p7);                 // 5 SPI MOSI, 6 SPI MOSO, 7 SPI CLK
//DigitalOut SPICS(p8);                 // 8
SDFileSystem SDCard(p5, p6, p7, p8, "SDCard");
                                        
//I2C I2C1(p9, p10);                    // 9 Tx / I2C SDA, 10 Rx / I2C SCL
SerialBuffered TelemetrySerial(p9, p10);
DigitalIn Armed(p11);                   // 11 SPI MOSI
DigitalOut PWMCamPitch(p12);            // 12 SPI MOSO
                                        
Serial GPSSerial(p13, p14);             // 13 Tx1 / SPI CLK, 14 Rx1

AnalogIn PitchADC(p15);                 // 15 AN0
AnalogIn RollADC(p16);                  // 16 AN1
AnalogIn YawADC(p18);                   // 17 AN2 (has DAC capability)

AnalogIn RangefinderADC(p17);           // 18 AN3
AnalogIn BatteryCurrentADC(p19);        // 19 AN4
AnalogIn BatteryVoltsADC(p20);          // 20 AN5 

PwmOut Out0(p21);                       // 21
PwmOut Out1(p22);                       // 22
PwmOut Out2(p23);                       // 23
PwmOut Out3(p24);                       // 24

PwmOut Out4(p25);                       // 25
PwmOut Out5(p26);                       // 26

I2C I2C0(p28, p27);                     // 27, 28

DigitalIn RCIn(p29);                    // 29 CAN
DigitalOut PWMCamRoll(p30);             // 30 CAN 

//Serial TelemetrySerial(USBTX, USBRX);   
                                        // 31 USB +, 32 USB -
                                        // 34 -37 Ethernet
                                        // 38 IF +
                                        // 39 IF -
                                        // 40 3.3V Out 
                
// order L-R end of card                        
DigitalOut BlueLED(LED1);
DigitalOut GreenLED(LED2);
DigitalOut RedLED(LED3);
DigitalOut YellowLED(LED4);

InterruptIn RCInterrupt(p29);

char RTCString[32], RTCLogfile[32];
struct tm* RTCTime;

void UpdateRTC(void)
{
    time_t s = time(NULL);
    RTCTime = localtime(&s);
    strftime(RTCString, 32, "%a %b %d %H:%M:%S %Y", RTCTime );   
} // UpdateRTCString

void InitHarness(void){

    I2C0.frequency(I2C_MAX_RATE_HZ);
  
    TelemetrySerial.baud(115200);
    TelemetrySerial.set_tx_buffer_size(512);  

    GPSSerial.baud(115200); // reduce baud rate to lighten interrupt traffic?
    
    PWMCamRoll.write(false);
    PWMCamPitch.write(false);
    
    Armed.mode(PullUp);

    CheckSDCardValid();
    
    UpdateRTC();

} // InitHarness