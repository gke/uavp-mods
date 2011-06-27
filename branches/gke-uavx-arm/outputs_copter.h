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

void WriteT580ESC(uint8, uint8, uint8);
void WriteT580ESCs(uint8,  uint8, uint8, uint8, uint8);
void T580ESCs(uint8, uint8, uint8, uint8);
void OutSignals(void);

void OutSignals(void) {
    static int8 m;
    static uint8 r;

    if ( !F.MotorsArmed )
        StopMotors();

    for ( m = 0; m < (uint8)6; m++)
        PWM[m] = Limit(PWM[m], 0, ESCMax);

#if !( defined SIMULATE | defined TESTING )

    Out0.pulsewidth_us(1000 + (int16)( PWM[FrontC] * PWMScale ) );
    Out1.pulsewidth_us(1000 + (int16)( PWM[RightC] * PWMScale ) );
    Out2.pulsewidth_us(1000 + (int16)( PWM[BackC] * PWMScale ) );
    Out3.pulsewidth_us(1000 + (int16)( PWM[LeftC] * PWMScale ) );

#ifdef MULTICOPTER
    // in X3D and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL.
    // ACK (r) not checked as no recovery is possible.
    // Octocopters may have ESCs paired with common address so ACK is meaningless.
    // All motors driven with fourth motor ignored for Tricopter.

    switch ( P[ESCType ] ) {
        case  ESCHolger:
            for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
                I2CESC.start();
                r = I2CESC.write(0x52 + ( m*2 ));        // one command, one data byte per motor
                r += I2CESC.write( PWM[m] );
                ESCI2CFail[m] += r;
                I2CESC.stop();
            }
            break;
        case  ESCLRCI2C:
            T580ESCs(PWM[FrontC], PWM[BackC], PWM[RightC], PWM[LeftC]);
            break;
        case  ESCYGEI2C:
            for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
                I2CESC.start();
                r = I2CESC.write(0x62 + ( m*2) );    // one cmd, one data byte per motor
                r += I2CESC.write( PWM[m] * 0.5 );
                ESCI2CFail[m] += r;
                I2CESC.stop();
            }
            break;
        case ESCX3D:
            I2CESC.start();
            r = I2CESC.write(0x10);                // one command, 4 data bytes
            r += I2CESC.write( PWM[FrontC] );
            r += I2CESC.write( PWM[RightC] );
            r += I2CESC.write( PWM[BackC] );
            r += I2CESC.write( PWM[LeftC] );
            ESCI2CFail[0] += r;
            //  other ESCs if a Hexacopter
            I2CESC.stop();
            break;
        default:
            break;
    } // switch
#endif //  MULTICOPTER

#endif // !(SIMULATE | TESTING)

} // OutSignals

//________________________________________________________________

// LotusRC T580 I2C ESCs

enum T580States { T580Starting = 0, T580Stopping = 1, T580Constant = 2 };

boolean T580Running = false;
uint32 T580Available = 0;

void WriteT580ESC(uint8 a, uint8 s, uint8 d2) {

    I2CESC.start();
    I2CESC.write(a);
    I2CESC.write(0xa0 | s );
    I2CESC.write(d2);
    I2CESC.stop();

} // WriteT580ESC

void WriteT580ESCs(uint8 s,  uint8 f, uint8 b, uint8 r, uint8 l) {

//   static uint32 Timeout;

//     Timeout = mSClock() + 2;
    if ( ( s == T580Starting ) || ( s == T580Stopping ) ) {
//       if ( s == T580Starting )
//           Timeout = timer.read_us() + 2000;//436000;
//       else
//           Timeout = timer.read_us() + 2000;//403000;
        WriteT580ESC(0xd0, s, 0);
        WriteT580ESC(0xd2, s, 0);
        WriteT580ESC(0xd4, s, 0);
        WriteT580ESC(0xd6, s, 0);
    } else {
//        Timeout = mSClock() + 2;
        WriteT580ESC(0xd2, s, f);
        WriteT580ESC(0xd4, s, b);
        WriteT580ESC(0xd6, s, r);
        WriteT580ESC(0xd0, s, l);
    }
//    while ( mSClock() < Timeout);

} // WriteT580ESCs

void T580ESCs(uint8 f, uint8 b, uint8 r, uint8 l) {

    static boolean Run;
    static uint8 i;

    Run = ( f > 0 ) || ( b > 0 ) || ( r > 0 ) || ( l > 0 );

    if ( T580Running )
        if ( Run )
            WriteT580ESCs(T580Constant, f, r, b, l);
        else {
            WriteT580ESCs(T580Stopping, 0, 0, 0, 0);
            Delay1mS(2);
            for ( i = 0; i < 50; i++ ) {
                WriteT580ESCs(T580Constant, 0, 0, 0, 0);
                Delay1mS(2);
            }
        }
    else
        if ( Run ) {
            for ( i = 0; i < 50; i++ ) {
                WriteT580ESCs(T580Constant, 0, 0, 0, 0);
                Delay1mS(2);
            }
            WriteT580ESCs(T580Starting, 0, 0, 0, 0);
        } else
            WriteT580ESCs(T580Constant, f, r, b, l);

    T580Running = Run;

} // T580ESCs





