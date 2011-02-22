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


void OutSignals(void) {
    static int8 m;
    static uint8 r;

#if !( defined SIMULATE | defined TESTING )

    if ( !F.MotorsArmed )
        StopMotors();

    PWM[FrontC] = TC(PWM[FrontC]);
    PWM[LeftC] = TC(PWM[LeftC]);
    PWM[RightC] = TC(PWM[RightC]);
#ifdef TRICOPTER
    PWM[BackC] = Limit(PWM[BackC], 1, OUT_MAXIMUM);
#else
    PWM[BackC] = TC(PWM[BackC]);
#endif

    Out0.pulsewidth_us(1000 + (int16)( PWM[FrontC] * PWMScale ) );
    Out1.pulsewidth_us(1000 + (int16)( PWM[RightC] * PWMScale ) );
    Out2.pulsewidth_us(1000 + (int16)( PWM[BackC] * PWMScale ) );
    Out3.pulsewidth_us(1000 + (int16)( PWM[LeftC] * PWMScale ) );
                      
#ifdef MULTICOPTER
    // in X3D and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL.
    // ACK (r) not checked as no recovery is possible.
    // Octocopters may have ESCs paired with common address so ACK is meaningless.
    // All motors driven with fourth motor ignored for Tricopter.

    if ( P[ESCType] ==  ESCHolger )
        for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
            I2CESC.start();
            r = I2CESC.write(0x52 + ( m*2 ));        // one command, one data byte per motor
            r += I2CESC.write( PWM[m] );
            ESCI2CFail[m] += r;
            I2CESC.stop();
        }
    else
        if ( P[ESCType] == ESCYGEI2C )
            for ( m = 0 ; m < NoOfI2CESCOutputs ; m++ ) {
                I2CESC.start();
                r = I2CESC.write(0x62 + ( m*2) );    // one cmd, one data byte per motor
                r += I2CESC.write( PWM[m] * 0.5 );
                ESCI2CFail[m] += r;
                I2CESC.stop();
            }
        else
           if ( P[ESCType] == ESCX3D ) {
                I2CESC.start();
                r = I2CESC.write(0x10);                // one command, 4 data bytes
                r += I2CESC.write( PWM[FrontC] );
                r += I2CESC.write( PWM[RightC] );
                r += I2CESC.write( PWM[BackC] );
                r += I2CESC.write( PWM[LeftC] );
                ESCI2CFail[0] += r;
                //  other ESCs if a Hexacopter
                I2CESC.stop();
            }
#endif //  MULTICOPTER

#else

    PWM[FrontC] = Limit(PWM[FrontC], ESCMin, ESCMax);
    PWM[LeftC] = Limit(PWM[LeftC], ESCMin, ESCMax);
    PWM[RightC] = Limit(PWM[RightC], ESCMin, ESCMax);
#ifdef TRICOPTER
    PWM[BackC] = Limit(PWM[BackC], 1, OUT_MAXIMUM);
#else
    PWM[BackC] = Limit(PWM[BackC], ESCMin, ESCMax);
#endif

#endif // !(SIMULATE | TESTING)

} // OutSignals




