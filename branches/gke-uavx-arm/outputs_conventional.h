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


void OutSignals(void) {
    static int8 m;
    static uint8 r;

    for ( m = 0; m < (uint8)6; m++ )
        PWM[m] = Limit(PWM[m], ESCMin, ESCMax);

#if !( defined SIMULATE | defined TESTING )

    if ( !F.MotorsArmed )
        StopMotors();

#ifdef ELEVON
    Out0.pulsewidth_us(1000 + (int16)( PWM[ThrottleC] * PWMScale ) );
    Out1.pulsewidth_us(1000 + (int16)( PWM[RightElevonC] * PWMScale ) );
    Out2.pulsewidth_us(1000 + (int16)( PWM[LeftElevonC] * PWMScale ) );
    Out3.pulsewidth_us(1000 + (int16)( PWM[RudderC] * PWMScale ) );
#else
    Out0.pulsewidth_us(1000 + (int16)( PWM[ThrottleC] * PWMScale ) );
    Out1.pulsewidth_us(1000 + (int16)( PWM[AileronC] * PWMScale ) );
    Out2.pulsewidth_us(1000 + (int16)( PWM[ElevatorC] * PWMScale ) );
    Out3.pulsewidth_us(1000 + (int16)( PWM[RudderC] * PWMScale ) );
#endif

} // OutSignals








