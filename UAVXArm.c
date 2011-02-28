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

volatile Flags     F;

int main(void) {

    InitMisc();
    InitHarness();

    InitRC();
    InitTimersAndInterrupts();
    InitLEDs();
         
    InitParameters();
    ReadStatsPX();
    InitMotors();
    InitBattery();

    LEDYellow_ON;
    InitAccelerometers();
    InitGyros();
    InitIRSensors();
 
    InitCompass();
    InitRangefinder();

    InitGPS();
    InitNavigation();

    InitTemperature();
    InitBarometer();

    ShowSetup(true);

    I2C0.frequency(MinI2CRate);

    FirstPass = true;

    while ( true ) {
        StopMotors();

        LightsAndSirens();    // Check for Rx signal, disarmed on power up, throttle closed, gyros ONLINE
        
        GetAttitude();
        MixAndLimitCam();

        State = Starting;
        F.FirstArmed = false;

        while ( Armed.read() ) { // no command processing while the Quadrocopter is armed

            UpdateGPS();
            if ( F.RCNewValues )
                UpdateControls();

            if ( ( F.Signal ) && ( FailState == MonitoringRx ) ) {
                switch ( State  ) {
                    case Starting:    // this state executed once only after arming

                        LEDYellow_OFF;

                        CreateLogfile();

                        if ( !F.FirstArmed ) {
                            mS[StartTime] = mSClock();
                            F.FirstArmed = true;
                        }

                        InitControl();
                        InitGPS();
                        InitNavigation();

                        DesiredThrottle = 0;
                        ErectGyros(); // DO NOT MOVE AIRCRAFT!
                        ZeroStats();
                        DoStartingBeeps(3);
                        
                        SendParameters(ParamSet);

                        mS[ArmedTimeout] = mSClock() + ARMED_TIMEOUT_MS;
                        mS[RxFailsafeTimeout] = mSClock() + RC_NO_CHANGE_TIMEOUT_MS;
                        ControlUpdateTimeuS = uSClock();
                        F.ForceFailsafe = F.LostModel = false;

                        State = Landed;
                        break;
                    case Landed:
                        DesiredThrottle = 0;
                        if ( mSClock() > mS[ArmedTimeout] )
                            DoShutdown();
                        else
                            if ( StickThrottle < IdleThrottle ) {
                                SetGPSOrigin();
                                GetHeading();
                                if ( F.NewCommands )
                                    F.LostModel = F.ForceFailsafe;
                            } else {
#ifdef SIMULATE
                                FakeBaroRelAltitude = 0;
#endif // SIMULATE                        
                                LEDPattern = 0;
                                mS[NavActiveTime] = mSClock() + NAV_ACTIVE_DELAY_MS;
                                Stats[RCGlitchesS] = RCGlitches; // start of flight
                                SaveLEDs();
                                if ( ParameterSanityCheck() )
                                    State = InFlight;
                                else
                                    ALL_LEDS_ON;
                            }
                        break;
                    case Landing:
                        if ( StickThrottle > IdleThrottle ) {
                            DesiredThrottle = 0;
                            State = InFlight;
                        } else
                            if ( mSClock() < mS[ThrottleIdleTimeout] )
                                DesiredThrottle = IdleThrottle;
                            else {
                                DesiredThrottle = 0; // to catch cycles between Rx updates
                                F.MotorsArmed = false;
                                Stats[RCGlitchesS] = RCGlitches - Stats[RCGlitchesS];
                                WriteStatsPX();
                                WritePXImagefile();
                                mS[ArmedTimeout] = mSClock() + ARMED_TIMEOUT_MS;
                                State = Landed;
                            }
                        break;
                    case Shutdown:
                        // wait until arming switch is cycled
                        F.LostModel = true;
                        DesiredRoll = DesiredPitch = DesiredYaw = DesiredThrottle = 0;
                        StopMotors();
                        break;
                    case InFlight:
                        F.MotorsArmed = true;
                        DoNavigation();
                        LEDChaser();

                        DesiredThrottle = SlewLimit(DesiredThrottle, StickThrottle, 1);

                        if ( StickThrottle < IdleThrottle ) {
                            mS[ThrottleIdleTimeout] = mSClock() + THROTTLE_LOW_DELAY_MS;
                            RestoreLEDs();
                            State = Landing;
                        }
                        break;
                } // Switch State
                mS[FailsafeTimeout] = mSClock() + FAILSAFE_TIMEOUT_MS;
                FailState = MonitoringRx;
            } else
                DoPPMFailsafe();
                
            while ( uSClock() < ControlUpdateTimeuS ) {}; // CAUTION: uS clock wraps at about an hour
            ControlUpdateTimeuS = uSClock() + PID_CYCLE_US; 

            DoControl();

            MixAndLimitMotors();
            OutSignals();
            
            MixAndLimitCam();
            
            GetTemperature();  
            GetBattery();
            CheckAlarms();
            CheckTelemetry();

            SensorTrace();

        } // flight while armed
    }
} // main

