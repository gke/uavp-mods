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

#pragma	config OSC=HSPLL, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC, XINST = OFF

#include "uavx.h"

Flags 	F;
uint8 p;
uint32 NowmS, CyclemS;
#pragma udata access statevars
int8 near State, NavState, FailState;
boolean near SpareSlotTime;
#pragma udata

void main(void)
{
	DisableInterrupts;

	InitPortsAndUSART();
	InitTimersAndInterrupts();

	InitMisc();
	InitADC();
	InitI2C(); // selects 400KHz
	ReadStatsEE();	
	InitRC();
	InitMotors();

    EnableInterrupts;

	LEDYellow_ON;
	Delay1mS(500);

	InitParameters(); // inits Acc/Gyros

	InitCompass();
	InitHeading();
	InitRangefinder();
	InitGPS();
	InitNavigation();
	InitBarometer();

	ShowSetup();

	FirstPass = true;
	
	while( true )
	{
		StopMotors();

		ReceivingGPSOnly(false);
		EnableInterrupts;

		LightsAndSirens();	// Check for Rx signal, disarmed on power up, throttle closed, gyros ONLINE

		State = Starting;
		mS[RxFailsafeTimeout] = NowmS + FAILSAFE_TIMEOUT_MS;
		FailState = MonitoringRx;
		F.FirstArmed = false;

		while ( Armed )
		{ // no command processing while the Quadrocopter is armed
	
			ReceivingGPSOnly(true); 

			while ( WaitingForSync ) {};

			DisableInterrupts; // protect 1mS clock
				WaitingForSync = true;
				NowmS = MilliSec;		
				PIDUpdate = NowmS + PIDCyclemS;
			EnableInterrupts;

			SpareSlotTime = true; // token used by nav, compass, baro and telemetry

			DoControl();

			#ifndef TESTING
				#ifdef INC_CYCLE_STATS
				if ( State == InFlight )
				{
					CyclemS = NowmS - mS[LastPIDUpdate];
					mS[LastPIDUpdate] = NowmS;
					CyclemS = Limit(CyclemS, 0, 15);
					CycleHist[CyclemS]++;
				}
				#endif // INC_CYCLE_STATS
			#endif // TESTING

			UpdateGPS();
	
			if ( F.RCNewValues )
				UpdateControls();

			if ( ( F.Signal ) && ( FailState == MonitoringRx ) )
			{
				switch ( State  ) {
				case Starting:	// this state executed once only after arming

					LEDYellow_OFF;

					if ( !F.FirstArmed )
					{
						mS[StartTime] = NowmS;
						F.FirstArmed = true;
					}

					InitControl();
					CaptureTrims();
					InitGPS();
					InitNavigation();

					DesiredThrottle = 0;
					ErectGyros(256);				// DO NOT MOVE AIRCRAFT!
					ZeroStats();
					WriteMagCalEE();

					DoStartingBeepsWithOutput(3);

					mS[ArmedTimeout] = NowmS + ARMED_TIMEOUT_MS;
					mS[RxFailsafeTimeout] = NowmS + RC_NO_CHANGE_TIMEOUT_MS;
					F.ForceFailsafe = F.LostModel = false;

					State = Landed;
					break;
				case Landed:
					DesiredThrottle = 0;
					F.OriginAltValid = false;
					InitHeading();
					if ( NowmS > mS[ArmedTimeout] )
						DoShutdown();
					else	
						if ( StickThrottle < IdleThrottle )
						{
							SetGPSOrigin();
							DecayNavCorr();
	    					if ( F.NewCommands )
								F.LostModel = F.ForceFailsafe;
						}
						else
						{						
							LEDPattern = 0;
							mS[NavActiveTime] = NowmS + NAV_ACTIVE_DELAY_MS;
							Stats[RCGlitchesS] = RCGlitches; // start of flight
							SaveLEDs();

							mS[RxFailsafeTimeout] = NowmS + RC_NO_CHANGE_TIMEOUT_MS;
							F.ForceFailsafe = F.LostModel = false;

							if ( ParameterSanityCheck() )
							{
								#ifndef SIMULATE
								while ( !F.NewBaroValue )
									GetBaroAltitude(); 
								OriginBaroPressure = BaroPressure;
								OriginBaroTemperature = BaroTemperature;
								#endif // !SIMULATE
								F.OriginAltValid = true;
								F.NewBaroValue = false;
								State = InFlight;
							}
							else
								ALL_LEDS_ON;	
						}						
					break;
				case Landing:
					GetBaroAltitude();
					if ( StickThrottle > IdleThrottle )
					{
						DesiredThrottle = 0;
						State = InFlight;
					}
					else
						if ( NowmS < mS[ThrottleIdleTimeout] )
							DesiredThrottle = IdleThrottle;
						else
						{
							DecayNavCorr();
							DesiredThrottle = AltComp = 0; // to catch cycles between Rx updates
							F.DrivesArmed = false;
							Stats[RCGlitchesS] = RCGlitches - Stats[RCGlitchesS];	
							WriteStatsEE();
							WriteMagCalEE();
							mS[ArmedTimeout] = NowmS + ARMED_TIMEOUT_MS;
							State = Landed;
						}
					break;
				case Shutdown:
					LEDChaser();
					GetBaroAltitude(); // may as well!
					if ((StickThrottle < IdleThrottle) && !(F.ReturnHome || F.Navigate)) {
						mSTimer(ArmedTimeout, ARMED_TIMEOUT_MS);
						mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);
						F.ForceFailsafe = F.LostModel = false;
						DoStartingBeepsWithOutput(3);
						State = Landed;
					}
					break;
				case InFlight:
					F.DrivesArmed = true;
		
					LEDChaser();

					DesiredThrottle = SlewLimit(DesiredThrottle, StickThrottle, 1);
 
					DoNavigation();
		
					if (State != Shutdown) {
						if (StickThrottle < IdleThrottle) {
							AltComp = 0;
							mSTimer(ThrottleIdleTimeout, THROTTLE_LOW_DELAY_MS);
							RestoreLEDs();
							State = Landing;
						} else
							AltitudeHold();
					}
					break;
				} // Switch State
				mS[FailsafeTimeout] = NowmS + FAILSAFE_TIMEOUT_MS;
				FailState = MonitoringRx;
			}
			else
				if ( F.FailsafesEnabled )
					DoFailsafe();

			CheckTelemetry();

			CheckBatteries();
			CheckAlarms();

		} // flight while armed
	}

} // main

