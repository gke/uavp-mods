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

// Autonomous flight routines

#include "UAVX.h"

void DoShutdown(void);
void DoPolarOrientation(void);
void Navigate(int32, int32);
void SetDesiredAltitude(int16);
void DoFailsafeLanding(void);
void AcquireHoldPosition(void);
void NavGainSchedule(int16);
void DoNavigation(void);
void DoPPMFailsafe(void);
void UAVXNavCommand(void);
void GetWayPointPX(int8);
void InitNavigation(void);

real32 NavCorr[3], NavCorrp[3];
real32 NavE[3], NavEp[3], NavIntE[3];
int16 NavYCorrLimit;

#ifdef SIMULATE
int16 FakeDesiredRoll, FakeDesiredPitch, FakeDesiredYaw, FakeHeading;
#endif // SIMULATE

typedef union {
    uint8 b[256];
    struct {
        uint8 u0;
        int16 u1;
        int8 u3;
        int8 u4;
        uint16 u5;
        uint16 u6;
        uint8 NoOfWPs;
        uint8 ProximityAltitude;
        uint8 ProximityRadius;
        int16 OriginAltitude;
        int32 OriginLatitude;
        int32 OriginLongitude;
        int16 RTHAltitude;
        struct {
            int32 Latitude;
            int32 Longitude;
            int16 Altitude;
            int8 Loiter;
        } WP[23];
    };
} UAVXNav;

uint8 BufferPX[256];

int8     CurrWP;
int8     NoOfWayPoints;
int16    WPAltitude;
int32    WPLatitude, WPLongitude;
int24    WPLoiter;

real32   WayHeading;
real32   NavPolarRadius, NavNeutralRadius, NavProximityRadius, NavProximityAltitude;
int24    NavRTHTimeoutmS;

int8     NavState;
int16    NavSensitivity, RollPitchMax;
int16    AltSum;

void DoShutdown(void)
{
    State = Shutdown;
    DesiredThrottle = 0;
    StopMotors();
} // DoShutdown

void SetDesiredAltitude(int16 NewDesiredAltitude) { // Metres
    if ( F.AllowNavAltitudeHold && F.AltHoldEnabled ) {
        AltSum = 0;
        DesiredAltitude = NewDesiredAltitude * 10L; // Decimetres
    }
} // SetDesiredAltitude

void DoFailsafeLanding(void) {
    // InTheAir micro switch RC0 Pin 11 to ground when landed
    
    const boolean InTheAir = true;

    DesiredAltitude = -20.0;
if ( F.BaroAltitudeValid )
    {
        if ( Altitude > LAND_M )
            mS[NavStateTimeout] = mSClock() + NAV_RTH_LAND_TIMEOUT_MS;

        if ( !InTheAir || (( mSClock() > mS[NavStateTimeout] ) 
            && ( F.ForceFailsafe || ( NavState == Touchdown ) || (FailState == Terminated))) )
            DoShutdown();
        else
            DesiredThrottle = CruiseThrottle;
    }
    else
    {
        if ( mSClock() > mS[DescentUpdate] )
        {
            mS[DescentUpdate] = mSClock() + ALT_DESCENT_UPDATE_MS;
            DesiredThrottle = CruiseThrottle - DescentComp;
            if ( DesiredThrottle < IdleThrottle )
                StopMotors();
            else
                if ( DescentComp < CruiseThrottle )
                    DescentComp++;        
        }
    }
} // DoFailsafeLanding

void FailsafeHoldPosition(void) {
    DesiredRoll = DesiredPitch = DesiredYaw = 0;
    if ( F.GPSValid && F.CompassValid )
        Navigate(HoldLatitude, HoldLongitude);
} // FailsafeHoldPosition

void AcquireHoldPosition(void) {
    static int8 i;

    for ( i = 0; i < (uint8)3; i++ )
        NavCorr[i] = NavCorrp[i]  =  0;

    F.NavComputed = false;

    HoldLatitude = GPSLatitude;
    HoldLongitude = GPSLongitude;
    F.WayPointAchieved = F.WayPointCentred = F.AcquireNewPosition = false;

    NavState = HoldingStation;
} // AcquireHoldPosition

void DoPolarOrientation(void) {
    static real32 EastDiff, NorthDiff, Radius;
    static real32 DesiredRelativeHeading;
    static int16 P;

    F.UsingPolar = F.UsingPolarCoordinates && F.NavValid && ( NavState == HoldingStation );

    if ( F.UsingPolar ) { // needs rethink - probably arm using RTH switch
        EastDiff = (OriginLongitude - GPSLongitude) * GPSLongitudeCorrection;
        NorthDiff = OriginLatitude - GPSLatitude;

        Radius = Max(abs(EastDiff), abs(NorthDiff));
        if ( Radius > NavPolarRadius ) {
            DesiredRelativeHeading = Make2Pi(atan2((real32)EastDiff, (real32)NorthDiff) - PI - Heading );

            P = ( (int24)DesiredRelativeHeading * 24L + HALFPI )/ PI + Orientation;

            while ( P > 24 ) P -=24;
            while ( P < 0 ) P +=24;

        } else
            P = 0;
    } else
        P = 0;

    PolarOrientation = P;;

} // DoPolarOrientation

void Navigate(int32 NavLatitude, int32 NavLongitude ) {
    // F.GPSValid must be true immediately prior to entry
    // This routine does not point the quadrocopter at the destination
    // waypoint. It simply rolls/pitches towards the destination.
    // GPS coordinates MUST be int32 to allow sufficient range - real32 is insufficient.

    static real32 RelHeadingSin, RelHeadingCos;
    static real32 Radius;
    static real32 AltE;
    static real32 EastDiff, NorthDiff;
    static real32 RelHeading;
    static uint8 a;
    static real32 Diff, NavP, NavI, NavD;

    DoPolarOrientation();

    DesiredLatitude = NavLatitude;
    DesiredLongitude = NavLongitude;

    EastDiff = (real32)(DesiredLongitude - GPSLongitude) * GPSLongitudeCorrection;
    NorthDiff = (real32)(DesiredLatitude - GPSLatitude);

    Radius = sqrt( Sqr(EastDiff) + Sqr(NorthDiff) );

    F.WayPointCentred =  Radius < NavNeutralRadius;
    AltE = DesiredAltitude - Altitude;
    F.WayPointAchieved = ( Radius < NavProximityRadius ) && ( abs(AltE) < NavProximityAltitude );

    WayHeading = Make2Pi(atan2((real32)EastDiff, (real32)NorthDiff));
    RelHeading = MakePi(WayHeading - Heading); // make +/- MilliPi

    if ( ( NavSensitivity > NAV_SENS_THRESHOLD ) && !F.WayPointCentred ) {
#ifdef NAV_WING

        // no Nav for conventional aircraft - yet!
        NavCorr[Pitch] = -5; // always moving forward
        // Just use simple rudder only for now.
        if ( !F.WayPointAchieved ) {
            NavCorr[Yaw] = -(RelHeading * NAV_YAW_LIMIT) / HALFPI;
            NavCorr[Yaw] = Limit(NavCorr[Yaw], -NAV_YAW_LIMIT, NAV_YAW_LIMIT); // gently!
        }

#else // MULTICOPTER

        // revert to original simpler version from UAVP->UAVX transition

        RelHeadingSin = sin(RelHeading);
        RelHeadingCos = cos(RelHeading);

        NavE[Roll] = Radius * RelHeadingSin;
        NavE[Pitch] = -Radius * RelHeadingCos;

        // Roll & Pitch

        for ( a = 0; a < (uint8)2 ; a++ ) {
            NavP = Limit(NavE[a], -NAV_MAX_ROLL_PITCH, NAV_MAX_ROLL_PITCH);

            NavIntE[a] += NavE[a];
            NavIntE[a] = Limit(NavIntE[a], -K[NavIntLimit], K[NavIntLimit]);
            NavI = NavIntE[a] * K[NavKi] * GPSdT;
            NavIntE[a] = Decay1(NavIntE[a]);

            Diff = (NavEp[a] - NavE[a]);
            Diff = Limit(Diff, -256, 256);
            NavD = Diff * K[NavKd] * GPSdTR;
            NavD = Limit(NavD, -NAV_DIFF_LIMIT, NAV_DIFF_LIMIT);

            NavEp[a] = NavE[a];

            NavCorr[a] = (NavP + NavI + NavD ) * NavSensitivity;

            NavCorr[a] = SlewLimit(NavCorrp[a], NavCorr[a], NAV_CORR_SLEW_LIMIT);
            NavCorr[a] = Limit(NavCorr[a], -NAV_MAX_ROLL_PITCH, NAV_MAX_ROLL_PITCH);

            NavCorrp[a] = NavCorr[a];
        }

        // Yaw
        if ( F.AllowTurnToWP && !F.WayPointAchieved ) {
            RelHeading = MakePi(WayHeading - Heading); // make +/- MilliPi
            NavCorr[Yaw] = -(RelHeading * NavYCorrLimit) / HALFPI;
            NavCorr[Yaw] = Limit(NavCorr[Yaw], -NavYCorrLimit, NavYCorrLimit); // gently!
        } else
            NavCorr[Yaw] = 0;

#endif // NAV_WING    
    } else {
        // Neutral Zone - no GPS influence
        NavCorr[Pitch] = DecayX(NavCorr[Pitch], 2);
        NavCorr[Roll] = DecayX(NavCorr[Roll], 2);
        NavCorr[Yaw] = 0;
        NavIntE[Roll] = NavIntE[Pitch] = NavEp[Roll] = NavEp[Pitch] = 0;
    }

    F.NavComputed = true;

} // Navigate

void DoNavigation(void) {

    F.NavigationActive = F.GPSValid && F.CompassValid && ( mSClock() > mS[NavActiveTime]);

    if ( F.NavigationActive ) {
    
            F.LostModel = F.ForceFailsafe = false;

        if ( !F.NavComputed )
            switch ( NavState ) { // most case last - switches in C18 are IF chains not branch tables!
                case Touchdown:
                    Navigate(OriginLatitude, OriginLongitude);
                    DoFailsafeLanding();
                    break;
                case Descending:
                    Navigate( OriginLatitude, OriginLongitude );
                    if ( F.ReturnHome || F.Navigate )
#ifdef NAV_WING
                    {
                        // needs more thought - runway direction?
                        DoFailsafeLanding();
                    }
#else
                        if ( Altitude < LAND_M ) {
                            mS[NavStateTimeout] = mSClock() + NAV_RTH_LAND_TIMEOUT_MS;
                            NavState = Touchdown;
                        } else
                            DoFailsafeLanding();
#endif // NAV_WING
                    else
                        AcquireHoldPosition();
                    break;
                case AtHome:
                    Navigate(OriginLatitude, OriginLongitude);
                    SetDesiredAltitude((int16)P[NavRTHAlt]);
                    if ( F.ReturnHome || F.Navigate )
                        if ( F.WayPointAchieved ) { // check still @ Home
                            if ( F.UsingRTHAutoDescend && ( mSClock() > mS[NavStateTimeout] ) )
                                NavState = Descending;
                        } else
                            NavState = ReturningHome;
                    else
                        AcquireHoldPosition();
                    break;
                case ReturningHome:
                    Navigate(OriginLatitude, OriginLongitude);
                    SetDesiredAltitude((int16)P[NavRTHAlt]);
                    if ( F.ReturnHome || F.Navigate ) {
                        if ( F.WayPointAchieved ) {
                            mS[NavStateTimeout] = mSClock() + NavRTHTimeoutmS;
                            NavState = AtHome;
                        }
                    } else
                        AcquireHoldPosition();
                    break;
                case Loitering:
                    Navigate(WPLatitude, WPLongitude);
                    SetDesiredAltitude(WPAltitude);
                    if ( F.Navigate ) {
                        if ( F.WayPointAchieved && (mSClock() > mS[NavStateTimeout]) )
                            if ( CurrWP == NoOfWayPoints ) {
                                CurrWP = 1;
                                NavState = ReturningHome;
                            } else {
                                GetWayPointPX(++CurrWP);
                                NavState = Navigating;
                            }
                    } else
                        AcquireHoldPosition();
                    break;
                case Navigating:
                    Navigate(WPLatitude, WPLongitude);
                    SetDesiredAltitude(WPAltitude);
                    if ( F.Navigate ) {
                        if ( F.WayPointAchieved ) {
                            mS[NavStateTimeout] = mSClock() + WPLoiter;
                            NavState = Loitering;
                        }
                    } else
                        AcquireHoldPosition();
                    break;
                case HoldingStation:
                    if ( F.AttitudeHold ) {
                        if ( F.AcquireNewPosition && !( F.Ch5Active & F.UsingPositionHoldLock ) ) {
                            F.AllowTurnToWP = SaveAllowTurnToWP;
                            AcquireHoldPosition();
#ifdef NAV_ACQUIRE_BEEPER
                            if ( !F.BeeperInUse ) {
                                mS[BeeperTimeout] = mSClock() + 500L;
                                Beeper_ON;
                            }
#endif // NAV_ACQUIRE_BEEPER
                        }
                    } else
                        F.AcquireNewPosition = true;

                    Navigate(HoldLatitude, HoldLongitude);

                    if ( F.NavValid && F.NearLevel )  // Origin must be valid for ANY navigation!
                        if ( F.Navigate ) {
                            GetWayPointPX(CurrWP); // resume from previous WP
                            SetDesiredAltitude(WPAltitude);
                            NavState = Navigating;
                        } else
                            if ( F.ReturnHome )
                                NavState = ReturningHome;
                    break;
            } // switch NavState
    } else
        if ( F.ForceFailsafe && F.NewCommands )
        {
            F.AltHoldEnabled = F.AllowNavAltitudeHold = true;
            F.LostModel = true;
            DoFailsafeLanding();
        }
        else // kill nav correction immediately
             NavCorr[Pitch] = NavCorr[Roll] = NavCorr[Yaw] = 0; // zzz

    F.NewCommands = false;    // Navigate modifies Desired Roll, Pitch and Yaw values.

} // DoNavigation

void CheckFailsafeAbort(void) {
    if ( mSClock() > mS[AbortTimeout] ) {
        if ( F.Signal ) {
            LEDGreen_ON;
            mS[NavStateTimeout] = 0;
            mS[FailsafeTimeout] = mSClock() + FAILSAFE_TIMEOUT_MS; // may be redundant?
            NavState = HoldingStation;
            FailState = MonitoringRx;
        }
    } else
        mS[AbortTimeout] += ABORT_UPDATE_MS;
} // CheckFailsafeAbort

void DoPPMFailsafe(void) { // only relevant to PPM Rx or Quad NOT synchronising with Rx

    if ( State == InFlight )
        switch ( FailState ) { // FailStates { MonitoringRx, Aborting, Terminating, Terminated }
            case Terminated: // Basic assumption is that aircraft is being flown over a safe area!
                FailsafeHoldPosition();
                DoFailsafeLanding();
                break;
            case Terminating:
                FailsafeHoldPosition();
                if ( Altitude < LAND_M ) {
                    mS[NavStateTimeout] = mSClock() + NAV_RTH_LAND_TIMEOUT_MS;
                    NavState = Touchdown;
                    FailState = Terminated;
                }
                DoFailsafeLanding();
                break;
            case Aborting:
                FailsafeHoldPosition();
                F.AltHoldEnabled = true;
                SetDesiredAltitude((int16)P[NavRTHAlt]);
                if ( mSClock() > mS[NavStateTimeout] ) {
                    F.LostModel = true;
                    LEDGreen_OFF;
                    LEDRed_ON;

                    mS[NavStateTimeout] = mSClock() + NAV_RTH_LAND_TIMEOUT_MS;
                    NavState = Descending;
                    FailState = Terminating;
                } else
                    CheckFailsafeAbort();
                break;
            case MonitoringRx:
                if ( mSClock() > mS[FailsafeTimeout] ) {
                    // use last "good" throttle
                    Stats[RCFailsafesS]++;
                    if ( F.GPSValid && F.CompassValid )
                        mS[NavStateTimeout] = mSClock() + ABORT_TIMEOUT_GPS_MS;
                    else
                        mS[NavStateTimeout] = mSClock() + ABORT_TIMEOUT_NO_GPS_MS;
                    mS[AbortTimeout] = mSClock() + ABORT_UPDATE_MS;
                    FailState = Aborting;
                }
                break;
        } // Switch FailState
    else
        DesiredRoll = DesiredPitch = DesiredYaw = DesiredThrottle = 0;

} // DoPPMFailsafe

void UAVXNavCommand(void) {

    static int16 b;
    static uint8 c, d, csum;

    c = RxChar();
    LEDBlue_ON;

    switch ( c ) {
        case '0': // hello
            TxChar(ACK);
            break;
        case '1': // write
            csum = 0;
            for ( b = 0; b < 256; b++) { // cannot write fast enough so buffer
                d = RxChar();
                csum ^= d;
                BufferPX[b] = d;
            }
            if ( csum == (uint8)0 ) {
                for ( b = 0; b < 256; b++)
                    WritePX(NAV_ADDR_PX + b, BufferPX[b]);
                TxChar(ACK);
            } else
                TxChar(NAK);

            InitNavigation();

            break;
        case '2':
            csum = 0;
            for ( b = 0; b < 255; b++) {
                d = ReadPX(NAV_ADDR_PX + b);
                csum ^= d;
                BufferPX[b] = d;
            }
            BufferPX[255] = csum;
            for ( b = 0; b < 256; b++)
                TxChar(BufferPX[b]);
            TxChar(ACK);
            break;
        case '3':
            csum = 0;
            for ( b = 0; b < 63; b++) {
                d = ReadPX(STATS_ADDR_PX + b);
                csum ^= d;
                BufferPX[b] = d;
            }
            BufferPX[63] = csum;
            for ( b = 0; b < 64; b++)
                TxChar(BufferPX[b]);
            TxChar(ACK);
            break;
        default:
            break;
    } // switch

    WritePXImagefile();
    LEDBlue_OFF;

} // UAVXNavCommand

void GetWayPointPX(int8 wp) {
    static uint16 w;

    if ( wp > NoOfWayPoints )
        CurrWP = wp = 0;
    if ( wp == 0 ) { // force to Origin
        WPLatitude = OriginLatitude;
        WPLongitude = OriginLongitude;
        WPAltitude = (int16)P[NavRTHAlt];
        WPLoiter = 30000; // mS
    } else {
        w = NAV_WP_START + (wp-1) * WAYPOINT_REC_SIZE;
        WPLatitude = Read32PX(w + 0);
        WPLongitude = Read32PX(w + 4);
        WPAltitude = Read16PX(w + 8);

#ifdef NAV_ENFORCE_ALTITUDE_CEILING
        if ( WPAltitude > NAV_CEILING )
            WPAltitude = NAV_CEILING;
#endif // NAV_ENFORCE_ALTITUDE_CEILING
        WPLoiter = (int16)ReadPX(w + 10) * 1000L; // mS
    }

    F.WayPointCentred =  F.WayPointAchieved = false;

} // GetWaypointPX

void InitNavigation(void) {
    static uint8 i;

    HoldLatitude = HoldLongitude = WayHeading = 0;

    for ( i = 0; i < (uint8)3; i++ )
        NavEp[i] = NavIntE[i] = NavCorr[i] = NavCorrp[i] = 0;

    NavState = HoldingStation;
    AttitudeHoldResetCount = 0;
    CurrMaxRollPitch = 0;
    F.WayPointAchieved = F.WayPointCentred = false;
    F.NavComputed = false;

    if ( ReadPX(NAV_NO_WP) <= 0 ) {
        NavProximityRadius = ConvertMToGPS(NAV_PROXIMITY_RADIUS);
        NavProximityAltitude = NAV_PROXIMITY_ALTITUDE * 10L; // Decimetres
    } else {
        // need minimum values in UAVXNav?
        NavProximityRadius = ConvertMToGPS(ReadPX(NAV_PROX_RADIUS));
        NavProximityAltitude = ReadPX(NAV_PROX_ALT) * 10L; // Decimetres
    }

    NoOfWayPoints = ReadPX(NAV_NO_WP);

    if ( NoOfWayPoints <= 0 )
        CurrWP = 0;
    else
        CurrWP = 1;
    GetWayPointPX(0);

} // InitNavigation


