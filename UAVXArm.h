
// Commissioning defines

#define SW_I2C                               // define for software I2C - TRAGICALLY SLOW

#define MAGIC 1.0                           // rescales the sensitivity of all PID loop params

#define I2C_MAX_RATE_HZ    400000       

#define PWM_UPDATE_HZ       200             // MUST BE LESS THAN OR EQUAL TO 450HZ

#define DISABLE_EXTRAS                       // suppress altitude hold, position hold and inertial compensation
#define SUPPRESS_SDCARD                     // no logging to check if buffering backup is an issue

//BMP occasional returns bad results - changes outside this rate are deemed sensor/buss errors
#define BARO_SANITY_CHECK_MPS    10.0       // dm/S 20,40,60,80 or 100

#define SIX_DOF                             // effects acc and gyro addresses

#define SOFTWARE_CAM_PWM

#define BATTERY_VOLTS_SCALE   57.85         // 51.8144    // Volts scaling for internal divider

//#define DCM_YAW_COMP

#define USING_MBED

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

#include "mbed.h"
#include "SDFileSystem.h"
#include "SerialBuffered.h" // used in preference to MODSERIAL

// Additional Types
typedef uint8_t uint8 ;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef int32_t int24;
typedef uint32_t uint24;
typedef int32_t int32;
typedef uint32_t uint32;
typedef uint8_t boolean;
typedef float real32;

extern Timer timer;

//________________________________________________________________________________________


// Useful Constants
#define NUL     0
#define SOH     1
#define EOT     4
#define ACK     6
#define HT      9
#define LF      10
#define CR      13
#define NAK     21
#define ESC     27
#define true    1
#define false   0

#define PI              3.141592654
#define HALFPI          (PI*0.5)
#define QUARTERPI       (PI*0.25)
#define SIXTHPI         (PI/6.0)
#define TWOPI           (PI*2.0)
#define RADDEG          (180.0/PI)
#define MILLIANGLE      (180000.0/PI)
#define DEGRAD          (PI/180.0)

#define MILLIRAD        18
#define CENTIRAD        2

#define MAXINT32        0x7fffffff
#define MAXINT16        0x7fff

//#define BATCHMODE

#ifndef BATCHMODE
//#define RX6CH
//#define EXPERIMENTAL
//#define TESTING
//#define RX6CH                     // 6ch Receivers
//#define SIMULATE
//#define VTCOPTER
//#define Y6COPTER
#define QUADROCOPTER
//#define TRICOPTER
//#define HELICOPTER
//#define AILERON
// #define ELEVON
#endif // !BATCHMODE

//________________________________________________________________________________________________

#define USE_PPM_FAILSAFE

// Airframe

#if ( defined TRICOPTER | defined QUADROCOPTER | defined VTCOPTER | defined Y6COPTER )
#define MULTICOPTER
#endif

#if ( defined HELICOPTER | defined AILERON | defined ELEVON )
#if ( defined AILERON | defined ELEVON )
#define NAV_WING
#endif
#endif

#ifdef QUADROCOPTER
#define AF_TYPE QuadAF
#endif
#ifdef TRICOPTER
#define AF_TYPE TriAF
#endif
#ifdef VTCOPTER
#define AF_TYPE VTAF
#endif
#ifdef Y6COPTER
#define AF_TYPE Y6AF
#endif
#ifdef HELICOPTER
#define AF_TYPE HeliAF
#endif
#ifdef ELEVON
#define AF_TYPE ElevAF
#endif
#ifdef AILERON
#define AF_TYPE AilAF
#endif

#define GPS_INC_GROUNDSPEED                     // GPS groundspeed is not used for flight but may be of interest

// Timeouts and Update Intervals

#define FAILSAFE_TIMEOUT_MS             1000L   // mS. hold last "good" settings and then restore flight or abort
#define ABORT_TIMEOUT_GPS_MS            5000L   // mS. go to descend on position hold if GPS valid.
#define ABORT_TIMEOUT_NO_GPS_MS         0L      // mS. go to descend on position hold if GPS valid.  
#define ABORT_UPDATE_MS                 1000L   // mS. retry period for RC Signal and restore Pilot in Control
#define ARMED_TIMEOUT_MS                150000L    // mS. automatic disarming if armed for this long and landed

#define ALT_DESCENT_UPDATE_MS           1000L    // mS time between throttle reduction clicks in failsafe descent without baro

#define RC_STICK_MOVEMENT                5L        // minimum to be recognised as a stick input change without triggering failsafe

#define THROTTLE_LOW_DELAY_MS           1000L   // mS. that motor runs at idle after the throttle is closed
#define THROTTLE_UPDATE_MS              3000L   // mS. constant throttle time for altitude hold

#define NAV_ACTIVE_DELAY_MS             10000L  // mS. after throttle exceeds idle that Nav becomes active
#define NAV_RTH_LAND_TIMEOUT_MS         10000L  // mS. Shutdown throttle if descent lasts too long

#define UAVX_TEL_INTERVAL_MS            125L    // mS. emit an interleaved telemetry packet
#define ARDU_TEL_INTERVAL_MS            200L    // mS. Ardustation
#define UAVX_CONTROL_TEL_INTERVAL_MS    10L     // mS. flight control only
#define CUSTOM_TEL_INTERVAL_MS          250L    // mS.
#define UAVX_MIN_TEL_INTERVAL_MS        1000L    // mS. emit minimum FPV telemetry packet slow rate for example to FrSky

#define GPS_TIMEOUT_MS                  2000L   // mS.

#define ALT_UPDATE_HZ                   20L     // Hz based on 50mS update time for Baro 

#ifdef MULTICOPTER
//#define PWM_UPDATE_HZ                   450L    // PWM motor update rate must be <450 and >100
#else
#define PWM_UPDATE_HZ                   40L     // standard RC servo update rate
#endif // MULTICOPTER

// Accelerometers

#define DAMP_HORIZ_LIMIT                3L      // equivalent stick units - no larger than 5
#define DAMP_VERT_LIMIT_LOW             -5L     // maximum throttle reduction
#define DAMP_VERT_LIMIT_HIGH            20L     // maximum throttle increase

// Gyros

#define ATTITUDE_FF_DIFF                (24.0/16.0)   // 0 - 32 max feedforward speeds up roll/pitch recovery on fast stick change

#define ATTITUDE_ENABLE_DECAY                   // enables decay to zero angle when roll/pitch is not in fact zero!
// unfortunately there seems to be a leak which cause the roll/pitch
// to increase without the decay.

#define ATTITUDE_SCALE                  0.5     // scaling of stick units for attitude angle control 

// Enable "Dynamic mass" compensation Roll and/or Pitch
// Normally disabled for pitch only
//#define DISABLE_DYNAMIC_MASS_COMP_ROLL
//#define DISABLE_DYNAMIC_MASS_COMP_PITCH

// Altitude Hold


#define ALT_HOLD_MAX_ROC_MPS            0.5      // Must be changing altitude at less than this for alt. hold to be detected

// the range within which throttle adjustment is proportional to altitude error
#define ALT_BAND_M                      5.0     // Metres

#define LAND_M                          3.0     // Metres deemed to have landed when below this height

#define ALT_MAX_THR_COMP                40L     // Stick units was 32

#define ALT_INT_WINDUP_LIMIT            16L

#define ALT_RF_ENABLE_M                 5.0   // altitude below which the rangefiner is selected as the altitude source
#define ALT_RF_DISABLE_M                6.0    // altitude above which the rangefiner is deselected as the altitude source

// Navigation

#define NAV_ACQUIRE_BEEPER

//#define ATTITUDE_NO_LIMITS                // full stick range is available otherwise it is scaled to Nav sensitivity

#define NAV_RTH_LOCKOUT                 ((10.0*PI)/180.0)    // first number is degrees

#define NAV_MAX_ROLL_PITCH              25L     // Rx stick units
#define NAV_CONTROL_HEADROOM            10L     // at least this much stick control headroom above Nav control    
#define NAV_DIFF_LIMIT                  24L     // Approx double NAV_INT_LIMIT
#define NAV_INT_WINDUP_LIMIT            64L     // ???

#define NAV_ENFORCE_ALTITUDE_CEILING            // limit all autonomous altitudes
#define NAV_CEILING                     120L    // 400 feet
#define NAV_MAX_NEUTRAL_RADIUS          3L      // Metres also minimum closing radius
#define NAV_MAX_RADIUS                  99L     // Metres

#ifdef NAV_WING
#define NAV_PROXIMITY_RADIUS           20.0     // Metres if there are no WPs
#define NAV_PROXIMITY_ALTITUDE         5.0      // Metres
#else
#define NAV_PROXIMITY_RADIUS           5.0      // Metres if there are no WPs
#define NAV_PROXIMITY_ALTITUDE         3.0      // Metres
#endif // NAV_WING

// reads $GPGGA sentence - all others discarded

#define GPS_MIN_SATELLITES              6       // preferably > 5 for 3D fix
#define GPS_MIN_FIX                     1       // must be 1 or 2 
#define GPS_ORIGIN_SENTENCES            30L     // Number of sentences needed to obtain reasonable Origin
#define GPS_MIN_HDILUTE                 130L    // HDilute * 100

#define NAV_SENS_THRESHOLD              40L     // Navigation disabled if Ch7 is less than this
#define NAV_SENS_ALTHOLD_THRESHOLD      20L     // Altitude hold disabled if Ch7 is less than this
#define NAV_SENS_6CH                    80L     // Low GPS gain for 6ch Rx

#define NAV_YAW_LIMIT                   10L     // yaw slew rate for RTH
#define NAV_MAX_TRIM                    20L     // max trim offset for altitude hold
#define NAV_CORR_SLEW_LIMIT             1L      // *5L maximum change in roll or pitch correction per GPS update

#define ATTITUDE_HOLD_LIMIT             8L      // dead zone for roll/pitch stick for position hold
#define ATTITUDE_HOLD_RESET_INTERVAL    25L     // number of impulse cycles before GPS position is re-acquired

//#define NAV_PPM_FAILSAFE_RTH                // PPM signal failure causes RTH with Signal sampled periodically

// Throttle

#define THROTTLE_MAX_CURRENT            40L     // Amps total current at full throttle for estimated mAH
#define CURRENT_SENSOR_MAX              50L     // Amps range of current sensor - used for estimated consumption - no actual sensor yet.
#define THROTTLE_CURRENT_SCALE          ((THROTTLE_MAX_CURRENT * 1024L)/(200L * CURRENT_SENSOR_MAX ))

#define THROTTLE_SLEW_LIMIT             0       // limits the rate at which the throttle can change (=0 no slew limit, 5 OK)
#define THROTTLE_MIDDLE                 10      // throttle stick dead zone for baro 
#define THROTTLE_MIN_ALT_HOLD           75      // min throttle stick for altitude lock

// RC

#define RC_INIT_FRAMES                  32      // number of initial RC frames to allow filters to settle

#define RC_MIN_WIDTH_US                 900
#define RC_MAX_WIDTH_US                 2100

#define RC_NO_CHANGE_TIMEOUT_MS         20000L        // mS.

typedef union {
    int16 i16;
    uint16 u16;
    struct {
        uint8 b0;
        uint8 b1;
    };
    struct {
        int8 pad;
        int8 i1;
    };
} i16u;

typedef union {
    int24 i24;
    uint24 u24;
    struct {
        uint8 b0;
        uint8 b1;
        uint8 b2;
    };
    struct {
        uint8 pad;
        int16 i2_1;
    };
} i24u;

typedef union {
    int32 i32;
    uint32 u32;
    struct {
        uint8 b0;
        uint8 b1;
        uint8 b2;
        uint8 b3;
    };
    struct {
        uint16 w0;
        uint16 w1;
    };
    struct {
        int16 pad;
        int16 iw1;
    };

    struct {
        uint8 padding;
        int24 i3_1;
    };
} i32u;

#define TX_BUFF_MASK    511
#define RX_BUFF_MASK    511

typedef struct { // PPM
    uint8 Head;
    int16 B[4][8];
} int16x8x4Q;

typedef struct { // Baro
    uint8 Head, Tail;
    int24 B[8];
} int24x8Q;

typedef struct { // GPS
    uint8 Head, Tail;
    int32 Lat[4], Lon[4];
} int32x4Q;

// Macros

#define Sign(i)                 (((i)<0) ? -1 : 1)

#define Max(i,j)                ((i<j) ? j : i)
#define Min(i,j)                ((i<j) ? i : j )
#define Decay1(i)               (((i) < 0) ? (i+1) : (((i) > 0) ? (i-1) : 0))
#define Limit(i,l,u)            (((i) < l) ? l : (((i) > u) ? u : (i)))
#define Sqr(v)                  ( v * v )

// To speed up NMEA sentence processing
// must have a positive argument
#define ConvertDDegToMPi(d)     (((int32)d * 3574L)>>11)
#define ConvertMPiToDDeg(d)     (((int32)d * 2048L)/3574L)

#define ToPercent(n, m)         (((n)*100L)/m)

// Simple filters using weighted averaging
#define VerySoftFilter(O,N)     (SRS16((O)+(N)*3, 2))
#define SoftFilter(O,N)         (SRS16((O)+(N), 1))
#define MediumFilter(O,N)       (SRS16((O)*3+(N), 2))
#define HardFilter(O,N)         (SRS16((O)*7+(N), 3))

// Unsigned
#define VerySoftFilterU(O,N)    (((O)+(N)*3+2)>>2)
#define SoftFilterU(O,N)        (((O)+(N)+1)>>1)
#define MediumFilterU(O,N)      (((O)*3+(N)+2)>>2)
#define HardFilterU(O,N)        (((O)*7+(N)+4)>>3)

#define NoFilter(O,N)           (N)

#define DisableInterrupts       (INTCONbits.GIEH=0)
#define EnableInterrupts        (INTCONbits.GIEH=1)
#define InterruptsEnabled       (INTCONbits.GIEH)

// PARAMS

#define PARAMS_ADDR_PX      0         // code assumes zero!
#define MAX_PARAMETERS      64        // parameters in PXPROM start at zero

#define STATS_ADDR_PX       ( PARAMS_ADDR_PX + (MAX_PARAMETERS *2) )
#define MAX_STATS           64

// uses second Page of PXPROM
#define NAV_ADDR_PX         256L
// 0 - 8 not used

#define NAV_NO_WP           (NAV_ADDR_PX + 9)
#define NAV_PROX_ALT        (NAV_ADDR_PX + 10)
#define NAV_PROX_RADIUS     (NAV_ADDR_PX + 11)
#define NAV_ORIGIN_ALT      (NAV_ADDR_PX + 12)
#define NAV_ORIGIN_LAT      (NAV_ADDR_PX + 14)
#define NAV_ORIGIN_LON      (NAV_ADDR_PX + 18)
#define NAV_RTH_ALT_HOLD    (NAV_ADDR_PX + 22)    // P[NavRTHAlt]
#define NAV_WP_START        (NAV_ADDR_PX + 24)

#define WAYPOINT_REC_SIZE    (4 + 4 + 2 + 1)      // Lat + Lon + Alt + Loiter
#define NAV_MAX_WAYPOINTS    ((256 - 24 - 1)/WAYPOINT_REC_SIZE)

//______________________________________________________________________________________________

// main.c

enum Directions {BF, LR, UD, Alt }; // x forward, y left and z down
enum Angles { Pitch, Roll, Yaw };   // X, Y, Z

#define FLAG_BYTES 10
#define TELEMETRY_FLAG_BYTES 6
typedef union {
    uint8 AllFlags[FLAG_BYTES];
    struct { // Order of these flags subject to change
        uint8
AltHoldEnabled:
        1,
AllowTurnToWP:
        1,            // stick programmed
GyroFailure:
        1,
LostModel:
        1,
NearLevel:
        1,
LowBatt:
        1,
GPSValid:
        1,
NavValid:
        1,

BaroFailure:
        1,
AccFailure:
        1,
CompassFailure:
        1,
GPSFailure:
        1,
AttitudeHold:
        1,
ThrottleMoving:
        1,
HoldingAlt:
        1,
Navigate:
        1,

ReturnHome:
        1,
WayPointAchieved:
        1,
WayPointCentred:
        1,
Unused2: // was UsingGPSAlt:
        1,
UsingRTHAutoDescend:
        1,
BaroAltitudeValid:
        1,
RangefinderAltitudeValid:
        1,
UsingRangefinderAlt:
        1,

        // internal flags not really useful externally

AllowNavAltitudeHold:
        1,    // stick programmed
UsingPositionHoldLock:
        1,
Ch5Active:
        1,
Simulation:
        1,
AcquireNewPosition:
        1,
MotorsArmed:
        1,
NavigationActive:
        1,
ForceFailsafe:
        1,

Signal:
        1,
RCFrameOK:
        1,
ParametersValid:
        1,
RCNewValues:
        1,
NewCommands:
        1,
AccelerationsValid:
        1,
CompassValid:
        1,
CompassMissRead:
        1,

UsingPolarCoordinates:
        1,
UsingAngleControl:
        1,
GPSPacketReceived:
        1,
NavComputed:
        1,
AltitudeValid:
        1,
UsingSerialPPM:
        1,
UsingTxMode2:
        1,
UsingAltOrientation:
        1,

        // outside telemetry flags

UsingTelemetry:
        1,
TxToBuffer:
        1,
NewBaroValue:
        1,
BeeperInUse:
        1,
RFInInches:
        1,
FirstArmed:

        1,
HaveGPRMC:
        1,
UsingPolar:
        1,
RTCValid:
        1,
SDCardValid:
        1,
PXImageStale:
        1,
UsingLEDDriver:
        1,
Using9DOF:
        1,
HaveBatterySensor:
        1;
    };
} Flags;

enum FlightStates { Starting, Landing, Landed, Shutdown, InFlight};
extern volatile Flags F;
extern int8 State;

//______________________________________________________________________________________________

// accel.c

#define ACC_FREQ  50     // Hz must be less than 100Hz
const real32 OneOnTwoPiAccF = ( 1.0 / ( TWOPI * ACC_FREQ ));

enum AccelerometerTypes { LISLAcc, ADXL345Acc, AccUnknown };

extern void ShowAccType(void);
extern void ReadAccelerometers(void);
extern void GetNeutralAccelerations(void);
extern void GetAccelerations(void);
extern void AccelerometerTest(void);
extern void InitAccelerometers(void);

// ADXL345 3-Axis Accelerometer

#ifdef SIX_DOF
#define ADXL345_ID         0xA6
#else
#define ADXL345_ID         0x53
#endif // 6DOF

#define ADXL345_WR           ADXL345_ID
#define ADXL345_RD           (ADXL345_ID+1)

extern const float GRAVITY_ADXL345;

extern void ReadADXL345Acc(void);
extern void InitADXL345Acc(void);
extern boolean ADXL345AccActive(void);

// LIS3LV02DG 3-Axis Accelerometer 400KHz

extern const float GRAVITY_LISL;

#define LISL_WHOAMI      0x0f
#define LISL_OFFSET_X    0x16
#define LISL_OFFSET_Y    0x17
#define LISL_OFFSET_Z    0x18
#define LISL_GAIN_X      0x19
#define LISL_GAIN_Y      0x1A
#define LISL_GAIN_Z      0x1B
#define LISL_CTRLREG_1   0x20
#define LISL_CTRLREG_2   0x21
#define LISL_CTRLREG_3   0x22
#define LISL_STATUS      0x27
#define LISL_OUTX_L      0x28
#define LISL_OUTX_H      0x29
#define LISL_OUTY_L      0x2A
#define LISL_OUTY_H      0x2B
#define LISL_OUTZ_L      0x2C
#define LISL_OUTZ_H      0x2D
#define LISL_FF_CFG      0x30
#define LISL_FF_SRC      0x31
#define LISL_FF_ACK      0x32
#define LISL_FF_THS_L    0x34
#define LISL_FF_THS_H    0x35
#define LISL_FF_DUR      0x36
#define LISL_DD_CFG      0x38
#define LISL_INCR_ADDR   0x40
#define LISL_READ        0x80
#define LISL_ID          0x3a

extern void WriteLISL(uint8, uint8);
extern void ReadLISLAcc(void);
extern boolean LISLAccActive(void);

// other accelerometers

extern real32 Vel[3], Acc[3], AccNeutral[3];
extern int16 NewAccNeutral[3];
extern uint8 AccelerometerType;

//______________________________________________________________________________________________

// analog.c

extern real32 ADC(uint8);
extern void GetBattery(void);
extern void BatteryTest(void);
extern void InitBattery(void);

extern void GetRangefinderAltitude(void);
extern void InitRangefinder(void);

extern real32 BatteryVolts, BatteryCurrentADCEstimated, BatteryChargeUsedAH;
extern real32 BatteryCharge, BatteryCurrent;
extern real32 BatteryVoltsScale;

extern real32 RangefinderAltitude;

//______________________________________________________________________________________________

// attitude.c

enum AttitudeMethods { PremerlaniDCM,  MadgwickIMU,  MadgwickAHRS};

extern void GetAttitude(void);
extern void DoLegacyYawComp(void);
extern void AttitudeTest(void);
extern void InitAttitude(void);

extern real32 dT, dTR;
extern uint32 PrevDCMUpdate;
extern uint8 AttitudeMethod;

// DCM Premerlani

extern void DCMNormalise(void);
extern void DCMDriftCorrection(void);
extern void AccAdjust(void);
extern void DCMMotionCompensation(void);
extern void DCMUpdate(void);
extern void DCMEulerAngles(void);

extern real32 RollPitchError[3];
extern real32 AccV[3];
extern real32 GyroV[3];
extern real32 OmegaV[3];
extern real32 OmegaP[3];
extern real32 OmegaI[3];
extern real32 Omega[3];
extern real32 DCM[3][3];
extern real32 U[3][3];
extern real32 TempM[3][3];

// IMU & AHRS S.O.H. Madgwick

extern void IMUupdate(real32, real32, real32, real32, real32, real32);
extern void AHRSupdate(real32, real32, real32, real32, real32, real32, real32, real32, real32);
extern void EulerAngles(void);

extern real32 q0, q1, q2, q3;    // quaternion elements representing the estimated orientation

//______________________________________________________________________________________________

// autonomous.c

extern void DoShutdown(void);
extern void FailsafeHoldPosition(void);
extern void DoPolarOrientation(void);
extern void Navigate(int32, int32);
extern void SetDesiredAltitude(int16);
extern void DoFailsafeLanding(void);
extern void AcquireHoldPosition(void);
extern void NavGainSchedule(int16);
extern void DoNavigation(void);
extern void FakeFlight(void);
extern void DoPPMFailsafe(void);
extern void WriteWayPointPX(uint8, int32, int32, int16, uint8);
extern void UAVXNavCommand(void);
extern void GetWayPointPX(int8);
extern void InitNavigation(void);

typedef struct {
    int32 E, N;
    int16 A;
    uint8 L;
} WayPoint;

enum NavStates { HoldingStation, ReturningHome, AtHome, Descending, Touchdown, Navigating, Loitering};
enum FailStates { MonitoringRx, Aborting, Terminating, Terminated };

extern real32 NavCorr[3], NavCorrp[3];
extern real32 NavE[3], NavEp[3], NavIntE[3];
extern int16 NavYCorrLimit;

extern int8 FailState;
extern WayPoint WP;
extern int8 CurrWP;
extern int8 NoOfWayPoints;
extern int16 WPAltitude;
extern int32 WPLatitude, WPLongitude;

extern real32 WayHeading;
extern real32 NavPolarRadius, NavNeutralRadius, NavProximityRadius, NavProximityAltitude;

extern int24 NavRTHTimeoutmS;
extern int8 NavState;
extern int16 NavSensitivity, RollPitchMax;
extern int16 AltSum;

extern int16 EffNavSensitivity;
extern int16 EastP, EastDiffSum, EastI, EastCorr, NorthP, NorthDiffSum, NorthI, NorthCorr;
extern int24 EastD, EastDiffP, NorthD, NorthDiffP;

//______________________________________________________________________________________________

// baro.c

#define BARO_INIT_RETRIES    10    // max number of initialisation retries

enum BaroTypes { BaroBMP085, BaroSMD500, BaroMPX4115, BaroUnknown };

#define ADS7823_TIME_MS     50      // 20Hz
#define ADS7823_MAX         4095    // 12 bits
#define ADS7823_ID          0x90    // ADS7823 ADC
#define ADS7823_WR          ADS7823_ID      // ADS7823 ADC
#define ADS7823_RD          (ADS7823_ID+1)  // ADS7823 ADC
#define ADS7823_CMD         0x00

extern uint8 MCP4725_ID_Actual;

#define MCP4725_MAX         4095    // 12 bits
#define MCP4725_ID_0xC8     0xc8
#define MCP4725_ID_0xCC     0xcc
#define MCP4725_CMD         0x40    // write to DAC registor in next 2 bytes
#define MCP4725_EPROM       0x60    // write to DAC registor and eprom

extern void SetFreescaleMCP4725(int16);
extern void SetFreescaleOffset(void);
extern void ReadFreescaleBaro(void);
extern real32 FreescaleToDM(int24);
extern void GetFreescaleBaroAltitude(void);
extern boolean IsFreescaleBaroActive(void);
extern boolean IdentifyMCP4725(void);
extern void InitFreescaleBarometer(void);

#define BOSCH_ID_BMP085         0x55
#define BOSCH_ID                0xee
#define BOSCH_WR                BOSCH_ID_BMP085
#define BOSCH_RD                (BOSCH_ID_BMP085+1)
#define BOSCH_TEMP_BMP085       0x2e
#define BOSCH_TEMP_SMD500       0x6e
#define BOSCH_PRESS             0xf4
#define BOSCH_CTL               0xf4                // OSRS=3 for BMP085 25.5mS, SMD500 34mS                
#define BOSCH_ADC_MSB           0xf6
#define BOSCH_ADC_LSB           0xf7
#define BOSCH_ADC_XLSB          0xf8                // BMP085
#define BOSCH_TYPE              0xd0

extern void StartBoschBaroADC(boolean);
extern void ReadBoschBaro(void);
extern int24 CompensatedBoschPressure(uint16, uint16);
extern void GetBoschBaroAltitude(void);
extern boolean IsBoschBaroActive(void);
extern void InitBoschBarometer(void);

extern void GetBaroAltitude(void);
extern void InitBarometer(void);

extern void ShowBaroType(void);
extern void BaroTest(void);

extern int32 OriginBaroPressure, CompBaroPressure;
extern uint16 BaroPressure, BaroTemperature;
extern boolean AcquiringPressure;
extern real32 BaroRelAltitude, BaroRelAltitudeP;
extern i16u BaroVal;
extern int8 BaroType;
extern int16 AltitudeUpdateRate;
extern int8 BaroRetries;

extern real32 FakeBaroRelAltitude;

//______________________________________________________________________________________________

// compass.c

enum CompassTypes { HMC5843, HMC6352, NoCompass };

#define COMPASS_UPDATE_MS               50
#define COMPASS_UPDATE_S                (real32)(COMPASS_UPDATE_MS * 0.001)
#define COMPASS_FREQ                    10      // Hz must be less than 10Hz

#define COMPASS_MAXDEV                30        // maximum yaw compensation of compass heading 
#define COMPASS_MIDDLE                10        // yaw stick neutral dead zone

const real32 OneOnTwoPiCompassF = ( 1.0 / ( TWOPI * COMPASS_FREQ ));

extern void ReadCompass(void);
extern void GetHeading(void);
extern void ShowCompassType(void);
extern void DoCompassTest(void);
extern void CalibrateCompass(void);
extern void InitCompass(void);

// HMC5843 Compass

#define HMC5843_ID      0x3C        // 0x1E 9DOF
#define HMC5843_WR      HMC5843_ID
#define HMC5843_RD      (HMC5843_ID+1)

extern void ReadHMC5843(void);
extern void GetHMC5843Parameters(void);
extern void DoHMC5843Test(void);
extern void CalibrateHMC5843(void);
extern void InitHMC5843(void);
extern boolean IsHMC5843Active(void);

// HMC6352

#define HMC6352_ID       0x42
#define HMC6352_WR       HMC6352_ID
#define HMC6352_RD       (HMC6352_ID+1)

extern void ReadHMC6352(void);
extern uint8 WriteByteHMC6352(uint8);
extern void GetHMC6352Parameters(void);
extern void DoHMC6352Test(void);
extern void CalibrateHMC6352(void);
extern void InitHMC6352(void);
extern boolean HMC6352Active(void);

typedef struct {
    real32 V;
    real32 Offset;
} MagStruct;
extern MagStruct Mag[3];
extern real32 MagDeviation, CompassOffset;
extern real32 MagHeading, Heading, FakeHeading;
extern real32 HeadingSin, HeadingCos;
extern uint8 CompassType;

//______________________________________________________________________________________________

// control.c

extern real32 PTerm, ITerm, DTerm; //zzz

extern void DoAltitudeHold(void);
extern void UpdateAltitudeSource(void);
extern void AltitudeHold(void);

extern void LimitYawSum(void);
extern void InertialDamping(void);
extern void DoOrientationTransform(void);
extern void DoControl(void);

extern void LightsAndSirens(void);
extern void InitControl(void);

extern real32 Angle[3], Anglep[3], Rate[3], Ratep[3];
extern real32 Comp[4];
extern real32 DescentComp;
extern real32 Rl, Pl, Yl, Ylp;
extern real32 GS;

extern int16 HoldYaw, YawSlewLimit;
extern int16 CruiseThrottle, MaxCruiseThrottle, DesiredThrottle, IdleThrottle, InitialThrottle, StickThrottle;
extern int16 DesiredRoll, DesiredPitch, DesiredYaw, DesiredHeading, DesiredCamPitchTrim;
extern real32 ControlRoll, ControlPitch, ControlRollP, ControlPitchP;
extern real32 CameraRollAngle, CameraPitchAngle;
extern int16 CurrMaxRollPitch;

extern int16 AttitudeHoldResetCount;
extern real32 AltDiffSum, AltD, AltDSum;
extern real32 DesiredAltitude, Altitude, AltitudeP;
extern real32 ROC;
extern boolean FirstPass;

extern uint32 AltuSp;
extern int16 DescentLimiter;

extern int16 FakeDesiredPitch, FakeDesiredRoll, FakeDesiredYaw;

//______________________________________________________________________________________________

// gps.c

extern void UpdateField(void);
extern int32 ConvertGPSToM(int32);
extern int32 ConvertMToGPS(int32);
extern int24 ConvertInt(uint8, uint8);
extern real32 ConvertReal(uint8, uint8);
extern int32 ConvertLatLonM(uint8, uint8);
extern void ConvertUTime(uint8, uint8);
extern void ConvertUDate(uint8, uint8);
extern void ParseGPGGASentence(void);
extern void ParseGPRMCSentence(void);
extern void SetGPSOrigin(void);
extern void ParseGPSSentence(void);
extern void RxGPSPacket(uint8);
extern void SetGPSOrigin(void);
extern void DoGPS(void);
extern void GPSTest(void);
extern void UpdateGPS(void);
extern void InitGPS(void);

#define MAXTAGINDEX         4
#define GPSRXBUFFLENGTH     80
typedef struct {
    uint8 s[GPSRXBUFFLENGTH];
    uint8 length;
} NMEAStruct;

#define MAX_NMEA_SENTENCES 2
#define NMEA_TAG_INDEX 4

enum GPSPackeType { GPGGAPacketTag, GPRMCPacketTag,  GPSUnknownPacketTag };
extern NMEAStruct NMEA;
extern const uint8 NMEATags[MAX_NMEA_SENTENCES][5];

extern uint8 GPSPacketTag;
extern tm GPSTime;
extern int32 GPSStartTime, GPSSeconds;
extern int32 GPSLatitude, GPSLongitude;
extern int32 OriginLatitude, OriginLongitude;
extern real32 GPSAltitude, GPSRelAltitude, GPSOriginAltitude;
extern int32 DesiredLatitude, DesiredLongitude;
extern int32 LatitudeP, LongitudeP, HoldLatitude, HoldLongitude;
extern real32 GPSLongitudeCorrection;
extern real32 GPSHeading, GPSMagHeading, GPSMagDeviation, GPSVel, GPSVelp, GPSROC;
extern int8 GPSNoOfSats;
extern int8 GPSFix;
extern int16 GPSHDilute;

extern real32 GPSdT, GPSdTR;
extern uint32 GPSuSp;

extern int32 FakeGPSLongitude, FakeGPSLatitude;

extern uint8 ll, tt, ss, RxCh;
extern uint8 GPSCheckSumChar, GPSTxCheckSum;

//______________________________________________________________________________________________

// gyro.c

enum GyroTypes { MLX90609Gyro, ADXRS150Gyro, IDG300Gyro, LY530Gyro, ADXRS300Gyro, ITG3200Gyro,
                 IRSensors, UnknownGyro
               };

extern void ReadGyros(void);
extern void GetGyroRates(void);
extern void CheckGyroFault(uint8, uint8, uint8);
extern void ErectGyros(void);
extern void GyroTest(void);
extern void InitGyros(void);
extern void ShowGyroType(void);

// Generic Analog Gyrso
extern void ReadAnalogGyros(void);
extern void InitAnalogGyros(void);
extern void CheckAnalogGyroFault(uint8, uint8, uint8);
extern void AnalogGyroTest(void);

// ITG3200 3 Axis Gyro

#define ITG3200_WHO     0x00
#define ITG3200_SMPL    0x15
#define ITG3200_DLPF    0x16
#define ITG3200_INT_C   0x17
#define ITG3200_TMP_H   0x18
#define ITG3200_TMP_L   0x1C
#define ITG3200_GX_H    0x1D
#define ITG3200_GX_L    0x1E
#define ITG3200_GY_H    0x1F
#define ITG3200_GY_L    0x20
#define ITG3200_GZ_H    0x21
#define ITG3200_GZ_L    0x22
#define ITG3200_PWR_M   0x3E

#ifdef SIX_DOF
#define ITG3200_ID      0xD0
#else
#define ITG3200_ID      0xD2
#endif // 6DOF

#define ITG3200_WR       ITG3200_ID
#define ITG3200_RD       (ITG3200_ID+1)

extern void ReadITG3200Gyro(void);
extern uint8 ReadByteITG3200(uint8);
extern void WriteByteITG3200(uint8, uint8);
extern void InitITG3200Gyro(void);
extern void ITG3200Test(void);
extern boolean ITG3200GyroActive(void);

extern const real32 GyroToRadian[];
extern real32 GyroADC[3], GyroNeutral[3], Gyro[3]; // Radians
extern uint8 GyroType;

//______________________________________________________________________________________________

// harness.c

extern void UpdateRTC(void);
extern void InitHarness(void);

extern LocalFileSystem Flash;

// 1 GND
// 2 4.5-9V
// 3 VBat
// 4 NReset

//extern SPI SPI0;                  // 5 SPI MOSI, 6 SPI MOSO, 7 SPI CLK
//extern DigitalOut SPICS;          // 8
extern SDFileSystem SDCard;

//extern I2C I2C1;                  // 9, 10
extern SerialBuffered TelemetrySerial;
extern DigitalIn Armed;             // 11 SPI MOSI
extern DigitalOut PWMCamPitch;      // 12 SPI MOSO    // 12 SPI MOSO
extern Serial GPSSerial;            // 13 Tx1 / SPI CLK, 14 Rx1

extern AnalogIn PitchADC;           // 15 AN0
extern AnalogIn RollADC;            // 16 AN1
extern AnalogIn YawADC;             // 17 AN2

extern AnalogIn RangefinderADC;     // 18 AN3
extern AnalogIn BatteryCurrentADC;  // 19 AN4
extern AnalogIn BatteryVoltsADC;    // 20 AN5

enum ADCValues { ADCPitch, ADCRoll, ADCYaw, ADCRangefinder, ADCBatteryCurrent, ADCBatteryVolts };

extern PwmOut Out0;                 // 21
extern PwmOut Out1;                 // 22
extern PwmOut Out2;                 // 23
extern PwmOut Out3;                 // 24

//extern PwmOut Out4;                 // 25
//extern PwmOut Out5;                 // 26
extern DigitalOut DebugPin;           // 25

#ifdef SW_I2C

class MyI2C {

private:

    boolean waitclock(void);

public:

    void frequency(uint32 f);
    void start(void);
    void stop(void);
    uint8 blockread(uint8 r, char* b, uint8);
    uint8 read(uint8 r);
    void blockwrite(uint8 a, const char* b, uint8 l);
    uint8 write(uint8 d);
};

extern MyI2C I2C0;                    // 27, 28
extern DigitalInOut I2C0SCL;
extern DigitalInOut I2C0SDA;
#else

extern I2C I2C0;                    // 27, 28
#define blockread   read
#define blockwrite  write

#endif // SW_I2C

extern DigitalIn  RCIn;             // 29 CAN
extern DigitalOut PWMCamRoll;      // 30 CAN

//extern Serial TelemetrySerial;
// 31 USB +, 32 USB -
// 34 -37 Ethernet
// 38 IF +
// 39 IF -
// 40 3.3V Out

extern DigitalOut BlueLED;
extern DigitalOut GreenLED;
extern DigitalOut RedLED;
extern DigitalOut YellowLED;

extern InterruptIn RCInterrupt;

extern char RTCString[], RTCLogfile[];
extern struct tm* RTCTime;

#define I2CTEMP I2C0
#define I2CBARO I2C0
#define I2CBAROAddressResponds I2C0AddressResponds
#define I2CGYRO I2C0
#define I2CGYROAddressResponds I2C0AddressResponds
#define I2CACC I2C0
#define I2CACCAddressResponds I2C0AddressResponds
#define I2CCOMPASS I2C0
#define I2CCOMPASSAddressResponds I2C0AddressResponds
#define I2CESC I2C0
#define I2CLED I2C0

#define SPIACC SPI0

#define mSClock timer.read_ms
#define uSClock timer.read_us

//______________________________________________________________________________________________

// irq.c

#define CONTROLS                7
#define MAX_CONTROLS            12     // maximum Rx channels

#define RxFilter            MediumFilterU
//#define RxFilter            SoftFilterU
//#define RxFilter            NoFilter

#define RC_GOOD_BUCKET_MAX      20
#define RC_GOOD_RATIO           4

#define RC_MINIMUM              0

#define RC_MAXIMUM              238

#define RC_NEUTRAL              ((RC_MAXIMUM-RC_MINIMUM+1)/2)

#define RC_MAX_ROLL_PITCH       (170)

#define RC_THRES_STOP           ((6L*RC_MAXIMUM)/100)
#define RC_THRES_START          ((10L*RC_MAXIMUM)/100)

#define RC_FRAME_TIMEOUT_MS     25
#define RC_SIGNAL_TIMEOUT_MS    (5L*RC_FRAME_TIMEOUT_MS)
#define RC_THR_MAX              RC_MAXIMUM

#define MAX_ROLL_PITCH          RC_NEUTRAL    // Rx stick units - rely on Tx Rate/Exp

#ifdef RX6CH
#define RC_CONTROLS 5
#else
#define RC_CONTROLS CONTROLS
#endif //RX6CH

extern void InitTimersAndInterrupts(void);

extern void enableTxIrq0(void);
extern void disableTxIrq0(void);
extern void enableTxIrq1(void);
extern void disableTxIrq1(void);

extern void RCISR(void);
extern void RCNullISR(void);
extern void RCTimeoutISR(void);
extern void GPSInISR(void);
extern void GPSOutISR(void);
extern void TelemetryInISR(void);
extern void TelemetryOutISR(void);
extern void RazorInISR(void);
extern void RazorOutISR(void);

enum { Clock, GeneralCountdown, UpdateTimeout, RCSignalTimeout, BeeperTimeout, ThrottleIdleTimeout,
       FailsafeTimeout, AbortTimeout, RxFailsafeTimeout, DescentUpdate, StickChangeUpdate, NavStateTimeout, LastValidRx,
       LastGPS, StartTime, GPSTimeout, LEDChaserUpdate, LastBattery, TelemetryUpdate, NavActiveTime, BeeperUpdate,
       ArmedTimeout, ThrottleUpdate, RazorUpdate, VerticalDampingUpdate, CamUpdate, BaroUpdate, CompassUpdate
     };

enum WaitPacketStates { WaitSentinel, WaitTag, WaitBody, WaitCheckSum};

extern uint32 mS[];
extern int16 PPM[];
extern int8 PPM_Index;
extern uint32 PrevEdge, CurrEdge;
extern uint8 Intersection, PrevPattern, CurrPattern;
extern uint32 Width;
extern uint8 RxState;
extern boolean WaitingForSync;

extern int8 SignalCount;
extern uint16 RCGlitches;

//______________________________________________________________________________________________

// ir.c

extern void GetIRAttitude(void);
extern void TrackIRMaxMin(uint8);
extern void InitIRSensors(void);

extern real32 IR[3], IRMax, IRMin, IRSwing;

//______________________________________________________________________________________________

// i2c.c

#ifdef SW_I2C
#define I2C_ACK  0
#define I2C_NACK 1
#else
#define I2C_ACK  1
#define I2C_NACK 0
#endif // SW_I2C

extern boolean I2C0AddressResponds(uint8);
#ifdef HAVE_I2C1
extern boolean I2C1AddressResponds(uint8);
#endif // HAVE_I2C1
extern void TrackMinI2CRate(uint32);
extern void ShowI2CDeviceName(uint8);
extern uint8 ScanI2CBus(void);
extern boolean ESCWaitClkHi(void);
extern void ProgramSlaveAddress(uint8);
extern void ConfigureESCs(void);

extern uint32 MinI2CRate;

//______________________________________________________________________________________________

// leds.c

#define PCA9551_ID         0xc0

#define DRV0M              0x0001
#define DRV1M              0x0002
#define DRV2M              0x0004
#define DRV3M              0x0008

#define AUX0M              0x0010
#define AUX1M              0x0020
#define AUX2M              0x0040
#define AUX3M              0x0080

#define BeeperM            DRV0M

#define YellowM            0x0100
#define RedM               0x0200
#define GreenM             0x0400
#define BlueM              0x0800

#define ALL_LEDS_ON        LEDsOn(BlueM|RedM|GreenM|YellowM)
#define ALL_LEDS_OFF       LEDsOff(BlueM|RedM|GreenM|YellowM)

#define AUX_LEDS_ON        LEDsOn(AUX0M|AUX1M|AUX2M|AUX3M)
#define AUX_LEDS_OFF       LEDsOff(AUX0M|AUX1M|AUX2M|AUX3M)

#define AUX_DRVS_ON        LEDsOn(DRV0M|DRV1M|DRV2M|DRV3M)
#define AUX_DRVS_OFF       LEDsOff(DRV0M|DRV1M|DRV2M|DRV3M)

#define ALL_LEDS_ARE_OFF   ( (LEDShadow&(BlueM|RedM|GreenM|YellowM))== (uint8)0 )

#define BEEPER_IS_ON       ( (LEDShadow & BeeperM) != (uint8)0 )
#define BEEPER_IS_OFF      ( (LEDShadow & BeeperM) == (uint8)0 )

#define LEDRed_ON          LEDsOn(RedM)
#define LEDBlue_ON         LEDsOn(BlueM)
#define LEDGreen_ON        LEDsOn(GreenM)
#define LEDYellow_ON       LEDsOn(YellowM)
#define LEDAUX1_ON         LEDsOn(AUX1M)
#define LEDAUX2_ON         LEDsOn(AUX2M)
#define LEDAUX3_ON         LEDsOn(AUX3M)
#define LEDRed_OFF         LEDsOff(RedM)
#define LEDBlue_OFF        LEDsOff(BlueM)
#define LEDGreen_OFF       LEDsOff(GreenM)
#define LEDYellow_OFF      LEDsOff(YellowM)
#define LEDYellow_TOG      if( (LEDShadow&YellowM) == (uint8)0 ) LEDsOn(YellowM); else LEDsOff(YellowM)
#define LEDRed_TOG         if( (LEDShadow&RedM) == (uint8)0 ) LEDsOn(RedM); else LEDsOff(RedM)
#define LEDBlue_TOG        if( (LEDShadow&BlueM) == (uint8)0 ) LEDsOn(BlueM); else LEDsOff(BlueM)
#define LEDGreen_TOG       if( (LEDShadow&GreenM) == (uint8)0 ) LEDsOn(GreenM); else LEDsOff(GreenM)
#define Beeper_OFF         LEDsOff(BeeperM)
#define Beeper_ON          LEDsOn(BeeperM)
#define Beeper_TOG         if( (LEDShadow&BeeperM) == (uint8)0 ) LEDsOn(BeeperM); else LEDsOff(BeeperM)

extern void SendLEDs(void);
extern void SaveLEDs(void);
extern void RestoreLEDs(void);
extern void LEDsOn(uint16);
extern void LEDsOff(uint16);
extern void LEDChaser(void);
extern void DoLEDs(void);
extern void PowerOutput(int8);
extern void LEDsAndBuzzer(void);

extern void PCA9551Test(void);
extern void WritePCA9551(uint8);
extern boolean IsPCA9551Active(void);

extern void InitLEDs(void);

extern uint16 LEDShadow, PrevLEDShadow, SavedLEDs, LEDPattern;
extern uint8 PrevPCA9551LEDShadow;

//______________________________________________________________________________________________

// math.c

extern int16 SRS16(int16, uint8);
extern int32 SRS32(int32, uint8);
extern real32 Make2Pi(real32);
extern real32 MakePi(real32);
extern int16 Table16(int16, const int16 *);

extern real32 VDot(real32 v1[3], real32 v2[3]);
extern void VCross(real32 VOut[3], real32 v1[3], real32 v2[3]);
extern void VScale(real32 VOut[3], real32 v[3], real32 s);
extern void VAdd(real32 VOut[3],real32 v1[3], real32 v2[3]);
extern void VSub(real32 VOut[3],real32 v1[3], real32 v2[3]);

//______________________________________________________________________________________________

// menu.c

extern void ShowPrompt(void);
extern void ShowRxSetup(void);
extern void ShowSetup(boolean);
extern uint8 MakeUpper(uint8);
extern void ProcessCommand(void);

extern const uint8 SerHello[];
extern const uint8 SerSetup[];
extern const uint8 SerPrompt[];

extern const uint8 RxChMnem[];

//______________________________________________________________________________________________

// nonvolatiles.c

extern void CheckSDCardValid(void);

extern void CreateLogfile(void);
extern void CloseLogfile(void);
extern void TxLogChar(uint8);

extern void WritePXImagefile(void);
extern boolean ReadPXImagefile(void);
extern int8 ReadPX(uint16);
extern int16 Read16PX(uint16);
extern int32 Read32PX(uint16);
extern void WritePX(uint16, int8);
extern void Write16PX(uint16, int16);
extern void Write32PX(uint16, int32);

extern FILE *pxfile;
extern FILE *newpxfile;

extern const int PX_LENGTH;
extern int8 PX[], PXNew[];

//______________________________________________________________________________________________

// outputs.c

#define OUT_MINIMUM            1.0              // Required for PPM timing loops
#define OUT_MAXIMUM            200.0            // to reduce Rx capture and servo pulse output interaction
#define OUT_NEUTRAL            105.0            // 1.503mS @ 105 16MHz
#define OUT_HOLGER_MAXIMUM     225.0
#define OUT_YGEI2C_MAXIMUM     240.0
#define OUT_X3D_MAXIMUM        200.0

extern uint8 SaturInt(int16);
extern void DoMulticopterMix(real32 CurrThrottle);
extern void CheckDemand(real32 CurrThrottle);
extern void MixAndLimitMotors(void);
extern void MixAndLimitCam(void);
extern void OutSignals(void);
extern void InitI2CESCs(void);
extern void StopMotors(void);
extern void ExercisePWM(void);
extern void InitMotors(void);

// Using clockwise numbering - NOT the same as UAVXPIC
enum PWMCamTags { CamRollC = 6, CamPitchC = 7 };
enum PWMQuadTags {FrontC=0, RightC, BackC, LeftC }; // order is important for X3D & Holger ESCs
enum PWMConvTags {ThrottleC=0, AileronC, ElevatorC, RudderC, RTHC };
enum PWMWingTags3 {RightElevonC=1, LeftElevonC=2};
enum PWMVTTags {FrontRightC=0, FrontLeftC};
enum PWMY6Tags {FrontTC=0, RightTC, LeftTC, FrontBC, RightBC, LeftBC };
enum PWMHexTags {FrontHC=0, FrontRightHC, BackRightHC, BackHC, BackLeftHC,FrontLeftHC };

//#define NoOfPWMOutputs            4
#ifdef HEXACOPTER
#define NoOfI2CESCOutputs         6
#else
#define NoOfI2CESCOutputs         4
#endif // HEXACOPTER

extern const real32 PWMScale;

extern real32 PWM[8];
extern real32 PWMSense[8];
extern int16 ESCI2CFail[6];
extern int16 CurrThrottle;
extern int16 CamRollPulseWidth, CamPitchPulseWidth;
extern int16 ESCMin, ESCMax;

//______________________________________________________________________________________________

// params.c

extern void ReadParameters(void);
extern void UseDefaultParameters(void);
extern void UpdateParamSetChoice(void);
extern boolean ParameterSanityCheck(void);
extern void InitParameters(void);

enum TxRxTypes {
    FutabaCh3, FutabaCh2, FutabaDM8, JRPPM, JRDM9, JRDXS12,
    DX7AR7000, DX7AR6200, FutabaCh3_6_7, DX7AR6000, GraupnerMX16s, DX6iAR6200, FutabaCh3_R617FS,
    DX7aAR7000, ExternalDecoder, FrSkyDJT_D8R, UnknownTxRx, CustomTxRx
};
enum RCControls {ThrottleRC, RollRC, PitchRC, YawRC, RTHRC, CamPitchRC, NavGainRC};
enum ESCTypes { ESCPPM, ESCHolger, ESCX3D, ESCYGEI2C };
enum AFs { QuadAF, TriAF, VTAF, Y6AF, HeliAF, ElevAF, AilAF };

enum Params { // MAX 64
    RollKp,                // 01
    RollKi,                // 02
    RollKd,                // 03
    HorizDampKp,           // 04c
    RollIntLimit,          // 05
    PitchKp,               // 06
    PitchKi,               // 07
    PitchKd,               // 08
    AltKp,                 // 09c
    PitchIntLimit,         // 10

    YawKp,                 // 11
    YawKi,                 // 12
    YawKd,                 // 13
    YawLimit,              // 14
    YawIntLimit,           // 15
    ConfigBits,            // 16c
    unused17 ,             // 17 TimeSlots
    LowVoltThres,          // 18c
    CamRollKp,             // 19
    PercentCruiseThr,      // 20c

    VertDampKp,            // 21c
    MiddleUD,              // 22c
    PercentIdleThr,        // 23c
    MiddleLR,              // 24c
    MiddleBF,              // 25c
    CamPitchKp,            // 26
    CompassKp,             // 27
    AltKi,                 // 28c
    unused29 ,             // 29 NavRadius
    NavKi,                 // 30

    GSThrottle,            // 31
    Acro,                  // 32
    NavRTHAlt,             // 33
    NavMagVar,             // 34c
    GyroRollPitchType,     // 35
    ESCType,               // 36
    TxRxType,              // 37
    NeutralRadius,         // 38
    PercentNavSens6Ch,     // 39
    CamRollTrim,           // 40
    NavKd,                 // 41
    VertDampDecay,         // 42
    HorizDampDecay,        // 43
    BaroScale,             // 44
    TelemetryType,         // 45
    MaxDescentRateDmpS,    // 46
    DescentDelayS,         // 47
    NavIntLimit,           // 48
    AltIntLimit,           // 49
    unused50,              // 50 GravComp
    unused51 ,             // 51 CompSteps
    ServoSense,            // 52
    CompassOffsetQtr,      // 53
    BatteryCapacity,       // 54
    unused55,              // 55 GyroYawType
    AltKd,                 // 56
    Orient,                // 57
    NavYawLimit,           // 58
    Balance                // 59

    // 56 - 64 unused currently
};

//#define FlyXMode                 0
//#define FlyAltOrientationMask    0x01

#define UsePositionHoldLock      0
#define UsePositionHoldLockMask  0x01

#define UseRTHDescend            1
#define UseRTHDescendMask        0x02

#define TxMode2                  2
#define TxMode2Mask              0x04

#define RxSerialPPM              3
#define RxSerialPPMMask          0x08

#define RFInches                 4
#define RFInchesMask             0x10

// bit 4 is pulse polarity for 3.15

#define UseUseAngleControl       5
#define UseAngleControlMask      0x20

#define UsePolar                 6
#define UsePolarMask             0x40

// bit 7 unusable in UAVPSet

extern const int8 DefaultParams[MAX_PARAMETERS][2];

extern const uint8 ESCLimits [];

extern real32 OSin[], OCos[];
extern uint8 Orientation, PolarOrientation;

extern uint8 ParamSet;
extern boolean ParametersChanged, SaveAllowTurnToWP;
extern int8 P[];
extern real32 K[MAX_PARAMETERS]; // Arm rescaled legacy parameters as appropriate

extern uint8 UAVXAirframe;

//__________________________________________________________________________________________

// rc.c

extern void DoRxPolarity(void);
extern void InitRC(void);
extern void MapRC(void);
extern void CheckSticksHaveChanged(void);
extern void UpdateControls(void);
extern void CaptureTrims(void);
extern void CheckThrottleMoved(void);
extern void ReceiverTest(void);

extern const boolean PPMPosPolarity[];
extern const uint8 Map[CustomTxRx+1][CONTROLS];
extern int8 RMap[];

#define PPMQMASK 3
extern int16 PPMQSum[];
extern int16x8x4Q PPMQ;
extern boolean RCPositiveEdge;
extern int16 RC[], RCp[];
extern int16 Trim[3];
extern int16 ThrLow, ThrNeutral, ThrHigh;

//__________________________________________________________________________________________

// serial.c

extern void TxString(const uint8*);
extern void TxChar(uint8);
extern void TxValU(uint8);
extern void TxValS(int8);
extern void TxBin8(uint8);
extern void TxNextLine(void);
extern void TxNibble(uint8);
extern void TxValH( uint8);
extern void TxValH16(uint16);
extern uint8 RxChar(void);
extern uint8 PollRxChar(void);
extern uint8 RxNumU(void);
extern int8 RxNumS(void);
extern void TxVal32(int32, int8, uint8);
extern void TxChar(uint8);
extern void TxESCu8(uint8);
extern void Sendi16(int16);
extern void TxESCi8(int8);
extern void TxESCi16(int16);
extern void TxESCi24(int24);
extern void TxESCi32(int32);

//______________________________________________________________________________________________

// stats.c

extern void ZeroStats(void);
extern void ReadStatsPX(void);
extern void WriteStatsPX(void);
extern void ShowStats(void);

enum Statistics {
    GPSAltitudeS, BaroRelAltitudeS, ESCI2CFailS, GPSMinSatsS, MinROCS, MaxROCS, GPSVelS,
    AccFailS, CompassFailS, BaroFailS, GPSInvalidS, GPSMaxSatsS, NavValidS,
    MinHDiluteS, MaxHDiluteS, RCGlitchesS, GPSBaroScaleS, GyroFailS, RCFailsafesS, I2CFailS, MinTempS, MaxTempS, BadS, BadNumS
}; // NO MORE THAN 32 or 64 bytes

extern int16 Stats[];

//______________________________________________________________________________________________

// telemetry.c

extern void RxTelemetryPacket(uint8);
extern void InitTelemetryPacket(void);
extern void BuildTelemetryPacket(uint8);

extern void SendPacketHeader(void);
extern void SendPacketTrailer(void);

extern void SendTelemetry(void);
extern void SendUAVX(void);
extern void SendUAVXControl(void);
extern void SendFlightPacket(void);
extern void SendNavPacket(void);
extern void SendControlPacket(void);
extern void SendStatsPacket(void);
extern void SendParamPacket(uint8, uint8);
extern void SendParameters(uint8);
extern void SendMinPacket(void);
extern void SendArduStation(void);
extern void SendCustom(void);
extern void SensorTrace(void);
extern void CheckTelemetry(void);

enum TelemetryStates { WaitRxSentinel, WaitRxESC,  WaitRxBody };


enum PacketTags {UnknownPacketTag = 0, LevPacketTag, NavPacketTag, MicropilotPacketTag, WayPacketTag,
                 AirframePacketTag, NavUpdatePacketTag, BasicPacketTag, RestartPacketTag, TrimblePacketTag,
                 MessagePacketTag, EnvironmentPacketTag, BeaconPacketTag, UAVXFlightPacketTag,
                 UAVXNavPacketTag, UAVXStatsPacketTag, UAVXControlPacketTag, UAVXParamPacketTag, UAVXMinPacketTag, 
                 UAVXArmParamPacketTag, UAVXStickPacketTag, FrSkyPacketTag = 99
                };

enum TelemetryTypes { NoTelemetry, GPSTelemetry, UAVXTelemetry, UAVXControlTelemetry, UAVXMinTelemetry,
                      ArduStationTelemetry, CustomTelemetry
                    };

extern uint8 UAVXCurrPacketTag;
extern uint8 RxPacketLength, RxPacketByteCount;
extern uint8 RxCheckSum;
extern uint8 RxPacketTag, ReceivedPacketTag;
extern uint8 PacketRxState;
extern boolean CheckSumError, TelemetryPacketReceived;

extern int16 RxLengthErrors, RxTypeErrors, RxCheckSumErrors;

extern uint8 TxCheckSum;

extern FILE *logfile;
extern boolean EchoToLogFile, LogfileIsOpen;
extern uint32 LogChars;

//______________________________________________________________________________________________

// temperature.c

#define TMP100_MAX_ADC      4095      // 12 bits
#define TMP100_ID           0x96
#define TMP100_WR           0x96      // Write
#define TMP100_RD           0x97      // Read    
#define TMP100_TMP          0x00      // Temperature
#define TMP100_CMD          0x01
#define TMP100_LOW          0x02      // Alarm low limit
#define TMP100_HI           0x03      // Alarm high limit
#define TMP100_CFG          0         // 0.5 deg resolution continuous

extern void GetTemperature(void);
extern void InitTemperature(void);

extern i16u AmbientTemperature;

//______________________________________________________________________________________________

// utils.c

extern void InitMisc(void);
extern void Delay1mS(int16);
extern void Delay100mS(int16);
extern void DoBeep100mS(uint8, uint8);
extern void DoStartingBeeps(uint8);
extern real32 SlewLimit(real32, real32, real32);
extern real32 DecayX(real32, real32);
extern void LPFilter(real32*, real32*, real32, real32);
extern void CheckAlarms(void);
extern void Timing(uint8, uint32);

typedef struct {
    uint32 T;
    uint32 Count;
} TimingRec;

enum Timed { GetAttitudeT , UnknownT };
extern TimingRec Times[];

#define TimeS  uint32 TStart;TStart=timer.read_us();
#define TimeF(w, tf) Time(w, tf)

//______________________________________________________________________________________________


// Sanity checks

#if (( defined TRICOPTER + defined QUADROCOPTER + defined VTCOPTER + defined Y6COPTER + defined HELICOPTER + defined AILERON + defined ELEVON ) != 1)
#error None or more than one aircraft configuration defined !
#endif
