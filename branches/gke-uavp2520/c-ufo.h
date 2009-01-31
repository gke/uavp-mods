
#define COMMISSIONING
#define READABLE
//#define RAW_BOOT
//#define BOOTONLY

#ifndef BATCHMODE
// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =              Ported to 18F2520 2008 by Prof. Greg Egan              =
// =                          http://www.uavp.org                        =
// =======================================================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include <p18cxxx.h> 
#include <math.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>
#include <capture.h>

#undef BATCHMODE

// if you want the led outputs for test purposes
#define NOLEDGAME

// Only one of the following 3 defines must be activated:
// When using 3 ADXRS300 gyros
//#define OPT_ADXRS300

// When using 3 ADXRS150 gyros
//#define OPT_ADXRS150

// When using 1 ADXRS300 and 1 IDG300 gyro
#define OPT_IDG

// Select what speeed controllers to use:
// to use standard PPM pulse
#define ESC_PPM
// to use X3D BL controllers (not yet tested. Info courtesy to Jast :-)
//#define ESC_X3D
// to use Holgers ESCs (tested and confirmed by ufo-hans)
//#define ESC_HOLGER
// to use YGE I2C controllers (for standard YGEs use ESC_PPM)
//#define ESC_YGEI2C

// defined: serial PPM pulse train from receiver
// RX_DEFAULT: standard servo pulses from CH1, 3, 5 and 7
#define RX_DEFAULT
//#define RX_PPM

// uncomment this to enable Tri-Copter Mixing.
// connector K1 = front motor
//           K2 = rear left motor 
//           K3 = rear right motor 
//           K4 = yaw servo output
// Camera controlling can be used!
//#define TRICOPTER

// Debugging output of the 4 motors on RS232
// When throttling up, every cycle  4 numbers are output
// on the com port:
// <front>;<rear>;<left>;<right> CR LF
// these are the throttle value to the 4 speed controllers
// log them with a terminal proggie to make nice charts :-)
// Reg. 26 ( CamFactor in UAVPSet) controls if motors should run or not
// bit 0 = front, 1 = rear, 2 = left, 3 = right
// if a bit is 1 motor is always stopped, only comport output
// if a bit is 0 motor is running as normal.
// so you can test individual axis easily!
// Reg. 26 bit 4: 1=serial data output on
// NOTE: enable DEBUG_MOTORS only on 3.1 boards (black PCB)!
//#define DEBUG_MOTORS

// special mode for sensor data output (with UAVPset)
#define DEBUG_SENSORS

#endif // !BATCHMODE

// Performs initial configuration of HMC6532 Compass. Can also be 
// performed using TestSoftware 
#define CFG_COMPASS

// Includes GPS functionality and limited autonomous capability
//#define USE_GPS
// Use standard NMEA protocol otherwise Trimble TSIP protocol
#define GPS_NMEA

// 18F2520 PIC can be run at 32MHz with an 8MHz XTAL. At 16MHz there is no 
// camera control to reduce the complexity/reliability of output pulse
// generation. Comment out if using 32MHz
#define CLOCK_16MHZ

// =====================================
// end of user-configurable section!
// =====================================

// this enables common code for all ADXRS gyros
// leave this untouched!
#if defined OPT_ADXRS300 || defined OPT_ADXRS150
#define OPT_ADXRS
#endif

// Misc Constants

#define NUL 0
#define SOH 1
#define ETX 3
#define EOT 4
#define ENQ 5
#define ACK 6
#define HT 9
#define LF 10
#define CR 13
#define DLE 16
#define NAK 21
#define ESC 27

#define EarthR (real32)(6378140.0)
#define Pi (real32)(3.141592654)
#define RecipEarthR (real32)(1.5678552E-7)

#define DEGTOM (real32)(111319.5431531) 
#define MILLIPI 3142 
#define CENTIPI 314 
#define HALFMILLIPI 1571 
#define TWOMILLIPI 6184

#define MILLIRAD 18 
#define CENTIRAD 2 
#define DEGMILLIRAD (real32)(17.453292) 
#define MILLIRADDEG (real32)(0.05729578)
#define MILLIRADDEG100 (real32)(5.729578)
#define CENTIRADDEG (real32)(0.5729578)
#define DEGRAD (real32)(0.017453292)
#define RADDEG (real32)(57.2957812)
#define RADDEG1000000 (real32)(57295781.2)

#define KTTODMPS 5.15 

// Types

#define true 1
#define false 0

#define MAXINT32 0x7fffffff;
#define	MAXINT16 0x7fff;

typedef unsigned char uint8 ;
typedef signed char int8;
typedef unsigned int uint16;
typedef int int16;
typedef long int32;
typedef unsigned long uint32;
typedef float real32;
typedef uint8 boolean;

// Serial I/O
#define RXBUFFMASK 127

// Macros - could be turned into functions if space gets tight

#define USE_MACROS
#define Set(S,b) 		((uint8)(S|=(1<<b)))
#define Clear(S,b) 		((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 		((uint8)((S>>b)&1))
#define IsClear(S,b) 	((uint8)(!(S>>b)&1))
#define Invert(S,b) 	((uint8)(S^=(1<<b)))

#define Upper8(i) 		(i>>8)
#define Lower8(i) 		(i&0xff)
#define Abs(a) 			(a<0 ? -a: a)
#define Sense(a) 		(a<0 ? -1 : (a>0 ? 1 : 1))
 	
#define Max(i,j) 		((i<j) ? j : ((i>j) ? j : i))
#define Limit(i,l,u) 	((i<l) ? l : ((i>u) ? u : i))
#define DecayBand(i,l,u,d) 	((i<l) ? i+d : ((i>u) ? i-d : i))
#define Decay(i) 		((i<0) ? i+1 : ((i>0) ? i-1 : 0))

#define DisableInterrupts (INTCONbits.GIEH=0)
#define EnableInterrupts (INTCONbits.GIEH=1)

// Simple filters using weighted averaging
#ifdef SUPPRESSFILTERS
  #define VerySoftFilter(O,N)		(N)
  #define SoftFilter(O,N) 			(N)
  #define MediumFilter(O,N) 			(N)
  #define AccelerometerFilter(O,N) 	(N)
#else
  #define VerySoftFilter(O,N) 		(SRS16(O+N*3+2, 2))
  #define SoftFilter(O,N) 			(SRS16(O+N+1, 1))
  #define MediumFilter(O,N) 		(SRS16(O*3+N+2, 2))
  #define HardFilter(O,N) 			(SRS16(O*7+N+4, 3))
#endif
#define NoFilter(O,N)				(N)

// Timers

#define _ClkOut		(160/4)								// 16.0 MHz quartz
#define _PreScale0	16									// 1:16 TMR0 prescaler
#define _PreScale1	8									// 1:8 TMR1 prescaler 
#define _PreScale2	16
#define _PostScale2	16

// Because of the (original) CC5X stupid compiler one must calculate by hand :-(
//#define TMR2_9MS	(9000*_ClkOut/(10*_PreScale2*_PostScale2))
//#define TMR2_9MS	141									// 2x 9ms = 18ms pause time 
// modified for Spectrum DX6/DX7 and AR6000
#define TMR2_5MS	78									// 1 x 5ms +  
#define TMR2_14MS	234									// 1 x 15ms = 20ms pause time 

// Motors

// RC pulse times in 10-microseconds units
// Warning: pay attention to numeric overflow (presumably in folding of constants)!
#ifdef ESC_PPM
#define	_Minimum 	1									/* ((105*_ClkOut/(2*_PreScale1))&0xFF)	 -100% */
#define _Maximum 	251									/* 255 causes overflow of Camera pre-pules to 2mS */
#endif
#ifdef ESC_X3D
#define _Minimum	0
#define _Maximum	255
#endif
#ifdef ESC_HOLGER
#define _Minimum	0
#define _Maximum	225	/* ESC demanded */
#endif
#ifdef ESC_YGEI2C
#define _Minimum	0
#define _Maximum	240	/* ESC demanded */
#endif

#define _Neutral	((150* _ClkOut/(2*_PreScale1))&0xFF)    /*   0%  */
#define _ThresStop	((113* _ClkOut/(2*_PreScale1))&0xFF)	/* -90% ab hier Stopp! */
#define _ThresStart	((116* _ClkOut/(2*_PreScale1))&0xFF)	/* -85% ab hier Start! */

// Sanity checks
// please leave them as they are!

// check the PPM RX and motor values
#if _Minimum >= _Maximum
#error _Minimum < _Maximum!
#endif
#if _ThresStart <= _ThresStop
#error _ThresStart <= _ThresStop!
#endif
#if (_Maximum < _Neutral)
#error Maximum < _Neutral !
#endif

// for Drive saturation
#if (_Maximum > 255)
#error Maximum > 255 (uint8) !
#endif

#if (_Maximum < 0)
#error Maximum < 0 (uint8) !
#endif

#if (_Maximum > 255)
#error Maximum > 255 (uint8) !
#endif

#if (_Minimum < 1)
#error Minimum < 1 (uint8) !
#endif

// check gyro model
#if defined OPT_ADXRS150 + defined OPT_ADXRS300 + defined OPT_IDG != 1
#error Define only ONE out of OPT_ADXRS150 OPT_ADXRS300 OPT_IDG
#endif

// check ESC model
#if defined ESC_PPM + defined ESC_X3D + defined ESC_HOLGER + defined ESC_YGEI2C != 1
#error Define only ONE out of ESC_PPM ESC_X3D ESC_HOLGER ESC_YGEI2C
#endif

// check RX model
#if defined RX_DEFAULT + defined RX_PPM + defined RX_DSM2 != 1
#error Define only ONE out of RX_DEFAULT RX_PPM RX_DSM2
#endif

// check debug settings
#if defined DEBUG_MOTORS + defined DEBUG_SENSORS > 1
#error Define only ONE or NONE out of DEBUG_MOTORS DEBUG_SENSORS
#endif
// end of sanity checks
#if defined DEBUG_MOTORS + defined DEBUG_SENSORS > 0
#define DEBUG
#endif

// Timer constants in MilliSec
#define THROTTLE_TIMEOUT		2000
		
#define FAILSAFE_TIMEOUT		1500
#define FAILSAFE_CANCEL			1000

#define AUTONOMOUS_TIMEOUT		5000
#define AUTONOMOUS_CANCEL		10000

// Parameters for UART port
#define _B9600		(_ClkOut*100000/(4*9600) - 1)
#define _B19200		(_ClkOut*100000/(4*19200) - 1)
#define _B38400		(_ClkOut*100000/(4*38400) - 1)
#define _B115200	(_ClkOut*104000/(4*115200) - 1)
#define _B230400	(_ClkOut*100000/(4*115200) - 1)

// Filter selection
#define GyroFilter SoftFilter
#define AccelerometerFilter SoftFilter
#define	StickFilter NoFilter

// The following is to attempt to unify the coordinate system
// to Right Hand Cartesian
//	+X forward
//	+Y right
//	+Z down

//	+P roll right
//	+Q pitch up
//	+R yaw clockwise

// Controls 
//  Aileron + roll right
//	Elevator + pitch up
//	Rudder + yaw anti-clockwise

// Accelerometers
// LIS3LV02DQ Inertial Sensor (Accelerometer)
//  Ax=> Y + right
//	Ay=> Z + up
//	Az=> X + back

#define ACCSIGN_X	(1)
#define ACCSIGN_Y	(-1)
#define ACCSIGN_Z	(-1)

// IDG300
// 3.3V Reference +-500 Deg/Sec
// 0.4882815 Deg/Sec/LSB 
//	+ roll right
//	+ pitch down
//	+ yaw clockwise

// ADRSX300 Yaw
// 5V Reference +-300 Deg/Sec
// 0.2926875 Deg/Sec/LSB 
//	+ roll left
//	+ pitch down
//	+ yaw clockwise

// ADXRS150
// 5V Reference +-150 Deg/Sec
// 0.146484375 Deg/Sec/LSB

// Gyro combinations

#define ADCPORTCONFIG 0b00001010 // AAAAA
#ifdef OPT_IDG
	#define MAXDEGSEC_PITCHROLL 	500
	#define ADCEXTVREF_PITCHROLL 	1
	#define	GYROSIGN_ROLL 			(1)
	#define	GYROSIGN_PITCH 			(-1)
#else
#ifdef OPT_ADXRS150
	#define MAXDEGSEC_PITCHROLL 	150
	#define ADCEXTVREF_PITCHROLL 	0
	#define	GYROSIGN_ROLL 			(-1)
	#define	GYROSIGN_PITCH 			(-1)
#else // OPT_ADXRS300
	#define MAXDEGSEC_PITCHROLL 	300
	#define ADCEXTVREF_PITCH 		0
	#define	GYROSIGN_ROLL 			(-1)
	#define	GYROSIGN_PITCH 			(-1)
#endif
#endif

// Max stick output ???
#define IMAX 50

// Max attitude angles in attitude angle units for now
#define MAX_ROLL				200	
#define	MAX_PITCH				200

// Control limits for autonomous flight - not in PID loop!
#define MAX_ROLL_CTL			((_Maximum+5)/10)			
#define MAX_PITCH_CTL			((_Maximum+5)/10)

#define ADCEXTVREF_YAW 		0
#define	GYROSIGN_YAW 		(1)
#define ADCVREF5V 			0

#define Version	"3.15-18F2520"

// External variables - declarations in c-ufo.c

// Assembly language variables in Bank0
extern uint8 SHADOWB, CAMTOGGLE, MF, MB, ML, MR, MT, ME;			

// Globals 									
enum 	States {Initialising, Landed, Flying, Failsafe, Autonomous};
extern uint8	State;
extern int32	ClockMilliSec, TimerMilliSec, LostTimer0Clicks;
extern int32	FailsafeTimeoutMilliSec, AutonomousTimeoutMilliSec, ThrottleClosedMilliSec;

extern int32	CycleCount;
extern int8	TimeSlot;

// RC
enum RCChannels {ThrottleC=1, RollC, PitchC, YawC, ParamC, CamTiltC, CamRollC}; 
#define FirstC ThrottleC
#define LastC CamRollC

extern uint8	Map[8];
extern const	uint8	MapCh1[8], MapCh3[8];
extern int16	PPM[8], RC[8], Desired[8];
extern int32	BadRCFrames;
extern uint8	GoodRCFrames;

// Gyros
extern int32	RollAngle, PitchAngle, YawAngle;		// PID integral (angle)
extern int16	RollGyroRate, PitchGyroRate, YawGyroRate;// PID rate (raw gyro values)
extern int16	RollRate, PitchRate, YawRate;			// PID rate (scaled gyro values)
extern int16	MidRoll, MidPitch, MidYaw;				// PID gyro neutrals

// Acceleration Corrections
extern int32	UDVelocity;
extern int16	Ax, Ay, Az;								// LISL sensor accelerations								
extern int16	NeutralLR, NeutralFB, NeutralUD;		// LISL scaled neutral values					

// PID
enum Vars {ThrottleV, RollV, PitchV, YawV, BaroAltV };
extern int32	Rl, Pl, Yl;								// PID output values
extern int16	RE, PE, YE;								// PID error
extern int16	REp, PEp, YEp;							// PID previous error
extern int16	DesiredThrottle, DesiredRoll, DesiredPitch, DesiredYaw, DesiredYawRate;
extern int16	DesiredCamRoll, DesiredCamPitch;
		
// Altitude
extern uint16	OriginBaroTemp;
extern int16	OriginBaroAltitude, DesiredBaroAltitude, CurrBaroAltitude, CurrBaroTemp;
extern 	int16	BE, BEp, BESum;	
extern int16	VBaroComp;
extern int32	Vud;

// Compass
extern int16 	Compass;								// raw compass value
extern int16	AbsDirection;							// desired heading (240 = 360 deg)
extern int16	CurrDeviation;							// deviation from correct heading

// Motors
extern uint8	MFront, MBack, MLeft, MRight, MCamPitch, MCamRoll;

// GPS
extern boolean GPSSentenceReceived; 
extern real32 GPSLatitude, GPSLongitude, GPSOriginLatitude, GPSOriginLongitude;
extern int16 GPSHeading, GPSAltitude, GPSOriginAltitude, GPSGroundSpeed;
extern real32 LongitudeCorrection;

// Serial I/O
extern uint8 RxCheckSum, RxHead, RxTail;
extern uint8 RxBuff[RXBUFFMASK+1];

// Misc
extern uint8	Flags[8];
extern uint8	Flags2[8];
										
extern uint8	LedShadow;								// shadow register
extern uint8	LedCount;
extern  int16	BatteryVolts; 

// Die Reihenfolge dieser Variablen MUSS gewahrt bleiben!!!!
// These variables MUST keep in this order!!!
extern int8	RollPropFactor; 	// 01
extern int8	RollIntFactor;		// 02
extern int8	RollDiffFactor;		// 03
extern int8	BaroTempCoeff;		// 04
extern int8	RollIntLimit;		// 05
extern int8	PitchPropFactor;	// 06
extern int8	PitchIntFactor;		// 07
extern int8	PitchDiffFactor;	// 08
extern int8	BaroThrottleProp;	// 09
extern int8	PitchIntLimit;		// 10
extern int8	YawPropFactor; 		// 11
extern int8	YawIntFactor;		// 12
extern int8	YawDiffFactor;		// 13
extern int8	YawLimit;			// 14
extern int8 	YawIntLimit;		// 15

extern int8	ConfigParam;		// 16
extern int8	NoOfTimeSlots;		// 17
extern int8	LowVoltThres;		// 18

extern int8	LinLRIntFactor;		// 19
extern int8	LinFBIntFactor;		// 20
extern int8	LinUDIntFactor;		// 21
extern int8	MiddleUD;			// 22 unused?
extern int8	MotorLowRun;		// 23
extern int8	MiddleLR;			// 24
extern int8	MiddleFB;			// 25
extern int8	CamIntFactor;		// 26
extern int8	CompassFactor;		// 27
extern int8	BaroThrottleDiff;	// 28

#define FirstProgReg RollPropFactor
#define	LastProgReg BaroThrottleDiff
// EEPROM parameter set addresses
#define _EESet1	0		// first set starts at address 0
#define _EESet2	32	// second set starts at address 32

// end of "order-block"

// Prototypes

// ADC.c
extern int16 ADC(uint8, uint8);
extern void InitADC(void);

// autonomous.c
extern void AcquireSatellites(void);
extern void DoAutonomous(void);
extern void ResetTimeOuts(void);
extern 	uint8 RCLinkRestored(int32);

// compass_altimeter.c
extern void InitDirection(void);
extern void GetDirection(void);
extern void InitBarometer(void);
extern void GetBaroAltitude(void);
extern void SetDesiredBaroAltitude(uint16);

// control.c
extern void InitAttitude(void);
extern void UpdateControls(void);
extern void DoControl(void);

// gps.c
extern 	void InitGPS(void);
extern void ParseGPSSentence(void);
extern void UpdateGPS(void);

// i2c.c
extern void I2CStart(void);
extern void I2CStop(void);
extern uint8 SendI2CByte(uint8);
extern uint8 RecvI2CByte(uint8);

// irq.c
extern void InitTimersAndInterrupts(void);
extern void MapRC(void);

// outputs.c
extern void OutSignals(void);

// serial.c
extern void ProcessCommand(void);
extern void ShowSetup(uint8);
extern uint8 RxChar(void);
extern uint8 PollRxChar(void);
extern int16 RxNumS(void);
extern uint8 RxNumU(void);
extern void TxValH16(uint16);
extern void TxValH(uint8);
extern void TxChar(uint8);
extern void TxNextLine(void);
extern void TxValU(uint8);
extern void TxValS(int8);

#ifdef READABLE
extern void TxVal(int32, uint8, uint8);
#endif

// spi.c
extern void IsLISLactive(void);
extern void ReadLISLXYZ(void);

// utils.c
extern int32 SRS32(int32, uint8);
extern int16 SRS16(int16, uint8);
extern void CheckAlarms(void);
extern void ReadParametersEE(void);
extern void WriteParametersEE(uint8);
extern 	void WriteEE(uint8, int8);
extern 	int8 ReadEE(uint8);
extern void LoadRCMap(void);
extern void SendLeds(void);
extern 	void SwitchLedsOn(uint8);
extern void SwitchLedsOff(uint8);
extern void LEDGame(void);
extern void DoDebugTraces(void);
extern void Delay100mSec(uint8);
extern void InitPorts(void);

// End of c-ufo.h

