
#define COMMISSIONING
#define READABLE

//#define RAW_BOOT
//#define BOOTONLY

#ifndef BATCHMODE
// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =             Ported 2008 to 18F2520 by Prof. Greg Egan               =
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
// Reg. 26 controls if motors should run or not
// bit 0 = front, 1 = rear, 2 = left, 3 = right
// if a bit is 1 motor is always stopped, only comport output
// if a bit is 0 motor is running as normal.
// so you can test individual axis easily!
// Reg. 26 bit 4: 1=serial data output on
// NOTE: enable DEBUG_MOTORS only on 3.1 boards (black PCB)!
//#define DEBUG_MOTORS

// special mode for sensor data output (with UAVPset)
#define DEBUG_SENSORS

// Switched Roll and Pitch channels for Conrad mc800 transmitter
//#define EXCHROLLNICK

#endif // !BATCHMODE

// Performs inial configuration of HMC6532 Compass. Can alos be 
// performed using TestSoftware 
#define CFG_COMPASS

// =====================================
// end of user-configurable section!
// =====================================

// this enables common code for all ADXRS gyros
// leave this untouched!
#if defined OPT_ADXRS300 || defined OPT_ADXRS150
#define OPT_ADXRS
#endif

#define true 1
#define false 0

#define MAXINT32 0x7fffffff;
#define	MAXINT16 0x7fff;

typedef unsigned char uint8 ;
typedef signed char int8;
typedef unsigned int uint16;
typedef int int16;
typedef long int32;
typedef uint8 boolean;

// macros - could be turned into functions if space gets tight
#define USE_MACROS
#define Set(S,b) 		((uint8)(S|=(1<<b)))
#define Clear(S,b) 		((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 		((uint8)((S>>b)&1))
#define IsClear(S,b) 	((uint8)(!(S>>b)&1))
#define Invert(S,b) 	((uint8)(S^=(1<<b)))

#define Upper8(i) 		(SRS16(i,8))
#define Lower8(i) 		(i&0xff)
#define Abs(a) 			(a<0 ? -a: a)
#define Sense(a) 		(a<0 ? -1 : (a>0 ? 1 : 0))
 	
#ifdef USE_MACROS
#define Max(i,j) 		((i<j) ? j : ((i>j) ? j : i))
#define Limit(i,l,u) 	((i<l) ? l : ((i>u) ? u : i))
#define DecayBand(i,l,u,d) 	((i<l) ? i+d : ((i>u) ? i-d : i))
#define Decay(i) 		((i<0) ? i+1 : ((i>0) ? i-1 : i))
#endif

#define DisableInterrupts (INTCONbits.GIEH=0)
#define EnableInterrupts (INTCONbits.GIEH=1)

#ifdef SUPPRESSFILTERS
  #define VerySoftFilter(O,N) 	(N)
  #define SoftFilter(O,N) 		(N)
  #define MediumFilter(O,N) 	(N)
  #define HardFilter(O,N) 		(N)
#else
  #define VerySoftFilter(O,N) 	(SRS32((int32)O+N*3+2, 2))
  #define SoftFilter(O,N) 		(SRS32((int32)O+N+1, 1))
  #define MediumFilter(O,N) 	(SRS32((int32)O*3+N+2, 2))
  #define HardFilter(O,N) 		(SRS32((int32)O*7+(int32)N+4, 3))
#endif

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

#define MAXDROPOUT	200	/* max 200x 20ms = 4sec. dropout allowable */

// Counter for flashing Low-Power LEDs
#define BLINK_LIMIT 100	// should be a number divisable by 4!

// Parameters for UART port
#define _B9600		(_ClkOut*100000/(4*9600) - 1)
#define _B19200		(_ClkOut*100000/(4*19200) - 1)
#define _B38400		(_ClkOut*100000/(4*38400) - 1)
#define _B115200	(_ClkOut*104000/(4*115200) - 1)
#define _B230400	(_ClkOut*100000/(4*115200) - 1)

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

// Gyros

// IDG300
//	+ roll right
//	+ pitch down
//	+ yaw clockwise

// ADXRS 300/150
//	+ roll left
//	+ pitch down
//	+ yaw clockwise

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

#define RollLimit			100				/* ??? nominal values */
#define	PitchLimit			100

#define ADCEXTVREF_YAW 		0
#define	GYROSIGN_YAW 		(1)
#define ADCVREF5V 			0

#define Version	"3.15-18F2520"

// External variables - declarations in c-ufo.c

// Assembly language variables in Bank0
extern	uint8 SHADOWB, CAMTOGGLE, MF, MB, ML, MR, MT, ME;			

// Globals 									
extern	int32	ClockMilliSec, TimerMilliSec;
extern	int32	RCTimeOutMilliSec, ThrottleClosedMilliSec;
extern	int32	CycleCount;
extern	int8	TimeSlot;

// RC
extern	uint8	IThrottle;
extern	int16 	IRoll,IPitch,IYaw;
extern	uint8	IK5, IK6, IK7;
extern	int16	MidRoll, MidPitch, MidYaw;				// mid RC stick values

// Gyros
extern	int32	RollAngle, PitchAngle, YawAngle;		// PID integral (angle)
extern	int16	RollGyroRate, PitchGyroRate, YawGyroRate;// PID rate (raw gyro values)
extern	int16	RollRate, PitchRate, YawRate;			// PID rate (scaled gyro values)
extern	int16 	PrevYawRate;
extern	int16	MidRoll, MidPitch, MidYaw;				// PID gyro neutrals

// Acceleration Corrections
extern	int32	UDVelocity;
extern	int16	Ax, Ay, Az;								// LISL sensor accelerations								
extern	int16	LRIntKorr, FBIntKorr;
extern	int16	NeutralLR, NeutralFB, NeutralUD;		// LISL scaled neutral values					

// PID 
extern	int32	Rl,Pl,Yl;								// PID output values						// Progressive corrections
extern	int16	RE, PE, YE;								// PID error
extern	int16	REp, PEp, YEp;							// PID previous error
extern	int16	RollFailsafe, PitchFailsafe, YawFailsafe;
			
// Altitude
extern	uint16	BasePressure, BaseTemp, TempCorr;
extern	int16	VBaroComp, BaroVal;
extern	int32	Vud;
extern  int32	BaroCompSum;

// Compass
extern	int16 	Compass;								// raw compass value
extern	int16	AbsDirection;							// desired heading (240 = 360 deg)
extern	int16	CurrDeviation;							// deviation from correct heading

// Motors
extern	uint8	MFront,MLeft,MRight,MBack;			// output channels
extern	uint8	MCamRoll,MCamPitch;

// Misc
extern	uint8	Flags[8];
extern	uint8	Flags2[8];
										
//extern	uint8	CurrThrottle;

extern	uint8	LedShadow;								// shadow register
extern	uint8	LedCount;
extern  int16	BatteryVolts; 

// Die Reihenfolge dieser Variablen MUSS gewahrt bleiben!!!!
// These variables MUST keep in this order!!!
extern	int8	RollPropFactor; 	// 01
extern	int8	RollIntFactor;		// 02
extern	int8	RollDiffFactor;		// 03
extern	int8	BaroTempCoeff;		// 04
extern	int8	RollIntLimit;		// 05
extern	int8	PitchPropFactor;	 	// 06
extern	int8	PitchIntFactor;		// 07
extern	int8	PitchDiffFactor;		// 08
extern	int8	BaroThrottleProp;	// 09
extern	int8	PitchIntLimit;		// 10
extern	int8	YawPropFactor; 	// 11
extern	int8	YawIntFactor;		// 12
extern	int8	YawDiffFactor;		// 13
extern	int8	YawLimit;			// 14
extern	int8 	YawIntLimit;		// 15

extern	int8	ConfigParam;		// 16
extern	int8	NoOfTimeSlots;		// 17
extern	int8	LowVoltThres;		// 18

extern	int8	LinLRIntFactor;		// 19
extern	int8	LinFBIntFactor;		// 20
extern	int8	LinUDIntFactor;		// 21
extern	int8	MiddleUD;			// 22 unused?
extern	int8	MotorLowRun;		// 23
extern	int8	MiddleLR;			// 24
extern	int8	MiddleFB;			// 25
extern	int8	CamIntFactor;		// 26
extern	int8	CompassFactor;		// 27
extern	int8	BaroThrottleDiff;	// 28

#define FirstProgReg RollPropFactor
#define	LastProgReg BaroThrottleDiff
// EEPROM parameter set addresses
#define _EESet1	0		// first set starts at address 0
#define _EESet2	32	// second set starts at address 32

// end of "order-block"

// Prototypes
extern	void ResetTimeOuts(void);
extern	void CheckThrottleClosed(void);

extern	void BootStart(void);
extern	void TxValH16(uint16);
extern	void TxValH(uint8);
extern	void TxChar(uint8);
extern	void TxNextLine(void);
extern	void TxValU(uint8);
extern	void TxValS(int8);
extern	uint8 RxChar(void);
extern	int16 RxNumS(void);
extern	uint8 RxNumU(void);
extern	void ShowSetup(uint8);
extern	void DoDebugTraces(void);
extern	void ProcessCommand(void);

extern	void ReadParametersEE(void);
extern	void WriteParametersEE(uint8);
extern  void WriteEE(uint8, int8);
extern  int8 ReadEE(uint8);

extern	void InitPorts(void);
extern	void InitArrays(void);
extern	void InitTimersAndInterrupts(void);

extern	void InitADC(void);
extern	void InitGyros(void);
extern	int16 ADC(uint8, uint8);
extern	int16 GetRollRate(void);
extern	int16 GetPitchRate(void);
extern	int16 GetYawRate(void);
extern	void InitAttitude(void);
extern	void DetermineAttitude(void);
extern	void CompensateGyros(void);

extern	void InitAccelerometers(void);
extern	void IsLISLactive(void);
extern	void WriteLISLByte(uint8);
extern	void WriteLISL(uint8, uint8);
extern 	uint8 ReadLISL(uint8);
extern 	uint8 ReadLISLNext(void);
extern	void ReadLISLXYZ(void);

extern	void OutSSP(uint8);
extern	void InitDirection(void);
extern	void GetDirection(void);
extern	void DoHeadingLock(void);

extern	void InitAltimeter(void);
extern	void GetAltitude(void);
extern	uint8 ReadBaro(void);
extern	uint8 StartBaroADC(uint8);

extern  void PID(void);
extern	void InitInertial(void);
extern	void DoControl(void);
extern	void MixAndLimitMotors(void);
extern	void MixAndLimitCam(void);
extern	void Delay100mSec(uint8);

extern	void GetBatteryVolts(void);
extern	void CheckAlarms(void);

extern	void SendLeds(void);
extern	void SwitchLedsOn(uint8);
extern	void SwitchLedsOff(uint8);
extern	void LEDGame(void);

extern	void I2CStart(void);
extern	void I2CStop(void);
extern	uint8 I2CWaitClkHi(void);
extern	void I2CDelay(void);
extern	uint8 SendI2CByte(uint8);
extern	uint8 RecvI2CByte(uint8);

extern	void OutSignals(void);
extern	void EscI2CStart(void);
extern	void EscI2CStop(void);
extern	void EscI2CDelay(void);
extern	void EscWaitClkHi(void);
extern	void SendEscI2CByte(uint8);

extern	int32 SRS32(int32, uint8);
extern	int16 SRS16(int16, uint8);

#ifndef USE_MACROS
extern	int16 Limit(int16, int16, int16);
extern	int16 Max(int16, int16);
extern	int16 DecayBand(int16, int16, int16, int16);
extern	int16 Decay(int16);
#endif

#ifdef READABLE
extern	void TxVal(int32, uint8, uint8);
#endif
// End of c-ufo.h

