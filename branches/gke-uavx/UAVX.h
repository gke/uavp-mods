// EXPERIMENTAL

// Simulates a FAKE GPS estimating position from control inputs and outputs to 
// Hyperterm lines of the form: 
// 294      75n     33e    -> r= 33        p= 0 hov rth
// compass   uav posn	       controls		state - in this case hovering and returning home
// This permits quadrocopter to be "flown on the bench". Motors WILL run but this is NOT flight code.
//#define FAKE_GPS

//#define DisablePpEqualsMinusPp
#define ChangeSignOfPitchOffset

// Navigation
 
#define GPS_HZ					5
// Number of good GPS sentences required to establish origin
#define INITIAL_GPS_SENTENCES 	60*GPS_HZ

// minimum no of satellites for sentence to be acceptable	
#define	MIN_SATELLITES			5		// preferably >5 for 3D fix
#define MIN_FIX					1		// must be 1 or 2 	
#define MIN_HDILUTE				150L	// HDilute * 100

// The "Ls" are important
#define MAGNETIC_VARIATION		6L		// Positive East degrees
#define COMPASS_OFFSET_DEG		90L		// North degrees CW from Front
#define MAX_ANGLE 				30L		// Rx stick units ~= degrees
#define CLOSING_RADIUS			20L		// closing radius in metres 

// Turn to heading to home and use pitch control only
#define TURN_TO_HOME
// Turn to heading is not enabled until this grounspeed is reached
#define MIN_GROUNDSPEED_TO_ARM	1		// metres per second
// Reduce value to reduce yaw slew rate for "Turn to Home"
#define	NAV_YAW_LIMIT			75L		// was 50

// GPS is active if sticks are close to Neutral
#define MAX_CONTROL_CHANGE 		10		// new hold point if the roll/pitch stick change more
// Using IK7 as GPS gain for corrective action full CCW is no correction. 
#define GPS_IK7_GAIN

// JIM increase this if you wish to a maximum of say 4
#define NavIntLimit 			1		// integral term for windy conditions! 	

// Accelerometer

// Enable vertical accelerometer compensation of vertical velocity 
//#define ENABLE_VERTICAL_VELOCITY_DAMPING

// Gyros


// Barometer

// Reduces the update rate and the additionally the descent rate 
#define ENABLE_NEW_ALT_HOLD

// Modifications which have been adopted are included BELOW.

#ifndef BATCHMODE
// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
// =======================================================================

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

// ==============================================
// == Global compiler switches
// ==============================================

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
// #define ESC_HOLGER
// to use YGE I2C controllers (for standard YGEs use ESC_PPM)
//#define ESC_YGEI2C

// defined: serial PPM pulse train from receiver
//#define RX_PPM

// defined: Spektrum DSM2 various channel shuffles
//#define RX_DSM2

// defined: Interleaved odd channel PPM pulse train from receiver
#define RX_DEFAULT

// uncomment this to enable Tri-Copter Mixing.
// connector K1 = front motor
//           K2 = rear left motor 
//           K3 = rear right motor 
//           K4 = yaw servo output
// Camera controlling can be used!
//#define TRICOPTER

// special mode for sensor data output (with UAVPset)
//#define DEBUG_SENSORS

#endif // !BATCHMODE

// UAVX Extensions

// Baro

// Attempts reconnection to the barometer if there is an I2C error
// I2C comms seems now to be reliable with the BMP085 baro.
//#define BARO_RETRY

// Increase the severity of the filter on the barometer pressure readings
// may give better altitude hold ( New=(Old*7+New+4)/8) ).
#define BARO_HARD_FILTER

// Make a "scratchy" beeper noise while altitude hold is engaged.
// A better method is to use the auxilliary LEDs - see Manual.
#define BARO_SCRATCHY_BEEPER

// Accelerometers

// Enable this to use the Accelerometer sensor 
// Normally ENABLED
#define USE_ACCELEROMETER

// Gyros

// Enable "Dynamic mass" compensation Roll and/or Pitch
// Normally disabled for pitch only 
//#define DISABLE_DYNAMIC_MASS_COMP_ROLL
//#define DISABLE_DYNAMIC_MASS_COMP_PITCH

// Navigation

// reads $GPGGA and $GPRMC sentences - all others discarded
	
// Misc

// Loads a "representative" parameter set into EEPROM
//#define INIT_PARAMS

// =====================================
// end of user-configurable section!
// =====================================

// Bit Definitions

// Bit definitions

#define Armed		(PORTAbits.RA4)

#define	I2C_ACK		((uint8)(0))
#define	I2C_NACK	((uint8)(1))

#define SPI_CS		PORTCbits.RC5
#define SPI_SDA		PORTCbits.RC4
#define SPI_SCL		PORTCbits.RC3
#define SPI_IO		TRISCbits.TRISC4

#define	RD_SPI	1
#define WR_SPI	0
#define DSEL_LISL  1
#define SEL_LISL  0

// The LEDs and the beeper
#define ON	1
#define OFF	0

#define MODELLOSTTIMER		20 	/*in 0,2sec until first beep 
								after lost xmit signal */
#define MODELLOSTTIMERINT	2 	/* in 0,2 sec units
								interval beep when active */


// LEDs

#define LEDYellow	LED6
#define LEDGreen	LED4
#define	LEDBlue		LED2
#define LEDRed		LED3
#define LEDAUX1		LED5
#define LEDAUX2		LED1
#define LEDAUX3		LED7

#define LED1	0x01	/* Aux2 */
#define LED2	0x02	/* blue */
#define LED3	0x04	/* red */ 
#define LED4	0x08	/* green */
#define LED5	0x10	/* Aux1 */
#define LED6	0x20	/* yellow */
#define LED7	0x40	/* Aux3 */
#define Beeper	0x80

#define ALL_LEDS_ON		SwitchLEDsOn(LEDBlue|LEDRed|LEDGreen|LEDYellow)
#define AUX_LEDS_ON		SwitchLEDsOn(LEDAUX1|LEDAUX2|LEDAUX3)

#define ALL_LEDS_OFF	SwitchLEDsOff(LEDBlue|LEDRed|LEDGreen|LEDYellow)
#define AUX_LEDS_OFF	SwitchLEDsOff(LEDAUX1|LEDAUX2|LEDAUX3)

#define ARE_ALL_LEDS_OFF if((LEDShadow&(LEDBlue|LEDRed|LEDGreen|LEDYellow))==0)

#define LEDRed_ON		SwitchLEDsOn(LEDRed)
#define LEDBlue_ON		SwitchLEDsOn(LEDBlue)
#define LEDGreen_ON		SwitchLEDsOn(LEDGreen)
#define LEDYellow_ON	SwitchLEDsOn(LEDYellow)
#define LEDAUX1_ON		SwitchLEDsOn(LEDAUX1)
#define LEDAUX2_ON		SwitchLEDsOn(LEDAUX2)
#define LEDAUX3_ON		SwitchLEDsOn(LEDAUX3)
#define LEDRed_OFF		SwitchLEDsOff(LEDRed)
#define LEDBlue_OFF		SwitchLEDsOff(LEDBlue)
#define LEDGreen_OFF	SwitchLEDsOff(LEDGreen)
#define LEDYellow_OFF	SwitchLEDsOff(LEDYellow)
#define LEDRed_TOG		if( (LEDShadow&LEDRed) == 0 ) SwitchLEDsOn(LEDRed); else SwitchLEDsOff(LEDRed)
#define LEDBlue_TOG		if( (LEDShadow&LEDBlue) == 0 ) SwitchLEDsOn(LEDBlue); else SwitchLEDsOff(LEDBlue)
#define LEDGreen_TOG	if( (LEDShadow&LEDGreen) == 0 ) SwitchLEDsOn(LEDGreen); else SwitchLEDsOff(LEDGreen)
#define Beeper_OFF		SwitchLEDsOff(Beeper)
#define Beeper_ON		SwitchLEDsOn(Beeper)
#define Beeper_TOG		if( (LEDShadow&Beeper) == 0 ) SwitchLEDsOn(Beeper); else SwitchLEDsOff(Beeper);

// compass sensor
#define COMPASS_I2C_ID	0x42	/* I2C slave address */
#define COMPASS_MAXDEV	30		/* maximum yaw compensation of compass heading */
#define COMPASS_MAX		240		/* means 360 degrees */
#define COMPASS_INVAL	(COMPASS_MAX+15)	/* 15*4 cycles to settle */
#define COMPASS_MIDDLE	10		/* yaw stick neutral dead zone */

#define COMPASS_TIME	50	/* 20Hz */

// baro (altimeter) sensor
#define BARO_I2C_ID			0xee
#define BARO_TEMP_BMP085	0x2e
#define BARO_TEMP_SMD500	0x6e
#define BARO_PRESS			0xf4
#define BARO_CTL			0xf4
#define BARO_ADC_MSB		0xf6
#define BARO_ADC_LSB		0xf7
#define BARO_TYPE			0xd0
//#define BARO_ID_SMD500		??
#define BARO_ID_BMP085		((uint8)(0x55))

#define BARO_TEMP_TIME	10
#define BARO_PRESS_TIME 35

#define THR_DOWNCOUNT	255		/* 128 PID-cycles (=3 sec) until current throttle is fixed */
#define THR_MIDDLE		10  	/* throttle stick dead zone for baro */
#define THR_HOVER		75		/* min throttle stick for alti lock */

// Status 

#define	_Signal				Flags[0]	/* if no valid signal is received */
#define	_Flying				Flags[1]	/* UFO is flying */
#define	_NewValues			Flags[2]	/* new RX channel values sampled */
#define _FirstTimeout		Flags[3]	/* is 1 after first 9ms TO expired */

#define _LowBatt			Flags[4]	/* if Batt voltage is low */
#define _AccelerationsValid Flags[5]	/* 1 if LISL Sensor is used */
#define	_CompassValid		Flags[6]	/* 1 if compass sensor is enabled */
#define _CompassMissRead 	Flags[7]
#define _BaroAltitudeValid	Flags[8]	/* 1 if baro sensor active */
#define _BaroTempRun		Flags[9]	/* 1 if baro temp a/d conversion is running */
#define _BaroRestart		Flags[10] 	/* Baro restart required */
#define _OutToggle			Flags[11]	/* cam servos only evers 2nd output pulse */								
#define _UseCh7Trigger		Flags[12]	/* 1: don't use Ch7 */
									/* 0: use Ch7 as Cam Roll trim */
#define _ReceivingGPS 		Flags[16]
#define _GPSValid 			Flags[17]
#define _LostModel			Flags[18]
#define _Hovering			Flags[19]
#define _NavComputed 		Flags[20]
#define _GPSHeadingValid 	Flags[21]
#define _GPSAltitudeValid	Flags[22]

#define _GPSTestActive		Flags[31]

// Mask Bits of ConfigParam
#define FlyCrossMode 	IsSet(ConfigParam,0)
#define FutabaMode		IsSet(ConfigParam,1)
#define IntegralTest	IsSet(ConfigParam,2)
#define DoubleRate		IsSet(ConfigParam,3)
#define NegativePPM		IsSet(ConfigParam,4)
#define CompassTest		IsSet(ConfigParam,5)

// this enables common code for all ADXRS gyros
// leave this untouched!
#if defined OPT_ADXRS300 || defined OPT_ADXRS150
#define OPT_ADXRS
#endif

#define Version	"1.0m3"

// 18Fxxx C18 includes

#include <p18cxxx.h> 
#include <math.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>
#include <capture.h>
#include <adc.h>

// Additional Types

typedef unsigned char uint8 ;
typedef signed char int8;
typedef unsigned int uint16;
typedef int int16;
typedef short long int24;
typedef unsigned short long uint24;
typedef long int32;
typedef unsigned long uint32;
typedef uint8 boolean;
typedef float real32;

typedef union {
	int16 i16;
	uint16 u16;
	struct {
		unsigned char low8;
		unsigned char high8;
	};
} i16u;

// Useful Constants

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
#define true 1
#define false 0

#define EarthR (real32)(6378140.0)
#define Pi (real32)(3.141592654)
#define RecipEarthR (real32)(1.5678552E-7)

#define DEGTOM (real32)(111319.5431531) 
#define MILLIPI 3142 
#define CENTIPI 314 
#define HALFMILLIPI 1571 
#define TWOMILLIPI 6284

#define MILLIRAD 18 
#define CENTIRAD 2
#define DEGMILLIRAD (real32)(17.453292) 
#define DECIDEGMILLIRAD (real32)(1.7453292)
#define MILLIRADDEG (real32)(0.05729578)
#define MILLIRADDEG100 (real32)(5.729578)
#define CENTIRADDEG (real32)(0.5729578)
#define DEGRAD (real32)(0.017453292)
#define RADDEG (real32)(57.2957812)
#define RADDEG1000000 (real32)(57295781.2)

#define MAXINT32 0x7fffffff;
#define	MAXINT16 0x7fff;

// Macros

#define Set(S,b) 		((uint8)(S|=(1<<b)))
#define Clear(S,b) 		((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 		((uint8)((S>>b)&1))
#define IsClear(S,b) 	((uint8)(!(S>>b)&1))
#define Invert(S,b) 	((uint8)(S^=(1<<b)))

#define Abs(i)			((i<0) ? -i : i)
#define Sign(i)			((i<0) ? -1 : 1)

#define Max(i,j) 		((i<j) ? j : i)
#define Min(i,j) 		((i<j) ? i : j )
#define Limit(i,l,u) 	((i<l) ? l : ((i>u) ? u : i))
#define DecayBand(i,l,u,d) 	((i<l) ? i+d : ((i>u) ? i-d : i))
#define Decay(i) 		((i<0) ? i+1 : ((i>0) ? i-1 : 0))

// To speed up NMEA sentence porocessing 
// must have a positive argument
#define ConvertDDegToMPi(d) (((int32)d * 3574L)>>11)
#define ConvertMPiToDDeg(d) (((int32)d * 2048L)/3574L)

// Simple filters using weighted averaging
#ifdef SUPPRESSFILTERS
  #define VerySoftFilter(O,N)		(N)
  #define SoftFilter(O,N) 			(N)
  #define MediumFilter(O,N) 			(N)
  #define AccelerometerFilter(O,N) 	(N)
#else
  #define VerySoftFilter(O,N) 		(SRS16(O+N*3, 2))
  #define SoftFilter(O,N) 			(SRS16(O+N, 1))
  #define MediumFilter(O,N) 		(SRS16(O*3+N, 2))
  #define HardFilter(O,N) 			(SRS16(O*7+N, 3))
#endif
#define NoFilter(O,N)				(N)

#define DisableInterrupts (INTCONbits.GIEH=0)
#define EnableInterrupts (INTCONbits.GIEH=1)
#define InterruptsEnabled (INTCONbits.GIEH)

// Constants 

// ADC Channels
#define ADCPORTCONFIG 0b00001010 // AAAAA
#define ADCBattVoltsChan 	0 

#ifdef OPT_ADXRS
	#define ADCRollChan 	1
	#define ADCPitchChan 	2
#else // OPT_IDG
	#define ADCRollChan 	2
	#define ADCPitchChan 	1
#endif // OPT_ADXRS

#define ADCVRefChan 		3 
#define ADCYawChan			4

#define ADCVREF5V 			0
#define ADCVREF 			1

//#ifdef CLOCK_16MHZ
#define _ClkOut		(160/4)	/* 16.0 MHz Xtal */
//#else // CLOCK_40MHZ
//NOT IMPLEMENTED YET #define _ClkOut		(400/4)	/* 10.0 MHz Xtal * 4 PLL */
//#endif

#define _PreScale0	16	/* 1:16 TMR0 prescaler */
#define _PreScale1	8	/* 1:8 TMR1 prescaler */
#define _PreScale2	16
#define _PostScale2	16

#define TMR2_5MS	78	/* 1x 5ms +  */
#define TMR2_14MS	234	/* 1x 15ms = 20ms pause time */

#ifdef ESC_PPM
#define	_Minimum	1 /*((105* _ClkOut/(2*_PreScale1))&0xFF)*/
#define _Maximum	240					/* reduced from 255 */
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

#define _Neutral	((150* _ClkOut/(2*_PreScale1))&0xFF)    /*   0% */
#define _ThresStop	((113* _ClkOut/(2*_PreScale1))&0xFF)	/*-90% ab hier Stopp! */
#define _ThresStart	((116* _ClkOut/(2*_PreScale1))&0xFF)	/*-85% ab hier Start! */

#define MAXDROPOUT	400L	// 400 x 16 x 7ms = 40sec. dropout allowable
#define GPSDROPOUT	20L		// 2sec.

// Parameters for UART port
// ClockHz/(16*(BaudRate+1))

#define _B9600		104 
#define _B19200		(_ClkOut*100000/(4*19200) - 1)
#define _B38400		26 
#define _B115200	9 
#define _B230400	(_ClkOut*100000/(4*115200) - 1)

// EEPROM parameter set addresses

#define _EESet1	0		// first set starts at address 0x00
#define _EESet2	0x20	// second set starts at address 0x20

// Sanity checks

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

// end of sanity checks

// Prototypes

// accel.c
extern void SendCommand(int8);
extern uint8 ReadLISL(uint8);
extern uint8 ReadLISLNext(void);
extern void WriteLISL(uint8, uint8);
extern void IsLISLActive(void);
extern void InitLISL(void);
extern void ReadAccelerations(void);
extern void GetNeutralAccelerations(void);

// adc.c
extern int16 ADC(uint8, uint8);
extern void InitADC(void);

// autonomous.c
extern void Descend(void);
extern void Navigate(int16, int16);
extern void CheckAutonomous(void);

// compass_altimeter.c
extern void InitDirection(void);
extern void GetDirection(void);
extern uint8 ReadValueFromBaro(void);
extern uint8 StartBaroADC(uint8);
extern void InitBarometer(void);
extern void ComputeBaroComp(void);

// control.c
extern void GyroCompensation(void);
extern void LimitRollSum(void);
extern void LimitPitchSum(void);
extern void LimitYawSum(void);
extern void GetGyroValues(void);
extern void CalcGyroValues(void);
extern void PID(void);

extern void WaitThrottleClosed(void);
extern void CheckThrottleMoved(void);
extern void WaitForRxSignal(void);

// irq.c
extern void InitTimersAndInterrupts(void);
extern void ReceivingGPSOnly(uint8);

// gps.c
extern void UpdateField(void);
extern int16 ConvertInt(uint8, uint8);
extern int32 ConvertLatLonM(uint8, uint8);
extern int32 ConvertUTime(uint8, uint8);
extern void ParseGPRMCSentence(void);
extern void ParseGPGGASentence(void);
extern void ParseGPSSentence(void);
extern void PollGPS(uint8);
extern void InitGPS(void);
extern void UpdateGPS(void);

// i2c.c
void I2CDelay(void);
extern void I2CStart(void);
extern void I2CStop(void);
extern uint8 SendI2CByte(uint8);
extern uint8 RecvI2CByte(uint8);

extern void EscI2CDelay(void);
extern void EscWaitClkHi(void);
extern void EscI2CStart(void);
extern void EscI2CStop(void);
extern void SendEscI2CByte(uint8);

// menu.c
extern void ShowPrompt(void);
extern void ShowSetup(uint8);
extern void ProcessComCommand(void);

// outputs.c
extern uint8 SaturInt(int16);
extern void DoMix(int16 CurrThrottle);
extern void CheckDemand(int16 CurrThrottle);
extern void MixAndLimitMotors(void);
extern void MixAndLimitCam(void);
extern void OutSignals(void);

// serial.c
extern void TxString(const rom uint8 *);
extern void TxChar(uint8);
extern void TxValU(uint8);
extern void TxValS(int8);
extern void TxNextLine(void);
extern void TxNibble(uint8);
extern void TxValH(uint8);
extern void TxValH16(uint16);
extern uint8 PollRxChar(void);
extern uint8 RxNumU(void);
extern int8 RxNumS(void);
extern void TxVal32(int32, int8, uint8);

// utils.c
extern void Delay1mS(int16);
extern void Delay100mSWithOutput(int16);
extern int16 SRS16(int16, uint8);
extern int32 SRS32(int32, uint8);
extern void InitPorts(void);
extern void InitArrays(void);

extern int16 ConvertGPSToM(int16);
extern int16 ConvertMToGPS(int16);

extern 	int8 ReadEE(uint8);
extern void ReadParametersEE(void);
extern 	void WriteEE(uint8, int8);
extern void WriteParametersEE(uint8);
extern void InitParams(void);

extern int16 Make2Pi(int16);
extern int16 MakePi(int16);
extern int16 Table16(int16, const int16 *);
extern int16 int16sin(int16);
extern int16 int16cos(int16);
extern int16 int16atan2(int16, int16);
extern int16 int16sqrt(int16);

extern void SendLEDs(void);
extern void SwitchLEDsOn(uint8);
extern void SwitchLEDsOff(uint8);
extern void LEDGame(void);
void DoPIDDisplays(void);
extern void CheckAlarms(void);

extern void DumpTrace(void);

// bootl18f.asm
extern void BootStart(void);

// tests.c
extern void LinearTest(void);
extern uint8 ScanI2CBus(void);
extern void ReceiverTest(void);
extern void DoCompassTest(void);
extern void CalibrateCompass(void);
extern void BaroTest(void);
extern void PowerOutput(int8);
extern void GPSTest(void);

extern void AnalogTest(void);extern void Program_SLA(uint8);

extern void DoLEDs(void);

extern int16	TestTimeSlot;
extern uint16	PauseTime;
extern int16 	NewK1, NewK2, NewK3, NewK4, NewK5, NewK6, NewK7;

// Menu strings
extern const rom uint8  SerHello[];
extern const rom uint8  SerSetup[];
extern const rom uint8 SerPrompt[];

// External Variables

enum TraceTags {TAbsDirection,TVBaroComp,TBaroRelPressure,				TRollRate,TPitchRate,TYE,				TRollSum,TPitchSum,TYawSum,
				TAx,TAz,TAy,
				TUDSum, TVud,
				TIGas,
				TIRoll, TIPitch, TIYaw,
				TMFront, TMBack, TMLeft, TMRight,
				TMCamRoll, TMCamPitch,
				TLRAcc, TLRGrav, TLRDyn, TLRIntKorr, TFBAcc, TFBGrav, TFBDyn, TFBIntKorr,
				LastTrace
				};
#define TopTrace TFBIntKorr

enum MotorTags {Front, Left, Right, Back};
#define NoOfMotors 4

extern uint8	IGas;
extern int8 	IRoll,IPitch,IYaw;
extern uint8	IK5,IK6,IK7;

extern int16	RE, PE, YE;
extern int16	REp,PEp,YEp;
extern int16	PitchSum, RollSum, YawSum;
extern int16	RollRate, PitchRate, YawRate;
extern int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
extern int16	DesiredThrottle, DesiredRoll, DesiredPitch, DesiredYaw, Heading;
extern i16u		Ax, Ay, Az;
extern int8		LRIntKorr, FBIntKorr;
extern int8		NeutralLR, NeutralFB, NeutralUD;
extern int16 	UDSum;

// GPS
extern uint8 GPSMode;
extern int16 GPSGroundSpeed, GPSHeading, GPSLongitudeCorrection;
extern uint8 GPSNoOfSats;
extern uint8 GPSFix;
extern int16 GPSHDilute;
extern int16 GPSNorth, GPSEast, GPSNorthHold, GPSEastHold;
extern int16 GPSRelAltitude;

// Failsafes
extern uint8	ThrNeutral;
			
// Variables for barometric sensor PD-controller
extern int24	BaroBasePressure, BaroBaseTemp;
extern int16	BaroRelPressure, BaroRelTempCorr;
extern i16u		BaroVal;
extern int16	VBaroComp;
extern uint8	BaroType, BaroTemp, BaroRestarts;

extern uint8	MCamRoll,MCamPitch;
extern int16	Motor[NoOfMotors];
extern int16	Rl,Pl,Yl;	// PID output values
extern int16	Vud;

extern uint8	Flags[32];

extern int16	IntegralCount, ThrDownCount, GPSCount, DropoutCount, LEDCount, BaroCount;
extern int16	FakeGPSCount;
extern uint32	BlinkCount;
extern uint24	RCGlitchCount;
extern int8		BatteryVolts; 

extern uint8	LEDShadow;	// shadow register
extern int16	AbsDirection;	// wanted heading (240 = 360 deg)
extern int16	CurDeviation;	// deviation from correct heading

extern uint8 	RxCheckSum;

#ifdef DEBUG_SENSORS
extern int16	Trace[LastTrace];
#endif // DEBUG_SENSORS

// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
extern int8	RollPropFactor; 	// 01
extern int8	RollIntFactor;		// 02
extern int8	RollDiffFactor;		// 03
extern int8 BaroTempCoeff;		// 04
extern int8	RollIntLimit;		// 05
extern int8	PitchPropFactor;	// 06
extern int8	PitchIntFactor;		// 07
extern int8	PitchDiffFactor;	// 08
extern int8 BaroThrottleProp;	// 09
extern int8	PitchIntLimit;		// 10
extern int8	YawPropFactor; 		// 11
extern int8	YawIntFactor;		// 12
extern int8	YawDiffFactor;		// 13
extern int8	YawLimit;			// 14
extern int8 YawIntLimit;		// 15
extern int8	ConfigParam;		// 16
extern int8 TimeSlot;			// 17
extern int8	LowVoltThres;		// 18
extern int8	CamRollFactor;		// 19
extern int8	LinFBIntFactor;		// 20 free
extern int8	LinUDIntFactor;		// 21
extern int8 MiddleUD;			// 22
extern int8	MotorLowRun;		// 23
extern int8	MiddleLR;			// 24
extern int8	MiddleFB;			// 25
extern int8	CamPitchFactor;		// 26
extern int8	CompassFactor;		// 27
extern int8	BaroThrottleDiff;	// 28

#define FirstProgReg RollPropFactor
#define	LastProgReg BaroThrottleDiff

// End of c-ufo.h

