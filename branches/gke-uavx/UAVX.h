// EXPERIMENTAL

// Navigation
// tune this up and down to see if it has any effect on compass fail

#define EMIT_TONE

#define CAPTURE_TRIMS
// Accelerometer

// Gyros

//#define YAW_RESET

// Outstanding issue with gyro compensation offset sense yet to be resolved
#define REVERSE_OFFSET_SIGNS

// Barometer

// Modifications which have been adopted are included BELOW.

#ifndef BATCHMODE
// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

// ==============================================
// == Global compiler switches
// ==============================================

// uncomment this to enable Tri-Copter Mixing.
// connector K1 = front motor
//           K2 = rear left motor 
//           K3 = rear right motor 
//           K4 = yaw servo output
// Camera controlling can be used!
//#define TRICOPTER

// uncomment for 6 channel recievers
//#define RX6CH 

// special mode for sensor data output (with UAVPset)
//#define DEBUG_SENSORS

#endif // !BATCHMODE

// UAVX Extensions

// Baro

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
// Enable vertical accelerometer compensation of vertical velocity 
#define ENABLE_VERTICAL_VELOCITY_DAMPING

// Gyros

// Enable "Dynamic mass" compensation Roll and/or Pitch
// Normally disabled for pitch only 
//#define DISABLE_DYNAMIC_MASS_COMP_ROLL
//#define DISABLE_DYNAMIC_MASS_COMP_PITCH

// Navigation

#define	NAV_GAIN_THRESHOLD 		20		// Navigation disabled if Ch7 is less than this
// reads $GPGGA and $GPRMC sentences - all others discarded
	
#define	MIN_SATELLITES			5		// preferably >5 for 3D fix
#define MIN_FIX					1		// must be 1 or 2 
#define INITIAL_GPS_SENTENCES 	90		// Number of sentences needed with set HDilute
#define MIN_HDILUTE				130L	// HDilute * 100	
#define COMPASS_OFFSET_DEG		270L	// North degrees CW from Front
#define MAX_ANGLE 				15L		// Rx stick units ~= degrees

#define MAX_CONTROL_CHANGE 		3L		// new hold point if the roll/pitch stick change more
#define	NAV_YAW_LIMIT			10L		// yaw slew rate for RTH

// Misc

//#define SUPPRESS_COMPASS_SR				// turns off periodic compass set/reset 
	
// =====================================
// end of user-configurable section!
// =====================================

#include "UAVXRevision.h"

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
#define HT 9
#define LF 10
#define CR 13
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

// To speed up NMEA sentence processing 
// must have a positive argument
#define ConvertDDegToMPi(d) (((int32)d * 3574L)>>11)
#define ConvertMPiToDDeg(d) (((int32)d * 2048L)/3574L)

// Simple filters using weighted averaging
#ifdef SUPPRESSFILTERS
  #define VerySoftFilter(O,N)		(N)
  #define SoftFilter(O,N) 			(N)
  #define MediumFilter(O,N) 		(N)
  #define AccelerometerFilter(O,N) 	(N)
#else
  #define VerySoftFilter(O,N) 		(SRS16((O)+(N)*3, 2))
  #define SoftFilter(O,N) 			(SRS16((O)+(N), 1))
  #define MediumFilter(O,N) 		(SRS16((O)*3+(N), 2))
  #define HardFilter(O,N) 			(SRS16((O)*7+(N), 3))
#endif
#define NoFilter(O,N)				(N)

#define DisableInterrupts (INTCONbits.GIEH=0)
#define EnableInterrupts (INTCONbits.GIEH=1)
#define InterruptsEnabled (INTCONbits.GIEH)

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

#define FAILSAFE_TIMEOUT	1000L 		// mS hold last settings
#define ABORT_TIMEOUT		3000L 		// mS full flight abort 
#define LOW_THROTTLE_DELAY	1000L		// mS
#define THROTTLE_UPDATE		3000L		// mS constant throttle time for hover
#define VERT_DAMPING_UPDATE 50L 		// mS vertical velocity damping

#define THR_MIDDLE			10  		// throttle stick dead zone for baro 
#define THR_HOVER			75			// min throttle stick for alti lock

#define GPS_TIMEOUT			2000L		// mS

// RC

#define RC_MINIMUM			0
#define RC_MAXIMUM			238
#define RC_NEUTRAL			((RC_MAXIMUM-RC_MINIMUM+1)/2)

#define RC_THRES_STOP		((15L*RC_MAXIMUM)/100)		
#define RC_THRES_START		((25L*RC_MAXIMUM)/100)		

#define RC_FRAME_TIMEOUT 	25
#define RC_SIGNAL_TIMEOUT 	(5L*RC_FRAME_TIMEOUT)
#define RC_THR_MAX 			RC_MAXIMUM

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

#define BARO_TEMP_TIME		10
#define BARO_PRESS_TIME 	35
#define BARO_SAMPLES		16	

// Status 

#define	_Signal				Flags[0]	/* if no valid signal is received */
#define _NegativePPM		Flags[1]	/* Futaba */
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
#define _Failsafe			Flags[12]
#define _GyrosErected		Flags[13]
#define _NewBaroValue		Flags[14]
#define _TrimsCaptured		Flags[15]

#define _ReceivingGPS 		Flags[16]
#define _GPSValid 			Flags[17]
#define _LostModel			Flags[18]
#define _ThrottleMoving		Flags[19]
#define _Hovering			Flags[20]
#define _NavComputed 		Flags[21]
#define _GPSHeadingValid 	Flags[22]
#define _GPSAltitudeValid	Flags[23]
#define _RTHAltitudeHold	Flags[24]
#define _ReturnHome			Flags[25]
#define _TurnToHome			Flags[26]
#define _Proximity			Flags[27]

#define _ParametersValid	Flags[30]
#define _GPSTestActive		Flags[31]

// Mask Bits of ConfigParam
#define FlyCrossMode 	IsSet(ConfigParam,0)
//#define FutabaMode		IsSet(ConfigParam,1)
#define TxMode2			IsSet(ConfigParam,2)
#define RxPPM			IsSet(ConfigParam,3)
//#define NegativePPM		IsSet(ConfigParam,4)
#define UseGPSAlt		IsSet(ConfigParam,5)

// Constants 


// ADC Channels
#define ADCPORTCONFIG 0b00001010 // AAAAA
#define ADCBattVoltsChan 	0 
#define NonIDGADCRollChan 	1
#define NonIDGADCPitchChan 	2
#define IDGADCRollChan 		2
#define IDGADCPitchChan 	1
#define ADCVRefChan 		3 
#define ADCYawChan			4
#define ADCVREF5V 			0
#define ADCVREF3V3 			1


// ------------------------------------------------------------------------------------

// Accelerometers
// LIS3LV02DQ Inertial Sensor (Accelerometer)
//  Ax=> Y + right
//	Ay=> Z + up
//	Az=> X + back

#define ACCSIGN_X	+	
#define ACCSIGN_Y	-
#define ACCSIGN_Z	+	

// Gyros

// Coordinates are NOT right hand cartesian for LEGACY reasons
// and the desire to keep the current parameters sets!!

//	+P roll right
//	+Q pitch up
//	+R yaw clockwise

// Controls 
//  Aileron + roll right
//	Elevator + pitch up
//	Rudder + yaw anti-clockwise


// IDG300
// 3.3V Reference +-500 Deg/Sec
// 0.4882815 Deg/Sec/LSB 
//	+ roll right
//	+ pitch up
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

//#ifdef CLOCK_16MHZ
#define _ClkOut		(160/4)	/* 16.0 MHz Xtal */
//#else // CLOCK_40MHZ
//NOT IMPLEMENTED YET #define _ClkOut		(400L/4)	/* 10.0 MHz Xtal * 4 PLL */
//#endif

#define _PreScale0	16	/* 1:16 TMR0 prescaler */
#define _PreScale1	8	/* 1:8 TMR1 prescaler */
#define _PreScale2	16
#define _PostScale2	16

#define TMR2_5MS	78	/* 1x 5ms +  */
#define TMR2_TICK	2	// uSec

#define OUT_MINIMUM	1
#define OUT_MAXIMUM	240
#define _HolgerMaximum	225 

#define OUT_NEUTRAL	((150* _ClkOut/(2*_PreScale1))&0xFF)    /*   0% */

#define ToPercent(n, m) (((n)*100)/m)
// Parameters for UART port
// ClockHz/(16*(BaudRate+1))

#define _B9600		104 
#define _B19200		(_ClkOut*100000/(4*19200) - 1)
#define _B38400		26 
#define _B115200	9 
#define _B230400	(_ClkOut*100000/(4*115200) - 1)

// Sanity checks

// check the PPM RX and motor values
#if OUT_MINIMUM >= OUT_MAXIMUM
#error OUT_MINIMUM < OUT_MAXIMUM!
#endif
#if (OUT_MAXIMUM < OUT_NEUTRAL)
#error OUT_MAXIMUM < OUT_NEUTRAL !
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
extern void DoNavigation(void);
extern void CheckThrottleMoved(void);
extern void DoFailsafe(void);

// compass_altimeter.c
extern void InitDirection(void);
extern void GetDirection(void);
extern void StartBaroADC(void);
extern void ReadBaro(void);
extern void GetBaroPressure(void);
extern void InitBarometer(void);
extern void CheckForHover(void);
extern void BaroAltitudeHold(int16);

// control.c
extern void GyroCompensation(void);
extern void LimitRollSum(void);
extern void LimitPitchSum(void);
extern void LimitYawSum(void);
extern void GetGyroValues(void);
extern void ErectGyros(void);
extern void CalcGyroValues(void);
extern void DoControl(void);

extern void WaitThrottleClosed(void);
extern void WaitForRxSignal(void);
extern void UpdateControls(void);
extern void CaptureTrims(void);

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
extern void ProcessCommand(void);

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
extern void WriteEE(uint8, int8);
extern void WriteParametersEE(uint8);
extern void UpdateParamSetChoice(void);

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

// Menu strings
extern const rom uint8  SerHello[];
extern const rom uint8  SerSetup[];
extern const rom uint8 SerPrompt[];

// External Variables

enum {Clock,  UpdateTimeout, RCSignalTimeout, AlarmUpdate, ThrottleIdleTimeout, FailsafeTimeout, 
      AbortTimeout, GPSTimeout, ThrottleUpdate, VerticalDampingUpdate, BaroUpdate, CompassUpdate};
	
enum RCControls {ThrottleC, RollC, PitchC, YawC, RTHC, CamTiltC, NavGainC}; 
#define CONTROLS  (NavGainC+1)

enum FlightStates { Starting, Landing, Landed, InFlight};

enum ESCTypes { ESCPPM, ESCHolger, ESCX3D, ESCYGEI2C };
enum GyroTypes { ADXRS300, ADXRS150, IDG300};
enum TxRxTypes { FutabaCh3, FutabaCh2, FutabaDM8, JRPPM, JRDM9, JRDXS12, DX7AR7000, DX7AR6200, CustomTxRx };

enum TraceTags {TAbsDirection,TVBaroComp,TBE,
				TRollRate,TPitchRate,TYE,
				TRollSum,TPitchSum,TYawSum,
				TAx,TAz,TAy,
				TUDSum, TVUDComp,
				TIThrottle,
				TIRoll, TIPitch, TIYaw,
				TMFront, TMBack, TMLeft, TMRight,
				TMCamRoll, TMCamPitch,
				TLRAcc, TLRGrav, TLRDyn, TLRIntKorr, TFBAcc, TFBGrav, TFBDyn, TFBIntKorr,
				LastTrace
				};
#define TopTrace TFBIntKorr

enum MotorTags {Front, Left, Right, Back};
#define NoOfMotors 4

extern uint24	mS[CompassUpdate+1];

extern uint8	CurrentParamSet;

extern i16u		PPM[CONTROLS];
extern int16	RC[CONTROLS];
extern boolean	RCFrameOK;
extern int8		PPM_Index;
extern int24	PrevEdge;
extern int16	PauseTime; // for tests

extern int16	RE, PE, YE;
extern int16	REp,PEp,YEp;
extern int16	PitchSum, RollSum, YawSum;
extern int16	RollRate, PitchRate, YawRate;
extern int16	RollTrim, PitchTrim;
extern int16	RollIntLimit256, PitchIntLimit256, YawIntLimit256, NavIntLimit256;
extern int16	GyroMidRoll, GyroMidPitch, GyroMidYaw;
extern int16	HoverThrottle, DesiredThrottle, IdleThrottle;
extern int16	DesiredRoll, DesiredPitch, DesiredYaw, Heading;
extern i16u		Ax, Ay, Az;
extern int8		LRIntKorr, FBIntKorr;
extern int8		NeutralLR, NeutralFB, NeutralUD;
extern int16 	UDAcc, UDSum, VUDComp;

// GPS
extern uint8 	GPSMode;
extern int16 	GPSGroundSpeed, GPSHeading, GPSLongitudeCorrection;
extern uint8 	GPSNoOfSats;
extern uint8 	GPSFix;
extern int16 	GPSHDilute;
extern int16 	GPSNorth, GPSEast, GPSNorthHold, GPSEastHold;
extern int16 	GPSRelAltitude;

extern int16 	SqrNavClosingRadius, NavClosingRadius, CompassOffset;

enum NavStates { PIC, HoldingStation, ReturningHome, Navigating, Terminating };
extern uint8 	NavState;
extern uint8 	NavSensitivity;
extern int16 	AltSum, AE;

// Failsafes
extern int16	ThrLow, ThrHigh, ThrNeutral;
			
// Variables for barometric sensor PD-controller
extern int24	OriginBaroPressure;
extern int16	DesiredBaroPressure, CurrentBaroPressure;
extern int16	BE, BEp;
extern i16u		BaroVal;
extern int8		BaroSample;
extern int16	VBaroComp;
extern uint8	BaroType, BaroTemp;

extern uint8	MCamRoll,MCamPitch;
extern int16	Motor[NoOfMotors];
extern int16	Rl,Pl,Yl;	// PID output values

extern boolean	Flags[32];

extern uint8	LEDCycles;		// for light display
extern int8		IntegralCount;
extern uint24	RCGlitches;
extern int8		BatteryVolts; 

extern uint8	LEDShadow;		// shadow register
extern int16	AbsDirection;	// wanted heading (240 = 360 deg)
extern int16	CurDeviation;	// deviation from correct heading

extern uint8 	RxCheckSum;

extern int16	Trace[LastTrace];

// Principal quadrocopter parameters - MUST remain in this order
// for block read/write to EEPROM
// Really should be a vector - later!
extern int8	RollKp; 			// 01
extern int8	RollKi;				// 02
extern int8	RollKd;				// 03
extern int8 BaroTempCoeff;		// 04c
extern int8	RollIntLimit;		// 05
extern int8	PitchKp;			// 06
extern int8	PitchKi;			// 07
extern int8	PitchKd;			// 08
extern int8 BaroCompKp;			// 09c
extern int8	PitchIntLimit;		// 10

extern int8	YawKp; 				// 11
extern int8	YawKi;				// 12
extern int8	YawKd;				// 13
extern int8	YawLimit;			// 14
extern int8 YawIntLimit;		// 15
extern int8	ConfigParam;		// 16c
extern int8 TimeSlots;			// 17c
extern int8	LowVoltThres;		// 18c
extern int8	CamRollKp;			// 19
extern int8	PercentHoverThr;	// 20c 

extern int8	VertDampKp;			// 21c
extern int8 MiddleUD;			// 22c
extern int8	PercentIdleThr;		// 23c
extern int8	MiddleLR;			// 24c
extern int8	MiddleFB;			// 25c
extern int8	CamPitchKp;			// 26
extern int8	CompassKp;			// 27
extern int8	BaroCompKd;			// 28c
extern int8	NavRadius;			// 29
extern int8	NavIntLimit;		// 30

extern int8	NavAltKp;			// 31
extern int8	NavAltKi;			// 32
extern int8	NavRTHAlt;			// 33
extern int8	NavMagVar;			// 34c
extern int8 GyroType;			// 35c
extern int8 ESCType;			// 36c
extern int8 TxRxType;			// 37c

extern const rom int8 ComParms[];

extern const rom uint8 Map[CustomTxRx+1][CONTROLS];

#define _EESet1		0		// first set starts at address 0x00
#define _EESet2		0x40	// second set starts at address 0x40

#define FirstProgReg RollKp
#define	LastProgReg TxRxType

// End of c-ufo.h

