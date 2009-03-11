
// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =           Extensively modified 2008-9 by Prof. Greg Egan            =
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

// C-Ufo Header File
#ifndef BATCHMODE
// ==============================================
// == Global compiler switches
// ==============================================

// use this to allow debugging with MPLAB's Simulator
// #define SIMU
//
// Only one of the following 2 defines must be activated:
// When using 3 ADXRS300 or -150 gyros
//#define OPT_ADXRS
// When using 1 ADXRS300 and 1 IDG300 gyro
#define OPT_IDG

// defined: serial PPM pulse train from receiver
// undefined: standard servo pulses from CH1, 3, 5 and 7
//#define RX_PPM

// Select what speeed controllers to use
// to use standard PPM pulse
#define ESC_PPM

// to use Holgers ESCs (tested by ufo-hans)
//#define ESC_HOLGER

#endif // !BATCHMODE

// Board version
#define Version	"3.14m3"

#include <p18cxxx.h> 
#include <math.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>
#include <capture.h>
#include <adc.h>

// Types

typedef unsigned char uint8 ;
typedef signed char int8;
typedef unsigned int uint16;
typedef int int16;
typedef long int32;
typedef unsigned long uint32;
typedef short long int24;
typedef unsigned short long uint24;

#define IsSet(S,b) 		((uint8)((S>>b)&1))
#define Invert(S,b) 	((uint8)(S^=(1<<b)))

#define DisableInterrupts (INTCONbits.GIEH=0)
#define EnableInterrupts (INTCONbits.GIEH=1)

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

#define ADCVRefChan 			3 
#define ADCYawChan			4

#define ADCVREF5V 			0

// ==============================================
// == External variables
// ==============================================

extern	 uint8	Flags[8] , Flags2[8];
extern	 uint8	RecFlags;	// Interrupt save registers for FSR
extern	 uint8	LedShadow;	// shadow register
extern	 uint8	ConfigReg;
// the CurrK1..CurrK7 variables MUST be in this order!
extern	 uint16	CurrK1,CurrK2,CurrK3,CurrK4;
extern	 uint16	CurrK5,CurrK6,CurrK7;
extern	 uint16	PauseTime; 
extern	 uint8	EscI2CFlags;
// special locations
extern	 int24	val;
extern	 int8 	TimeSlot;
extern	 uint8	mSTick;

// EEPROM parameter set addresses

#define _EESet1	0		// first set starts at address 0x00
#define _EESet2	0x20	// second set starts at address 0x20

#define DELAY_MS(ms) for( mSTick=ms; mSTick!=0; mSTick--){INTCONbits.T0IF=0;while(INTCONbits.T0IF == 0);}

#define _ClkOut		(160/4)	/* 16.0 MHz quartz */
#define TMR2_5MS	78	/* 1x 5ms +  */
#define TMR2_14MS	234	/* 1x 15ms = 20ms pause time */

// RX impuls times in 10-microseconds units
#ifdef ESC_PPM
#define	_Minimum	1
#define _Maximum	240	
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

#define _Neutral	((_Maximum+_Minimum+1)/2)

// ==============================================
// == Sanity checks of the #defines
// ==============================================

#if _Minimum >= _Maximum
#error _Minimum < _Maximum!
#endif

#if (_Maximum < _Neutral)
#error Maximum < _Neutral !
#endif

#if defined OPT_ADXRS && defined OPT_IDG
#error OPT_ADXRS and OPT_IDG set!
#endif
#if !defined OPT_ADXRS && !defined OPT_IDG
#error OPT_ADXRS and OPT_IDG both not set!
#endif

// Parameters for UART port

#define _B9600	(_ClkOut*100000/(4*9600) - 1)
#define _B19200	(_ClkOut*100000/(4*19200) - 1)
#define _B38400	(_ClkOut*100000/(4*38400) - 1)

// defines for TxValUL
#define NKSMASK 0b00000111
#define	NKS0	0b00000000
#define	NKS1	0b00000001
#define	NKS2	0b00000010
#define	NKS3	0b00000011
#define	NKS4	0b00000100

#define LENMASK 0b00111000
#define LEN5	0b00101000
#define LEN4	0b00100000
#define LEN3	0b00011000
#define LEN2	0b00010000

#define VZ		0b01000000

extern const uint8  SerHello[];
extern const uint8  SerSetup[];
extern const uint8  SerLSavail[];
extern const uint8 SerLSnone[];
extern const uint8 SerCompass[];
extern const uint8 SerAlti[];
extern const uint8 SerHelp[];
extern const uint8 SerAnTest[];
extern const uint8 SerLinTst[];
extern const uint8 SerLinErr[];
extern const uint8 SerSrvRun[];
extern const uint8 SerI2CRun[];
extern const uint8 SerI2CCnt[];
extern const uint8 SerMagTst[];
extern const uint8 SerPowTst[];
extern const uint8 SerPowAux1[];
extern const uint8 SerPowAux2[];
extern const uint8 SerPowAux3[];
extern const uint8 SerPowBlue[];
extern const uint8 SerPowRed[];
extern const uint8 SerPowGreen[];
extern const uint8 SerPowYellow[];
extern const uint8 SerPowBeep[];
extern const uint8 SerSrvOK[];
extern const uint8 SerPrompt[];
extern const uint8 SerVolt[];
extern const uint8 SerFail[];
extern const uint8 SerGrad[];
extern const uint8 SerI2CFail[];
extern const uint8 SerMS[];
extern const uint8 SerPPMP[];
extern const uint8 SerPPMN[];
extern const uint8 SerLinG[];
extern const uint8 SerBaroOK[];
extern const uint8 SerBaroT[];
extern const uint8 SerCCalib1[];
extern const uint8 SerCCalib2[]; 
extern const uint8 SerCCalib3[];
extern const uint8 SerCCalibE[];
extern const uint8 SerBaroSMD500[];
extern const uint8 SerBaroBMP085[];
extern const uint8 SerBaroComp[];

extern const uint8 SerRxTest[];
extern const uint8 SerRxRes[];
extern const uint8 SerRxFail[];
extern const uint8 SerRxOK[];
extern const uint8 SerRxNN[];

// Prototypes

extern	 void BootStart(void);

extern	 void ShowSetup(uint8);
extern	 uint8 RecvComChar(void);
extern	 void ProcessComCommand(void);
extern	 void TxChar(uint8);
extern	 void TxNextLine(void);
extern	 void TxValH(uint8);
extern	 void TxValUL(uint8);
extern	 void AnalogTest(void);
extern	 void OutSignals(void);
extern	 void TestServos(void);
extern	 void ConfigureESCs(void);

extern	 void ReceiverTest(void);
extern	 void TogglePPMPolarity(void);

extern	 void Delay100mS(uint8);
extern	 void AcqTime(void);
extern	 void SendLeds(void);
extern	 void SwitchLedsOn(uint8);
extern	 void SwitchLedsOff(uint8);
extern	 uint8 ScanI2CBus(void);
extern	 void CompassTest(void);
extern	 void CalibrateCompass(void);
extern	 void BaroTest(void);
extern	 void I2CStart(void);
extern	 void I2CStop(void);
extern	 uint8 SendI2CByte(uint8);
extern	 uint8 RecvI2CByte(uint8);
extern	 void PowerOutput(uint8);
extern	 void TxText(const uint8 *);

extern	 void ReadParametersEE(void);

extern	int16 ADC(uint8, uint8);
extern	void InitADC(void);

extern	 void IsLISLactive(void);
extern	 void LinearTest(void);

extern	 void nop2(void);

// End of pu-test.h

