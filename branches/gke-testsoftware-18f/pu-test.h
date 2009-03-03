
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

typedef unsigned char uns8 ;
typedef signed char int8;
typedef unsigned int uns16;
typedef int int16;
typedef long int32;
typedef unsigned long uint32;
typedef short long int24;
typedef unsigned short long uint24;

#define IsSet(S,b) 		((uns8)((S>>b)&1))
#define Invert(S,b) 	((uns8)(S^=(1<<b)))

#define DisableInterrupts (INTCONbits.GIEH=0)
#define EnableInterrupts (INTCONbits.GIEH=1)

// ==============================================
// == External variables
// ==============================================

extern	 uns8	Flags[8] , Flags2[8];
extern	 uns8	RecFlags;	// Interrupt save registers for FSR
extern	 uns8	LedShadow;	// shadow register
extern	 uns8	ConfigReg;
// the CurrK1..CurrK7 variables MUST be in this order!
extern	 uns16	CurrK1,CurrK2,CurrK3,CurrK4;
extern	 uns16	CurrK5,CurrK6,CurrK7;
extern	 uns16	PauseTime; 
extern	 uns8	MFront, MBack, MLeft, MRight;
extern	 uns8	MCamRoll, MCamPitch;
extern	 uns8	EscI2CFlags;
// special locations
extern	 uns16	nilgval;
extern	 int8 	TimeSlot;
extern	 uns8	mSTick;

// EEPROM parameter set addresses

#define _EESet1	0		// first set starts at address 0x00
#define _EESet2	0x20	// second set starts at address 0x20

#define DELAY_MS(ms) for( mSTick=ms; mSTick!=0; mSTick--){INTCONbits.T0IF=0;while(INTCONbits.T0IF == 0);}

#define _ClkOut		(160/4)	/* 16.0 MHz quartz */
#define _PreScale0	16	/* 1:16 TMR0 prescaler */
#define _PreScale1	8	/* 1:8 TMR1 prescaler */
#define _PreScale2	16
#define _PostScale2	16

// wegen dem dummen Compiler muss man händisch rechnen :-(
#define TMR2_5MS	78	/* 1x 4ms +  */
#define TMR2_14MS	219	/* 1x 15ms = 20ms pause time */

//                    RX impuls times in 10-microseconds units
//                    vvv   ACHTUNG: Auf numerischen Überlauf achten!
#ifdef ESC_PPM
#define	_Minimum	((105* _ClkOut/(2*_PreScale1))&0xFF)	/*-100% */
#endif
#ifdef ESC_HOLGER
#define _Minimum	0
#endif

#define _Neutral	((150* _ClkOut/(2*_PreScale1))&0xFF)    /*   0% */
#define _Maximum	((195* _ClkOut/(2*_PreScale1))&0xFF)	/*+100% */
#define _ThresStop	((113* _ClkOut/(2*_PreScale1))&0xFF)	/*-90% ab hier Stopp! */
#define _ThresStart	((116* _ClkOut/(2*_PreScale1))&0xFF)	/*-85% ab hier Start! */
#define _ProgMode	((160* _ClkOut/(2*_PreScale1))&0xFF)	/*+75% */
#define _ProgUp		((150* _ClkOut/(2*_PreScale1))&0xFF)	/*+60% */
#define _ProgDown	((130* _ClkOut/(2*_PreScale1))&0xFF)	/*-60% */

// ==============================================
// == Sanity checks of the #defines
// ==============================================

#if _Minimum >= _Maximum
#error _Minimum < _Maximum!
#endif
#if _ThresStart <= _ThresStop
#error _ThresStart <= _ThresStop!
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

// defines for SendComValUL
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

extern const char  SerHello[];
extern const char  SerSetup[];
extern const char  SerLSavail[];
extern const char SerLSnone[];
extern const char SerCompass[];
extern const char SerAlti[];
extern const char SerHelp[];
extern const char SerAnTest[];
extern const char SerLinTst[];
extern const char SerLinErr[];
extern const char SerSrvRun[];
extern const char SerI2CRun[];
extern const char SerI2CCnt[];
extern const char SerMagTst[];
extern const char SerPowTst[];
extern const char SerPowAux1[];
extern const char SerPowAux2[];
extern const char SerPowAux3[];
extern const char SerPowBlue[];
extern const char SerPowRed[];
extern const char SerPowGreen[];
extern const char SerPowYellow[];
extern const char SerPowBeep[];
extern const char SerSrvOK[];
extern const char SerPrompt[];
extern const char SerVolt[];
extern const char SerFail[];
extern const char SerGrad[];
extern const char SerI2CFail[];
extern const char SerMS[];
extern const char SerPPMP[];
extern const char SerPPMN[];
extern const char SerLinG[];
extern const char SerBaroOK[];
extern const char SerBaroT[];
extern const char SerCCalib1[];
extern const char SerCCalib2[]; 
extern const char SerCCalib3[];
extern const char SerCCalibE[];
extern const char SerBaroSMD500[];
extern const char SerBaroBMP085[];
extern const char SerBaroComp[];

extern const char SerRxTest[];
extern const char SerRxRes[];
extern const char SerRxFail[];
extern const char SerRxOK[];
extern const char SerRxNN[];

// Prototypes

extern	 void BootStart(void);

extern	 void ShowSetup(uns8);
extern	 char RecvComChar(void);
extern	 void ProcessComCommand(void);
extern	 void SendComChar(char);
extern	 void SendComCRLF(void);
extern	 void SendComValH(uns8);
extern	 void SendComValUL(uns8);
extern	 void AnalogTest(void);
extern	 void OutSignals(void);
extern	 void TestServos(void);
extern	 void ConfigureESCs(void);

extern	 void ReceiverTest(void);
extern	 void TogglePPMPolarity(void);

extern	 void Delay100mS(uns8);
extern	 void AcqTime(void);
extern	 void SendLeds(void);
extern	 void SwitchLedsOn(uns8);
extern	 void SwitchLedsOff(uns8);
extern	 uns8 ScanI2CBus(void);
extern	 void CompassTest(void);
extern	 void CalibrateCompass(void);
extern	 void BaroTest(void);
extern	 void I2CStart(void);
extern	 void I2CStop(void);
extern	 uns8 SendI2CByte(uns8);
extern	 uns8 RecvI2CByte(uns8);
extern	 void PowerOutput(uns8);
extern	 void SendComText(const char *);

extern	 void ReadParametersEE(void);

extern	 void IsLISLactive(void);
extern	 void LinearTest(void);

extern	 void nop2(void);

// End of pu-test.h

