
// =================================================================================================
// =                                  UAVX Quadrocopter Controller                                 =
// =                             Copyright (c) 2008 by Prof. Greg Egan                             =
// =                   Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                       http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// =================================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even 
//    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
//    General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#ifndef BATCHMODE

#endif // !BATCHMODE

#ifdef I2C_HW
	#include "i2c.h"
#else
	#define MASTER 		0
	#define SLEW_ON 	0
#endif // I2C_HW

//________________________________________________________________________________________________


//________________________________________________________________________________________

#include "UAVXRxRevision.h"

// 18Fxxx C18 includes

#include <p18cxxx.h> 
#include <delays.h>
#include <timers.h>
#include <usart.h>

// Useful Constants
#define NUL 	0
#define SOH 	1
#define EOT 	4
#define ACK		6
#define HT 		9
#define LF 		10
#define CR 		13
#define NAK 	21
#define ESC 	27
#define true 	1
#define false 	0

#define MILLIPI 			3142 
#define CENTIPI 			314 
#define HALFMILLIPI 		1571 
#define QUARTERMILLIPI		785
#define TWOMILLIPI 			6284

#define MILLIRAD 			18 
#define CENTIRAD 			2

#define MAXINT32 			0x7fffffff
#define	MAXINT16 			0x7fff

// Additional Types
typedef unsigned char 		uint8 ;
typedef signed char 		int8;
typedef unsigned int 		uint16;
typedef int 				int16;
typedef short long 			int24;
//typedef long 			int24;
typedef unsigned short long uint24;
typedef long 				int32;
typedef unsigned long 		uint32;
typedef uint8 				boolean;
typedef float 				real32;

typedef union {
	int16 i16;
	uint16 u16;
	struct {
		uint8 b0;
		uint8 b1;
	};
	struct {
		uint8 i0;
		uint8 i1;
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
} i24u;

typedef union {
	int32 i32;
	uint32 u32;
	struct {
		uint16 w0;
		uint16 w1;
	};
	struct {
		uint8 b0;
		uint8 b1;
		uint8 b2;
		uint8 b3;
	};
} i32u;

typedef struct {
	uint8 Head, Tail;
	uint8 B[128];
	} uint8Q;	

typedef struct {
	uint8 Head, Tail;
	int24 B[8];
	} int24Q;	

typedef struct {
	uint8 Head, Tail;
	int32 B[4];
	} int32Q;	

// Macros
#define Set(S,b) 			((uint8)(S|=(1<<b)))
#define Clear(S,b) 			((uint8)(S&=(~(1<<b))))
#define IsSet(S,b) 			((uint8)((S>>b)&1))
#define IsClear(S,b) 		((uint8)(!(S>>b)&1))
#define Invert(S,b) 		((uint8)(S^=(1<<b)))

#define Abs(i)				(((i)<0) ? -(i) : (i))
#define Sign(i)				(((i)<0) ? -1 : 1)

#define Max(i,j) 			((i<j) ? j : i)
#define Min(i,j) 			((i<j) ? i : j )
#define Decay1(i) 			(((i) < 0) ? (i+1) : (((i) > 0) ? (i-1) : 0))

#define USE_LIMIT_MACRO
#ifdef USE_LIMIT_MACRO
	#define Limit(i,l,u) 	(((i) < l) ? l : (((i) > u) ? u : (i)))
#else
	#define Limit			ProcLimit
#endif

// To speed up NMEA sentence processing 
// must have a positive argument
#define ConvertDDegToMPi(d) (((int32)d * 3574L)>>11)
#define ConvertMPiToDDeg(d) (((int32)d * 2048L)/3574L)

#define ToPercent(n, m) (((n)*100L)/m)

// Simple filters using weighted averaging
#define VerySoftFilter(O,N) 	(SRS16((O)+(N)*3, 2))
#define SoftFilter(O,N) 		(SRS16((O)+(N), 1))
#define MediumFilter(O,N) 		(SRS16((O)*3+(N), 2))
#define HardFilter(O,N) 		(SRS16((O)*7+(N), 3))

// Unsigned
#define VerySoftFilterU(O,N)	(((O)+(N)*3+2)>>2)
#define SoftFilterU(O,N) 		(((O)+(N)+1)>>1)
#define MediumFilterU(O,N) 		(((O)*3+(N)+2)>>2)
#define HardFilterU(O,N) 		(((O)*7+(N)+4)>>3)

#define NoFilter(O,N)			(N)

#define DisableInterrupts 	(INTCONbits.GIEH=0)
#define EnableInterrupts 	(INTCONbits.GIEH=1)
#define InterruptsEnabled 	(INTCONbits.GIEH)

// Clock
#ifdef CLOCK_16MHZ
	#define	TMR0_1MS		0
#else // CLOCK_40MHZ
	#define	TMR0_1MS		(65536-640) // actually 1.0248mS to clear PWM
#endif // CLOCK_16MHZ

#define _PreScale0		16				// 1 6 TMR0 prescaler 
#define _PreScale1		8				// 1:8 TMR1 prescaler 
#define _PreScale2		16
#define _PostScale2		16

// Parameters for UART port ClockHz/(16*(BaudRate+1))
#ifdef CLOCK_16MHZ
#define _B9600			104
#define _B38400			26 
#else // CLOCK_40MHZ
#define _B9600			65
#define _B38400			65
#endif // CLOCK_16MHZ

#define	I2C_ACK			((uint8)(0))
#define	I2C_NACK		((uint8)(1))

// RC

#define CONTROLS 			7
#define MAX_CONTROLS 		12 	// maximum Rx channels

#define RxFilter			MediumFilterU
//#define RxFilter			SoftFilterU
//#define RxFilter			NoFilter

#define	RC_GOOD_BUCKET_MAX	20
#define RC_GOOD_RATIO		4

#define RC_MINIMUM			0
#define RC_MAXIMUM			238
#define RC_NEUTRAL			((RC_MAXIMUM-RC_MINIMUM+1)/2)

#define RC_MAX_ROLL_PITCH	(170)	

#define RC_THRES_STOP		((6L*RC_MAXIMUM)/100)		
#define RC_THRES_START		((10L*RC_MAXIMUM)/100)		

#define RC_FRAME_TIMEOUT_MS 	25
#define RC_SIGNAL_TIMEOUT_MS 	(5L*RC_FRAME_TIMEOUT_MS)
#define RC_THR_MAX 			RC_MAXIMUM

#define MAX_ROLL_PITCH		RC_NEUTRAL	// Rx stick units - rely on Tx Rate/Exp

#ifdef RX6CH 
	#define RC_CONTROLS 5			
#else
	#define RC_CONTROLS CONTROLS
#endif //RX6CH

// ESC
#define OUT_MINIMUM			1			// Required for PPM timing loops
#define OUT_MAXIMUM			200			// to reduce Rx capture and servo pulse output interaction
#define OUT_NEUTRAL			105			// 1.503mS @ 105 16MHz
#define OUT_HOLGER_MAXIMUM	225
#define OUT_YGEI2C_MAXIMUM	240
#define OUT_X3D_MAXIMUM		200

// Compass sensor
#define COMPASS_I2C_ID		0x42		// I2C slave address
#define COMPASS_MAXDEV		30			// maximum yaw compensation of compass heading 
#define COMPASS_MIDDLE		10			// yaw stick neutral dead zone
#define COMPASS_TIME_MS		50			// 20Hz



#if RC_MINIMUM >= RC_MAXIMUM
#error RC_MINIMUM < RC_MAXIMUM!
#endif
#if (RC_MAXIMUM < RC_NEUTRAL)
#error RC_MAXIMUM < RC_NEUTRAL !
#endif

#define TX_BUFF_SIZE 128
#define TX_BUFF_MASK (TX_BUFF_SIZE-1)
extern uint8Q TxQ;

typedef union {
		uint8 AllFlags;
		struct {
		uint8
		Signal:1,
		RCFrameOK:1, 
		ParametersValid:1,
		RCNewValues:1,
		NewCommands:1,
		TxToBuffer:1;
		};
} Flags;

extern Flags F;



