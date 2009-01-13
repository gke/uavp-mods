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

// Utilities and subroutines

#include "c-ufo.h"
#include "bits.h"

// Outputs signals to the speed controllers
// using timer 0
// all motor outputs are driven simultaneously
//
// 0x80 gives 1,52ms, 0x60 = 1,40ms, 0xA0 = 1,64ms
//
// This routine needs exactly 3 ms to complete:
//
// Motors:      ___________
//          ___|     |_____|_____________
//
// Camservos:         ___________
//          _________|     |_____|______
//
//             0     1     2     3 ms

uint8 Saturate(int32 l)
{
	// do not let a motor drop below idle
	return(Limit(l,Max(_Minimum, MotorLowRun),_Maximum));
} // SaturateInt8
 
int16 ThrottleCurve(int16 G)
{
#ifdef ABANDONED_USE_THROTTLECURVE
	// Lookup table based throttle curve low gain at hover
	// Working but ABANDONED - use Tx based throttle curve shaping

	static const uint8 ThrottleTable[16]={
	0, 16, 32,
	36, 40, 44, 48, 52,56, 60, 64, 80, 
	96, 112, 128, 144
	};
	uint8 Index, Low, High, Offset;
	int16 Temp;

	if( _NewValues )
	{
		if (G > 127) G=127;
		Index = G>>3;
		Offset = G&0x07;
		Low = ThrottleTable[Index];
		High = ThrottleTable[++Index];
		Temp = (High-Low)*Offset;
		Temp >>= 3;
		G = Low + Temp; 
	}	
#endif	// USE_THROTTLECURVE
 
	return(G);
}  // ThrottleCurve

// mix the PID-results (Rl, Pl and Yl) and the throttle
// on the motors and check for numerical overrun
void MixAndLimitMotors(void)
{
	uint8 Th;							// Local variable to protect during interrupts
	int16 Temp;
	int32 Ml, Mr, Mf, Mb;					// excessive range but safer for now		

	if ( _MotorsEnabled )
		Th = ThrottleCurve(IThrottle); 		// Th snapshots IThrottle which may change in irq.c
	else
		Th = _Minimum;

#ifdef TRICOPTER
	Mf = Th + Pl;	// front motor
	Ml = Th + Rl;
	Mr = Th - Rl;
	Rl >>= 1;
	Ml -= Rl;								// rear left
    Mr -= Pl;								// rear right
	Mb = -Yl + _Neutral;					// yaw servo

	if( Th > MotorLowRun )
	{
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			// Mf += Mb - MotorLowRun
			Ml += Mr;
			Ml -= MotorLowRun;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{
			// Mb += Mf - MotorLowRun
			Mr += Ml;
			Mr -= MotorLowRun;
		}
	}
#else // QUADROCOPTER
	if( FlyCrossMode )
	{	// "Cross" Mode
		Ml = Th + Pl;	Ml -= Rl;
		Mr = Th - Pl;	Mr += Rl;
		Mf = Th - Pl;	Mf -= Rl;
		Mb = Th + Pl;	Mb += Rl;
	}
	else
	{	// "Plus" Mode
		Ml = Th - Rl;
		Mr = Th + Rl;
		Mf = Th - Pl;
		Mb = Th + Pl;
	}

	Mf -= Yl;
	Mb -= Yl;
	Ml += Yl;
	Mr += Yl;

	// Altitude stabilization from vertical accelerometer
	Mf += Vud;
	Mb += Vud;
	Ml += Vud;
	Mr += Vud;

	// Altitude stabilization from barometer/altimeter
	Mf += VBaroComp;
	Mb += VBaroComp;
	Ml += VBaroComp;
	Mr += VBaroComp;

	// if low-throttle limiting occurs, must limit other motor too
	// to prevent flips!
	// needs further thought ???
	if( Th > MotorLowRun )
	{
		if( (Mf > Mb) && (Mb < MotorLowRun) )
		{
			Temp = Mb - MotorLowRun;
			Mf += Temp;
			Ml += Temp;
			Mr += Temp;
		}
		if( (Mb > Mf) && (Mf < MotorLowRun) )
		{
			Temp = Mf - MotorLowRun;
			Mb += Temp;
			Ml += Temp;
			Mr += Temp;
		}
		if( (Ml > Mr) && (Mr < MotorLowRun) )
		{
			Temp = Mr - MotorLowRun;
			Ml += Temp;
			Mf += Temp;
			Mb += Temp;
		}
		if( (Mr > Ml) && (Ml < MotorLowRun) )
		{	
			Temp = Ml - MotorLowRun;
			Mr += Temp;
			Mf += Temp;
			Mb += Temp;
		}
	}
#endif // TRICOPTER

	// limit to motor_idle.._Maximum
	MFront = Saturate(Mf);
	MLeft = Saturate(Ml);
	MRight = Saturate(Mr);
	MBack = Saturate(Mb);

} // MixAndLimitMotors

// Mix the Camera tilt channel (Ch6) and the quadrocopter 
// attitude angles (roll and nick) to the camera servos. 
void MixAndLimitCam(void)
{
	int32 Rp, Pp;

	if (CamIntFactor != 0)
	{
		Rp = RollAngle/CamIntFactor;
		Pp = PitchAngle/CamIntFactor;
	}
	else
	{
		Rp = 0;
		Pp = 0;
	}

	if( _UseCh7Trigger )
		Rp += _Neutral;
	else
		Rp += IK7;
		
	Pp += IK6;								// only Pitch servo is controlled by channel 6

	MCamRoll = Limit(Rp, _Minimum , _Maximum);
	MCamPitch = Limit(Pp, _Minimum , _Maximum);

} // MixAndLimitCam

void OutSignals(void)
{
	// for positive PID coeffs MF/MB anticlockwise, ML/MR clockwise
	int32 NowMilliSec;

	MixAndLimitMotors();

	if ( _MotorsEnabled )
	{
		MF = MFront;
		MB = MBack;
		ML = MLeft;
		MR = MRight;
	}
	else
	{
		MF = _Minimum;
		MB = _Minimum;
		ML = _Minimum;
		MR = _Minimum;
	}

	MixAndLimitCam();
	MT = MCamRoll;
	ME = MCamPitch;

#ifndef DEBUG

#ifdef ESC_PPM
	DisableInterrupts;
	while( INTCONbits.T0IF == 0 ) ;			// wait for overflow
	INTCONbits.T0IF=0;						// quit TMR0 interrupt
	EnableInterrupts;

_asm
	MOVLB	0								// select Bank0
	MOVLW	0x3F							// turn on camera servos
	MOVWF	PORTB,0							// setup PORTB shadow
	MOVWF	SHADOWB,1
_endasm

	// simply wait for nearly 1 ms
	// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
	while( ReadTimer0() <  0x100-3-16 ) ;	// ??? need to understand this

	// now stop CCP1 interrupt - capture can survive 1ms without service!
	// To be strictly correct it must be less than the minimum valid
	// Rx pulse/gap width (1027us less interrupt overheads) otherwise 
	// a CCP1 capture event may be over-written	
	DisableInterrupts;
	while( INTCONbits.T0IF == 0 ) ;			// wait for first overflow
	INTCONbits.T0IF=0;						// quit TMR0 interrupt
	
// This loop should be exactly 16 cycles long
// under no circumstances should the loop cycle time be changed
_asm
OS005:
	MOVF	SHADOWB,0,1						// Cannot read PORTB!
	MOVWF	PORTB,0
	ANDLW	0x0F							// output ports 0 to 3
	BZ		OS006

	DECFSZ	MF,1,1							// front motor
	GOTO	OS007

	BCF		SHADOWB,PulseFront,1			// stop Front pulse
OS007:
	DECFSZ	ML,1,1							// left motor
	GOTO	OS008

	BCF		SHADOWB,PulseLeft,1			// stop Left pulse
OS008:
	DECFSZ	MR,1,1							// right motor
	GOTO	OS009

	BCF		SHADOWB,PulseRight,1			// stop Right pulse
OS009:
	DECFSZ	MB,1,1							// rear motor
	GOTO	OS005
	
	BCF		SHADOWB,PulseBack,1			// stop Back pulse
	GOTO	OS005
OS006:

_endasm

	// This will be the corresponding C code:
	//	while( ALL_OUTPUTS != 0 )
	//	{	// remain in loop as long as any output is still high
	//		if( TMR2 = MFront  ) PulseFront  = 0;
	//		if( TMR2 = MBack ) PulseBack = 0;
	//		if( TMR2 = MLeft  ) PulseLeft  = 0;
	//		if( TMR2 = MRight ) PulseRight = 0;
	//	}

	EnableInterrupts;
    
    // Allow interupts to breath - may cause some jitter on camera servos

	// simply wait for balance of 1 ms
	// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
	while( ReadTimer0() <  0x100-3-16 ) ;

	DisableInterrupts;
	while( INTCONbits.T0IF == 0 ) ;			// wait for first overflow
	INTCONbits.T0IF=0;						// quit TMR0 interrupt

// This loop should be exactly 16 cycles long
// under no circumstances should the loop cycle time be changed
_asm
OS001:
	MOVF	SHADOWB,0,1						// Cannot read PORTB!
	MOVWF	PORTB,0
	ANDLW	0x30							// output ports 4 and 5
	BZ		OS002

	DECFSZ	MT,1,1
	GOTO	OS003

	BCF		SHADOWB,PulseCamRoll,1	
OS003:
	DECFSZ	ME,1,1
	GOTO	OS004

	BCF		SHADOWB,PulseCamPitch,1
OS004:
_endasm

	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();	// currently only 15 cycles  but acceptable ???

_asm
	GOTO	OS001
OS002:
_endasm

	EnableInterrupts; 	

#endif // ESC_PPM

#if defined ESC_X3D || defined ESC_HOLGER || defined ESC_YGEI2C

	DisableInterrupts;
	while( INTCONbits.T0IF == 0 ) ;			// wait for overflow
	INTCONbits.T0IF=0;						// quit TMR0 interrupt
	EnableInterrupts();
_asm
	MOVLB	0								// select Bank0
//	MOVF	CAMTOGGLE,0,1
	MOVLW	0x30							// turn on camera servos
	MOVWF	PORTB,0							// setup PORTB shadow
	MOVWF	SHADOWB,1
_endasm
	
// in X3D- and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL
#ifdef ESC_X3D
	EscI2CStart();
	SendEscI2CByte(0x10);					// one command, 4 data bytes
	SendEscI2CByte(MF); 					// for all motors
	SendEscI2CByte(MB);
	SendEscI2CByte(ML);
	SendEscI2CByte(MR);
	EscI2CStop();
#endif	// ESC_X3D

#ifdef ESC_HOLGER
	EscI2CStart();
	SendEscI2CByte(0x52);					// one cmd, one data byte per motor
	SendEscI2CByte(MF); 					// for all motors
	EscI2CStop();

	EscI2CStart();
	SendEscI2CByte(0x54);
	SendEscI2CByte(MB);
	EscI2CStop();

	EscI2CStart();
	SendEscI2CByte(0x58);
	SendEscI2CByte(ML);
	EscI2CStop();

	EscI2CStart();
	SendEscI2CByte(0x56);
	SendEscI2CByte(MR);
	EscI2CStop();
#endif	// ESC_HOLGER

#ifdef ESC_YGEI2C
	EscI2CStart();
	SendEscI2CByte(0x62);					// one cmd, one data byte per motor
	SendEscI2CByte(MF>>1); 					// for all motors
	EscI2CStop();

	EscI2CStart();
	SendEscI2CByte(0x64);
	SendEscI2CByte(MB>>1);
	EscI2CStop();

	EscI2CStart();
	SendEscI2CByte(0x68);
	SendEscI2CByte(ML>>1);
	EscI2CStop();

	EscI2CStart();
	SendEscI2CByte(0x66);
	SendEscI2CByte(MR>>1);
	EscI2CStop();
#endif	// ESC_YGEI2C

	// simply wait for balance of 1 ms
	// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
	while( ReadTimer0() < 0x100-3-16 ) ;

	DisableInterrupts;
	while( INTCONbits.T0IF == 0 ) ;			// wait for first overflow
	INTCONbits.T0IF=0;						// quit TMR0 interrupt					

// This loop should be exactly 16 cycles long
// under no circumstances should the loop cycle time be changed
_asm
OS001:
	MOVF	SHADOWB,0,1						// Cannot read PORTB - analog config problems!
	MOVWF	PORTB,0
	ANDLW	0x30							// output ports 4 and 5
	BZ		OS002

	DECFSZ	MT,1,1
	GOTO	OS003

	BCF		SHADOWB,PulseCamRoll,1	
OS003:
	DECFSZ	ME,1,1
	GOTO	OS004

	BCF		SHADOWB,PulseCamPitch,1
OS004:
_endasm

	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();	// currently only 15 cycles  but acceptable ???

_asm
	GOTO	OS001
OS002:
_endasm

	EnableInterrupts; 					// re-enable interrupts

#endif	// ESC_X3D or ESC_HOLGER or ESC_YGEI2C
#endif	// !DEBUG

} // OutSignals




