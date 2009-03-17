// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://www.uavp.org                        =
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

// Utilities and subroutines

#include "c-ufo.h"
#include "bits.h"

#if (defined ESC_X3D || defined ESC_HOLGER || defined ESC_YGEI2C) && !defined DEBUG_SENSORS

void EscI2CDelay(void)
{
	nop2();
	nop2();
	nop2();
} // EscI2CDelay

void EscWaitClkHi(void)
{
	EscI2CDelay();
	ESC_CIO=1;	// set SCL to input, output a high
	while( ESC_SCL == 0 ) ;	// wait for line to come hi
	EscI2CDelay();
} // EscWaitClkHi

// send a start condition
void EscI2CStart(void)
{
	ESC_DIO=1;	// set SDA to input, output a high
	EscWaitClkHi();
	ESC_SDA = 0;	// start condition
	ESC_DIO = 0;	// output a low
	EscI2CDelay();
	ESC_SCL = 0;
	ESC_CIO = 0;	// set SCL to output, output a low
} // EscI2CStart

// send a stop condition
void EscI2CStop(void)
{
	ESC_DIO=0;	// set SDA to output
	ESC_SDA = 0;	// output a low
	EscWaitClkHi();

	ESC_DIO=1;	// set SDA to input, output a high, STOP condition
	EscI2CDelay();		// leave clock high
} // EscI2CStop


// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
void SendEscI2CByte(uint8 nidata)
{
	uint8 s;

	for(s=8; s!=0; s--)
	{
		if( IsSet(nidata,7) )
		{
			ESC_DIO = 1;	// switch to input, line is high
		}
		else
		{
			ESC_SDA = 0;			
			ESC_DIO = 0;	// switch to output, line is low
		}
	
		ESC_CIO=1;	// set SCL to input, output a high
		while( ESC_SCL == 0 ) ;	// wait for line to come hi
		EscI2CDelay();
		ESC_SCL = 0;
		ESC_CIO = 0;	// set SCL to output, output a low
		nidata <<= 1;
	}
	ESC_DIO = 1;	// set SDA to input
	EscI2CDelay();
	ESC_CIO=1;	// set SCL to input, output a high
	while( ESC_SCL == 0 ) ;	// wait for line to come hi

	EscI2CDelay();		// here is the ACK
//	nii = I2C_SDA;	

	ESC_SCL = 0;
	ESC_CIO = 0;	// set SCL to output, output a low
	EscI2CDelay();
//	I2C_IO = 0;		// set SDA to output
//	I2C_SDA = 0;	// leave output low
	return;
} // SendEscI2CByte


#endif	// ESC_X3D || ESC_HOLGER || ESC_YGEI2C

// to avoid stopping motors in the air, the
// motor values are limited to a minimum and
// a maximum
// the eventually corrected value is returned
int8 SaturInt(int16 l)
{
	#if defined ESC_PPM || defined ESC_HOLGER || defined ESC_YGEI2C
	if( l > _Maximum )
		return(_Maximum);
	if( l < MotorLowRun )
		return(MotorLowRun);
	// just for safety
	if( l < _Minimum )
		return(_Minimum);
	#endif

	#ifdef ESC_X3D
	l -= _Minimum;
	if( l > 200 )
		return(200);
	if( l < 1 )
		return(1);
	#endif
	return((int8)l);
} // SaturInt

// mix the PID-results (Rl, Pl and Yl) and the throttle
// on the motors and check for numerical overrun
void MixAndLimit(void)
{
	int16 CurrGas;
    int16 Temp;

	CurrGas = IGas;	// to protect against IGas being changed in interrupt
 
	#ifndef TRICOPTER
	if( FlyCrossMode )
	{	// "Cross" Mode
		Ml = CurrGas + Pl; Ml -= Rl;
		Mr = CurrGas - Pl; Mr += Rl;
		Mf = CurrGas - Pl; Mf -= Rl;
		Mb = CurrGas + Pl; Mb += Rl;
	}
	else
	{	// "Plus" Mode
		#ifdef MOUNT_45
		Ml = CurrGas - Rl; Ml -= Pl;	// K2 -> Front right
		Mr = CurrGas + Rl; Mr += Pl;	// K3 -> Rear left
		Mf = CurrGas + Rl; Mf -= Pl;	// K1 -> Front left
		Mb = IGas - Rl; Mb += Pl;	// K4 -> Rear rigt
		#else
		Ml = CurrGas - Rl;	// K2 -> Front right
		Mr = CurrGas + Rl;	// K3 -> Rear left
		Mf = CurrGas - Pl;	// K1 -> Front left
		Mb = CurrGas + Pl;	// K4 -> Rear rigt
		#endif
	}

	Mf += Yl;
	Mb += Yl;
	Ml -= Yl;
	Mr -= Yl;

	// Altitude stabilization factor
	Mf += Vud;
	Mb += Vud;
	Ml += Vud;
	Mr += Vud;

	Mf += VBaroComp;
	Mb += VBaroComp;
	Ml += VBaroComp;
	Mr += VBaroComp;

	// if low-throttle limiting occurs, must limit other motor too
	// to prevent flips!

	if( CurrGas > MotorLowRun )
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
	#else	// TRICOPTER
	Mf = CurrGas + Pl;	// front motor
	Ml = CurrGas + Rl;
	Mr = CurrGas - Rl;
	Rl >>= 1;
	Ml -= Rl;	// rear left
    Mr -= Pl;	// rear right
	Mb = Yl + _Neutral;	// yaw servo

	if( CurrGas > MotorLowRun )
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
	#endif

	// Ergebnisse auf Überlauf testen und korrigieren
	MFront = SaturInt(Mf);
	MLeft = SaturInt(Ml);
	MRight = SaturInt(Mr);
	MBack = SaturInt(Mb);
} // MixAndLimit

// Mix the Camera tilt channel (Ch6) and the
// ufo air angles (roll and nick) to the 
// camera servos. 
void MixAndLimitCam(void)
{
// Cam Servos

	if( IntegralCount > 0 )
		Rp = Pp = _Minimum;

	if( _UseCh7Trigger )
		Rp += _Neutral;
	else
		Rp += IK7;
		
	Pp += IK6;		// only Pitch servo is controlled by channel 6

	MCamRoll = Limit(Rp, _Minimum, _Maximum);
	MCamPitch = Limit(Pp, _Minimum, _Maximum);

} // MixAndLimitCam

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

#pragma udata assembly_language=0x080 
uint8 SHADOWB, MF, MB, ML, MR, MT, ME; // motor/servo outputs
// Bootloader ???
#pragma udata

void OutSignals(void)
{
	#ifdef NADA
	TxValH(MCamRoll);
	TxValH(MCamPitch);
	TxNextLine();
	#endif

	#ifndef DEBUG_SENSORS

	#ifdef DEBUG_MOTORS
	if( _Flying && IsSet(CamPitchFactor,4) )
	{
		TxValU(IGas);
		TxChar(';');
		TxValS(IRoll);
		TxChar(';');
		TxValS(IPitch);
		TxChar(';');
		TxValS(IYaw);
		TxChar(';');
		TxValU(MFront);
		TxChar(';');
		TxValU(MBack);
		TxChar(';');
		TxValU(MLeft);
		TxChar(';');
		TxValU(MRight);
		TxChar(0x0d);
		TxChar(0x0a);
	}
	#endif

	WriteTimer0(0);
	INTCONbits.TMR0IF = false;

	#ifdef ESC_PPM
	_asm
	MOVLB	0						// select Bank0
	MOVLW	0x0f				// turn on motors
	MOVWF	SHADOWB,1
	_endasm	
	PORTB |= 0x0f;
	#endif

	MF = MFront;
	MB = MBack;
	ML = MLeft;
	MR = MRight;

	#ifdef DEBUG_MOTORS
	// if DEBUG_MOTORS is active, CamIntFactor is a bitmap:
	// bit 0 = no front motor
	// bit 1 = no rear motor
	// bit 2 = no left motor
	// bit 3 = no right motor
	// bit 4 = turns on the serial output

	if( IsSet(CamPitchFactor,0) )
		MF = _Minimum;
	if( IsSet(CamPitchFactor,1) )
		MB = _Minimum;
	if( IsSet(CamPitchFactor,2) )
		ML = _Minimum;
	if( IsSet(CamPitchFactor,3) )
		MR = _Minimum;
	#else
	#ifdef INTTEST
	MF = _Minimum;
	MB = _Minimum;
	ML = _Minimum;
	MR = _Minimum;
	#endif
	#endif

	#ifdef ESC_PPM

	// simply wait for nearly 1 ms
	// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
	while( ReadTimer0() < (uint16)(0x100-3-16) ) ;

	// now stop CCP1 interrupt
	// capture can survive 1ms without service!

	// Strictly only if the masked interrupt region below is
	// less than the minimum valid Rx pulse/gap width which
	// is 1027uS less capture time overheads

	DisableInterrupts;	// BLOCK ALL INTERRUPTS for NO MORE than 1mS
	while( !INTCONbits.TMR0IF ) ;	// wait for first overflow
	INTCONbits.TMR0IF=0;		// quit TMR0 interrupt

	#if !defined DEBUG && !defined DEBUG_MOTORS
	if( _OutToggle )	// driver cam servos only every 2nd pulse
	{
		_asm
		MOVLB	0					// select Bank0
		MOVLW	0x3f				// turn on motors
		MOVWF	SHADOWB,1
		_endasm	
		PORTB |= 0x3f;
	}
	_OutToggle ^= 1;
	#endif

// This loop is exactly 16 cycles int16
// under no circumstances should the loop cycle time be changed
_asm
	MOVLB	0						// select Bank0
OS005:
	MOVF	SHADOWB,0,1				// Cannot read PORTB ???
	MOVWF	PORTB,0
	ANDLW	0x0f
	BZ		OS006
			
	DECFSZ	MF,1,1					// front motor
	GOTO	OS007
			
	BCF		SHADOWB,PulseFront,1	// stop Front pulse
OS007:
	DECFSZ	ML,1,1					// left motor
	GOTO	OS008
			
	BCF		SHADOWB,PulseLeft,1		// stop Left pulse
OS008:
	DECFSZ	MR,1,1					// right motor
	GOTO	OS009
			
	BCF		SHADOWB,PulseRight,1	// stop Right pulse
OS009:
	DECFSZ	MB,1,1					// rear motor
	GOTO	OS005
				
	BCF		SHADOWB,PulseBack,1		// stop Back pulse			

	GOTO	OS005
OS006:
_endasm
	// This will be the corresponding C code:
	//	while( ALL_OUTPUTS != 0 )
	//	{	// remain in loop as int16 as any output is still high
	//		if( TMR2 = MFront  ) PulseFront  = 0;
	//		if( TMR2 = MBack ) PulseBack = 0;
	//		if( TMR2 = MLeft  ) PulseLeft  = 0;
	//		if( TMR2 = MRight ) PulseRight = 0;
	//	}

	EnableInterrupts;	// Re-enable interrupt

	#endif	// ESC_PPM

	#if defined ESC_X3D || defined ESC_HOLGER || defined ESC_YGEI2C

	#if !defined DEBUG && !defined DEBUG_MOTORS
	if( _OutToggle )	// driver cam servos only every 2nd pulse
	{
		_asm
		MOVLB	0					// select Bank0
		MOVLW	0x3f				// turn on motors
		MOVWF	SHADOWB,1
		_endasm	
		PORTB |= 0x3f;
	}
	_OutToggle ^= 1;
	#endif

	// in X3D- and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL
	#ifdef ESC_X3D
	EscI2CStart();
	SendEscI2CByte(0x10);	// one command, 4 data bytes
	SendEscI2CByte(MF); // for all motors
	SendEscI2CByte(MB);
	SendEscI2CByte(ML);
	SendEscI2CByte(MR);
	EscI2CStop();
	#endif	// ESC_X3D

	#ifdef ESC_HOLGER
	EscI2CStart();
	SendEscI2CByte(0x52);	// one cmd, one data byte per motor
	SendEscI2CByte(MF); // for all motors
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
	SendEscI2CByte(0x62);	// one cmd, one data byte per motor
	SendEscI2CByte(MF>>1); // for all motors
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

	#endif	// ESC_X3D or ESC_HOLGER or ESC_YGEI2C

	#ifndef DEBUG_MOTORS
	while( ReadTimer0() < (uint16)(0x100-3-16) ) ; 	// wait for 2nd TMR0 near overflow

	INTCONbits.GIE = false;					// Int wieder sperren, wegen Jitter
	while( !INTCONbits.TMR0IF ) ;		// wait for 2nd overflow (2 ms)

	#if !defined DEBUG && !defined DEBUG_SENSORS
	// This loop is exactly 16 cycles int16
	// under no circumstances should the loop cycle time be changed
_asm
	MOVLB	0
OS001:
	MOVF	SHADOWB,0,1				// Cannot read PORTB ???
	MOVWF	PORTB,0
	ANDLW	0x30		// output ports 4 and 5
	BZ		OS002		// stop if all 2 outputs are 0

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
	Delay1TCY();
_asm
	GOTO	OS001
OS002:
_endasm
#endif	// DEBUG
	EnableInterrupts;	// re-enable interrupt

#endif	// DEBUG_MOTORS

#endif  // !DEBUG_SENSORS
} // OutSignals


