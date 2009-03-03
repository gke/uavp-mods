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


// Utilities and subroutines

#include "pu-test.h"
#include "bits.h"

#ifdef ESC_HOLGER

void EscI2CDelay(void)
{
	nop2();
}

// send a start condition
void EscI2CStart(void)
{
	ESC_DIO=1;	// set SDA to input, output a high
	EscI2CDelay();
	ESC_CIO=1;	// set SCL to input, output a high
	while( ESC_SCL == 0 ) ;	// wait for line to come hi
	EscI2CDelay();
	ESC_SDA = 0;	// start condition
	ESC_DIO = 0;	// output a low
	EscI2CDelay();
	ESC_SCL = 0;
	ESC_CIO = 0;	// set SCL to output, output a low
}

// send a stop condition
void EscI2CStop(void)
{
	ESC_DIO=0;	// set SDA to output
	ESC_SDA = 0;	// output a low
	EscI2CDelay();
	ESC_CIO=1;	// set SCL to input, output a high
	while( ESC_SCL == 0 ) ;	// wait for line to come hi
	EscI2CDelay();

	ESC_DIO=1;	// set SDA to input, output a high, STOP condition
	EscI2CDelay();		// leave clock high
}


// send a byte to I2C slave and return ACK status
// 0 = ACK
// 1 = NACK
uns8 SendEscI2CByte(uns8 nidata)
{
	uns8 nii;

	for(nii=0; nii<8; nii++)
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
	nii = ESC_SDA;	

	ESC_SCL = 0;
	ESC_CIO = 0;	// set SCL to output, output a low
	EscI2CDelay();
//	ESC_IO = 0;		// set SDA to output
//	ESC_SDA = 0;	// leave output low
	return(nii);
}

#endif	// ESC_HOLGER


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
uns8 SHADOWB, MF, MB, ML, MR, MT, ME; // motor/servo outputs
// Bootloader ???
#pragma udata

void OutSignals(void){

#ifdef ESC_HOLGER
	uns8 niret;	// return status

	EscI2CFlags = 0;
#endif

	WriteTimer0(0);
	INTCONbits.TMR0IF = 0;

	#ifdef ESC_PPM
	_asm
	MOVLB	0						// select Bank0
	MOVLW	0x0f				// turn on motors
	MOVWF	SHADOWB,1
	_endasm	
	PORTB |= 0x0f;
	#endif

    // mo motor test for 18f version
	MF = _Minimum;
	MB = _Minimum;
	ML = _Minimum;
	MR = _Minimum;


	#ifdef ESC_PPM

	// simply wait for nearly 1 ms
	// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
	while( ReadTimer0() < 0x100-3-16 ) ;

	// now stop CCP1 interrupt
	// capture can survive 1ms without service!

	// Strictly only if the masked interrupt region below is
	// less than the minimum valid Rx pulse/gap width which
	// is 1027uS less capture time overheads

	DisableInterrupts;	// BLOCK ALL INTERRUPTS for NO MORE than 1mS
	while( INTCONbits.TMR0IF == 0 ) ;	// wait for first overflow
	INTCONbits.TMR0IF=0;		// quit TMR0 interrupt

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

// This loop is exactly 16 cycles int16
// under no circumstances should the loop cycle time be changed
_asm
OS005:
	MOVLB	0						// select Bank0
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

	while( ReadTimer0() < 0x100-3-16 ) ; // wait for 2nd TMR0 near overflow

	INTCONbits.GIE = 0;					// Int wieder sperren, wegen Jitter

	while( INTCONbits.TMR0IF == 0 ) ;	// wait for 2nd overflow (2 ms)

	// avoid servo overrun when MCamxx == 0
	MT = ME = _Neutral;

	// This loop is exactly 16 cycles int16
	// under no circumstances should the loop cycle time be changed
_asm
OS001:
	MOVLB	0
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
	nop2();
	nop2();
_asm
	GOTO	OS001
OS002:
_endasm

	EnableInterrupts;	// re-enable interrupt

	while( INTCONbits.TMR0IF == 0 ) ;	// wait for 3rd TMR2 overflow

} // OutSignals


