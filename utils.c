// ==============================================
// =    U.A.V.P Brushless UFO Test software     =
// =           Professional Version             =
// = Copyright (c) 2007 Ing. Wolfgang Mahringer =
// ==============================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// ==============================================
// =  please visit http://www.uavp.de           =
// =               http://www.mahringer.co.at   =
// ==============================================

// Utilities and subroutines

#pragma codepage=0

#include "pu-test.h"
#include "bits.h"

// Math Library
#include "mymath16.h"

#if defined ESC_X3D || defined ESC_HOLGER

void EscI2CDelay(void)
{
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
	nop2();
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
		if( nidata.7 )
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

#endif	// ESC_X3D || ESC_HOLGER


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

void OutSignals(void){
	bank0 uns8 MV, MH, ML, MR;	// must reside on bank0
	uns8 MT@MV;	// cam tilt servo
	uns8 ME@MH; // cam tilt servo
#if defined ESC_HOLGER || defined ESC_X3D
	uns8 niret;	// return status
#endif

	TMR0 = 0;
	T0IF = 0;

#ifdef ESC_PWM
	ALL_PULSE_ON;	// turn on all motor outputs

	MV = MVorne;
	MH = MHinten;
	ML = MLinks;
	MR = MRechts;

// simply wait for nearly 1 ms
// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
	while( TMR0 < 0x100-3-16 ) ;

	// now stop CCP1 interrupt
	// capture can survive 1ms without service!

	GIE = 0;	// BLOCK ALL INTERRUPTS
	while( T0IF == 0 ) ;	// wait for first overflow
	T0IF=0;		// quit TMR0 interrupt

#ifndef DEBUG
	CAM_PULSE_ON;	// now turn camera servo pulses on too
#endif

// This loop is exactly 16 cycles long
// under no circumstances should the loop cycle time be changed
#asm
	BCF	RP0		// clear all bank bits
	BCF	RP1
OS005
	MOVF	PORTB,W
	ANDLW	0x0F		// output ports 0 to 3
	BTFSC	Zero_
	GOTO	OS006		// stop if all 4 outputs are done

	DECFSZ	MV,f		// front motor
	GOTO	OS007

	BCF	PulseVorne		// stop pulse
OS007
	DECFSZ	ML,f		// left motor
	GOTO	OS008

	BCF	PulseLinks		// stop pulse
OS008
	DECFSZ	MR,f		// right motor
	GOTO	OS009

	BCF	PulseRechts		// stop pulse
OS009
	DECFSZ	MH,f		// rear motor
	GOTO	OS005
	
	BCF	PulseHinten		// stop pulse
	GOTO	OS005
OS006
#endasm
// This will be the corresponding C code:
//	while( ALL_OUTPUTS != 0 )
//	{	// remain in loop as long as any output is still high
//		if( TMR2 = MVorne  ) PulseVorne  = 0;
//		if( TMR2 = MHinten ) PulseHinten = 0;
//		if( TMR2 = MLinks  ) PulseLinks  = 0;
//		if( TMR2 = MRechts ) PulseRechts = 0;
//	}

	GIE = 1;	// Re-enable interrupt
#endif

#if defined ESC_X3D || defined ESC_HOLGER

#ifndef DEBUG
	CAM_PULSE_ON;	// now turn camera servo pulses on too
#endif

// in X3D- and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL
#ifdef ESC_X3D
	EscI2CStart();
	niret = SendEscI2CByte(0x10);	// one command, 4 data bytes
	if( niret == 0 )
	{
		SendEscI2CByte(MVorne); // for all motors
		SendEscI2CByte(MHinten);
		SendEscI2CByte(MLinks);
		SendEscI2CByte(MRechts);
		EscI2CFlags |= 0x10;
	}
	EscI2CStop();
#endif	// ESC_X3D

#ifdef ESC_HOLGER
	EscI2CStart();
	niret = SendEscI2CByte(0x52);	// one cmd, one data byte per motor
	if( niret == 0 )
	{
		SendEscI2CByte(MVorne); // for all motors
		EscI2CFlags |= 0x01;
	}
	EscI2CStop();

	EscI2CStart();
	niret = SendEscI2CByte(0x54);
	if( niret == 0 )
	{
		SendEscI2CByte(MHinten);
		EscI2CFlags |= 0x08;
	}
	EscI2CStop();

	EscI2CStart();
	niret = SendEscI2CByte(0x58);
	if( niret == 0 )
	{
		SendEscI2CByte(MLinks);
		EscI2CFlags |= 0x02;
	}
	EscI2CStop();

	EscI2CStart();
	niret = SendEscI2CByte(0x56);
	if( niret == 0 )
	{
		SendEscI2CByte(MRechts);
		EscI2CFlags |= 0x04;
	}
	EscI2CStop();
#endif	// ESC_HOLGER

#endif	// ESC_X3D or ESC_HOLGER

	while( TMR0 < 0x100-3-16 ) ; // wait for 2nd TMR0 near overflow

	GIE = 0;	// Int wieder sperren, wegen Jitter

	while( T0IF == 0 ) ;	// wait for 2nd overflow (2 ms)

	ME = MCamRoll;
	MT = MCamNick;

#ifndef DEBUG
// This loop is exactly 16 cycles long
// under no circumstances should the loop cycle time be changed
#asm
	BCF	RP0		// clear all bank bits
	BCF	RP1
OS001
	MOVF	PORTB,W
	ANDLW	0x30		// output ports 4 and 5
	BTFSC	Zero_
	GOTO	OS002		// stop if all 2 outputs are 0

	DECFSZ	MT,f
	GOTO	OS003

	BCF	PulseCamRoll
OS003
	DECFSZ	ME,f
	GOTO	OS004

	BCF	PulseCamNick
OS004
#endasm
	nop2();
	nop2();
	nop2();
#asm
	GOTO	OS001
OS002
#endasm
#endif	// DEBUG

	GIE = 1;	// re-enable interrupt
	while( T0IF == 0 ) ;	// wait for 3rd TMR2 overflow
}

#ifdef ESC_HOLGER
const char page2 SerHolFOK[] = "Front ESC OK\r\n";
const char page2 SerHolHOK[] = "Rear  ESC OK\r\n";
const char page2 SerHolLOK[] = "Left  ESC OK\r\n";
const char page2 SerHolROK[] = "Right ESC OK\r\n";
#endif
#ifdef ESC_X3D
const char page2 SerX3DOK[] = "X3D ESC OK\r\n";
#endif

void MySendComText(const char *pch)
{
	while( *pch != '\0' )
	{
		SendComChar(*pch);
		pch++;
	}
}

void TestServos(void)
{
	uns8 nii;
// drive outputs slowly up, then stop suddenly

	MCamRoll = 
	MCamNick = _Maximum;

	while(1)
	{
		if( MVorne >= _Maximum )
			break;

		for( nii=0; nii < 5; nii++ )
		{
			TimeSlot = 10;
			while( TimeSlot > 0 )
			{
				while( T0IF == 0 ) 	// 1024us wait
				{	
					if( RecvComChar() != '\0' )
						goto EmergStop;	// emergency stop
				}
				T0IF = 0;
				TimeSlot--;
			}
			OutSignals();
		}
		MVorne++;
		MLinks++;
		MRechts++;
		MHinten++;
		MCamRoll--;
		MCamNick--;
	}
EmergStop:
	MVorne =
	MLinks =
	MRechts =
	MHinten = _Minimum;

	MCamRoll = 
	MCamNick = _Neutral;
#ifdef ESC_HOLGER
	if( EscI2CFlags & 0x01 )
		MySendComText(SerHolFOK);
	if( EscI2CFlags & 0x02 )
		MySendComText(SerHolHOK);
	if( EscI2CFlags & 0x04 )
		MySendComText(SerHolLOK);
	if( EscI2CFlags & 0x08 )
		MySendComText(SerHolROK);
#endif
#ifdef ESC_X3D
	if( EscI2CFlags & 0x10 )
		MySendComText(SerX3DOK);
#endif
}
