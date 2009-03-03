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

#pragma	config OSC=HS, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC 

#include "pu-test.h"
#include "bits.h"

// The globals


#pragma udata mainvars
uns8 ConfigReg;
uns8	LedShadow;	// shadow register
uns8	Flags[8], Flags2[8];
uns16	CurrK1,CurrK2,CurrK3,CurrK4,CurrK5,CurrK6,CurrK7;
uns16	PauseTime;
uns8	MFront, MBack, MLeft, MRight;
uns8	MCamRoll, MCamPitch;
uns8	EscI2CFlags;
int8	TimeSlot;
uns8	mSTick;
#pragma udata

void main(void)
{
	uns8	i;

	DisableInterrupts;

	// general ports setup
	TRISA = 0b00111111;	// all inputs
	ADCON1 = 0b00000010;	// uses 5V as Vref

	PORTB = 0b11000000;		// all outputs to low, except RB6 & 7 (I2C)!
	TRISB = 0b01000000;	// all servo and LED outputs
	PORTC = 0b01100000;		// all outputs to low, except TxD and CS
	TRISC = 0b10000100;	// RC7, RC2 are inputs
	INTCON2bits.RBPU  = 1;			// disable weak pullups
	SSPSTATbits.CKE = 1;		// default I2C - enable SMBus thresholds for 3.3V LISL

	LedShadow = 0;
    ALL_LEDS_OFF;

	// setup serial port for 8N1
	TXSTA = 0b00100100;	// async mode, BRGH = 1
	RCSTA = 0b10010000;	// receive mode
	SPBRG = _B38400;
	//SPBRG = _B115200;
	//SPBRG = _B230400;

	//W = RCREG;			// be sure to empty FIFO
	
	OpenTimer0(TIMER_INT_OFF&T0_8BIT&T0_SOURCE_INT&T0_PS_1_16);
	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);

	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge
	CCP1CONbits.CCP1M0 = NegativePPM;

	OpenTimer2(TIMER_INT_OFF&T2_PS_1_16&T2_POST_1_16);		
	PR2 = TMR2_5MS;		// set compare reg to 9ms

	// setup flags register
	for ( i = 0; i<8; i++ )
		Flags2[i] = Flags[i] = 0;

	LedShadow = 0;

    ALL_LEDS_OFF;
	LedBlue_ON;

	_NoSignal = 1;		// assume no signal present

	LedRed_ON;		// red LED on
	Delay100mS(1);	// wait 1/10 sec until LISL is ready to talk
	IsLISLactive();
#ifdef ICD2_DEBUG
	_UseLISL = 1;	// because debugger uses RB7 (=LISL-CS) :-(
#endif
		
	INTCONbits.PEIE = 1;		// enable peripheral interrupts

	ConfigReg = 0;
	CurrK1 =
	CurrK2 =
	CurrK3 =
	CurrK4 =
	CurrK5 =
	CurrK6 =
	CurrK7 = 0xFFFF;
	PauseTime = 0;
	MFront =
	MLeft =
	MRight =
	MBack = _Minimum;

	MCamRoll = 
	MCamPitch = _Neutral;

	// send hello text to serial COM
	Delay100mS(1);	// just to see the output after powerup

	EnableInterrupts;

	ShowSetup(1);

	while(1)
	{
		// turn red LED on of signal missing or invalid, green if OK
		if( _NoSignal )
		{
			LedRed_ON;
			LedGreen_OFF;
		}
		else
		{
			LedGreen_ON;
			LedRed_OFF;
		}
		// output servo values
		TimeSlot = 10;
		while( TimeSlot > 0 )
		{
			while( INTCONbits.T0IF == 0 ) 	// 1024us wait
			{	
				ProcessComCommand();
			}
			INTCONbits.T0IF = 0;
			TimeSlot--;
		}
		OutSignals();
	}
}

