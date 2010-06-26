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

#pragma	config OSC=HSPLL, WDT=OFF, PWRT=ON, MCLRE=OFF, LVP=OFF, PBADEN=OFF, CCP2MX = PORTC

#include "uavx.h"

#ifdef CLOCK_16MHZ
#define MIN_PPM_SYNC_PAUSE 2500  	// 2500 *2us = 5ms
#else // CLOCK_40MHZ
#define MIN_PPM_SYNC_PAUSE 6250  	// 6250 *0.8us = 5ms
#endif //  CLOCK_16MHZ
// no less than 1500

void InitTimersAndInterrupts(void);
void low_isr_handler(void);
void high_isr_handler(void);

#pragma udata access isrvars
near int24 	PrevEdge, CurrEdge;
near uint8 	Intersection, PrevPattern, CurrPattern;
near i16u 	PPM[MAX_CONTROLS];
near int8 	PPM_Index;
near int24 	PauseTime;
#pragma udata

Flags F;
int8	SignalCount;
uint16	RCGlitches;

#pragma udata rxchan
struct {
	int16 Width;
	int16 PrevEdge;
	boolean High;
} RxChannel[4];	
#pragma udata

#pragma udata rxq
#define RXQ_BUFF_SIZE 32
#define RXQ_BUFF_MASK (RXQ_BUFF_SIZE-1)
struct {
	uint8 Head, Tail;
	struct {
		int16 T;
		uint8 P;
		} B[RXQ_BUFF_SIZE];
	} RxQ;	

boolean RCSignal, RCNewValues;

void InitMisc(void)
{
	static uint8 i;

	TRISA = 0b00111111;							// all inputs
	ADCON1 = 0b00000010;						// Vref used for Rangefinder

	PORTB = 0b00000000;	
	TRISB = 0b1111111;	

	PORTC = 0b01100000;							// all outputs to low, except TxD and CS
	TRISC = 0b10000100;							// RC7, RC2 are inputs	

	F.AllFlags = 0;
	F.TxToBuffer = true;
} // InitMisc

void InitTimersAndInterrupts(void)
{
	static uint8 i;

	OpenTimer0(TIMER_INT_OFF&T0_16BIT&T0_SOURCE_INT&T0_PS_1_16);	
	OpenTimer1(T1_8BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);
	//OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge

	INTCONbits.RBIE = true; // interrupt on change PORTB bits 4..7 ??? zzz

	TxQ.Head = TxQ.Tail = 0;
	RxQ.Head = RxQ.Tail = CurrEdge = PrevEdge = 0;

	PrevPattern = 0;
	for (i = 0; i < 4; i++)
	{
		RxChannel[i].Width = RxChannel[i].PrevEdge = 0; 
		RxChannel[i].High = false;
	} 
} // InitTimersAndInterrupts

#pragma code high_isr = 0x08
void high_isr (void)
{
  _asm goto high_isr_handler _endasm
} // high_isr
#pragma code

#pragma code low_isr = 0x18
void low_isr (void)
{
  _asm goto low_isr_handler _endasm
} // low_isr
#pragma code

#pragma interrupt low_isr_handler
void low_isr_handler(void)
{
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void)
{
	if( INTCONbits.RBIF )
	{
		CurrEdge = CCPR1;
		CurrPattern = PORTB;
		if ( CurrEdge < PrevEdge )
			PrevEdge -= (int24)0x00ffff;		// Deal with wraparound

		RxQ.Tail = (RxQ.Tail+1) & RXQ_BUFF_MASK;
		RxQ.B[RxQ.Tail].T = CurrEdge - PrevEdge;
		RxQ.B[RxQ.Tail].P = CurrPattern;
		CurrEdge = PrevEdge;

		INTCONbits.RBIF = false;
	}
} // high_isr_handler
	
void main(void)
{
	static uint8	b, c, m;

	DisableInterrupts;
	InitMisc();
	OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);		
	InitTimersAndInterrupts();

	F.TxToBuffer = true;

	EnableInterrupts;
		
	while( true )
	{
		// check for boot

		while ( RxQ.Head != RxQ.Tail )
		{
			RxQ.Head = (RxQ.Head + 1) & RXQ_BUFF_MASK;
			CurrPattern = RxQ.B[RxQ.Head].P;
			CurrEdge = RxQ.B[RxQ.Head].T;
			
			Intersection = PrevPattern ^ CurrPattern;
			m = 0x10;
			for ( c = 0; c < 4; c++ )
			{ 
				if ( Intersection & m )
				{
					if ( RxChannel[c].High )
					{
						RxChannel[c].Width = CurrEdge - RxChannel[c].PrevEdge;
						RxChannel[c].High = false;
TxVal32((int32)c,0,' '); TxVal32((int32)RxChannel[c].Width,0,0); TxNextLine();
					}
					else
						RxChannel[c].High = true;
					RxChannel[c].PrevEdge = CurrEdge;	
				}
				m >>= 1;
			}
	
			PrevPattern = CurrPattern;
		}

		if (( TxQ.Head != TxQ.Tail) && PIR1bits.TXIF )
		{
			TXREG = TxQ.B[TxQ.Head];
			TxQ.Head = (TxQ.Head + 1) & TX_BUFF_MASK;
		}

		F.Signal = F.RCNewValues = false;
		// honour I2C request ????

	}
} // main


