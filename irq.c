// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

// Interrupt Routines

#include "uavx.h"

#define MIN_PPM_SYNC_PAUSE 3750  	// 3000 *0.8us = 2.4ms // was 6250 5mS

// no less than 1500 ?? 

void SyncToTimer0AndDisableInterrupts(void);
void ReceivingGPSOnly(uint8);
void InitTimersAndInterrupts(void);
uint24 mSClock(void);
void low_isr_handler(void);
void high_isr_handler(void);

#pragma udata clocks
volatile uint24	mS[CompassUpdate+1];
volatile uint24 PIDUpdate;
#pragma udata

#pragma udata access isrvars
near volatile uint24 MilliSec;
near i16u 	PPM[CONTROLS];
near uint8 	PPM_Index, NoOfControls;
near int24 	PrevEdge, CurrEdge;
near i16u 	Width, Timer0;
near int24 	PauseTime;
near uint8 	RxState;

near uint8 	ll, ss, tt, RxCh;
near uint8 	RxCheckSum, TxCheckSum, GPSCheckSumChar, GPSTxCheckSum;
near uint8 	RxQTail, RxQHead, TxQTail, TxQHead;
near boolean WaitingForSync;

#pragma udata

int8	SignalCount;
uint16	RCGlitches;

void SyncToTimer0AndDisableInterrupts(void)
{
	do { GetTimer0; } while ( Timer0.u16 < INT_LATENCY ); 
	// one interrupt service which may occur between the check and the disable! 	
	DisableInterrupts;

	while( !INTCONbits.TMR0IF ) ;		// now wait overflow
	FastWriteTimer0(TMR0_1MS);	
} // SyncToTimer0AndDisableInterrupts

void ReceivingGPSOnly(boolean r)
{
	if ( r != F.ReceivingGPS ) {
		PIE1bits.RCIE = false;
		F.ReceivingGPS = r;

		if ( F.ReceivingGPS )		
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_LOW, _B9600);
		else
			OpenUSART(USART_TX_INT_OFF&USART_RX_INT_OFF&USART_ASYNCH_MODE&
				USART_EIGHT_BIT&USART_CONT_RX&USART_BRGH_HIGH, _B38400);
   		PIE1bits.RCIE = r;

		DisableInterrupts;
		RxQTail = RxQHead = 0;
		EnableInterrupts;
	}
} // ReceivingGPSOnly

void InitTimersAndInterrupts(void)
{
	static uint8 i;

	OpenTimer0(TIMER_INT_OFF&T0_16BIT&T0_SOURCE_INT&T0_PS_1_16); 	

	OpenTimer1(T1_16BIT_RW&TIMER_INT_OFF&T1_PS_1_8&T1_SYNC_EXT_ON&T1_SOURCE_CCP&T1_SOURCE_INT);
	OpenCapture1(CAPTURE_INT_ON & C1_EVERY_FALL_EDGE); 	// capture mode every falling edge

	CCP1CONbits.CCP1M0 = true; //DoRxPolarity(); 

	RxQHead = RxQTail = TxQHead = TxQTail = RxCheckSum = 0;

	MilliSec = 0;
	for (i = StartTime; i<= (uint8)CompassUpdate; i++)
		mS[i] = 0;

	INTCONbits.PEIE = true;	
	INTCONbits.TMR0IE = true; 

   	ReceivingGPSOnly(false);
} // InitTimersAndInterrupts

uint24 mSClock(void) { // MUST make locked accesses to the millisecond clock
	static int24 m;

	DisableInterrupts;
	m = MilliSec;
	EnableInterrupts;
	return(m);
} // mSClock

#pragma interrupt low_isr_handler
void low_isr_handler(void) {
	return;
} // low_isr_handler

#pragma interrupt high_isr_handler
void high_isr_handler(void) {

	if( PIR1bits.CCP1IF ) {	// An Rx PPM pulse edge has been detected
		CurrEdge = CCPR1;
		if ( CurrEdge < PrevEdge )
			PrevEdge -= (int24)0x00ffff;		// Deal with wraparound

		Width.i16 = (int16)(CurrEdge - PrevEdge);
		PrevEdge = CurrEdge;		

		if ( Width.i16 > MIN_PPM_SYNC_PAUSE ) {	// A pause  > 5ms
			PPM_Index = 0;						// Sync pulse detected - next CH is CH1
			F.RCFrameOK = true;
			F.RCNewValues = false;
			PauseTime = Width.i16;	
		} else 
			if (PPM_Index < NoOfControls) {
		//		if ( (Width.i16 >= 1250 ) && (Width.i16 <= 2500) ) // Width in 0.8uS ticks 	
					PPM[PPM_Index].i16 = (int16) Width.i16 - 1250;		
		//		else {
					// preserve old value i.e. default hold
		//			RCGlitches++;
		//			F.RCFrameOK = false;
		//		}
				
				PPM_Index++;
				// MUST demand rock solid RC frames for autonomous functions not
				// to be cancelled by noise-generated partially correct frames
				if ( PPM_Index == NoOfControls ) {
					if ( F.RCFrameOK ) {
						F.RCNewValues = true;
 						SignalCount++;
					} else {
						F.RCNewValues = false;
						SignalCount -= RC_GOOD_RATIO;
					}

					SignalCount = Limit1(SignalCount, RC_GOOD_BUCKET_MAX);
					F.Signal = SignalCount > 0;

					if ( F.Signal )
						mS[LastValidRx] = MilliSec;
					mS[RCSignalTimeout] = MilliSec + RC_SIGNAL_TIMEOUT_MS;
				}
			}

		if ( !F.UsingCompoundPPM )						
			CCP1CONbits.CCP1M0 ^= 1;

		PIR1bits.CCP1IF = false;
	}

	if ( PIR1bits.RCIF & PIE1bits.RCIE ) { // RCIE enabled for GPS
		if ( RCSTAbits.OERR | RCSTAbits.FERR ) {
			RxCh = RCREG; // flush
			RCSTAbits.CREN = false;
			RCSTAbits.CREN = true;
		} else { // PollGPS in-lined to avoid EXPENSIVE context save and restore within irq
			RxQTail = (RxQTail + 1) & RX_BUFF_MASK;
			RxQ[RxQTail] = RCREG; 		
		}
		if ( Armed && ( P[TelemetryType] == GPSTelemetry) ) // piggyback GPS telemetry on GPS Rx
			TXREG = RxCh;
	
		PIR1bits.RCIF = false;
	}

	if ( INTCONbits.T0IF ) {
		Timer0.u16 = TMR0_1MS;
		TMR0H = Timer0.b1;
		TMR0L = Timer0.b0;

		MilliSec++;

		WaitingForSync = MilliSec < PIDUpdate;

		if ( F.Signal && (MilliSec > mS[RCSignalTimeout]) ) {
			F.Signal = false;
			SignalCount = -RC_GOOD_BUCKET_MAX;
		}

		if ( Armed  && ( P[TelemetryType] !=  GPSTelemetry ) )
			if (( TxQHead != TxQTail) && PIR1bits.TXIF ) {
				TXREG = TxQ[TxQHead];
				TxQHead = (TxQHead + 1) & TX_BUFF_MASK;
			}
 
		INTCONbits.TMR0IF = false;	
	}

} // high_isr_handler
	
#pragma code high_isr = 0x08
void high_isr (void) {
  _asm goto high_isr_handler _endasm
} // high_isr
#pragma code

#pragma code low_isr = 0x18
void low_isr (void) {
  _asm goto low_isr_handler _endasm
} // low_isr
#pragma code

