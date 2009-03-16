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

void Delay1mS(uint16 d)
{ 	// Timer0 interrupt at 1mS must be running
	uint8 i;
	uint8 T0IntEn;

	T0IntEn = INTCONbits.TMR0IE;	// not protected?
	INTCONbits.TMR0IE = false;

	// if d is 1 then delay can be less than 1mS due to 	
	for (i=0; i<d; i++)
	{						// compromises ClockMilliSec;
		while ( !INTCONbits.TMR0IF ) {};
		INTCONbits.TMR0IF = 0;
	}

	INTCONbits.TMR0IE = T0IntEn;

} // Delay1mS

// wait blocking for "dur" * 0.1 seconds
// Motor and servo pulses are still output every 10ms
void Delay100mSWithOutput(uint16 dur)
{ // Timer0 Interrupts must be off 
	uint16 i, j;
	uint8 T0IntEn;

	T0IntEn = INTCONbits.TMR0IE;	// not protected?
	INTCONbits.TMR0IE = false;

	for(i = 0; i < dur*10; i++)
		{
			for (j = 0; j<8; j++)
			{
				while ( !INTCONbits.TMR0IF ) {};
				INTCONbits.TMR0IF = 0;
			}
			OutSignals(); // 1-2 ms Duration
			// break loop if a serial command is in FIFO
			if( PIR1bits.RCIF )
			{
				INTCONbits.TMR0IE = T0IntEn;
				return;
			}
		}
	INTCONbits.TMR0IE = T0IntEn;
} // Delay100mSWithOutput

void nop2()
{
	Delay1TCY();
	Delay1TCY();
} // nop2

int16 SRS16(int16 x, uint8 s)
{
	return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS16

#ifdef DEBUGOUT
// output a value on connector K5
// for observation via an oscilloscope
// with 8 narrow (bit=0) or broad (bit=1) pulses
// MSB first
void Out(uint8 l)
{
	for(i=0; i<8; i++)
	{
		PORTBbits.RB5=1;
		if(l & 0x80)
		{
			nop2();
			nop2();
			nop2();
		}
		PORTBbits.RB5=0;
		if(!(l & 0x80))
		{
			nop2();
			nop2();
			nop2();
		}
		l<<=1;
	}
} // Out
#endif

#ifdef DEBUGOUTG
// output a signed value in a graphic manner on connector K5
// use an oscilloscope to observe
//     Trigger    ______________
// ____|_________|_____|________|_____|
//      128  negative  0  positive  127
void OutG(uint8 l)
{
	uint8 v;

	PORTB.5=1;
	PORTB.5=0;
	for(v=128; v!=1; v++)	// -128 .. 0
	{
		if(v==l)
			PORTB.5=1;
	}
	PORTB.5^=1;
	for(v=1; v!=128; v++)	// +1 .. +127
	{
		if(v==l)
			PORTB.5=0;
	}

	PORTB.5=0;
} // OutG
#endif

void InitPorts(void)
{
	// general ports setup
	TRISA = 0b00111111;								// all inputs
	ADCON1 = 0b00000010;							// uses 5V as Vref

	PORTB = 0b11000000;								// all outputs to low, except RB6 & 7 (I2C)!
	TRISB = 0b01000000;								// all servo and LED outputs
	PORTC = 0b01100000;								// all outputs to low, except TxD and CS
	TRISC = 0b10000100;								// RC7, RC2 are inputs

	SSPSTATbits.CKE = true;							// low logic threshold for LISL
	INTCON2bits.NOT_RBPU = true;					// enable weak pullups
} // InitPorts

// resets all important variables - Do NOT call that while in flight!
void InitArrays(void)
{
	MFront = MLeft = MRight = MBack = _Minimum;
	MCamPitch = MCamRoll = _Neutral;

	_Flying = false;
	REp = PEp = YEp = 0;
	
	Rp = Pp = Vud = VBaroComp = 0;
	
	UDSum = 0;
	LRIntKorr = FBIntKorr = 0;
	YawSum = RollSum = PitchSum = 0;

	BaroRestarts = 0;
	RCGlitchCount = 0;
} // InitArrays


void CheckAlarms(void)
{
	int16 NewBatteryVolts;

	NewBatteryVolts = ADC(ADCBattVoltsChan, ADCVREF5V) >> 3; 
	BatteryVolts = SoftFilter(BatteryVolts, NewBatteryVolts);
	_LowBatt =  (BatteryVolts < (int16) LowVoltThres) & 1;

	if( _LowBatt ) // repeating beep
	{
		if( BlinkCount < BLINK_LIMIT/2 )
		{
			Beeper_ON;
			LedRed_ON;
		}
		else
		{
			Beeper_OFF;
			LedRed_OFF;
		}	
	}
	else
	if ( _LostModel ) // 2 beeps with interval
		if( (BlinkCount < (BLINK_LIMIT/2)) && ( BlinkCycle < (BLINK_CYCLES/4 )) )
		{
			Beeper_ON;
			LedRed_ON;
		}
		else
		{
			Beeper_OFF;
			LedRed_OFF;
		}	
	else
	if ( _BaroRestart ) // 1 beep with interval
		if( (BlinkCount < (BLINK_LIMIT/2)) && ( BlinkCycle == 0 ) )
		{
			Beeper_ON;
			LedRed_ON;
		}
		else
		{
			Beeper_OFF;
			LedRed_OFF;
		}	
	else
	{
		Beeper_OFF;				
		LedRed_OFF;
	}

} // CheckAlarms

void LedGame(void)
{
	if( --LedCount == 0 )
	{
		LedCount = ((255-IGas)>>3) +5;	// new setup
		if( _Hovering )
		{
			AUX_LEDS_ON;	// baro locked, all aux-leds on
		}
		else
		if( LedShadow & LedAUX1 )
		{
			AUX_LEDS_OFF;
			LedAUX2_ON;
		}
		else
		if( LedShadow & LedAUX2 )
		{
			AUX_LEDS_OFF;
			LedAUX3_ON;
		}
		else
		{
			AUX_LEDS_OFF;
			LedAUX1_ON;
		}
	}
} // LedGame

void DoPIDDisplays()
{
	if ( IntegralTest )
	{
		ALL_LEDS_OFF;
		if( (int8)(RollSum>>8) > 0 )
			LedRed_ON;
		else
			if( (int8)(RollSum>>8) < -1 )
				LedGreen_ON;

		if( (int8)(PitchSum>>8) >  0 )
			LedYellow_ON;
		else
			if( (int8)(PitchSum>>8) < -1 )
				LedBlue_ON;
	}
	else
		if( CompassTest )
		{
			ALL_LEDS_OFF;
			if( CurDeviation > 0 )
				LedGreen_ON;
			else
				if( CurDeviation < 0 )
					LedRed_ON;
			if( AbsDirection > COMPASS_MAX )
				LedYellow_ON;
		}
} // DoPIDDisplays

void UpdateBlinkCount(void)
{
	if( BlinkCount == 0 )
	{
		BlinkCount = BLINK_LIMIT;
		if ( BlinkCycle == 0)
			BlinkCycle = BLINK_CYCLES;
		BlinkCycle--;
	}
	BlinkCount--;
} // UpdateBlinkCount

void SendLeds(void)
{
	uint8	i, s;

	i = LedShadow;
	SPI_CS = SEL_LEDS;	
	SPI_IO = 0;	// SDA is output
	SPI_SCL = 0;	// because shift is on positive edge
	
	for(s=8; s!=0; s--)
	{
		if( i & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		i<<=1;
		SPI_SCL = 1;
		SPI_SCL = 0;
	}

	PORTCbits.RC1 = 1;
	PORTCbits.RC1 = 0;	// latch into drivers
}

void SwitchLedsOn(uint8 l)
{
	LedShadow |= l;
	SendLeds();
} // SwitchLedsOn

void SwitchLedsOff(uint8 l)
{
	LedShadow &= ~l;
	SendLeds();
} // SwitchLedsOff


