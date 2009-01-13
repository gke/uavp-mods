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

// SRS16/32 are used for divisions by powers of 2. A simple shift 
// does not (with C18 at least) preserve the sign on negative 
// arguments even if declared signed hence the conditionals.
// It is defined as a function as a simple macro would result in 
// possibly quite complicated expressions for "x" being expanded
// inline and not recognised as common sub expressions by the
// compiler.
// Typically divisions take 200uS vs SRS 20uS on a 18F2520 @ 16MHz.
int32 SRS32(int32 x, uint8 s)
{
	return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS32

int16 SRS16(int16 x, uint8 s)
{
	return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS16

#ifndef USE_MACROS

int16	Limit(int16 i, int16 l, int16 u)
{
	return((i<l) ? l : ((i>u) ? u : i));
} // Limit

int16 Max(int16 i,int16 j)
{
 		return((i<j) ? j : ((i>j) ? j : i));
} // Max

int16 DecayBand(int16 i, int16 l, int16 u, int16 d)
{
	return ((i<l) ? i+d : ((i>u) ? i-d : i));
} // DecayBand

int16 Decay(int16 i)
{
	return((i<0) ? i+1 : ((i>0) ? i-1 : i));
} // Decay

#endif

// wait blocking for "dur" * 0.1 seconds
// Motor and servo pulses are still output every 10ms
void Delay100mSec(uint8 delay)
{
	TimerMilliSec = ClockMilliSec + delay * 100;
	while (ClockMilliSec < TimerMilliSec)
	{
#ifndef BOOTONLY
		if ( TimeSlot <= 0)
		{
			TimeSlot = NoOfTimeSlots; 
			OutSignals(); 							// 3 ms duration
		}

		// break loop if a serial command is in FIFO
		if( PIR1bits.RCIF )
			return;
#endif
	}
} // Delay100mSec

void InitPorts()
{
	// general ports setup
	TRISA = 0b00111111;								// all inputs
	ADCON1 = 0b00000010;							// uses 5V as Vref

	PORTB = 0b11000000;								// all outputs to low, except RB6 & 7 (I2C)!
	TRISB = 0b01000000;								// all servo and LED outputs
	PORTC = 0b01100000;								// all outputs to low, except TxD and CS
	TRISC = 0b10000100;								// RC7, RC2 are inputs
	INTCON2bits.NOT_RBPU = true;					// enable weak pullups
} // InitPorts

// _LowBatt flag is set if battery voltage is below threshold.
// Filter battery volts to remove ADC/Motor spikes and set _LowBatt alarm accordingly 
void GetBatteryVolts(void)
{
	int16 NewBatteryVolts;

	NewBatteryVolts = ADC(ADCBattVoltsChan, ADCVREF5V) >> 3; 
	BatteryVolts = SoftFilter(BatteryVolts, NewBatteryVolts);
	_LowBatt =  (BatteryVolts < (int16) LowVoltThres) & 1;
} // GetBatteryVolts

void CheckLowBattery()
{
	GetBatteryVolts();
	if( _LowBatt )
	{
		if( BlinkCount < BLINK_LIMIT/2 )
		{
			Beeper_OFF;
			LedRed_OFF;
		}
		else
		{
			Beeper_ON;
			LedRed_ON;
		}
	}
	else
	{
		Beeper_OFF;
		LedRed_OFF;
	}
} // CheckLowBattery

void SwitchLedsOn(uint8 LEDs)
{
	LedShadow |= LEDs;
	SendLeds();
} // SwitchLedsOn

void SwitchLedsOff(uint8 LEDs)
{
	LedShadow &= ~LEDs;
	SendLeds();
} // SwitchLedsOff

void LEDGame(void)
{
	if( --LedCount == 0 )
	{
		LedCount = 255-IGas;				// new setup
		LedCount >>= 3;
		LedCount += 5;
		if( _UseBaro && (ThrDownCount == 0) )
		{
			AUX_LEDS_ON;					// baro locked, all aux-leds ON
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
} // LEDGame

int8 ReadEE(uint8 addr)
{
	int8 b;

	EEADR = addr;
	EECON1bits.EEPGD = false;
	EECON1bits.RD = true;
	b=EEDATA;
	EECON1 = 0;
	return(b);	
} // ReadEE

void WriteEE(uint8 addr, int8 d)
{
	int8 rd;
	
	rd = ReadEE(addr);
	if ( rd != d )						// avoid redundant writes
	{
		EEDATA = d;				
		EEADR = addr;
		EECON1bits.EEPGD = false;
		EECON1bits.WREN = true;

		DisableInterrupts;
		EECON2 = 0x55;
		EECON2 = 0xaa;
		EECON1bits.WR = true;
		while(EECON1bits.WR);
		EnableInterrupts;

		EECON1bits.WREN = false;
	}

} // WriteEE

void ReadParametersEE(void)
{
	int8 *p; 
	uint16 addr;

	//LedBlue_ON;

	if( IK5 > _Neutral )
		addr = _EESet2;	
	else
		addr = _EESet1;	

	for(p = &FirstProgReg; p <= &LastProgReg; p++)
		*p = ReadEE(addr++);

	TimeSlot = Limit(NoOfTimeSlots, 4, 22);

	//LedBlue_OFF;

} // ReadParametersEE

void WriteParametersEE(uint8 s)
{
	int8 *p;
	uint8 b;
	uint16 addr;
	
	if( s == 1 )
		addr = _EESet1;	
	else
		addr = _EESet2;

	p = &FirstProgReg; 
	while ( p <= &LastProgReg)
		WriteEE(addr++, *p++);
} // WriteParametersEE

void DoDebugTraces()
{
#ifdef DEBUG_SENSORS
	/*
	1 AbsDirection
	2 VBaroComp
	3 VBaroCompSum effective density altitude
	4 Rp left-right accel
	5 Np fore-aft accel
	6 Tp vertical accel
	7 RollRate Roll rate
	8 NickRate Pitch rate
	9 TE Yaw rate
	10 RollSum Roll angle
	11 NickSum Pitch angle
	12 YawSum Yaw angle
	*/
//	if( IGas > _ThresStart )
	{
#ifdef READABLE
		TxVal((int32)Compass, 0, ';');
		TxVal((int32)AbsDirection, 0, ';');
		TxVal((int32)CurrDeviation, 0, ';');

		TxVal((int32)VBaroComp, 0, ';');
		TxVal((int32)BaroCompSum, 0, ';');

		// accelerations
		TxVal((int32)(ACCSIGN_X * Ax - NeutralLR), 0, ';');
		TxVal((int32)(ACCSIGN_Z * Az - NeutralFB), 0, ';');
		TxVal((int32)(ACCSIGN_Y * Ay - NeutralUD), 0, ';');

		// rates
		TxVal((int32)RollRate, 0, ';');
		TxVal((int32)PitchRate, 0, ';');
		TxVal((int32)YawRate, 0, ';');

		// angles
		TxVal(RollAngle, 0, ';');
		TxVal(PitchAngle, 0, ';');
		TxVal(YawAngle, 0, ';');

		TxVal((int32)MFront, 0, ';');
		TxVal((int32)MBack, 0, ';');
		TxVal((int32)MLeft, 0, ';');
		TxVal((int32)MRight, 0, ';');
#else
		SendComValH(AbsDirection);
		SendComChar(';');

		SendComValH(VBaroComp);
		SendComChar(';');
		SendComValH16(BaroCompSum);
		SendComChar(';');

		// accelerations
		SendComValH16(ACCSIGN_X * Ax);
		SendComChar(';');
		SendComValH16(ACCSIGN_Z * Az);
		SendComChar(';');
		SendComValH16(ACCSIGN_Y * Ay);
		SendComChar(';');

		// rates
		SendComValH16(RollRate);
		SendComChar(';');
		SendComValH16(PitchRate);
		SendComChar(';');
		SendComValH(YawRate);
		SendComChar(';');

		// angles
		SendComValH16(RollAngle);
		SendComChar(';');
		SendComValH16(PitchAngle);
		SendComChar(';');
		SendComValH16(YawAngle);
		SendComChar(';');

#ifdef SIMULATION
		SendComValH16(MFront);
		SendComChar(';');
		SendComValH16(Mback);
		SendComChar(';');
		SendComValH16(MLeft);
		SendComChar(';');
		SendComValH16(MRight);
		SendComChar(';');
#endif // SIMULATION

#endif // READABLE
		SendComNextLine();

	};
#else
#ifdef DEBUG_MOTORS

	if( _Flying ) //&& IsSet(CamNickFactor,4) )
	{
		SendComValU(IGas);
		SendComChar(';');
		SendComValS(IRoll);
		SendComChar(';');
		SendComValS(INick);
		SendComChar(';');
		SendComValS(ITurn);
		SendComChar(';');
		SendComValU(MVorne);
		SendComChar(';');
		SendComValU(MHinten);
		SendComChar(';');
		SendComValU(MLinks);
		SendComChar(';');
		SendComValU(MRechts);
		SendComChar(';');
#ifdef DEBUG_CAMERAS
		SendComValU(IK6);
		SendComChar(';');
		SendComValU(MCamRoll);
		SendComChar(';');
		SendComValU(IK7);
		SendComChar(';');
		SendComValU(MCamNick);
		SendComChar(';');
#endif
		SendComNextLine();
	}
#endif
#endif
} // DoDebugTraces



