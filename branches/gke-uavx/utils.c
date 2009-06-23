// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
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

#include "uavx.h"

// Prototypes

void Delay1mS(int16);
void Delay100mSWithOutput(int16);
int16 SRS16(int16, uint8);
int32 SRS32(int32, uint8);
void InitPorts(void);
void InitArrays(void);

int16 ConvertGPSToM(int16);
int16 ConvertMToGPS(int16);

int8 ReadEE(uint8);
void ReadParametersEE(void);
void WriteEE(uint8, int8);
void WriteParametersEE(uint8);
void UpdateWhichParamSet(void);

int16 Make2Pi(int16);
int16 Table16(int16, const int16 *);
int16 int16sin(int16);
int16 int16cos(int16);
int16 int16atan2(int16, int16);
int16 int16sqrt(int16);

void SendLEDs(void);
void SwitchLEDsOn(uint8);
void SwitchLEDsOff(uint8);
void LEDGame(void);
void CheckAlarms(void);

void DumpTrace(void);

void Delay1mS(int16 d)
{ 	// Timer0 interrupt at 1mS must be running
	int16 i;
	uint8 T0IntEn;

	T0IntEn = INTCONbits.TMR0IE;	// not protected?
	INTCONbits.TMR0IE = false;

	// if d is 1 then delay can be less than 1mS due to 	
	for (i=d; i; i--)
	{						// compromises ClockMilliSec;
		while ( !INTCONbits.TMR0IF ) {};
		INTCONbits.TMR0IF = 0;
	}

	INTCONbits.TMR0IE = T0IntEn;

} // Delay1mS

// wait blocking for "dur" * 0.1 seconds
// Motor and servo pulses are still output every 10ms
void Delay100mSWithOutput(int16 dur)
{ // Timer0 Interrupts must be off 
	int16 i, j;
	uint8 T0IntEn;

	T0IntEn = INTCONbits.TMR0IE;	// not protected?
	INTCONbits.TMR0IE = false;

	for(i = 0; i < dur*10; i++)
		{
			for (j = 8; j ; j--)
			{
				while ( !INTCONbits.TMR0IF ) {};
				INTCONbits.TMR0IF = 0;
			}
			OutSignals(); // 1-2 ms Duration
			if( PIR1bits.RCIF )
			{
				INTCONbits.TMR0IE = T0IntEn;
				return;
			}
		}
	INTCONbits.TMR0IE = T0IntEn;
} // Delay100mSWithOutput

int16 SRS16(int16 x, uint8 s)
{
	return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS16

int32 SRS32(int32 x, uint8 s)
{
	return((x<0) ? -((-x)>>s) : (x>>s));
} // SRS32

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
	INTCON2bits.NOT_RBPU = false;	// WEAK PULLUPS MUST BE ENABLED OTHERWISE I2C VERSIONS 
									// WITHOUT ESCS INSTALLED WILL PREVENT ANY FURTHER BOOTLOADS
} // InitPorts

// resets all important variables - Do NOT call that while in flight!
void InitArrays(void)
{
	int8 i;

	for (i=0; i <= TopTrace; i++)
		Trace[i] = 0;

	for (i = 0; i < NoOfMotors; i++)
		Motor[i] = _Minimum;
	MCamPitch = MCamRoll = _Neutral;

	_Flying = false;
	REp = PEp = YEp = 0;
	
	VUDComp = VBaroComp = 0;
	
	UDSum = 0;
	LRIntKorr = FBIntKorr = 0;
	YawSum = RollSum = PitchSum = 0;

	HoverThrottle = THR_HOVER;
	AE = AltSum = 0;

	BaroRestarts = 0;


} // InitArrays

int16 ConvertGPSToM(int16 c)
{	// approximately 0.18553257183 Metres per LSB
	// only for converting difference in coordinates to 32K
	return ( ((int32)c * (int32)18553)/((int32)100000) );
} // ConvertGPSToM

int16 ConvertMToGPS(int16 c)
{
	return ( ((int32)c * (int32)100000)/((int32)18553) );
} // ConvertMToGPS


int8 ReadEE(uint8 addr)
{
	static int8 b;

	EEADR = addr;
	EECON1bits.EEPGD = false;
	EECON1bits.RD = true;
	b=EEDATA;
	EECON1 = 0;
	return(b);	
} // ReadEE


void ReadParametersEE(void)
{
	static int8 *p, c; 
	static uint16 addr;

	if( CurrentParamSet == 1 )
		addr = _EESet1;	
	else
		addr = _EESet2;
	
	for(p = &FirstProgReg; p <= &LastProgReg; p++)
		*p = ReadEE(addr++);

	RollIntLimit256 = (int16)RollIntLimit * 256L;
	PitchIntLimit256 = (int16)RollIntLimit * 256L;
	YawIntLimit256 = (int16)RollIntLimit * 256L;

	NavIntLimit256 = NavIntLimit * 256L;
	NavClosingRadius = (int32)NavRadius * 5L;
	SqrNavClosingRadius = NavClosingRadius * NavClosingRadius;	
	CompassOffset = (((COMPASS_OFFSET_DEG + NavMagVar)*MILLIPI)/180L);

	BatteryVolts = LowVoltThres;
	
} // ReadParametersEE

void WriteEE(uint8 addr, int8 d)
{
	int8 rd;
	uint8 IntsWereEnabled;
	
	rd = ReadEE(addr);
	if ( rd != d )						// avoid redundant writes
	{
		EEDATA = d;				
		EEADR = addr;
		EECON1bits.EEPGD = false;
		EECON1bits.WREN = true;
		
		IntsWereEnabled = InterruptsEnabled;
		DisableInterrupts;
		EECON2 = 0x55;
		EECON2 = 0xaa;
		EECON1bits.WR = true;
		while(EECON1bits.WR);
		if ( IntsWereEnabled )
			EnableInterrupts;

		EECON1bits.WREN = false;
	}
} // WriteEE

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

void UpdateParamSetChoice(void)
{
	int8 NewParamSet, NewRTHGPSAlt;

	NewParamSet = CurrentParamSet;
	NewRTHGPSAlt = _UseRTHGPSAlt;

	if ( _Signal )
		while ( (Abs(IPitch) > 20) && ((Abs(IYaw) > 20)|| (Abs(IRoll) > 20)) )
		{
			if ( IPitch  > 20 ) // bottom
			{
				if ( (IYaw > 20) || ( IRoll < -20) ) // left
				{ // bottom left
					NewParamSet = 1;
					NewRTHGPSAlt = true;
				}
				else
					if ( (IYaw < -20) || (IRoll > 20) ) // right
					{ // bottom right
						NewParamSet = 2;
						NewRTHGPSAlt = true;
					}
			}	
			else
				if ( IPitch < -20 ) // top
					if ( (IYaw > 20) || ( IRoll < -20) ) // left
					{
						NewParamSet = 1;
						NewRTHGPSAlt = false;
					}
					else
						if ( (IYaw < -20) || (IRoll > 20) ) // right
						{
							NewParamSet = 2;
							NewRTHGPSAlt = false;
						}
			if ( ( NewParamSet != CurrentParamSet ) || ( NewRTHGPSAlt != _UseRTHGPSAlt) )
			{
				
				CurrentParamSet = NewParamSet;
				_UseRTHGPSAlt = NewRTHGPSAlt;
				LEDBlue_ON;
				Beeper_ON;
				Delay100mSWithOutput(2);
				Beeper_OFF;
				if ( CurrentParamSet == 2 )
				{
					Delay100mSWithOutput(2);
					Beeper_ON;
					Delay100mSWithOutput(2);
					Beeper_OFF;
				}
				if ( _UseRTHGPSAlt )
				{
					Delay100mSWithOutput(4);
					Beeper_ON;
					Delay100mSWithOutput(4);
					Beeper_OFF;
				}
				LEDBlue_OFF;
		 	}
		}
} // UpdateParamSetChoice

int16 Make2Pi(int16 A)
{
	while ( A < 0 ) A += TWOMILLIPI;
	while ( A >= TWOMILLIPI ) A -= TWOMILLIPI;
	return( A );
} // Make2Pi

int16 MakePi(int16 A)
{
	while ( A < -MILLIPI ) A += TWOMILLIPI;
	while ( A >= MILLIPI ) A -= TWOMILLIPI;
	return( A );
} // MakePi


#pragma idata sintable
const int16 SineTable[17]={ 
	0, 50, 98, 142, 180, 212, 236, 250, 255,
	250, 236, 212, 180, 142, 98, 50, 0
   };
#pragma idata

int16 Table16(int16 Val, const int16 *T)
{
	static uint8 Index,Offset;
	static int16 Temp, Low, High, Result;

	Index = (uint8) (Val >> 4);
	Offset = (uint8) (Val & 0x0f);
	Low = T[Index];
	High = T[++Index];
	Temp = (High-Low) * Offset;
	Result = Low + SRS16(Temp, 4);

	return(Result);
} // Table16

int16 int16sin(int16 A)
{	// A is in milliradian 0 to 2000Pi, result is -255 to 255
	static int16 	v;
	static uint8	Negate;

	while ( A < 0 ) A += TWOMILLIPI;
	while ( A >= TWOMILLIPI ) A -= TWOMILLIPI;

	Negate = A >= MILLIPI;
	if ( Negate )
		A -= MILLIPI;

	v = Table16(((int24)A * 256 + HALFMILLIPI)/MILLIPI, SineTable);

	if ( Negate )
		v= -v;

	return(v);
} // int16sin

int16 int16cos(int16 A)
{	// A is in milliradian 0 to 2000Pi, result is -255 to 255
	return(int16sin(A + HALFMILLIPI));
} // int16cos

#pragma idata arctan
const int16 ArctanTable[17]={
	0, 464, 785, 983, 1107, 1190, 1249, 1292, 1326,
	1352, 1373, 1391, 1406, 1418, 1429, 1438, 1446
   };
#pragma idata

int16 int16atan2(int16 y, int16 x)
{	// Result is in milliradian
	// Caution - this routine is intended to be acceptably accurate for 
	// angles less Pi/4 within a quadrant. Larger angles are directly interpolated
	// to Pi/2. 
 
	static int32 Absx, Absy, TL;
	static int16 A;

	Absy = Abs(y);
	Absx = Abs(x);

	if ( x == 0 )
		if ( y < 0 )
			A = -HALFMILLIPI;
		else
			A = HALFMILLIPI;
	else
		if (y == 0)
			if ( x < 0 )
				A=MILLIPI;
			else
				A = 0;
		else
		{
			TL = (Absy * 32)/Absx;
			if ( TL < 256 )
				A = Table16(TL, ArctanTable);
			else
			{  // extrapolate outside table
				TL -= 256;
				A =  ArctanTable[16] + (TL >> 2);
				A = Limit(A, 0, HALFMILLIPI);
			}

			if ( x < 0 )
				if ( y > 0 ) // 2nd Quadrant 
					A = MILLIPI - A;
				else // 3rd Quadrant 
					A = MILLIPI + A;
			else
				if ( y < 0 ) // 4th Quadrant 
					A = TWOMILLIPI-A;
	}
	return(A);
} // int16atan2

int16 int16sqrt(int16 n)
// 16 bit numbers 
{
  static int16 r, b;

  r=0;
  b=256;
  while (b>0) 
    {
    if (r*r>n)
      r-=b;
    b=(b>>1);
    r+=b;
    }
  return(r);
} // int16sqrt

void SendLEDs(void)
{
	static int8	i, s;

	i = LEDShadow;
	SPI_CS = DSEL_LISL;	
	SPI_IO = WR_SPI;	// SDA is output
	SPI_SCL = 0;		// because shift is on positive edge
	
	for(s=8; s!=0; s--)
	{
		if( i & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		i<<=1;
		Delay10TCY();
		SPI_SCL = 1;
		Delay10TCY();
		SPI_SCL = 0;
	}

	PORTCbits.RC1 = 1;
	PORTCbits.RC1 = 0;	// latch into drivers
	SPI_SCL = 1;		// rest state for LISL
	SPI_IO = RD_SPI;
}

void SwitchLEDsOn(uint8 l)
{
	LEDShadow |= l;
	SendLEDs();
} // SwitchLEDsOn

void SwitchLEDsOff(uint8 l)
{
	LEDShadow &= ~l;
	SendLEDs();
} // SwitchLEDsOff

void LEDGame(void)
{
	if( --LEDCycles == 0 )
	{
		LEDCycles = ((255-DesiredThrottle)>>3) +5;	// new setup
		if( _Hovering )
		{
			AUX_LEDS_ON;	// baro locked, all aux-leds on
		}
		else
		if( LEDShadow & LEDAUX1 )
		{
			AUX_LEDS_OFF;
			LEDAUX2_ON;
		}
		else
		if( LEDShadow & LEDAUX2 )
		{
			AUX_LEDS_OFF;
			LEDAUX3_ON;
		}
		else
		{
			AUX_LEDS_OFF;
			LEDAUX1_ON;
		}
	}
} // LEDGame

void CheckAlarms(void)
{
	static int16 NewBatteryVolts;

	NewBatteryVolts = ADC(ADCBattVoltsChan, ADCVREF5V) >> 3; 
	BatteryVolts = SoftFilter(BatteryVolts, NewBatteryVolts);
	_LowBatt =  (BatteryVolts < (int16) LowVoltThres) & 1;

// zzz fix later

	if( _LowBatt ) // repeating beep
	{
		if( ( Cycles & 0x0040) == 0 )
		{
			Beeper_ON;
			LEDRed_ON;
		}
		else
		{
			Beeper_OFF;
			LEDRed_OFF;
		}	
	}
	else
	if ( _LostModel ) // 2 beeps with interval
		if( (Cycles & 0x0080) == 0 )
		{
			Beeper_ON;
			LEDRed_ON;
		}
		else
		{
			Beeper_OFF;
			LEDRed_OFF;
		}	
	else
	{
		Beeper_OFF;				
		LEDRed_OFF;
	}

} // CheckAlarms

void DumpTrace(void)
{
#ifdef DEBUG_SENSORS
	int8 t;

	for (t=0; t <= TopTrace; t++)
	{
		TxValH16(Trace[t]);
		TxChar(';');
	}
	TxNextLine();

#endif // DEBUG_SENSORS
} // DumpTrace

