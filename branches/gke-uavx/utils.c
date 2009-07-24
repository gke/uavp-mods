// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "uavx.h"

// Prototypes

void Delay1mS(int16);
void Delay100mSWithOutput(int16);
int16 SRS16(int16, uint8);
int32 SRS32(int32, uint8);
void InitPorts(void);
void InitMisc(void);

int16 ConvertGPSToM(int16);
int16 ConvertMToGPS(int16);

int8 ReadEE(uint8);
void ReadParametersEE(void);
void WriteEE(uint8, int8);
void WriteParametersEE(uint8);
void UpdateWhichParamSet(void);
void InitParameters(void);

int16 Make2Pi(int16);
int16 Table16(int16, const int16 *);
int16 int16sin(int16);
int16 int16cos(int16);
int16 int16atan2(int16, int16);
int16 int16sqrt(int16);

void SendLEDs(void);
void SwitchLEDsOn(uint8);
void SwitchLEDsOff(uint8);
void CheckAlarms(void);

void DumpTrace(void);

void Delay1mS(int16 d)
{ 
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

void Delay100mSWithOutput(int16 dur)
{  // Motor and servo pulses are still output every 10ms
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
void InitMisc(void)
{
	int8 i;

	for (i=0; i <= TopTrace; i++)
		Trace[i] = 0;
	
	for ( i = 0; i<32 ; i++ )
		Flags[i] = false; 

	ThrNeutral = ThrLow = ThrHigh = MAXINT16;

	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;
	Beeper_OFF;
} // InitMisc

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
	static int8 p, c; 
	static uint16 addr;

	addr = (CurrentParamSet - 1)* MAX_PARAMETERS;	
	for(p = 0; p <= LastParam; p++)
		P[p] = ReadEE(addr + p);

	IdleThrottle = ((int16)P[PercentIdleThr] * OUT_MAXIMUM )/100;
	HoverThrottle = ((int16)P[PercentHoverThr] * OUT_MAXIMUM )/100;

	RollIntLimit256 = (int16)P[RollIntLimit] * 256L;
	PitchIntLimit256 = (int16)P[PitchIntLimit] * 256L;
	YawIntLimit256 = (int16)P[YawIntLimit] * 256L;

	NavIntLimit256 = P[NavIntLimit] * 256L; 
	NavClosingRadius = (int32)P[NavRadius] * 5L;
	NavClosingRadius = Limit(P[NavClosingRadius], 5, 40); // avoid divide by zero
	SqrNavClosingRadius = P[NavClosingRadius] * P[NavClosingRadius];	
	CompassOffset = (((COMPASS_OFFSET_DEG - P[NavMagVar])*MILLIPI)/180L);

	_NegativePPM = (( P[TxRxType] == JRPPM ) || ( P[TxRxType] == JRDM9 ) 
				|| ( P[TxRxType] == DX7AR7000 )|| ( P[TxRxType] == DX7AR6200 ));

	PIE1bits.CCP1IE = true;

	BatteryVolts = P[LowVoltThres];
	
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
	int8 p;
	uint8 b;
	uint16 addr;
	
	addr = (s - 1)* MAX_PARAMETERS;
	for ( p = 0; p <= LastParam; p++)
		WriteEE( addr + p,  P[p]);
} // WriteParametersEE

void UpdateParamSetChoice(void)
{
	#define STICK_WINDOW 30

	int8 NewParamSet, NewRTHAltitudeHold, NewTurnToHome, Selector;

	NewParamSet = CurrentParamSet;
	NewRTHAltitudeHold = _RTHAltitudeHold;
	NewTurnToHome = _TurnToHome;

	UpdateControls();

	if ( TxMode2 )
		Selector = RC[RollC];
	else
		Selector = -RC[YawC];

	if ( (Abs(RC[PitchC]) > STICK_WINDOW) && (Abs(Selector) > STICK_WINDOW) )
	{
		if ( RC[PitchC] > STICK_WINDOW ) // bottom
			if ( Selector < -STICK_WINDOW ) // left
			{ // bottom left
				NewParamSet = 1;
				NewRTHAltitudeHold = true;
			}
			else
			{ // bottom right
				NewParamSet = 2;
				NewRTHAltitudeHold = true;
			}	
		else
			if ( RC[PitchC] < -STICK_WINDOW ) // top
				if ( Selector < -STICK_WINDOW ) // left
				{
					NewParamSet = 1;
					NewRTHAltitudeHold = false;
				}
				else
				{ // right
					NewParamSet = 2;
					NewRTHAltitudeHold = false;
				}

		if ( ( NewParamSet != CurrentParamSet ) || ( NewRTHAltitudeHold != _RTHAltitudeHold) )
		{
					
			CurrentParamSet = NewParamSet;
			_RTHAltitudeHold = NewRTHAltitudeHold;
			LEDBlue_ON;
			Beeper_ON;
			Delay100mSWithOutput(2);
			Beeper_OFF;
			if ( CurrentParamSet == 2 )
			{
				Delay100mSWithOutput(2);
				Beeper_ON;
				Delay100mSWithOutput(2);
			}
			if ( _RTHAltitudeHold )
			{
				Delay100mSWithOutput(4);
				Beeper_ON;
				Delay100mSWithOutput(4);	
			}
			Beeper_OFF;
			LEDBlue_OFF;
		}
	}

	if ( TxMode2 )
		Selector = -RC[YawC];
	else
		Selector = RC[RollC];

	if ( (Abs(RC[ThrottleC]) < STICK_WINDOW) && (Abs(Selector) > STICK_WINDOW ) )
	{
		if ( Selector < -STICK_WINDOW ) // left
			NewTurnToHome = false;
		else
			NewTurnToHome = true; // right
			
		if ( NewTurnToHome != _TurnToHome )
		{		
			_TurnToHome = NewTurnToHome;
			LEDBlue_ON;
			if ( _TurnToHome )
			{
				Beeper_ON;
				Delay100mSWithOutput(4);
			}
			else
				Delay100mSWithOutput(2);
			Beeper_OFF;
			LEDBlue_OFF;
		}
	}
} // UpdateParamSetChoice

void InitParameters(void)
{
	uint16 addr;

	addr = ( CurrentParamSet - 1 ) * MAX_PARAMETERS;
	while ( ReadEE(addr) == 0xff )
		ProcessCommand();

	CurrentParamSet = 1;
	ReadParametersEE();
} // InitParamters

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
	static int16 Temp, Low, High;

	Index = (uint8) (Val >> 4);
	Offset = (uint8) (Val & 0x0f);
	Low = T[Index];
	High = T[++Index];
	Temp = (High-Low) * Offset;

	return( Low + SRS16(Temp, 4) );
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

void CheckAlarms(void)
{
	static int16 NewBatteryVolts;

	NewBatteryVolts = ADC(ADCBattVoltsChan, ADCVREF5V) >> 3; 
	BatteryVolts = SoftFilter(BatteryVolts, NewBatteryVolts);
	_LowBatt =  (BatteryVolts < (int16) P[LowVoltThres]) & 1;

	if( _LowBatt ) // repeating beep
	{
		if( ((int16)mS[Clock] & 0x0200) == 0 )
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
		if( ((int16)mS[Clock] & 0x0400) == 0 )
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

	if ( DesiredThrottle > IdleThrottle )
	{
		for (t=0; t <= TopTrace; t++)
		{
			TxValH16(Trace[t]);
			TxChar(';');
		}
		TxNextLine();
	}

#endif // DEBUG_SENSORS
} // DumpTrace

