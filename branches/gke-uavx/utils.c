// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =           http://code.google.com/p/uavp-mods/ http://uavp.ch        =
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
void UseDefaultParameters(void);
void UpdateWhichParamSet(void);
void InitParameters(void);

int16 Make2Pi(int16);
int16 Table16(int16, const int16 *);
int16 int16sin(int16);
int16 int16cos(int16);
int16 int16atan2(int16, int16);
int16 int16sqrt(int16);

void SendLEDs(void);
void LEDsOn(uint8);
void LEDsOff(uint8);
void LEDGame(void);
void CheckAlarms(void);

void DoBeep1MsWithOutput(uint8, uint8);
void DoStartingBeepsWithOutput(uint8);

void ZeroStats(void);
void ReadStatsEE(void);
void WriteStatsEE(void);
void ShowStats(void);

void DumpTrace(void);

void Delay1mS(int16 d)
{ 
	int16 i;
	boolean T0IntEn;

	T0IntEn = INTCONbits.TMR0IE;	// not protected?
	INTCONbits.TMR0IE = false;

	// if d is 1 then delay can be less than 1mS due to 	
	for (i = d; i ; i--)
	{						// compromises ClockMilliSec;
		while ( !INTCONbits.TMR0IF ) {};
		INTCONbits.TMR0IF = 0;
	}

	INTCONbits.TMR0IE = T0IntEn;

} // Delay1mS

void Delay100mSWithOutput(int16 dur)
{  // Motor and servo pulses are still output every 10ms
	int16 i;
	uint8 j;
	boolean T0IntEn;

	T0IntEn = INTCONbits.TMR0IE;	// not protected?
	INTCONbits.TMR0IE = false;

	for( i = 0; i < dur*10; i++)
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

#ifndef USE_LIMIT_MACRO
int16 ProcLimit(int16 i, int16 l, int16 u)
{
	return ((i<l) ? l : ((i>u) ? u : i));	
} // ProcLimit
#endif // USE_LIMIT_MACRO

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
	uint8 i;

	State = Starting;				// For trace preconditions

	for ( i = 0; i <= TopTrace; i++)
		Trace[i] = 0;
	
	for ( i = 0; i < 32 ; i++ )
		Flags[i] = false; 
	_RTHAltitudeHold = true;
	_ParametersValid = true;

	ThrNeutral = ThrLow = ThrHigh = MAXINT16;
	ESCMin = OUT_MINIMUM;
	ESCMax = OUT_MAXIMUM;

	GyroMidRoll = GyroMidPitch = GyroMidYaw = RollRate = PitchRate = YawRate = 0;

	LEDShadow = 0;
    ALL_LEDS_OFF;
	LEDRed_ON;
	Beeper_OFF;
} // InitMisc

int16 ConvertGPSToM(int16 c)
{	// approximately 0.18553257183 Metres per LSB at the Equator
	// only for converting difference in coordinates to 32K
	return ( ((int32)c * (int32)18553)/((int32)100000) );
} // ConvertGPSToM

int16 ConvertMToGPS(int16 c)
{
	return ( ((int32)c * (int32)100000)/((int32)18553) );
} // ConvertMToGPS

int16 SlewLimit(int16 Old, int16 New, int16 Slew)
{
  int16 Low, High;
  
  Low = Old - Slew;
  High = Old + Slew; 
  return(( New < Low ) ? Low : (( New > High ) ? High : New));
} // SlewLimit

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
	uint8 i;
	static uint16 addr;

	if ( ParametersChanged )
	{   // overkill if only a single parameter has changed but is not in flight loop
		addr = (CurrentParamSet - 1)* MAX_PARAMETERS;	
		for ( i = 0; i < MAX_PARAMETERS; i++)
			P[i] = ReadEE(addr + i);

		ESCMax = ESCLimits[P[ESCType]];
		if ( P[ESCType] == ESCPPM )
			ESCMin = 1;
		else
		{
			ESCMin = 0;
			for ( i = 0; i < NoOfMotors; i++ )
				ESCI2CFail[i] = false;
			InitI2CESCs();
		}

		#ifdef RX6CH
		NavSensitivity = ((int16)P[PercentNavSens6Ch] * RC_MAXIMUM)/100;
		#endif // RX6CH

		for ( i = 0; i < CONTROLS; i++) // make reverse map
			RMap[Map[P[TxRxType]][i]-1] = i+1;
	
		IdleThrottle = ((int16)P[PercentIdleThr] * OUT_MAXIMUM )/100;
		HoverThrottle = ((int16)P[PercentHoverThr] * OUT_MAXIMUM )/100;
	
		RollIntLimit256 = (int16)P[RollIntLimit] * 256L;
		PitchIntLimit256 = (int16)P[PitchIntLimit] * 256L;
		YawIntLimit256 = (int16)P[YawIntLimit] * 256L;
	
		NavIntLimit256 = P[NavIntLimit] * 256L; 
		NavClosingRadius = (int32)P[NavRadius] * METRES_TO_GPS;
		NavNeutralRadius = (int32)P[NeutralRadius] * METRES_TO_GPS;
		NavClosingRadius = Limit(P[NavClosingRadius], 5, 40); // avoid divide by zero
		SqrNavClosingRadius = P[NavClosingRadius] * P[NavClosingRadius];	
		CompassOffset = (((COMPASS_OFFSET_DEG - P[NavMagVar])*MILLIPI)/180L);
	
		PIE1bits.CCP1IE = false;
		DoRxPolarity();
		PPM_Index = PrevEdge = 0;
		PIE1bits.CCP1IE = true;
	
		BatteryVolts = P[LowVoltThres];
		
		ParametersChanged = false;
	}
	
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
	uint8 p;
	uint8 b;
	uint16 addr;
	
	addr = (s - 1)* MAX_PARAMETERS;
	for ( p = 0; p < MAX_PARAMETERS; p++)
		WriteEE( addr + p,  P[p]);
} // WriteParametersEE

void UseDefaultParameters(void)
{ // loads a representative set of initial parameters as a base for tuning
	uint8 p;

	for ( p = 0; p < MAX_PARAMETERS; p++ )
		P[p] = DefaultParams[p];

	WriteParametersEE(1);
	WriteParametersEE(2);
	CurrentParamSet = 1;
	TxString("\r\nDefault Parameters Loaded\r\n");	
} // UseDefaultParameters

void UpdateParamSetChoice(void)
{
	#define STICK_WINDOW 30

	int8 NewParamSet, NewRTHAltitudeHold, NewTurnToHome, Selector;

	NewParamSet = CurrentParamSet;
	NewRTHAltitudeHold = _RTHAltitudeHold;
	NewTurnToHome = _TurnToHome;

	if ( P[ConfigBits] & TxMode2Mask )
		Selector = RC[RollC];
	else
		Selector = -RC[YawC];

	if ( (Abs(RC[PitchC]) > STICK_WINDOW) && (Abs(Selector) > STICK_WINDOW) )
	{
		if ( RC[PitchC] > STICK_WINDOW ) // bottom
		{
			if ( Selector < -STICK_WINDOW ) // left
			{ // bottom left
				NewParamSet = 1;
				NewRTHAltitudeHold = true;
			}
			else
				if ( Selector > STICK_WINDOW ) // right
				{ // bottom right
					NewParamSet = 2;
					NewRTHAltitudeHold = true;
				}
		}		
		else
			if ( RC[PitchC] < -STICK_WINDOW ) // top
			{		
				if ( Selector < -STICK_WINDOW ) // left
				{
					NewRTHAltitudeHold = false;
					NewParamSet = 1;
				}
				else 
					if ( Selector > STICK_WINDOW ) // right
					{
						NewRTHAltitudeHold = false;
						NewParamSet = 2;
					}
			}

		if ( ( NewParamSet != CurrentParamSet ) || ( NewRTHAltitudeHold != _RTHAltitudeHold) )
		{	
			CurrentParamSet = NewParamSet;
			_RTHAltitudeHold = NewRTHAltitudeHold;
			LEDBlue_ON;
			DoBeep100mSWithOutput(2, 2);
			if ( CurrentParamSet == 2 )
				DoBeep100mSWithOutput(2, 2);
			if ( _RTHAltitudeHold )
				DoBeep100mSWithOutput(4, 4);
			ParametersChanged |= true;
			Beeper_OFF;
			LEDBlue_OFF;
		}
	}

	if ( P[ConfigBits] & TxMode2Mask )
		Selector = -RC[YawC];
	else
		Selector = RC[RollC];

	if ( (Abs(RC[ThrottleC]) < STICK_WINDOW) && (Abs(Selector) > STICK_WINDOW ) )
	{
		if ( Selector < -STICK_WINDOW ) // left
			NewTurnToHome = false;
		else
			if ( Selector > STICK_WINDOW ) // left
				NewTurnToHome = true; // right
			
		if ( NewTurnToHome != _TurnToHome )
		{		
			_TurnToHome = NewTurnToHome;
			LEDBlue_ON;
			if ( _TurnToHome )
				DoBeep100mSWithOutput(4, 2);

			LEDBlue_OFF;
		}
	}
} // UpdateParamSetChoice

void InitParameters(void)
{
	ALL_LEDS_ON;
	CurrentParamSet = 1;
	while ( ReadEE(TxRxType) == -1 ) 
		ProcessCommand();	
	CurrentParamSet = 1;
	ParametersChanged = true;
	ReadParametersEE();
	ALL_LEDS_OFF;
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
	static boolean	Negate;

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
					A = TWOMILLIPI - A;
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
	
	for(s = 8; s ; s--)
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

void LEDsOn(uint8 l)
{
	LEDShadow |= l;
	SendLEDs();
} // LEDsOn

void LEDsOff(uint8 l)
{
	LEDShadow &= ~l;
	SendLEDs();
} // LEDsOff

void LEDGame(void)
{
	if( --LEDCycles == 0 )
	{
		LEDCycles = (( 255 - DesiredThrottle ) >> 3) + 5;	// new setup
		if( _Hovering )
			AUX_LEDS_ON;	// baro locked, all aux-leds on
		else
			if( LEDShadow & AUX1M )
			{
				AUX_LEDS_OFF;
				LEDAUX2_ON;
			}
			else
				if( LEDShadow & AUX2M )
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

void DoBeep100mSWithOutput(uint8 t, uint8 d)
{
	Beeper_ON;
	Delay100mSWithOutput(t);
	Beeper_OFF;
	Delay100mSWithOutput(d);
} // DoBeep100mSWithOutput

void DoStartingBeepsWithOutput(uint8 b)
{
	uint8 i;

	for ( i = 0; i < b; i++ )
		DoBeep100mSWithOutput(2, 8);

	DoBeep100mSWithOutput(8,0);
} // DoStartingBeeps

void ZeroStats(void)
{
	uint8 s;

	for (s = 0 ; s < MaxStats ; s++ )
		Stats[s].i16 = 0;
} // ZeroStats

void ReadStatsEE(void)
{
	uint8 s;

	for (s = 0 ; s < MaxStats ; s++ )
	{
		Stats[s].low8 = ReadEE(STATS_ADDR_EE + s*2);
		Stats[s].high8 = ReadEE(STATS_ADDR_EE + s*2 + 1);
	}
} // InitStats

void WriteStatsEE()
{
	uint8 s;
	int16 Temp;

	for (s = 0 ; s < MaxStats ; s++ )
	{
		WriteEE(STATS_ADDR_EE + s*2, Stats[s].low8);
		WriteEE(STATS_ADDR_EE + s*2 + 1, Stats[s].high8);
	}

	Temp = ToPercent(HoverThrottle, OUT_MAXIMUM);
	WriteEE((CurrentParamSet-1) * MAX_PARAMETERS + PercentHoverThr, Temp);
} // WriteStatsEE

void ShowStats(void)
{
	static int16 Scale;

	Scale = 3000;
	if ( P[GyroType] == IDG300 )
		Scale = 5000;
	else
		if ( P[GyroType] == ADXRS150 )
			Scale = 1500;

	TxString("\r\nFlight Statistics\r\n");
	TxString("GPS:   \t");TxVal32(Stats[GPSAltitudeS].i16,1,' '); TxString("M\r\n"); 
	TxString("Baro:  \t");TxVal32(Stats[BaroPressureS].i16,0,' '); TxString("counts\r\n"); 
	TxNextLine();
	TxString("Roll:  \t"); TxVal32(((int32)(Stats[RollRateS].i16 - Stats[GyroMidRollS].i16) * Scale)>>10,1,' '); TxString("Deg/Sec\r\n");
	TxString("Pitch: \t"); TxVal32(((int32)(Stats[PitchRateS].i16 - Stats[GyroMidPitchS].i16) * Scale)>>10,1,' '); TxString("Deg/Sec\r\n");
	TxString("Yaw:   \t");TxVal32(((int32)(Stats[YawRateS].i16 - Stats[GyroMidYawS].i16) * 3000L)>>8,1,' '); TxString("Deg/Sec\r\n");
	TxNextLine();
	TxString("LRAcc: \t"); TxVal32(((int32)Stats[LRAccS].i16*1000+512)/1024, 3, 'G'); TxNextLine(); 
	TxString("FBAcc: \t"); TxVal32(((int32)Stats[FBAccS].i16*1000+512)/1024, 3, 'G'); TxNextLine();
	TxString("DUAcc: \t"); TxVal32(((int32)Stats[DUAccS].i16*1000+512)/1024, 3, 'G'); TxNextLine(); 
    TxNextLine();
	TxString("LRDrift:\t"); TxVal32((int32)Stats[LRDriftAccS].i16, 0, ' '); TxString("lsb\r\n");; 
	TxString("FBDrift:\t"); TxVal32((int32)Stats[FBDriftAccS].i16, 0, ' '); TxString("lsb\r\n");;
} // ShowStats

void DumpTrace(void)
{
#ifdef DEBUG_SENSORS
	uint8 t;

	if ( DesiredThrottle > 20 ) 
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

