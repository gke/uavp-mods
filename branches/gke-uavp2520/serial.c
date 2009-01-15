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

#include "c-ufo.h"
#include "bits.h"

// data strings
#pragma idata menu1
const char SerHello[] = "\r\nUAVP V" Version " (c) 2008"
							  " Ing. Wolfgang Mahringer\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";

const char  SerSetup[] = "\r\nUAVP V" Version " ready.\r\n"
							  "Gyro: "
#ifdef OPT_ADXRS300
							  "3x ADXRS300\r\n"
#endif
#ifdef OPT_ADXRS150
							  "3x ADXRS150\r\n"
#endif
#ifdef OPT_IDG
							  "1x ADXRS300, 1x IDG300\r\n"
#endif
							  "Linear sensors ";
const uint8 SerLSavail[]="ONLINE\r\n";
const uint8 SerLSnone[]= "not available\r\n";
const uint8 SerBaro[]=   "Baro sensor ";
const uint8 SerChannel[]="Channel mode: Throttle Ch";
const uint8 SerFM_Fut[]= "3";
const uint8 SerFM_Grp[]= "1";
const uint8 SerCompass[]="Compass sensor ";
#pragma idata

#pragma idata menu2
const uint8 SerHelp[]  = "\r\nCommands:\r\n"
					 		  "L...List param\r\n"
							  "M...Modify param\r\n"
							  "S...Show setup\r\n"
							  "N...Neutral values\r\n"
							  "R...Show receiver channels\r\n"
							  "B...start Boot-Loader\r\n";
const uint8 SerReg1[]  = "\r\nRegister ";
const uint8 SerReg2[]  = " = ";
const uint8 SerPrompt[]= "\r\n>";
// THE FOLLOWING LINE NOT TO BE CHANGED, it is important for UAVPset
const uint8 SerList[]  = "\r\nParameter list for set #";
const uint8 SerSelSet[]= "\r\nSelected parameter set: ";
const uint8 SerNeutralR[]="\r\nNeutral Roll:";
const uint8 SerNeutralN[]=" Nick:";
const uint8 SerNeutralY[]=" Yaw:";
const uint8 SerRecvCh[]=  "\r\nT:";
#pragma idata

void InitUSART(uint8 Rate)
{
	uint8 ch;

	TXSTA = 0b00100100;
	RCSTA = 0b10010000;
	SPBRG = Rate;

	ch = RCREG;
} // InitUSART
			
// transmit a fix text from a table
void TxText(const uint8 *pch)
{
	while( *pch != '\0' )
		TxChar(*pch++);
}// TxText

void ShowPrompt(void)
{
	TxText(SerPrompt);
} // ShowPrompt

// send a character to the serial port
void TxChar(uint8 ch)
{
	while( !PIR1bits.TXIF ) ;						// wait for transmit ready
	TXREG = ch;										// put new char
}

// converts an unsigned byte to decimal and send it
void TxValU(uint8 v)
{
	uint8 ch;

	ch = v / 100;
	TxChar(ch + '0');
	v %= 100;										// extract remainder
	
	ch = v / 10;
	TxChar(ch + '0');
	v %= 10;

	TxChar(v + '0');
} // TxValU

// converts a nibble to HEX and sends it
void TxNibble(uint8 v)
{
	if( v > 9 )
		v += ('A'-10);
	else
		v += '0';
	TxChar(v);
} // TxNibble 

// converts an unsigned byte to HEX and sends it
void TxValH(uint8 v)
{
	TxNibble(v >> 4);
	TxNibble(v & 0x0f);
} // TxValH

// converts an unsigned double byte to HEX and sends it
void TxValH16(uint16 v)
{
	TxValH(Upper8(v));
	TxValH(Lower8(v));
} // TxValH16

// converts a signed byte to decimal and send it
void TxValS(int8 v)
{
	if( v < 0 )
	{
		TxChar('-');							// send sign
		v = -v;
	}
	else
		TxChar('+');

	TxValU(v);
} // TxValS

void TxNextLine(void)
{
	TxChar(0x0d);
	TxChar(0x0a);
} // TxNextLine

#ifdef READABLE
void TxVal(int32 v, uint8 dp, uint8 sep)
{
  uint8 s[12];
  int8 c, rem;
  int32 newv;
 
  if (v<0)
    {
    TxChar('-');
    v=-v;
    }

  c=0;
  do
    {
    newv=v/10;
    rem=v-(newv*8+newv*2);
    s[c]=rem + '0';
    v=newv;
    c++;
    }
  while (v>0);
  
  if ((c==dp) && (dp>0))
    {
    TxChar('0');
    TxChar('.');
    }
  do
    {
    c--;
    TxChar(s[c]);
    if ((c==dp)&&(c>0)) 
      TxChar('.');
    }
  while (c>0);
  if (sep!=0)
    TxChar(sep);
} // TxVal
#endif // READABLE

// if a character is in the buffer
// return it. Else return the NUL character
uint8 RxChar(void)
{
	uint8	ch;	

	if( PIR1bits.RCIF )								// a character is waiting in the buffer
	{			
		if( RCSTAbits.OERR || RCSTAbits.FERR )		// overrun or framing error?
		{
			RCSTAbits.CREN = false;					// disable, then re-enable port to
			RCSTAbits.CREN = true;					// reset OERR and FERR bit
			ch = RCREG;								// dummy read
		}
		else
		{
			ch = RCREG;								// get the character
			TxChar(ch);						// echo it
			return(ch);								// and return it
		}
	}
	return( '\0' );									// nothing in buffer
} // RxChar

// enter an unsigned number 00 to 99
uint8 RxNumU(void)
{
	char ch;
	uint8 v;

	do
		ch = RxChar();
	while( (ch < '0') || (ch > '9') );
	v = (ch - '0') * 10;
	do
		ch = RxChar();
	while( (ch < '0') || (ch > '9') );
	v += ch - '0';
	return(v);
} // RxNumU

// enter a signed number -99 to 99 (always 2 digits)!
int16 RxNumS(void)
{
	char ch;
	uint8 v;
	uint8 neg;

	neg = false;
	do
		ch = RxChar();
	while( ((ch < '0') || (ch > '9')) && (ch != '-') );
	if( ch == '-' )
	{
		neg = true;
		do
			ch = RxChar();
		while( (ch < '0') || (ch > '9') );
	}
	v = (ch - '0') * 10;
	do
		ch = RxChar();
	while( (ch < '0') || (ch > '9') );
	v += ch - '0';
	if ( neg )
		v = -v;
	return(v);
} // RxNumS

// send the current configuration setup to serial port
void ShowSetup(uint8 Hello)
{
	if( Hello )
	{
		TxText(SerHello);
		IK5 = _Minimum;	
	}

	TxText(SerSetup);							// send hello message
	if( _UseLISL )
		TxText(SerLSavail);
	else
		TxText(SerLSnone);

	TxText(SerCompass);
	if( _UseCompass )
		TxText(SerLSavail);
	else
		TxText(SerLSnone);

	TxText(SerBaro);
	if( _UseBaro )
		TxText(SerLSavail);
	else
		TxText(SerLSnone);

	ReadParametersEE();
	TxText(SerChannel);
	if( FutabaMode )
		TxText(SerFM_Fut);
	else
		TxText(SerFM_Grp);

	TxText(SerSelSet);
	if( IK5 > _Neutral )
		TxChar('2');
	else
		TxChar('1');
	
	ShowPrompt();
} // ShowSetup

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessCommand(void)
{
	int8 *p;
	uint8 ch;
	uint8 addr;
	uint16 addrbase, curraddr;
	int8 d;

	ch = RxChar();

	if( islower(ch))							// check lower case
		ch=toupper(ch);

	if (ch == '\0') // switch would have been more elegant but code too big/slow!
	{
		// most common NULL case
	}
	else
	if (ch == 'L')	// List parameters
	{
			TxText(SerList);					// must send it (UAVPset!)
			if( IK5 > _Neutral )
				TxChar('2');
			else
				TxChar('1');
			ReadParametersEE();
			addr = 1;
			for(p = &FirstProgReg; p <= &LastProgReg; p++)
			{
				TxText(SerReg1);
				TxValU(addr++);
				TxText(SerReg2);
				d = *p;
				TxValS(d);
			}
			ShowPrompt();
	}
	else
	if (ch == 'M')// modify parameters
	{ 
			LedBlue_ON;
			TxText(SerReg1);
			addr = RxNumU()-1;
			TxText(SerReg2);
			d = RxNumS();
			if( IK5 > _Neutral )
				addrbase = _EESet2;
			else
				addrbase = _EESet1;
			WriteEE(addrbase + (uint16)addr, d);	

			// update transmitter config bits in the other parameter set
			if( addr ==  (&ConfigParam - &FirstProgReg) )
			{									
				if( IK5 > _Neutral )
					addrbase = _EESet1;				
				else
					addrbase = _EESet2;	
				// mask only bits _FutabaMode and _NegativePPM
				d &= 0x12;		
				d = (ReadEE(addrbase + (uint16)addr) & 0xed) | d;
				WriteEE(addrbase + (uint16)addr, d);
			}
			LedBlue_OFF;
			ShowPrompt();
	}
	else
	if (ch == 'S')	// show status
	{
			ShowSetup(0);
	}
	else
	if (ch == 'N')	// neutral values
	{
			TxText(SerNeutralR);
			TxValS(NeutralLR);

			TxText(SerNeutralN);
			TxValS(NeutralFB);

			TxText(SerNeutralY);
			TxValS(NeutralUD);
			ShowPrompt();
	}
	else
	if (ch == 'R')// receiver values
	{
			TxText(SerRecvCh);
			TxValU(IThrottle);
			TxChar(',');
			TxChar('R');
			TxChar(':');
			TxValS(IRoll);
			TxChar(',');
			TxChar('N');
			TxChar(':');
			TxValS(IPitch);
			TxChar(',');
			TxChar('Y');
			TxChar(':');
			TxValS(IYaw);
			TxChar(',');
			TxChar('5');
			TxChar(':');
			TxValU(IK5);
			TxChar(',');
			TxChar('6');
			TxChar(':');
			TxValU(IK6);
			TxChar(',');
			TxChar('7');
			TxChar(':');
			TxValU(IK7);
			ShowPrompt();
	}
	else
	if (ch == 'B')// call bootloader
	{									
//#asm
//			movlw	0x1f
//			movwf	PCLATH
//			dw	0x2F00
//#endasm
//			DisableInterrupts;
			BootStart();							// never comes back!
	}
	else
	if (ch == '?') // Help
	{
			TxText(SerHelp);
			ShowPrompt();
	}
} // ProcessCommand
