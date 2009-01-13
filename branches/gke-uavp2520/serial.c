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
const char SerHello[] = "\r\nU.A.V.P. V" Version " (c) 2008"
							  " Ing. Wolfgang Mahringer\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";

const char  SerSetup[] = "\r\nProfi-Ufo V" Version " ready.\r\n"
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
void SendComText(const uint8 *pch)
{
	while( *pch != '\0' )
		SendComChar(*pch++);
}// SendComText

void ShowPrompt(void)
{
	SendComText(SerPrompt);
} // ShowPrompt

// send a character to the serial port
void SendComChar(char ch)
{
	while( !PIR1bits.TXIF ) ;						// wait for transmit ready
	TXREG = ch;										// put new char
}

// converts an unsigned byte to decimal and send it
void SendComValU(uint8 v)
{
	uint8 ch;

	ch = v / 100;
	SendComChar(ch + '0');
	v %= 100;										// extract remainder
	
	ch = v / 10;
	SendComChar(ch + '0');
	v %= 10;

	SendComChar(v + '0');
} // SendComValU

// converts a nibble to HEX and sends it
void SendComNibble(uint8 v)
{
	if( v > 9 )
		v += ('A'-10);
	else
		v += '0';
	SendComChar(v);
} // SendComNibble 

// converts an unsigned byte to HEX and sends it
void SendComValH(uint8 v)
{
	SendComNibble(v >> 4);
	SendComNibble(v & 0x0f);
} // SendComValH

// converts an unsigned double byte to HEX and sends it
void SendComValH16(uint16 v)
{
	SendComValH(Upper8(v));
	SendComValH(Lower8(v));
} // SendComValH16

// converts a signed byte to decimal and send it
void SendComValS(int8 v)
{
	if( v < 0 )
	{
		SendComChar('-');							// send sign
		v = -v;
	}
	else
		SendComChar('+');

	SendComValU(v);
} // SendComValS

void SendComNextLine(void)
{
	SendComChar(0x0d);
	SendComChar(0x0a);
} // SendComNextLine

#ifdef READABLE
void TxVal(int32 v, uint8 dp, uint8 sep)
{
  uint8 s[12];
  int8 c, rem;
  int32 newv;
 
  if (v<0)
    {
    SendComChar('-');
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
    SendComChar('0');
    SendComChar('.');
    }
  do
    {
    c--;
    SendComChar(s[c]);
    if ((c==dp)&&(c>0)) 
      SendComChar('.');
    }
  while (c>0);
  if (sep!=0)
    SendComChar(sep);
} // TxVal
#endif // READABLE

// if a character is in the buffer
// return it. Else return the NUL character
uint8 RecvComChar(void)
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
			SendComChar(ch);						// echo it
			return(ch);								// and return it
		}
	}
	return( '\0' );									// nothing in buffer
} // RecvComChar

// enter an unsigned number 00 to 99
uint8 RecvComNumU(void)
{
	char ch;
	uint8 v;

	do
		ch = RecvComChar();
	while( (ch < '0') || (ch > '9') );
	v = (ch - '0') * 10;
	do
		ch = RecvComChar();
	while( (ch < '0') || (ch > '9') );
	v += ch - '0';
	return(v);
} // RecvComNumU

// enter a signed number -99 to 99 (always 2 digits)!
int16 RecvComNumS(void)
{
	char ch;
	uint8 v;
	uint8 neg;

	neg = false;
	do
		ch = RecvComChar();
	while( ((ch < '0') || (ch > '9')) && (ch != '-') );
	if( ch == '-' )
	{
		neg = true;
		do
			ch = RecvComChar();
		while( (ch < '0') || (ch > '9') );
	}
	v = (ch - '0') * 10;
	do
		ch = RecvComChar();
	while( (ch < '0') || (ch > '9') );
	v += ch - '0';
	if ( neg )
		v = -v;
	return(v);
} // RecvComNumS

// send the current configuration setup to serial port
void ShowSetup(uint8 Hello)
{
	if( Hello )
	{
		SendComText(SerHello);
		IK5 = _Minimum;	
	}

	SendComText(SerSetup);							// send hello message
	if( _UseLISL )
		SendComText(SerLSavail);
	else
		SendComText(SerLSnone);

	SendComText(SerCompass);
	if( _UseCompass )
		SendComText(SerLSavail);
	else
		SendComText(SerLSnone);

	SendComText(SerBaro);
	if( _UseBaro )
		SendComText(SerLSavail);
	else
		SendComText(SerLSnone);

	ReadParametersEE();
	SendComText(SerChannel);
	if( FutabaMode )
		SendComText(SerFM_Fut);
	else
		SendComText(SerFM_Grp);

	SendComText(SerSelSet);
	if( IK5 > _Neutral )
		SendComChar('2');
	else
		SendComChar('1');
	
	ShowPrompt();
} // ShowSetup

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
	int8 *p;
	uint8 ch;
	uint8 addr;
	uint16 addrbase, curraddr;
	int8 d;

	ch = RecvComChar();

	if( islower(ch))								// check lower case
		ch=toupper(ch);

	if (ch == '\0') // switch would have been more elegant but code too big/slow!
	{
		// most common NULL case
	}
	else
	if (ch == 'L')	// List parameters
	{
			SendComText(SerList);					// must send it (UAVPset!)
			if( IK5 > _Neutral )
				SendComChar('2');
			else
				SendComChar('1');
			ReadParametersEE();
			addr = 1;
			for(p = &FirstProgReg; p <= &LastProgReg; p++)
			{
				SendComText(SerReg1);
				SendComValU(addr++);
				SendComText(SerReg2);
				d = *p;
				SendComValS(d);
			}
			ShowPrompt();
	}
	else
	if (ch == 'M')// modify parameters
	{ 
			LedBlue_ON;
			SendComText(SerReg1);
			addr = RecvComNumU()-1;
			SendComText(SerReg2);
			d = RecvComNumS();
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
			SendComText(SerNeutralR);
			SendComValS(NeutralLR);

			SendComText(SerNeutralN);
			SendComValS(NeutralFB);

			SendComText(SerNeutralY);
			SendComValS(NeutralUD);
			ShowPrompt();
	}
	else
	if (ch == 'R')// receiver values
	{
			SendComText(SerRecvCh);
			SendComValU(IGas);
			SendComChar(',');
			SendComChar('R');
			SendComChar(':');
			SendComValS(IRoll);
			SendComChar(',');
			SendComChar('N');
			SendComChar(':');
			SendComValS(IPitch);
			SendComChar(',');
			SendComChar('Y');
			SendComChar(':');
			SendComValS(IYaw);
			SendComChar(',');
			SendComChar('5');
			SendComChar(':');
			SendComValU(IK5);
			SendComChar(',');
			SendComChar('6');
			SendComChar(':');
			SendComValU(IK6);
			SendComChar(',');
			SendComChar('7');
			SendComChar(':');
			SendComValU(IK7);
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
			SendComText(SerHelp);
			ShowPrompt();
	}
} // ProcessComCommand
