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

// Serial support (RS232 option)

#include "c-ufo.h"
#include "bits.h"

// data strings

#pragma idata menu1
const char SerHello[] = "\r\nU.A.V.P. V" Version " (c) 2007"
							  " Ing. Wolfgang Mahringer\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";

const char SerSetup[] = "\r\nProfi-Ufo V" Version " ready.\r\n"
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

const char  SerLSavail[]="ONLINE\r\n";
const char  SerLSnone[]= "not available\r\n";
const char  SerBaro[]=   "Baro ";
const char  SerBaroBMP085[]=   "BMP085\r\n";
const char  SerBaroSMD500[]=   "SMD500\r\n";
const char  SerChannel[]="Throttle Ch";
const char  SerFM_Fut[]= "3";
const char  SerFM_Grp[]= "1";

#pragma idata
#pragma idata menu2

const char  SerCompass[]="Compass ";
const char  SerReg1[]  = "\r\nRegister ";
const char  SerReg2[]  = " = ";
const char  SerPrompt[]= "\r\n>";
const char  SerHelp[]  = "\r\nCommands:\r\n"
					 		  "L...List param\r\n"
							  "M...Modify param\r\n"
							  "S...Show setup\r\n"
							  "N...Neutral values\r\n"
							  "R...Show receiver channels\r\n"
							  "B...start Boot-Loader\r\n";

// THE FOLLOWING LINE NOT TO BE CHANGED, it is important for UAVPset
const char  SerList[]  = "\r\nParameter list for set #";
const char  SerSelSet[]= "\r\nSelected parameter set: ";

const char  SerNeutralR[]="\r\nNeutral Roll:";
const char  SerNeutralN[]=" Ptch:";
const char  SerNeutralY[]=" Yaw:";

const char  SerRecvCh[]=  "\r\nT:";
#pragma idata

// transmit a fix text from a table
void SendComText(const char *pch)
{
	while( *pch != '\0' )
	{
		SendComChar(*pch);
		pch++;
	}
}

void ShowPrompt(void)
{
	SendComText(SerPrompt);
}

// send a character to the serial port
void SendComChar(char ch)
{
	while( PIR1bits.TXIF == 0 ) ;	// wait for transmit ready
	TXREG = ch;		// put new char
	// register W must be retained on exit!!!! Why???
}

static uns8 nival;
static char ch;

// converts an unsigned byte to decimal and send it
void SendComValU(uns8 v)
{
	nival = v;

	v = nival / 100;
	SendComChar(v+'0');
	nival %= 100;		// Einsparpotential: Modulo als Mathlib

	v = nival / 10;
	SendComChar(v+'0');
	nival %= 10;

	SendComChar(nival+'0');
}

// converts a nibble to HEX and sends it
void SendComNibble(uns8 v)
{
	nival = v + '0';
	if( nival > '9' )
		nival += 7;		// A to F
	SendComChar(nival);
}

// converts an unsigned byte to HEX and sends it
void SendComValH(uns8 v)
{
	uns8 nival2;

	nival2 = v;
	SendComNibble(nival2 >> 4);
	SendComNibble(nival2 & 0x0f);
}

// converts an unsigned double byte to HEX and sends it
void SendComValH16(uns16 v)
{
	SendComValH(v >> 8);
	SendComValH(v & 0xff);
} // SendComValH16

// converts a signed byte to decimal and send it
// because of dumb compiler nival must be declared as unsigned :-(
void SendComValS(uns8 v)
{
	nival = v;
	if( (int8)nival < 0 )
	{
		SendComChar('-');	// send sign
		nival = -(int8)nival;
	}
	else
		SendComChar('+');	// send sign

	SendComValU(nival);
}

// if a character is in the buffer
// return it. Else return the NUL character
char RecvComChar(void)
{
	uns8 Ch;
	
	if( PIR1bits.RCIF )	// a character is waiting in the buffer
	{
		if( RCSTAbits.OERR || RCSTAbits.FERR )	// overrun or framing error?
		{
			RCSTAbits.CREN = 0;	// diable, then re-enable port to
			RCSTAbits.CREN = 1;	// reset OERR and FERR bit
			Ch = RCREG;	// dummy read
		}
		else
		{
			Ch = RCREG;	// get the character
			SendComChar(Ch);	// echo it
			return(Ch);		// and return it
		}
	}
	return( '\0' );	// nothing in buffer
}


// enter an unsigned number 00 to 99
uns8 RecvComNumU(void)
{

	nival = 0;
	do
	{
		ch = RecvComChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival = ch - '0';
	nival *= 10;
	do
	{
		ch = RecvComChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	return(nival);
}


// enter a signed number -99 to 99 (always 2 digits)!
int8 RecvComNumS(void)
{
	int8 nival;

	nival = 0;

	_NegIn = 0;
	do
	{
		ch = RecvComChar();
	}
	while( ((ch < '0') || (ch > '9')) &&
           (ch != '-') );
	if( ch == '-' )
	{
		_NegIn = 1;
		do
		{
			ch = RecvComChar();
		}
		while( (ch < '0') || (ch > '9') );
	}
	nival = ch - '0';
	nival *= 10;

	do
	{
		ch = RecvComChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	if( _NegIn )
		nival = -nival;
	return(nival);
}

// send the current configuration setup to serial port
void ShowSetup(uns8 h)
{
	if( h )
	{
		SendComText(SerHello);
		IK5 = _Minimum;	
	}

	SendComText(SerSetup);	// send hello message
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
		if ( BaroType == BARO_ID_BMP085 )
			SendComText(SerBaroBMP085);
		else
			SendComText(SerBaroSMD500);
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
}

void ProgRegister(void)
{
	EECON1bits.EEPGD = 0;
	EECON1bits.WREN = 1;		// enable eeprom writes
	INTCONbits.GIE = 0;
	EECON2 = 0x55;	// fix prog sequence (see 16F628A datasheet)
	EECON2 = 0xAA;
	EECON1bits.WR = 1;			// start write cycle
	INTCONbits.GIE = 1;
	while( EECON1bits.WR == 1 );	// wait to complete
	EECON1bits.WREN = 0;	// disable EEPROM write
}

int16 nila1;

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
	int8  *p;
	uns8 ch;
	uns8 addr;
	uns16 addrbase, curraddr;
	int8 d;
	
	ch = RecvComChar();
	if( islower(ch))							// check lower case
		ch=toupper(ch);

	switch( ch )
	{
		case '\0' : break;
		case 'L'  :	// List parameters
			SendComText(SerList);	// must send it (UAVPset!)
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
			break;
		case 'M'  : // modify parameters
			LedBlue_ON;
			SendComText(SerReg1);
			addr = RecvComNumU()-1;
			SendComText(SerReg2);
			d = RecvComNumS();
			if( IK5 > _Neutral )
				addrbase = _EESet2;
			else
				addrbase = _EESet1;
			WriteEE(addrbase + (uns16)addr, d);	

			// update transmitter config bits in the other parameter set
			if( addr ==  (&ConfigParam - &FirstProgReg) )
			{									
				if( IK5 > _Neutral )
					addrbase = _EESet1;				
				else
					addrbase = _EESet2;	
				// mask only bits _FutabaMode and _NegativePPM
				d &= 0x12;		
				d = (ReadEE(addrbase + (uns16)addr) & 0xed) | d;
				WriteEE(addrbase + (uns16)addr, d);
			}
			LedBlue_OFF;
			ShowPrompt();
			break;
		case 'S' :	// show status
			ShowSetup(0);
			break;
		case 'N' :	// neutral values
			SendComText(SerNeutralR);
			SendComValS(NeutralLR);

			SendComText(SerNeutralN);
			SendComValS(NeutralFB);

			SendComText(SerNeutralY);
			Yp -= 1024;		// subtract 1g (vertical sensor)
			SendComValS(NeutralUD);
			ShowPrompt();
			break;
		case 'R':	// receiver values
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
			break;

		case 'B':	// call bootloader
//#asm
//			movlw	0x1f
//			movwf	PCLATH
//			dw	0x2F00
//#endasm
//			DisableInterrupts;
//zzz			BootStart();							// never comes back!
		
#ifndef TESTOUT	
		case 'T':
			RE = 10;
			PE = 20;
			Rw = 30;
			Pw = 40;
			//MatrixCompensate();
			ShowPrompt();
			break;
#endif

		case '?'  : // help
			SendComText(SerHelp);
			ShowPrompt();
	}
}

