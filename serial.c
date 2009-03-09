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
void TxText(const char *pch)
{
	while( *pch != '\0' )
	{
		TxChar(*pch);
		pch++;
	}
}

void ShowPrompt(void)
{
	TxText(SerPrompt);
}

// send a character to the serial port
void TxChar(char ch)
{
	while( PIR1bits.TXIF == 0 ) ;	// wait for transmit ready
	TXREG = ch;		// put new char
	// register W must be retained on exit!!!! Why???
}

// converts an uintigned byte to decimal and send it
void TxValU(uint8 v)
{	
	uint8 nival;

	nival = v;

	v = nival / 100;
	TxChar(v+'0');
	nival %= 100;		// Einsparpotential: Modulo als Mathlib

	v = nival / 10;
	TxChar(v+'0');
	nival %= 10;

	TxChar(nival+'0');
}

// converts a nibble to HEX and sends it
void TxNibble(uint8 v)
{
	uint8 nival;

	nival = v + '0';
	if( nival > '9' )
		nival += 7;		// A to F
	TxChar(nival);
}

// converts an uintigned byte to HEX and sends it
void TxValH(uint8 v)
{
	TxNibble(v >> 4);
	TxNibble(v & 0x0f);
}

// converts an uintigned double byte to HEX and sends it
void TxValH16(uint16 v)
{
	TxValH(v >> 8);
	TxValH(v & 0xff);
} // TxValH16

// converts a signed byte to decimal and send it
// because of dumb compiler nival must be declared as uintigned :-(
void TxValS(int8 v)
{
	if( v < 0 )
	{
		TxChar('-');	// send sign
		v = -v;
	}
	else
		TxChar('+');	// send sign

	TxValU(v);
}

void TxNextLine(void)
{
	TxChar(0x0d);
	TxChar(0x0a);
}

// if a character is in the buffer
// return it. Else return the NUL character
char RxChar(void)
{
	uint8 ch;
	
	if( PIR1bits.RCIF )	// a character is waiting in the buffer
	{
		if( RCSTAbits.OERR || RCSTAbits.FERR )	// overrun or framing error?
		{
			RCSTAbits.CREN = 0;	// diable, then re-enable port to
			RCSTAbits.CREN = 1;	// reset OERR and FERR bit
			ch = RCREG;	// dummy read
		}
		else
		{
			ch = RCREG;	// get the character
			TxChar(ch);	// echo it
			return(ch);		// and return it
		}
	}
	return( '\0' );	// nothing in buffer
}


// enter an uintigned number 00 to 99
uint8 RxNumU(void)
{
	char ch;
	uint8 nival;

	nival = 0;
	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival = ch - '0';
	nival *= 10;
	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	return(nival);
}


// enter a signed number -99 to 99 (always 2 digits)!
int8 RxNumS(void)
{
	char ch;
	int8 nival;

	nival = 0;

	_NegIn = 0;
	do
	{
		ch = RxChar();
	}
	while( ((ch < '0') || (ch > '9')) &&
           (ch != '-') );
	if( ch == '-' )
	{
		_NegIn = 1;
		do
		{
			ch = RxChar();
		}
		while( (ch < '0') || (ch > '9') );
	}
	nival = ch - '0';
	nival *= 10;

	do
	{
		ch = RxChar();
	}
	while( (ch < '0') || (ch > '9') );
	nival += ch - '0';
	if( _NegIn )
		nival = -nival;
	return(nival);
}

// send the current configuration setup to serial port
void ShowSetup(uint8 h)
{
	if( h )
	{
		TxText(SerHello);
		IK5 = _Minimum;	
	}

	TxText(SerSetup);	// send hello message
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
		if ( BaroType == BARO_ID_BMP085 )
			TxText(SerBaroBMP085);
		else
			TxText(SerBaroSMD500);
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

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
	int8  *p;
	uint8 ch;
	uint8 addr;
	uint16 addrbase, curraddr;
	int8 d;
	
	ch = RxChar();
	if( islower(ch))							// check lower case
		ch=toupper(ch);

	switch( ch )
	{
		case '\0' : break;
		case 'L'  :	// List parameters
			TxText(SerList);	// must send it (UAVPset!)
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
			break;
		case 'M'  : // modify parameters
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
			break;
		case 'S' :	// show status
			ShowSetup(0);
			break;
		case 'N' :	// neutral values
			TxText(SerNeutralR);
			TxValS(NeutralLR);

			TxText(SerNeutralN);
			TxValS(NeutralFB);

			TxText(SerNeutralY);	
			TxValS(NeutralUD);
			ShowPrompt();
			break;
		case 'R':	// receiver values
			TxText(SerRecvCh);
			TxValU(IGas);
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
			TxText(SerHelp);
			ShowPrompt();
	}
}

