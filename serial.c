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

#include "pu-test.h"
#include "bits.h"

int24	nilgval;

// data strings

void SendComCRLF(void)
{
	SendComChar(0x0D);
	SendComChar(0x0A);
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
}


// converts an unsigned byte to decimal and send it
void SendComValU(uns8 nival)
{
	uns16 i;

	i = nival / 100;
	SendComChar(i +'0');
	nival %= 100;

	i = nival / 10;
	SendComChar(i+'0');
	nival %= 10;

	SendComChar(nival+'0');
}


// converts a nibble to HEX and sends it
void SendComNibble(uns8 nival)
{
	nival += '0';
	if( nival > '9' )
		nival += 7;		// A to F
	SendComChar(nival);
}

// converts an unsigned byte to HEX and sends it
void SendComValH(uns8 nihex)
{
	SendComNibble(nihex >> 4);
	SendComNibble(nihex & 0x0F);
}

// converts an unsigned long to decimal and send it
void SendComValUL(uns8 niformat)
{
	int24 Temp, v;
	uns8 ninull=1;

	v = nilgval;

	if( niformat & VZ )
	{
		if( v >= 0 )
			SendComChar('+');
		else
		{
			SendComChar('-');
			v = -v;
		}
	}
	
	Temp = v / 10000;
	if( (Temp == 0) && (ninull == 1) && 
		((niformat & NKSMASK) < NKS4) )
	{
		if( (niformat & LENMASK) >= LEN5 )
			SendComChar(' ');
	}
	else
	{
		SendComChar(Temp+'0');
		ninull = 0;
	}
	v %= 10000;

	if( (niformat & NKSMASK) == NKS4 )
		SendComChar('.');

	Temp = v / 1000;
	if( (Temp == 0) && (ninull == 1)  && 
		((niformat & NKSMASK) < NKS3) )
	{
		if( (niformat & LENMASK) >= LEN4 )
			SendComChar(' ');
	}
	else
	{
		SendComChar(Temp+'0');
		ninull = 0;
	}
	v %= 1000;

	if( (niformat & NKSMASK) == NKS3 )
		SendComChar('.');

	Temp = v / 100;
	if( (Temp == 0) && (ninull == 1) && 
		((niformat & NKSMASK) < NKS2) )
	{
		if( (niformat & LENMASK) >= LEN3 )
			SendComChar(' ');
	}
	else
	{
		SendComChar(Temp+'0');
		ninull = 0;
	}
	v %= 100;

	if( (niformat & NKSMASK) == NKS2 )
		SendComChar('.');

	Temp = v / 10;
	if( (Temp == 0) && (ninull == 1) && 
		((niformat & NKSMASK) < NKS1)  )
	{
		if( (niformat & LENMASK) >= LEN2 )
			SendComChar(' ');
	}
	else
	{
		SendComChar(Temp+'0');
		ninull = 0;
	}
	v %= 10;

	if( (niformat & NKSMASK) == NKS1 )
		SendComChar('.');

	SendComChar(v+'0');
}

// if a character is in the buffer
// return it. Else return the NUL character
char RecvComChar(void)
{
	char chread;

	if( PIR1bits.RCIF )	// a character is waiting in the buffer
	{
		if( RCSTAbits.OERR || RCSTAbits.FERR )	// overrun or framing error?
		{
			RCSTAbits.CREN = 0;	// diable, then re-enable port to
			RCSTAbits.CREN = 1;	// reset OERR and FERR bit
			W = RCREG;	// dummy read
		}
		else
		{
			chread = RCREG;	// get the character
			SendComChar(chread);	// echo it
			return(chread);		// and return it
		}
	}
	return( '\0' );	// nothing in buffer
}

// send the current configuration setup to serial port
void ShowSetup(uns8 h)
{

	if( h )
		SendComText(SerHello);

	SendComText(SerSetup);	// send hello message
	if( _UseLISL )
		SendComText(SerLSavail);
	else
		SendComText(SerLSnone);


// check for compass device
	SendComText(SerCompass);	// send hello message
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) == I2C_ACK ) 
		SendComText(SerLSavail);
	else
		SendComText(SerLSnone);
	I2CStop();

// check for altimeter device
	SendComText(SerAlti);	// send hello message
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) == I2C_ACK ) 
		SendComText(SerLSavail);
	else
		SendComText(SerLSnone);
	I2CStop();

	ShowPrompt();
}

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
    int *p;
	char chcmd;

	chcmd = RecvComChar();
	if( islower(chcmd))							// check lower case
		chcmd=toupper(chcmd);

	switch( chcmd )
	{
		case '\0' : break;
		case 'R'  :	// Receiver test
			SendComText(SerRxTest);
			if( _NewValues )
			{
				ReceiverTest();
				_NewValues = 0;
				SendComText(SerRxRes);
				if( _NoSignal )
					SendComText(SerRxFail);
				else
					SendComText(SerRxOK);
			}
			else
				SendComText(SerRxNN);
			ShowPrompt();
			break;
		case 'A' :	// analog test
			SendComText(SerAnTest);
			AnalogTest();
			ShowPrompt();
			break;
		case 'L' :	// linear sensor
			SendComText(SerLinTst);
			if( _UseLISL )
				LinearTest();
			else
				SendComText(SerLinErr);
			ShowPrompt();
			break;
		case 'I':
			SendComText(SerI2CRun);
//			i = ScanI2CBus();
			SendComValU(ScanI2CBus());
			SendComText(SerI2CCnt);
			ShowPrompt();
			break;
		case 'C':
			SendComText(SerMagTst);
			CompassTest();
			ShowPrompt();
			break;
		case 'K':
			SendComText(SerCCalib1);
			CalibrateCompass();
			ShowPrompt();
			break;
		case 'H':	// barometer
			BaroTest();
			ShowPrompt();
			break;	
		case 'S' :	// show status
			ShowSetup(0);
			break;
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':

			SendComText(SerPowTst);
			SendComChar(chcmd);
			SendComChar(':');
			switch( chcmd )
			{
				case '1': SendComText(SerPowAux2);  break;
				case '2': SendComText(SerPowBlue);  break;
				case '3': SendComText(SerPowRed);   break;
				case '4': SendComText(SerPowGreen); break;
				case '5': SendComText(SerPowAux1);  break;
				case '6': SendComText(SerPowYellow);break;
				case '7': SendComText(SerPowAux3);  break;
				case '8': SendComText(SerPowBeep);  break;
			}
			SendComCRLF();
			PowerOutput(chcmd-'1');
			ShowPrompt();
			break;

		case 'N':	// toggle PPM polarity
			TogglePPMPolarity();
			ShowPrompt();
			break;

		case 'B':	// call bootloader
			SendComCRLF();
//_asm
//			movlw	0x1f
//			movwf	PCLATH
//			dw	0x2F00
//_endasm
//			BootStart();	// never comes back!
		case '?'  : // help
			SendComText(SerHelp);
			ShowPrompt();
	}
}

