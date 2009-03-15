// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =     Extensively rewritten Copyright (c) 2008-9 by Prof. Greg Egan   =
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

#pragma idata menu1
const uint8 SerHello[] = "\r\nUAVP V" Version " (c) 2007"
							  " Ing. Wolfgang Mahringer\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";

const uint8 SerSetup[] = "\r\nUAVP V" Version " ready.\r\n"
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

const uint8  SerLSavail[]="ONLINE\r\n";
const uint8  SerLSnone[]= "not available\r\n";
const uint8  SerBaro[]=   "Baro ";
const uint8  SerBaroBMP085[]=   "BMP085\r\n";
const uint8  SerBaroSMD500[]=   "SMD500\r\n";
const uint8  SerChannel[]="Throttle Ch";
const uint8  SerFM_Fut[]= "3";
const uint8  SerFM_Grp[]= "1";
#pragma idata

#pragma idata menu2
const uint8  SerCompass[]="Compass ";
const uint8  SerReg1[]  = "\r\nRegister ";
const uint8  SerReg2[]  = " = ";
const uint8  SerPrompt[]= "\r\n>";
const uint8  SerHelp[]  = "\r\nCommands:\r\n"
					 		  "L...List param\r\n"
							  "M...Modify param\r\n"
							  "S...Show setup\r\n"
							  "N...Neutral values\r\n"
							  "R...Show receiver channels\r\n"
							  "B...start Boot-Loader\r\n";

// THE FOLLOWING LINE NOT TO BE CHANGED, it is important for UAVPset
const uint8  SerList[]  = "\r\nParameter list for set #";
const uint8  SerSelSet[]= "\r\nSelected parameter set: ";

const uint8  SerNeutralR[]="\r\nNeutral Roll:";
const uint8  SerNeutralN[]=" Ptch:";
const uint8  SerNeutralY[]=" Yaw:";

const uint8  SerRecvCh[]=  "\r\nT:";
#pragma idata

void ShowPrompt(void)
{
	TxText(SerPrompt);
} // ShowPrompt

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
} // ShowSetup

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

			if( addr ==  (&ConfigParam - &FirstProgReg) )
				d &=0xf7; // no Double Rates

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
			BootStart();							// never comes back!
		
		case '?'  : // help
			TxText(SerHelp);
			ShowPrompt();
			break;
	}
} // ProcessComCommand

