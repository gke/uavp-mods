// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
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
const uint8 SerHello[] = "\r\nUAVP " Version " Copyright (c) 2007-9"
							  " G.K. Egan & W. Mahringer\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";

const uint8 SerSetup[] = "\r\nUAVP V" Version " ready.\r\n"

#ifdef DEBUG_SENSORS
	"Debug: Sensors\r\n"
#endif
#ifdef DEBUG_MOTORS
	"Debug: Motors\r\n"
#endif

#ifdef RX_DEFAULT
	"Rx: PPM Odd Channel\r\n"
#endif
#ifdef RX_PPM
	"Rx: PPM Composite\r\n"
#endif
#ifdef RX_DSM2
	"Rx: PPM DSM2\r\n"
#endif

#ifdef ESC_PPM
	"ESC: PPM\r\n"
#endif
#ifdef ESC_YGEI2C
	"ESC: YGE I2C\r\n"
#endif
#ifdef ESC_HOLGER
	"ESC: Holger I2C\r\n"
#endif
	
	"Gyros: "
#ifdef OPT_ADXRS300
	"ADXRS300 (Roll/Pitch/Yaw)\r\n"
#endif
#ifdef OPT_ADXRS150
	"ADXRS150 (Roll/Pitch/Yaw)\r\n"
#endif
#ifdef OPT_IDG
	"ADXRS300 (Yaw), IDG300 (Roll/Pitch)\r\n"
#endif
	"Accelerometers ";
#pragma idata

#pragma idata menu2
const uint8  SerHelp[]  = "\r\nCommands:\r\n"
	"L...List param\r\n"
	"M...Modify param\r\n"
	"S...Show setup\r\n"
	"N...Neutral values\r\n"
	"R...Show receiver channels\r\n"
	"B...start Boot-Loader\r\n";
#pragma idata

void ShowPrompt(void)
{
	TxString("\r\n>");
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
		TxString("ONLINE\r\n");
	else
		TxString("not available\r\n");

	TxString("Compass ");
	if( _UseCompass )
		TxString("ONLINE\r\n");
	else
		TxString("not available\r\n");

	TxString("Baro ");
	if( _UseBaro )
		if ( BaroType == BARO_ID_BMP085 )
			TxString("BMP085\r\n");
		else
			TxString("SMD500\r\n");
	else
		TxString("not available\r\n");

	ReadParametersEE();
	TxString("Throttle Ch");
	if( FutabaMode )
		TxChar('3');
	else
		TxChar('1');

	TxString("\r\nSelected parameter set: ");
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

	if ( ch != NUL )
	{
		if( islower(ch))							// check lower case
			ch=toupper(ch);
	
		switch( ch )
		{
			case 'L'  :	// List parameters
				TxString("\r\nParameter list for set #");	// do not change (UAVPset!)
				if( IK5 > _Neutral )
					TxChar('2');
				else
					TxChar('1');
				ReadParametersEE();
				addr = 1;
				for(p = &FirstProgReg; p <= &LastProgReg; p++)
				{
					TxString("\r\nRegister ");
					TxValU(addr++);
					TxString(" = ");
					d = *p;
					TxValS(d);
				}
				ShowPrompt();
				break;
			case 'M'  : // modify parameters
				LedBlue_ON;
				TxString("\r\nRegister ");
				addr = RxNumU()-1;
				TxString(" = ");
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
				TxString("\r\nNeutral Roll:");
				TxValS(NeutralLR);
	
				TxString(" Ptch:");
				TxValS(NeutralFB);
	
				TxString(" Yaw:");	
				TxValS(NeutralUD);
				ShowPrompt();
				break;
			case 'R':	// receiver values
				TxString("\r\nT:");
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
	}
} // ProcessComCommand

