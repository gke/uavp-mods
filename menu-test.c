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

// Menu for Test Software

#include "c-ufo.h"
#include "bits.h"

#pragma idata menuhello
const uint8  SerHello[] = "\r\nUAVPTest V18F Copyright (c) 2007-9"
							  " W. Mahringer & G.K. Egan\r\n"
							  "Comes with ABSOLUTELY NO WARRANTY\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n\0";
#pragma idata
#pragma idata menusetup
// THE FOLLOWING LINE MUST REMAIN IN THIS FORM, it is important for UAVPset!
const uint8  SerSetup[] = "\r\nUAVPTest V18F ready.\r\n"
							  "Gyro: "
#ifdef OPT_ADXRS
							  "3x ADXRS300\r\n"
#endif
#ifdef OPT_IDG
							  "1x ADXRS300, 1x IDG300\r\n"
#endif
							  "Linear sensor \0";
const uint8  SerLSavail[] = "ONLINE\r\n\0";
const uint8 SerLSnone[] = "not available\r\n\0";
const uint8 SerCompass[] = "Compass \0";
const uint8 SerAlti[] = "Barometer \0";
#pragma idata

#pragma idata menuhelp
const uint8 SerHelp[] = "\r\nCommands:\r\n"
							  "A..Analog ch.\r\n"
							  "B..Bootloader\r\n"
							  "C..Compass test\r\n"
							  "K..Calib. Compass\r\n"
							  "I..I2C bus scan\r\n"
							  "L..Linear test\r\n"
							  "H..Baro. test\r\n"
							  "N..Negated RX-PPM\r\n"
							  "R..RX test\r\n"
							  "S..Setup\r\n"
							  "1-8.."
/* CAUTION: The following line MUST REMAIN THE LAST in that help string!
   This is for UAVPset to correctly read the available commands! */
							  "Power output test\r\n\0";
#pragma idata

#pragma idata menu3
const uint8 SerAnTest[] = "\r\nAnalog ch. test:\r\n\0";
#pragma idata

#pragma idata menuacc
const uint8 SerLinTst[] = "\r\nAcc. test:\r\n\0";
const uint8 SerLinErr[] = "\r\n(Acc. not present)\r\n\0";
#pragma idata 

#pragma idata menui2c
const uint8 SerI2CRun[] = "\r\nI2C devices ...\r\n\0";
const uint8 SerI2CCnt[] = " device(s) found\r\n\0";
const uint8 SerI2CFail[] = "I2C fail!\r\n\0";
#pragma idata

#pragma idata menupower
const uint8 SerPowTst[] = "\r\nOutput test\r\n\0";
const uint8 SerPowAux1[] = "Aux1\0";
const uint8 SerPowAux2[] = "Aux2\0";
const uint8 SerPowAux3[] = "Aux3\0";
#pragma idata

#pragma idata menuleds
const uint8 SerPowBlue[] = "Blue\0";
const uint8 SerPowRed[] = "Red\0";
const uint8 SerPowGreen[] = "Green\0";
const uint8 SerPowYellow[] = "Yellow\0";
const uint8 SerPowBeep[] = "Beeper\0";
#pragma idata

#pragma idata menux
const uint8 SerSrvOK[] = "\r\nDone.\r\n\0";
const uint8 SerPrompt[] = "\r\n# \0";
#pragma idata

#pragma idata menu6
const uint8 SerFail[] = " FAIL\0";
const uint8 SerOK[] = "OK\r\n\0";
const uint8 SerGrad[] = " deg\r\n\0";
const uint8 SerMS[] = " ms\r\n\0";
#pragma idata

#pragma idata menurx
const uint8 SerPPMP[] = "Pos. Rx PPM\r\n\0";
const uint8 SerPPMN[] = "Neg. Rx PPM\r\n\0";
#pragma idata

#pragma idata menucompass
const uint8 SerMagTst[] = "\r\nCompass test\r\n\0";
const uint8 SerCCalib1[] = "\r\nCalib. compass. Any key to cont.\r\n\0";
const uint8 SerCCalib2[] = "\r\n720 deg in ~30 sec.!\r\nAny key to cont.\r\n\0"; 
#pragma idata 

#pragma idata menubaro
const uint8 SerBaroSMD500[] = "\r\nSMD500\0"; 
const uint8 SerBaroBMP085[] = "\r\nBMP085\0"; 
const uint8 SerBaroOK[] = "\r\nPress. is:\0";
const uint8 SerBaroT[] = "; Temp. is:\0"; 
#pragma idata
	  
#pragma idata menurx2
const uint8 SerRxTest[]="Rx vals:\r\n\0";
const uint8 SerRxNN[]="(no new vals)\r\n\0";
#pragma idata


void ShowPrompt(void)
{
	TxText(SerPrompt);
} // ShowPrompt

// send the current configuration setup to serial port
void ShowSetup(uint8 h)
{
	if( h )
		TxText(SerHello);

	TxText(SerSetup);	// send hello message
	if( _UseLISL )
		TxText(SerLSavail);
	else
		TxText(SerLSnone);

	// check for compass device
	TxText(SerCompass);	// send hello message
	I2CStart();
	if( SendI2CByte(COMPASS_I2C_ID) == I2C_ACK ) 
		TxText(SerLSavail);
	else
		TxText(SerLSnone);
	I2CStop();

	// check for altimeter device
	TxText(SerAlti);	// send hello message
	I2CStart();
	if( SendI2CByte(BARO_I2C_ID) == I2C_ACK ) 
		TxText(SerLSavail);
	else
		TxText(SerLSnone);
	I2CStop();

	ShowPrompt();
} // ShowSetup

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
    int8 *p;
	uint8 i, ch;

	ch = RxChar();
	if( islower(ch))							// check lower case
		ch=toupper(ch);

	switch( ch )
	{
		case NUL : break;
		case 'R'  :	// Receiver test
			ReceiverTest();
			ShowPrompt();
			break;
		case 'A' :	// analog test
			AnalogTest();
			ShowPrompt();
			break;
		case 'L' :	// linear sensor
			LinearTest();
			ShowPrompt();
			break;
		case 'I':
			TxText(SerI2CRun);
			TxVal32(ScanI2CBus(),0,0);
			TxText(SerI2CCnt);
			ShowPrompt();
			break;
		case 'C':
			DoCompassTest();
			ShowPrompt();
			break;
		case 'K':
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

			TxText(SerPowTst);
			TxChar(ch);
			TxChar(':');
			switch( ch )
			{
				case '1': TxText(SerPowAux2);  break;
				case '2': TxText(SerPowBlue);  break;
				case '3': TxText(SerPowRed);   break;
				case '4': TxText(SerPowGreen); break;
				case '5': TxText(SerPowAux1);  break;
				case '6': TxText(SerPowYellow);break;
				case '7': TxText(SerPowAux3);  break;
				case '8': TxText(SerPowBeep);  break;
			}
			TxNextLine();
			PowerOutput(ch-'1');
			ShowPrompt();
			break;

		case 'N':	// toggle PPM polarity
			TogglePPMPolarity();
			ShowPrompt();
			break;

		case 'B':	// call bootloader
			TxNextLine();
			BootStart();	// never comes back!
		case '?'  : // help
			TxText(SerHelp);
			ShowPrompt();
	}
} // ProcessComCommand

