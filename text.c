// ==============================================
// =    U.A.V.P Brushless UFO Test software     =
// =           Professional Version             =
// = Copyright (c) 2007 Ing. Wolfgang Mahringer =
// ==============================================
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// ==============================================
// =  please visit http://www.uavp.de           =
// =               http://www.mahringer.co.at   =
// ==============================================

// Text page

#pragma codepage=1
#pragma sharedAllocation

#include "pu-test.h"
#include "bits.h"

const char page2 SerHello[] = "\r\nU.A.V.P. TESTSOFTWARE V" Version " Copyright (c) 2007"
							  " Ing. Wolfgang Mahringer\r\n"
							  "U.A.V.P. comes with ABSOLUTELY NO WARRANTY\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";

// THE FOLLOWING LINE MUST REMAIN IN THIS FORM, it is important for UAVPset!
const char page2 SerSetup[] = "\r\nProfi-Ufo TEST V" Version " ready.\r\n"
							  "Gyro: "
#ifdef OPT_ADXRS
							  "3x ADXRS300\r\n"
#endif
#ifdef OPT_IDG
							  "1x ADXRS300, 1x IDG300\r\n"
#endif
							  "Linear sensors ";
const char page2 SerLSavail[]="ONLINE\r\n";
const char page2 SerLSnone[]= "not available\r\n";

const char page2 SerCompass[]="Compass sensor ";
const char page2 SerAlti[]   ="Altimeter sensor ";

const char page2 SerHelp[]  = "\r\nCommands:\r\n"
							  "A....analog voltages\r\n"
							  "B....call Bootloader\r\n"
#ifdef BOARD_3_1
							  "C....Compass test\r\n"
							  "I....I2C sensor bus scan\r\n"
							  "L....Linear sensor test\r\n"
							  "H....Barometer sensor test\r\n"
#endif
							  "N....negated RX-PPM\r\n"
							  "R....RX test\r\n"
							  "S....Show setup\r\n"
							  "V....Servo test (CAUTION!)\r\n"
#ifdef BOARD_3_0
							  "1-7.."
#endif
#ifdef BOARD_3_1
							  "1-8.."
#endif
/* CAUTION: The following line MUST REMAIN THE LAST in thas string!
            This is for UAVPset to correctly read the available commands! */
							       "Power output test\r\n";

const char page2 SerRxTest[]= "\r\nLast RX values received:\r\n";
const char page2 SerRxNN[]  = "(no new values)\r\n";
const char page2 SerRxRes[] = "received values are ";
const char page2 SerRxFail[]= "NOT valid!\r\n";
const char page2 SerRxOK[]  = "OK!\r\n";

const char page2 SerAnTest[]= "\r\nAnalog channels test:\r\n";
const char page2 SerLinTst[]= "\r\nLinear sensor test:\r\n";
const char page2 SerLinErr[]= "\r\n(linear sensor not present)\r\n";

const char page2 SerSrvRun[]= "\r\nServo test running...\r\n";

const char page2 SerI2CRun[]= "\r\nI2C bus scan running, listing slave addresses...\r\n";
const char page2 SerI2CCnt[]= " device(s) found\r\n";

const char page2 SerMagTst[]= "\r\nCompass device test\r\n";

const char page2 SerPowTst[]= "\r\nPower output test\r\n";
const char page2 SerPowAux1[]   = "Aux 1";
const char page2 SerPowAux2[]   = "Aux 2";
#ifdef BOARD_3_1
const char page2 SerPowAux3[]   = "Aux 3";
#endif
const char page2 SerPowBlue[]   = "Blue LED";
const char page2 SerPowRed[]    = "Red LED";
const char page2 SerPowGreen[]  = "Green LED";
const char page2 SerPowYellow[] = "Yellow LED";
const char page2 SerPowBeep[]   = "Beeper";

const char page2 SerSrvOK[] = "\r\nDone.\r\n";
const char page2 SerPrompt[]= "\r\n# ";
const char page2 SerVolt[]=	  " Volt\r\n";
const char page2 SerFail[]=   " FAILED";
// not possible because of dumb compiler CC5X
//const char page2 SerGrad[]=	  " °\r\n";
const char page2 SerGrad[]=	  " deg\r\n";
const char page2 SerI2CFail[]="I2C comm failed!\r\n";
const char page2 SerMS[]=	  " ms\r\n";

const char page2 SerPPMP[]=  "\r\nSetting pos. RX PPM\r\n";
const char page2 SerPPMN[]=  "\r\nSetting neg. RX PPM\r\n";

const char page2 SerLinG[]=  " G\r\n";

const char page2 SerBaroOK[]="\r\nAir pressure is:";

// transmit a fix text from a table
void SendComText(uns8 nitext)
{
	bank1 const char *pch;

	switch(nitext)
	{
        case _SerHello:		pch = SerHello; 	break;
        case _SerSetup:		pch = SerSetup; 	break;
        case _SerLSavail:	pch = SerLSavail; 	break;
        case _SerLSnone:	pch = SerLSnone; 	break;
		case _SerCompass:	pch = SerCompass;	break;
        case _SerHelp:		pch = SerHelp; 		break;
        case _SerRxTest:	pch = SerRxTest; 	break;
        case _SerRxNN:		pch = SerRxNN; 		break;
        case _SerRxRes:		pch = SerRxRes; 	break;
        case _SerRxFail:	pch = SerRxFail; 	break;
        case _SerRxOK:		pch = SerRxOK; 		break;
        case _SerAnTest:	pch = SerAnTest; 	break;

#ifdef BOARD_3_1
        case _SerLinTst:	pch = SerLinTst; 	break;
        case _SerLinErr:	pch = SerLinErr; 	break;
        case _SerI2CRun:	pch = SerI2CRun; 	break;
        case _SerI2CCnt:	pch = SerI2CCnt; 	break;
        case _SerMagTst:	pch = SerMagTst;	break;
#endif
        case _SerSrvRun:	pch = SerSrvRun; 	break;
        case _SerPowTst:	pch = SerPowTst;	break;
        case _SerPowAux1:	pch = SerPowAux1; 	break;
        case _SerPowAux2:	pch = SerPowAux2; 	break;
#ifdef BOARD_3_1
        case _SerPowAux3:	pch = SerPowAux3; 	break;
#endif
        case _SerPowBlue:	pch = SerPowBlue; 	break;
        case _SerPowRed:	pch = SerPowRed; 	break;
        case _SerPowGreen:	pch = SerPowGreen; 	break;
        case _SerPowYellow:	pch = SerPowYellow; break;
        case _SerPowBeep:	pch = SerPowBeep; 	break;
        case _SerSrvOK:		pch = SerSrvOK;		break;
        case _SerPrompt:	pch = SerPrompt; 	break;
        case _SerVolt:		pch = SerVolt;	 	break;
		case _SerFail:		pch = SerFail;		break;
#ifdef BOARD_3_1
		case _SerGrad:		pch = SerGrad;		break;
		case _SerI2CFail:	pch = SerI2CFail;	break;
#endif
		case _SerMS:		pch = SerMS;		break;
		case _SerPPMP:		pch = SerPPMP;		break;
		case _SerPPMN:		pch = SerPPMN;		break;
		case _SerLinG:		pch = SerLinG;		break;
#ifdef BOARD_3_1
		case _SerAlti:		pch = SerAlti;		break;
		case _SerBaroOK:	pch = SerBaroOK;	break;
#endif
	}
	while( *pch != '\0' )
	{
		SendComChar(*pch);
		pch++;
	}
}


