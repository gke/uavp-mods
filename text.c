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


// Text page

#include "pu-test.h"
#include "bits.h"


#pragma idata menuhello
const uint8  SerHello[] = "\r\nUAVP TEST 18f2xxx V" Version " Copyright (c) 2007"
							  " Ing. Wolfgang Mahringer\r\n"
							  "UAVP comes with ABSOLUTELY NO WARRANTY\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n\0";
#pragma idata
#pragma idata menusetup
// THE FOLLOWING LINE MUST REMAIN IN THIS FORM, it is important for UAVPset!
const uint8  SerSetup[] = "\r\nProfi-Ufo TEST V" Version " ready.\r\n"
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
							  "A..analog voltages\r\n"
							  "B..call Bootloader\r\n"
							  "C..Compass test\r\n"
							  "K..Calib. Compass\r\n"
							  "I..I2C sensor bus scan\r\n"
							  "L..Linear test\r\n"
							  "H..Barometer test\r\n"
							  "N..negated RX-PPM\r\n"
							  "R..RX test\r\n"
							  "S..Show setup\r\n"
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
const uint8 SerI2CFail[] = "I2C failed!\r\n\0";
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
const uint8 SerVolt[] = " V\r\n\0";
const uint8 SerFail[] = " FAILED\0";
const uint8 SerGrad[] = " deg\r\n\0";
const uint8 SerMS[] = " ms\r\n\0";
#pragma idata

#pragma idata menurx
const uint8 SerPPMP[] = "\r\nSet pos. Rx PPM\r\n\0";
const uint8 SerPPMN[] = "\r\nSet neg. Rx PPM\r\n\0";
const uint8 SerLinG[] = " G\r\n\0";
#pragma idata

#pragma idata menucompass
const uint8 SerMagTst[] = "\r\nCompass test\r\n\0";
const uint8 SerCCalib1[] = "\r\nCalib. compass. Press any key to continue\r\n\0";
const uint8 SerCCalib2[] = "\r\n720 deg turn in ~30 seconds!\r\nPress any key when done\r\n\0"; 
const uint8 SerCCalib3[] = "\r\nCalib. complete\r\n\0"; 
const uint8 SerCCalibE[] = "\r\nCalib. error!\r\n\0";
#pragma idata 

#pragma idata menubaro
const uint8 SerBaroSMD500[] = "\r\nSMD500\0"; 
const uint8 SerBaroBMP085[] = "\r\nBMP085\0"; 
const uint8 SerBaroOK[] = "\r\nPress. is:\0";
const uint8 SerBaroT[] = "; Temp. is:\0"; 
#pragma idata
	  
#pragma idata menurx2
const uint8 SerRxTest[]="\r\nRx values:\r\n\0";
const uint8 SerRxRes[]="received vals are \0";
const uint8 SerRxFail[]="NOT valid!\r\n\0";
const uint8 SerRxOK[]="OK!\r\n\0";
const uint8 SerRxNN[]="(no new values)\r\n\0";
#pragma idata


