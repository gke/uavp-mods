// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008-9 by Prof. Greg Egan               =
// =     Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer       =
// =                          http://uavp.ch                             =
// =======================================================================

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

#include "uavx.h"

// Prototypes

void ShowPrompt(void);
void ShowSetup(uint8);
void ProcessComCommand(void);

#pragma idata menuhello
const rom uint8  SerHello[] = "\r\nUAVX TEST V" Version "\r\nCopyright (c) 2007-9"
							  " G.K. Egan & W. Mahringer\r\n"
							  "Comes with ABSOLUTELY NO WARRANTY\r\n"
							  "This is FREE SOFTWARE, see GPL license!\r\n";
#pragma idata
#pragma idata menusetup
// THE FOLLOWING LINE MUST REMAIN IN THIS FORM, it is important for UAVPset!
const rom uint8  SerSetup[] = "\r\nUAVX TEST V" Version " ready.\r\n"

	"GPS enabled\r\n"

#ifdef DEBUG_SENSORS
	"Debug: Sensors\r\n"
#endif

#ifdef RX_DEFAULT
	"Rx: PPM odd channels only\r\n"
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

#pragma idata menuhelp
const rom uint8 SerHelp[] = "\r\nCommands:\r\n"
	"V..Analog ch.\r\n"
	"B..Boot\r\n"
	"C..Compass test\r\n"
	"K..Calib. Compass\r\n"
	"G..GPS test (Use HyperTerm)\r\n"
	"H..Baro. test\r\n"
	"I..I2C bus scan\r\n"
	"A..Linear test\r\n"
// use normal param change	"P..Negate Rx-PPM\r\n"
	"X..RX test\r\n"
	"S..Setup\r\n"
	#ifdef ESC_YGEI2C
	"Y..Prog. YGE\r\n"
	#endif
	"1-8..Power output test\r\n"; // last line must be in this form for UAVPSet

void ShowPrompt(void)
{
	TxString("\r\n# ");
} // ShowPrompt

// send the current configuration setup to serial port
void ShowSetup(uint8 h)
{
	if( h )
		TxString(SerHello);

	TxString(SerSetup);	// send hello message
	if( _UseLISL )
		TxString("ONLINE\r\n\0");
	else
		TxString("not available\r\n");

	// check for compass device
	TxString("Compass ");	// send hello message
	if( _UseCompass ) 
		TxString("ONLINE\r\n");
	else
		TxString("not available\r\n");

	// check for altimeter device
	TxString("Barometer ");	// send hello message
	if( _UseBaro ) 
		TxString("ONLINE\r\n");
	else
		TxString("not available\r\n");

	ShowPrompt();
} // ShowSetup

// if a command is waiting, read and process it.
// Do NOT call this routine while in flight!
void ProcessComCommand(void)
{
    int8 *p;
	uint8 i, ch;
	uint8 addr;
	uint16 addrbase, curraddr;
	int8 d;

	ch = PollRxChar();

	if ( !_ReceivingGPS )
	{
		if ( ch != NUL )
		{
			if( islower(ch))
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
				LEDBlue_ON;
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
	
				// This is not strictly necessary as UAVPSet enforces it.
				// Hovever direct edits of parameter files can easily exceed
				// intermediate arithmetic limits.
				if ( ((int16)Abs(YawIntFactor) * (int16)YawIntLimit) > 127 )
				{
					d = 127 / YawIntFactor;
					WriteEE(addrbase + (uint16)(&YawIntLimit - &FirstProgReg), d);
				}
	
				LEDBlue_OFF;
				ShowPrompt();
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
			case 'X'  :	// Receiver test			
				ReceiverTest();
				ShowPrompt();
				break;
			case 'R'  :	// Receiver display
				TxString("\r\nT:");TxValU(IGas);
				TxString(",R:");TxValS(IRoll);
				TxString(",N:");TxValS(IPitch);
				TxString(",Y:");TxValS(IYaw);
				TxString(",5:");TxValU(IK5);
				TxString(",6:");TxValU(IK6);
				TxString(",7:");TxValU(IK7);
				ShowPrompt();
				break;
			case 'V' :	// analog test
				AnalogTest();
				ShowPrompt();
				break;
			case 'G' : // GPS test
				GPSTest();
				ShowPrompt();
				break;			
			case 'A' :	// linear sensor
				LinearTest();
				ShowPrompt();
				break;
			case 'I':
				TxString("\r\nI2C devices ...\r\n");
				TxVal32(ScanI2CBus(),0,0);
				TxString(" device(s) found\r\n");
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
				TxString("\r\nOutput test\r\n");
				TxChar(ch);
				TxChar(':');
				switch( ch )
				{
					case '1': TxString("Aux2");  break;
					case '2': TxString("Blue");  break;
					case '3': TxString("Red");   break;
					case '4': TxString("Green"); break;
					case '5': TxString("Aux1");  break;
					case '6': TxString("Yellow");break;
					case '7': TxString("Aux3");  break;
					case '8': TxString("Beeper");  break;
				}
				TxNextLine();
				PowerOutput(ch-'1');
				ShowPrompt();
				break;
			/* use normal config change
			case 'P':	// toggle PPM polarity
				TogglePPMPolarity();
				ShowPrompt();
				break;
			*/
		#ifdef ESC_YGEI2C
		case 'Y':	// configure YGE30i EScs
			ConfigureESCs();
			ShowPrompt();
			break;
		#endif // ESC_YGEI2C

			case 'B':	// call bootloader
				TxNextLine();
				DisableInterrupts;
				BootStart();
			case '?'  : // help
				TxString(SerHelp);
				ShowPrompt();
			}
		}
	}
} // ProcessComCommand

