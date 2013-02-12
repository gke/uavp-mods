// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "uavx.h"

void ShowPrompt(void);
void ShowRxSetup(void);
void ShowSetup(void);
void ProcessCommand(void);

int8 PTemp[64];

#ifdef TESTING
const rom uint8 SerHello[] = "UAVX TEST " Version 
#else
const rom uint8 SerHello[] = "UAVX " Version 							 
#endif // TESTING
 							  " Copyright 2008 G.K. Egan & 2007 W. Mahringer\r\n"
							  "This is FREE SOFTWARE and comes with ABSOLUTELY NO WARRANTY "
							  "see http://www.gnu.org/licenses/!\r\n";
#pragma idata

#pragma idata menuhelp
const rom uint8 SerHelp[] = "\r\nCommands:\r\n"
	#ifdef TESTING

//	"B..Load UAVX hex file\r\n"
	"C..Comp/Mag test\r\n"
	"D..Load default params\r\n"
	"G..Gyro & Acc test\r\n"
	"H..Baro & RF test\r\n"
	"I..I2C bus scan\r\n"
	"K..Cal Comp/Mag\r\n"
//	"M..Modify params\r\n"
	"N..Cal Acc\r\n"
	"P..Rx test\r\n"
	"S..Show config\r\n"
	"V..Battery test\r\n"
	"X..Flight stats\r\n"
#ifdef INC_I2C_PROG
	"Y..Program YGE I2C ESC\r\n"
#endif

	#else

//	"B..Load UAVX hex file\r\n"
	"D..Load default param set\r\n"
	"S..Show config\r\n"
	"V..Battery test\r\n"
	"X..Flight stats\r\n"

	#endif // TESTING
	"1-8..Individual LED/buzzer test\r\n"; // last line must be in this form for UAVPSet
#pragma idata

#pragma idata rx_mnem_chars
const rom uint8 RxChMnem[] = "TAERG12345";
#pragma idata

void ShowPrompt(void)
{
	TxString("\r\n>");
} // ShowPrompt

void ShowRxSetup(void)
{
	if ( F.UsingCompoundPPM ) {
		TxString("Serial PPM frame (");
		if ( PPMPosPolarity )
			TxString("Pos. Polarity)");
		else
			TxString("Neg. Polarity)");
	} else
		TxString("Odd Rx Channels PPM");
} // ShowRxSetup

#pragma idata airframenames
const rom uint8 * AFName[AFUnknown+1] = { "Tri", "Quad", "Hex", "Y6", "Oct",
		"OctCoax", "Heli", "Elev", "Ail", "V", "VTOL", "Gimbal", "Unknown" };
#pragma idata

void ShowAFType(void)
{
	TxString(AFName[AF_TYPE]);
} // ShowAFType

void ShowSetup(void)
{
	static uint8 i;

	TxNextLine();
	TxString(SerHello);

	TxString("\r\nClock: 40MHz");

	TxString("\r\nAircraft: "); ShowAFType();

	TxString("\r\nPID Cycle (mS): ");
	TxVal32(PIDCyclemS,0,' ');
	TxNextLine();

	#ifdef INC_CYCLE_STATS
	for (i = 0 ; i <(uint8)15; i++ )
		if (CycleHist[i] > 0) {
			TxChar(HT);
			TxVal32(i,0,HT);
			TxVal32(CycleHist[i],0,0);
			TxNextLine();
		}
	#endif // INC_CYCLE_STATS

	#ifdef MULTICOPTER
		TxString("\r\nForward Flight: ");
		TxVal32((int16)Orientation * 75L, 1, 0);
		TxString("deg CW from K1 motor(s)");
	#endif // MULTICOPTER

	TxString("\r\nAccs: ");
	ShowAccType();	

	TxString("\r\nGyros: "); 
	ShowGyroType(GyroType);

	TxString("\r\nBaro: "); ShowBaroType();

	TxString("\r\nCompass: ");
	ShowCompassType();
	if(( F.MagnetometerValid) && ( CompassType != HMC6352Compass )) {
		TxString(" Offset ");
		TxVal32(COMPASS_OFFSET_QTR * 90,0,0);
		TxString("deg.");
	}

	TxString("\r\nESCs: ");	
	ShowESCType();
	if ( P[ESCType] != ESCPPM ) {
		TxString(" {");
		for ( i = 0; i < (uint8)NO_OF_I2C_ESCS; i++ )
			if ( ESCI2CFail[i] )
				TxString(" Fail");
			else
				TxString(" OK");
		TxString(" }");
	}	
	
	TxString("\r\nTx/Rx: ");
	ShowRxSetup();
	TxString("Tx Mode ");
	TxChar('1' + F.UsingTxMode2);

	TxString("\r\nParam set: "); // must be exactly this string as UAVPSet expects it
	TxChar('0' + ParamSet);	

	TxString("\r\n\r\nNav:\r\n\tAutoland ");
	if ( F.UsingRTHAutoDescend )
		TxString("ENABLED\r\n");
	else
		TxString("disabled\r\n");

	if ( F.AllowTurnToWP )
		TxString("\tTurn to WP\r\n");
	else
		TxString("\tHold heading\r\n");

	if ( !F.AllowNavAltitudeHold )
		TxString("\tAlt Hold Manual CAUTION\r\n");

	TxString("\r\nALARM (if any):\r\n");
	if ( (( NoOfControls&1 ) != (uint8)1 ) && !F.UsingCompoundPPM ) {
		TxString("\tODD Ch Inp selected, EVEN number used - reduced to ");
		TxVal32(NoOfControls,0,0);
		TxNextLine();
	}
	if ( !F.FailsafesEnabled )
		TxString("\tYOU have DISABLED Failsafes\r\n");

	#ifdef TESTING
		TxString("\tTEST VER. - No Motors\r\n");
	#endif // TESTING

	if ( !F.ParametersValid )
		TxString("\tINVALID flight params (PID)!\r\n");
	
	if ( !F.BaroAltitudeValid )
		TxString("\tBaro. FAIL\r\n");

	if ( !F.RangefinderAltitudeValid )
		TxString("\tRangefinder OFFLINE\r\n");

	if ( ( GyroType == GyroUnknown ) || F.GyroFailure )
		TxString("\tGyro FAIL\r\n");

	if ( ( AccType == AccUnknown ) || !F.AccelerationsValid )
		TxString("\tAccs. FAIL\r\n");

	if ( !F.MagnetometerValid)
		TxString("\tCompass OFFLINE\r\n");

	if ( !F.Signal )
		TxString("\tBad EPAs or Tx switched off?\r\n");
	if ( Armed && FirstPass ) 
		TxString("\tUAVX is armed - DISARM!\r\n");

	if (  F.ReturnHome || F.Navigate )
		TxString("\tNav/RTH selected - DESELECT!\r\n");

	if ( InitialThrottle >= RC_THRES_START )
		TxString("\tClose Throttle!\r\n");
	
	ShowPrompt();
} // ShowSetup

void ProcessCommand(void)
{
	static uint8 ch;
	static uint8 p;
	static int8 d;
	static int16 dd;


	if ( !Armed )
	{
		ch = PollRxChar();
		if ( ch != NUL ) {
			if( islower(ch))							// check lower case
				ch = toupper(ch);
			
			switch( ch ) {
			case 'B':	// call bootloader
				{ // arming switch must be OFF to call bootloader!!!
					DisableInterrupts;
					BootStart();		// never comes back!
				}
			case 'D':
				TxString("\r\nLoad default parameters - ");
				TxString("Click CONTINUE to confirm or CANCEL\r\n");
				do {
					ch = PollRxChar();
				} while ((ch != 'x') && (ch != 'z'));

				if (ch == 'x')
					UseDefaultParameters();
				else
					TxString("\r\nCancelled");
				ShowPrompt();
				break;
			case 'L'  :	// List parameters
				TxString("\r\nParameter list for set #");	// do not change (UAVPset!)
				TxChar('0' + ParamSet);
				ReadParametersEE();
				for( p = 0; p < MAX_PARAMETERS; p++) {
					TxString("\r\nRegister ");
					TxValU((uint8)(p+1));
					TxString(" = ");
					TxValS(P[p]);
				}
				ShowPrompt();
				break;
			case 'M'  : // modify parameters
				// no reprogramming in flight!!!!!!!!!!!!!!!
				LEDBlue_ON;
				TxString("\r\nRegister ");
				p = (uint8)(RxNumU()-1);
				TxString(" = ");
				dd = RxNumS();
				d = Limit(dd, -128, 127);
				if ( p == AFType)
					PTemp[p] = AF_TYPE;
				else
					PTemp[p] = d;
				
				if ( ( p == (MAX_PARAMETERS-1)) && ( P[RollRateKp] != 0 ) ) {
					PTemp[AFType] = AF_TYPE;
					PTemp[IMU] = Wolferl;
					for (p = 0; p<MAX_PARAMETERS;p++)
						if( ParamSet == (uint8)1 ) {
							WriteEE(p, PTemp[p]);
							if ( DefaultParams[p][1] )
								WriteEE(MAX_PARAMETERS + p, PTemp[p]);
						} else {
							if ( !DefaultParams[p][1] )
								WriteEE(MAX_PARAMETERS + p, PTemp[p]);
						}

					F.ParametersValid = true; 	// ALL parameters must be written 
					ParametersChanged = true;
				}
				LEDBlue_OFF;
				ShowPrompt();
				break;
			case 'Z' : // set Paramset
				p = RxNumU();
				if ( p != (int8)ParamSet ) {
					ParamSet = p;
					ParametersChanged = true;
					ReadParametersEE();
				}
				break;
			case 'W' :	// comms with UAVXNav utility NOT UAVPSet
				UAVXNavCommand();
				//ShowPrompt();
				break;
			case 'R':	// receiver values
				TxString("\r\nT:");
				TxValS(Limit1(((int32)RC[ThrottleRC]*500)/RC_MAXIMUM,999));
				TxString(",R:");
				TxValS(Limit1(((int32)RC[RollRC]*500)/RC_MAXIMUM, 999));
				TxString(",P:");
				TxValS(Limit1(((int32)RC[PitchRC]*500)/RC_MAXIMUM,999));
				TxString(",Y:");
				TxValS(Limit1(((int32)RC[YawRC]*500)/RC_MAXIMUM, 999));
				TxString(",5:");
				TxValS(Limit1(((int32)RC[RTHRC]*500)/RC_MAXIMUM, 999));
				TxString(",6:");
				TxValS(Limit1(((int32)RC[CamPitchRC]*500)/RC_MAXIMUM, 999));
				TxString(",7:");
				TxValS(Limit1(((int32)RC[NavGainRC]*500)/RC_MAXIMUM, 999));
				TxString(",8:");
				TxValS(Limit1(((int32)RC[BypassRC]*500)/RC_MAXIMUM, 999));
				TxString(",9:");
				TxValS(Limit1(((int32)RC[PolarCoordsRC]*500)/RC_MAXIMUM, 999));
				TxString(",X:");
				TxValS(Limit1(0, 999));
				ShowPrompt();
				break;
			case 'S' :	// show status
				ShowSetup();
				break;
			case 'X' :	// flight stats
				ShowStats();
				ShowPrompt();
				break;

			#ifdef TESTING

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
			case 'H':	// barometer
				BaroTest();
				ShowPrompt();
				break;
			case 'G':	// gyro
				GyrosAndAccsTest();
				ShowPrompt();
				break;	
			case 'K':
				CalibrateCompass();
				ShowPrompt();
				break;	
			case 'N' :	// neutral values
				GetNeutralAccelerations();
				ShowPrompt();
				break;		
			case 'P'  :	// Receiver test			
				ReceiverTest();
				ShowPrompt();
				break;
#ifdef INC_I2C_PROG
			case 'Y':	// configure YGE30i EScs
				ConfigureESCs();
				ShowPrompt();
				break;
#endif
			#endif // TESTING
			case 'V' :	// Battery test
				BatteryTest();
				ShowPrompt();
				break;
			case '?'  :  // help
				TxString(SerHelp);
				ShowPrompt();
				break;
			default: break;
			} // switch
		}
	}
} // ProcessCommand

