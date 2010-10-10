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

void SendTelemetry(void);
void SendUAVX(void);
void SendUAVXControl(void);
void SendFlightPacket(void);
void SendNavPacket(void);
void SendControlPacket(void);
void SendStatsPacket(void);
void SendArduStation(void);
void SendCustom(void);
void SensorTrace(void);

uint8 UAVXCurrPacketTag;

void CheckTelemetry(void)
{
	#ifndef TESTING // not used for testing - make space!
	if ( mSClock() > mS[TelemetryUpdate] )		
		switch ( P[TelemetryType] ) {
		case UAVXTelemetry:
			mS[TelemetryUpdate] = mSClock() + UAVX_TELEMETRY_INTERVAL_MS; 
			SendUAVX(); 	
			break;
		case UAVXControlTelemetry:
			mS[TelemetryUpdate] = mSClock() + UAVX_CONTROL_TELEMETRY_INTERVAL_MS; 
			SendUAVXControl(); 	
			break;
		case ArduStationTelemetry:
			mS[TelemetryUpdate] = mSClock() + ARDU_TELEMETRY_INTERVAL_MS; 
			SendArduStation(); 
			
			break;
		case CustomTelemetry: 
			mS[TelemetryUpdate] = mSClock() + CUSTOM_TELEMETRY_INTERVAL_MS;
			SendCustom(); 
			break;
		case GPSTelemetry: break;
		} 
	#endif // TESTING
} // CheckTelemetry

#define NAV_STATS_INTERLEAVE	10
static int8 StatsNavAlternate = 0; 

void ShowAttitude(void)
{
	TxESCi16(DesiredRoll);
	TxESCi16(DesiredPitch);
	TxESCi16(DesiredYaw);

	TxESCi16(Attitude.RollRate);
	TxESCi16(Attitude.PitchRate);
	TxESCi16(Attitude.YawRate);

	TxESCi16(-Attitude.RollAngle);
	TxESCi16(-Attitude.PitchAngle);
	TxESCi16(-Attitude.YawAngle);

	TxESCi16(Attitude.LRAcc);
	TxESCi16(Attitude.FBAcc);
	TxESCi16(Attitude.DUAcc);
} // ShowAttitude

void SendFlightPacket(void)
{
	static int8 b;

	TxESCu8(UAVXFlightPacketTag);
	TxESCu8(48 + TELEMETRY_FLAG_BYTES);
	for ( b = 0; b < TELEMETRY_FLAG_BYTES; b++ )
		TxESCu8(F.AllFlags[b]); 
		
	TxESCu8(State);	

	TxESCi16(BatteryVoltsADC);
	TxESCi16(BatteryCurrentADC);
	TxESCi16(BatteryChargeUsedmAH);
 
	TxESCi16(RCGlitches);			
	TxESCi16(DesiredThrottle);

	ShowAttitude();

	TxESCi8((int8)LRComp);
	TxESCi8((int8)FBComp);
	TxESCi8((int8)DUComp);
	TxESCi8((int8)AltComp);

	for ( b = 0; b < 6; b++ ) // motor/servo channels
	 	TxESCu8((uint8)PWM[b]);

	TxESCi24(mSClock() - mS[StartTime]);
} // SendFlightPacket

void SendControlPacket(void)
{
	static int8 b;

	TxESCu8(UAVXControlPacketTag);
	TxESCu8(33);
 			
	ShowAttitude();

	for ( b = 0; b < 4; b++ ) // motor/servo channels
	 	TxESCu8((uint8)PWM[b]);

	TxESCi24(mSClock() - mS[StartTime]);
} // SendControlPacket

void SendNavPacket(void)
{
	TxESCu8(UAVXNavPacketTag);
	TxESCu8(59);
		
	TxESCu8(0);
	TxESCu8(0);
	TxESCu8(0);
	TxESCu8(0);
	
	TxESCu8(0);	
	
	TxESCi16(BaroROC); 							// dm/S
	TxESCi24(BaroRelAltitude);
	
	TxESCi16(RangefinderROC); 					// dm/S 
	TxESCi16(RangefinderAltitude); 				// dm
	
	TxESCi16(0);
	TxESCi16(Heading);
	TxESCi16(0);
	
	TxESCi16(0);
	TxESCi16(0); 							// dm/S
	
	TxESCi24(0); 					// dm
	TxESCi32(0); 						// 5 decimal minute units
	TxESCi32(0); 
	
	TxESCi24(DesiredAltitude);
	TxESCi32(0); 
	TxESCi32(0);
	
	TxESCi24(mS[NavStateTimeout] - mSClock());	// mS
	
	TxESCi16(AmbientTemperature.i16);			// 0.1C
	TxESCi32(0);

	TxESCu8(NavSensitivity);
	TxESCi8(0);
	TxESCi8(0);
	TxESCi8(0);
} // SendNavPacket

void SendStatsPacket(void) 
{
	TxESCu8(UAVXStatsPacketTag);
	TxESCu8(44);
	
	TxESCi16(Stats[I2CFailS]);
	TxESCi16(Stats[GPSInvalidS]); 
	TxESCi16(Stats[AccFailS]); 
	TxESCi16(Stats[GyroFailS]); 
	TxESCi16(Stats[CompassFailS]); 
	TxESCi16(Stats[BaroFailS]); 
	TxESCi16(Stats[ESCI2CFailS]); 
			 
	TxESCi16(Stats[RCFailsafesS]); 
			
	TxESCi16(Stats[GPSAltitudeS]);
	TxESCi16(Stats[GPSVelS]);
	TxESCi16(Stats[GPSMinSatsS]);
	TxESCi16(Stats[GPSMaxSatsS]);
	TxESCi16(Stats[MinHDiluteS]);
	TxESCi16(Stats[MaxHDiluteS]);
			
	TxESCi16(Stats[BaroRelAltitudeS]);
	TxESCi16(Stats[MinBaroROCS]);
	TxESCi16(Stats[MaxBaroROCS]);
			
	TxESCi16(Stats[MinTempS]);
	TxESCi16(Stats[MaxTempS]);
	
	TxESCi16(Stats[BadS]);

	TxESCu8(UAVXAirframe);
	TxESCu8(Orientation);
	TxESCi16(Stats[BadNumS]);
} // SendStatsPacket

void SendUAVX(void) // 800uS at 40MHz?
{
	static int8 b;
	static i32u Temp;

	F.TxToBuffer = true;

	#ifdef TELEMETRY_PREAMBLE
	for (b=10;b;b--) 
		TxChar(0x55);
	#endif // TELEMETRY_PREAMBLE
	      
	TxChar(0xff); // synchronisation to "jolt" USART	
	TxChar(SOH);	
	TxCheckSum = 0;
	
	switch ( UAVXCurrPacketTag ) {
	case UAVXFlightPacketTag:
		SendFlightPacket();

		UAVXCurrPacketTag = UAVXNavPacketTag;
		break;
	
	case UAVXNavPacketTag:
		if ( ++StatsNavAlternate < NAV_STATS_INTERLEAVE)
			SendNavPacket();		
		else
		{
			SendStatsPacket();
			StatsNavAlternate = 0;
		}

		UAVXCurrPacketTag = UAVXFlightPacketTag;
		break;
	
	default:
		UAVXCurrPacketTag = UAVXFlightPacketTag;
		break;		
	}
		
	TxESCu8(TxCheckSum);	
	TxChar(EOT);
	
	TxChar(CR);
	TxChar(LF); 

	F.TxToBuffer = false; 
	
} // SendUAVX

void SendUAVXControl(void) // 0.516mS at 40MHz?
{
	static int8 b;
	static i32u Temp;

	F.TxToBuffer = true;

	#ifdef TELEMETRY_PREAMBLE
	for (b=10;b;b--) 
		TxChar(0x55);
	#endif // TELEMETRY_PREAMBLE
	      
	TxChar(0xff); // synchronisation to "jolt" USART	
	TxChar(SOH);	
	TxCheckSum = 0;
	
	SendControlPacket();

	TxESCu8(TxCheckSum);	
	TxChar(EOT);
	
	TxChar(CR);
	TxChar(LF); 

	F.TxToBuffer = false; 
	
} // SendUAVXControl

void SendArduStation(void)
{

	// junked

} // SendArduStation

void SendCustom(void) // 1.2mS @ 40MHz
{ // user defined telemetry human readable OK for small amounts of data < 1mS

	F.TxToBuffer = true;
	
	// insert values here using TxVal32(n, dp, separator)
	// dp is the scaling to decimal places, separator
	// separator may be a single 'char', HT for tab, or 0 (no space)
	// -> 

	// ~1mS @ 40MHz

	TxVal32(mSClock(), 3, HT);

	if ( F.HoldingAlt ) // are we holding
		TxChar('H');
	else
		TxChar('N');
	TxChar(HT);

	if (F.UsingRangefinderAlt ) // are we using the rangefinder
		TxChar('R');
	else
		TxChar('B');
	TxChar(HT);

	TxVal32(SRS32(AltComp,1), 1, HT);		// ~% throttle compensation

	TxVal32(0, 1, HT);
	TxVal32(BaroRelAltitude, 1, HT);
	TxVal32(RangefinderAltitude, 2, HT);

	TxVal32(BaroPressure, 0, HT);			// eff. sensor reading
	TxVal32(BaroTemperature, 0, HT); 		// eff. sensor reading redundant for MPX4115
	TxVal32(CompBaroPressure, 0, HT);  		// moving sum of last 8 readings

	// <-

	TxChar(CR);
	TxChar(LF);

	F.TxToBuffer = false;
} // SendCustom

void SensorTrace(void)
{
	#ifdef TESTING

	if ( DesiredThrottle > 20 ) 
	{
		F.TxToBuffer = false; // direct to USART

		TxValH16(((int24)Heading * 180)/MILLIPI); TxChar(';');

		TxValH16(BaroRelAltitude); TxChar(';');
		TxValH16(RangefinderAltitude); TxChar(';');
		TxValH16(0); TxChar(';');
			
		TxValH16(DesiredThrottle); TxChar(';');
		TxValH16(DesiredRoll); TxChar(';');
		TxValH16(DesiredPitch); TxChar(';');
		TxValH16(DesiredYaw); TxChar(';');

		TxValH16(RollRateADC - GyroMidRoll); TxChar(';');
		TxValH16(PitchRateADC - GyroMidPitch); TxChar(';');
		TxValH16(YawRateADC - GyroMidYaw); TxChar(';');

		TxValH16(RollAngle); TxChar(';');
		TxValH16(PitchAngle); TxChar(';');
		TxValH16(YawAngle); TxChar(';');

		TxValH16(LRAcc); TxChar(';');
		TxValH16(FBAcc); TxChar(';');
		TxValH16(DUAcc); TxChar(';');

		TxValH16(LRComp); TxChar(';');
		TxValH16(FBComp); TxChar(';');
		TxValH16(DUComp); TxChar(';');
		TxValH16(AltComp); TxChar(';');
		TxNextLine();
	} 
	#endif // TESTING
} // SensorTrace



