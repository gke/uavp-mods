// =======================================================================
// =                     UAVX Quadrocopter Controller                    =
// =               Copyright (c) 2008, 2009 by Prof. Greg Egan           =
// =   Original V3.15 Copyright (c) 2007, 2008 Ing. Wolfgang Mahringer   =
// =                          http://uavp.ch                             =
// =======================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "uavx.h"

extern uint8 SaturInt(int16);
extern void DoMix(int16 CurrThrottle);
extern void CheckDemand(int16 CurrThrottle);
extern void MixAndLimitMotors(void);
extern void MixAndLimitCam(void);
extern void OutSignals(void);

// Constants

#define MAGICNUMBER 84

#define	PulseFront		0
#define	PulseLeft		1
#define	PulseRight		2
#define	PulseBack		3

#define	PulseCamRoll	4
#define	PulseCamPitch	5

#define ALL_PULSE_ON	(PORTB |= 0b00001111)
#define ALL_OUTPUTS_OFF	(PORTB &= 0b11110000)
#define ALL_OUTPUTS		(PORTB & 0b00001111)

uint8 SaturInt(int16 l)
{
	static int16 r;

	if ( ESCType == ESCX3D )
	{
		l -= OUT_MINIMUM;
		r = Limit(l, 1, 200);
	}
	else
		r = Limit(l, OUT_MINIMUM, OUT_MAXIMUM );

	return((uint8) r);
} // SaturInt

void DoMix(int16 CurrThrottle)
{
	static int16 Temp;

	Motor[Front] = Motor[Left] = Motor[Right] = CurrThrottle;
	#ifndef TRICOPTER
	Motor[Back] = CurrThrottle;
	#endif // !TRICOPTER

	#ifndef TRICOPTER
	if( FlyXMode )
	{	// "Cross" Mode
		Motor[Left] +=   Pl - Rl - Yl;
		Motor[Right] += -Pl + Rl - Yl;
		Motor[Front] += -Pl - Rl + Yl;
		Motor[Back] +=   Pl + Rl + Yl; 
	}
	else
	{	// Normal "Plus" Mode
		Motor[Left]  += -Rl - Yl;	
		Motor[Right] +=  Rl - Yl;
		Motor[Front] += -Pl + Yl;
		Motor[Back]  +=  Pl + Yl;
	}

	#else	// TRICOPTER
	Temp = SRS16(Rl - Pl, 1); 
	Motor[Front] += Pl ;			// front motor
	Motor[Left]  += Temp;			// rear left
	Motor[Right] -= Temp; 			// rear right
	Motor[Back]   = Yl + RC_NEUTRAL;	// yaw servo
	#endif

} // DoMix

uint8 	MotorDemandRescale;

void CheckDemand(int16 CurrThrottle)
{
	static int8 s;
	static int24 Scale, ScaleHigh, ScaleLow, MaxMotor, DemandSwing;

	MaxMotor = Max(Motor[Front], Motor[Left]);
	MaxMotor = Max(MaxMotor, Motor[Right]);
	#ifndef TRICOPTER
	MaxMotor = Max(MaxMotor, Motor[Back]);
	#endif // TRICOPTER

	DemandSwing = MaxMotor - CurrThrottle;

	if ( DemandSwing > 0 )
	{		
		ScaleHigh = (( OUT_MAXIMUM - (int24)CurrThrottle) * 256 )/ DemandSwing;	 
		ScaleLow = (( (int24)CurrThrottle - IdleThrottle) * 256 )/ DemandSwing;
		Scale = Min(ScaleHigh, ScaleLow);
		if ( Scale < 256 )
		{
			MotorDemandRescale = true;
			Rl = (Rl * Scale)/256;  
			Pl = (Pl * Scale)/256; 
			Yl = (Yl * Scale)/256; 
		}
		else
			MotorDemandRescale = false;	
	}
	else
		MotorDemandRescale = false;	

} // CheckDemand

void MixAndLimitMotors(void)
{ 	// expensive ~400uSec @16MHz
    static int16 Temp, CurrThrottle;

	// Altitude stabilization factor
	CurrThrottle = DesiredThrottle + (VUDComp + VBaroComp); // vertical compensation not optional
	Temp = (int16)(OUT_MAXIMUM * 90 + 50) / 100; // 10% headroom for control
	CurrThrottle = Limit(CurrThrottle, 0, Temp ); 

	if ( CurrThrottle > IdleThrottle )
	{
		DoMix(CurrThrottle);

		CheckDemand(CurrThrottle);

		if ( MotorDemandRescale )
			DoMix(CurrThrottle);
	}
	else
		Motor[Front] = Motor[Back] = 
		Motor[Left] = Motor[Right] = CurrThrottle;

	Motor[Front] = SaturInt(Motor[Front]);
	Motor[Back] = SaturInt(Motor[Back]);
	Motor[Left] = SaturInt(Motor[Left]);
	Motor[Right] = SaturInt(Motor[Right]);

} // MixAndLimitMotors

void MixAndLimitCam(void)
{
	static int16 Cr, Cp;

	// use only roll/pitch angle estimates
	if( CamPitchKp != 0 )
		Cp = PitchSum / (int16)CamPitchKp;
	else
		Cp = OUT_NEUTRAL;
		
	Cp += RC[CamTiltC];	

	MCamRoll = OUT_NEUTRAL;
	MCamPitch = Limit(Cp, OUT_MINIMUM, OUT_MAXIMUM);

} // MixAndLimitCam

#pragma udata assembly_language=0x080 
uint8 SHADOWB, MF, MB, ML, MR, MT, ME; // motor/servo outputs
#pragma udata

void OutSignals(void)
{
	#ifdef DEBUG_SENSORS
	Trace[TIThrottle] = DesiredThrottle;

	Trace[TIRoll] = DesiredRoll;
	Trace[TIPitch] = DesiredPitch;
	Trace[TIYaw] = DesiredYaw;

	Trace[TMFront] = Motor[Front];
	Trace[TMBack] = Motor[Back];
	Trace[TMLeft] = Motor[Left];
	Trace[TMRight] = Motor[Right];

	Trace[TMCamRoll] = MCamRoll;
	Trace[TMCamPitch] = MCamPitch;
	#else // !DEBUG_SENSORS

	WriteTimer0(0);
	INTCONbits.TMR0IF = false;

	if ( ESCType == ESCPPM )
	{
		_asm
		MOVLB	0						// select Bank0
		MOVLW	0x0f					// turn on motors
		MOVWF	SHADOWB,1
		_endasm	
		PORTB |= 0x0f;
	
		} // ESC_PPM
	
		MF = Motor[Front];
		MB = Motor[Back];
		ML = Motor[Left];
		MR = Motor[Right];
	
		MT = MCamRoll;
		ME = MCamPitch;
	
		if ( ESCType == ESCPPM )
		{
	
		// simply wait for nearly 1 ms
		// irq service time is max 256 cycles = 64us = 16 TMR0 ticks
		while( ReadTimer0() < (uint16)(0x100-3-MAGICNUMBER) ) ; // 16
	
		// now stop CCP1 interrupt
		// capture can survive 1ms without service!
	
		// Strictly only if the masked interrupt region below is
		// less than the minimum valid Rx pulse/gap width which
		// is 1027uS less capture time overheads
	
		DisableInterrupts;	// BLOCK ALL INTERRUPTS for NO MORE than 1mS
		while( !INTCONbits.TMR0IF ) ;	// wait for first overflow
		INTCONbits.TMR0IF=0;			// quit TMR0 interrupt
	
		if( _OutToggle )	// driver cam servos only every 2nd pulse
		{
			_asm
			MOVLB	0					// select Bank0
			MOVLW	0x3f				// turn on motors
			MOVWF	SHADOWB,1
			_endasm	
			PORTB |= 0x3f;
		}
		_OutToggle ^= 1;
	
	// This loop is exactly 16 cycles int16
	// under no circumstances should the loop cycle time be changed
		_asm
		MOVLB	0						// select Bank0
OS005:
		MOVF	SHADOWB,0,1				// Cannot read PORTB ???
		MOVWF	PORTB,0
		ANDLW	0x0f
		BZ		OS006
				
		DECFSZ	MF,1,1					// front motor
		GOTO	OS007
				
		BCF		SHADOWB,PulseFront,1	// stop Front pulse
OS007:
		DECFSZ	ML,1,1					// left motor
		GOTO	OS008
				
		BCF		SHADOWB,PulseLeft,1		// stop Left pulse
OS008:
		DECFSZ	MR,1,1					// right motor
		GOTO	OS009
				
		BCF		SHADOWB,PulseRight,1	// stop Right pulse
OS009:
		DECFSZ	MB,1,1					// rear motor
		GOTO	OS005
					
		BCF		SHADOWB,PulseBack,1		// stop Back pulse			
	
		GOTO	OS005
OS006:
		_endasm
		// This will be the corresponding C code:
		//	while( ALL_OUTPUTS != 0 )
		//	{	// remain in loop as int16 as any output is still high
		//		if( TMR2 = MFront  ) PulseFront  = 0;
		//		if( TMR2 = MBack ) PulseBack = 0;
		//		if( TMR2 = MLeft  ) PulseLeft  = 0;
		//		if( TMR2 = MRight ) PulseRight = 0;
		//	}
	
		EnableInterrupts;
	
	} 
	else
	{
		if( _OutToggle )	// driver cam servos only every 2nd pulse
		{
			_asm
			MOVLB	0					// select Bank0
			MOVLW	0x3f				// turn on motors
			MOVWF	SHADOWB,1
			_endasm	
			PORTB |= 0x3f;
		}
		_OutToggle ^= 1;
		
		// in X3D- and Holger-Mode, K2 (left motor) is SDA, K3 (right) is SCL
		if ( ESCType == ESCX3D )
		{
			EscI2CStart();
			SendEscI2CByte(0x10);	// one command, 4 data bytes
			SendEscI2CByte(MF); // for all motors
			SendEscI2CByte(MB);
			SendEscI2CByte(ML);
			SendEscI2CByte(MR);
			EscI2CStop();
		}
		else
			if ( ESCType ==  ESCHolger)
			{
				EscI2CStart();
				SendEscI2CByte(0x52);	// one cmd, one data byte per motor
				SendEscI2CByte(MF); // for all motors
				EscI2CStop();
			
				EscI2CStart();
				SendEscI2CByte(0x54);
				SendEscI2CByte(MB);
				EscI2CStop();
			
				EscI2CStart();
				SendEscI2CByte(0x58);
				SendEscI2CByte(ML);
				EscI2CStop();
			
				EscI2CStart();
				SendEscI2CByte(0x56);
				SendEscI2CByte(MR);
				EscI2CStop();
			}
			else
		
				if ( ESCType == ESCYGEI2C )
				{
					EscI2CStart();
					SendEscI2CByte(0x62);	// one cmd, one data byte per motor
					SendEscI2CByte(MF>>1); // for all motors
					EscI2CStop();
				
					EscI2CStart();
					SendEscI2CByte(0x64);
					SendEscI2CByte(MB>>1);
					EscI2CStop();
				
					EscI2CStart();
					SendEscI2CByte(0x68);
					SendEscI2CByte(ML>>1);
					EscI2CStop();
				
					EscI2CStart();
					SendEscI2CByte(0x66);
					SendEscI2CByte(MR>>1);
					EscI2CStop();
				}
		}

	while( ReadTimer0() < (uint16)(0x100-3-MAGICNUMBER) ) ; 	// wait for 2nd TMR0 near overflow

	DisableInterrupts;				
	while( !INTCONbits.TMR0IF ) ;		// wait for 2nd overflow (2 ms)

	// This loop is exactly 16 cycles int16
	// under no circumstances should the loop cycle time be changed
_asm
	MOVLB	0
OS001:
	MOVF	SHADOWB,0,1				// Cannot read PORTB ???
	MOVWF	PORTB,0
	ANDLW	0x30		// output ports 4 and 5
	BZ		OS002		// stop if all 2 outputs are 0

	DECFSZ	MT,1,1
	GOTO	OS003

	BCF		SHADOWB,PulseCamRoll,1
OS003:
	DECFSZ	ME,1,1
	GOTO	OS004

	BCF		SHADOWB,PulseCamPitch,1
OS004:
_endasm
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
	Delay1TCY();
_asm
	GOTO	OS001
OS002:
_endasm

	EnableInterrupts;	

#endif  // DEBUG_SENSORS
} // OutSignals


