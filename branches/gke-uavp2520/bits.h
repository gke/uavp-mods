// =======================================================================
// =                   U.A.V.P Brushless UFO Controller                  =
// =                         Professional Version                        =
// =             Copyright (c) 2007 Ing. Wolfgang Mahringer              =
// =              Ported to 18F2520 2008 by Prof. Greg Egan              =
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

// ADC Channels
#define ADCBattVoltsChan 	0 

#ifdef OPT_ADXRS
	#define ADCRollChan 	1
	#define ADCPitchChan 	2
#else // OPT_IDG
	#define ADCRollChan 	2
	#define ADCPitchChan 	1
#endif // OPT_ADXRS

#define ADCVRef 			3 
#define ADCYawChan			4

// Bit definitions

#define	NULL	0

// when changing, see OutSignal() in utils.c
#define	PulseFront		0
#define	PulseLeft		1
#define	PulseRight		2
#define	PulseBack		3

#define	PulseCamRoll	4
#define	PulseCamPitch	5

#define	Switch			PORTAbits.RA4

#define ALL_OUTPUTS_OFF	(PORTB &= 0b11110000)

#if defined ESC_X3D || defined ESC_HOLGER || defined ESC_YGEI2C
// Wolfgang's SW I2C ESC
#define	 ESC_SDA		PORTBbits.RB1
#define	 ESC_SCL		PORTBbits.RB2
#define	 ESC_DIO		TRISBbits.TRISB1
#define	 ESC_CIO		TRISBbits.TRISB2
#endif

// the sensor bus lines
#define	LISL_CS			PORTCbits.RC5
#define LISL_SDA		PORTCbits.RC4
#define LISL_SCL		PORTCbits.RC3
#define	LISL_IO			TRISCbits.TRISC4

#define	I2C_DLAT		LATBbits.LATB6
#define	I2C_SLAT		LATBbits.LATB7	
#define I2C_SDA			PORTBbits.RB6
#define I2C_DIO			TRISBbits.TRISB6
#define I2C_SCL			PORTBbits.RB7
#define	I2C_CIO			TRISBbits.TRISB7

#define	I2C_ACK		0
#define	I2C_NACK	1

// The LEDs and the beeper
#define ON	1
#define OFF	0

#define LedYellow		LED6
#define LedGreen		LED4
#define	LedBlue			LED2
#define LedRed			LED3
#define LedAUX1			LED5
#define LedAUX2			LED1
#define LedAUX3			LED7

#define LED1	0x01	/* Aux2 */
#define LED2	0x02	/* blue */
#define LED3	0x04	/* red */ 
#define LED4	0x08	/* green */
#define LED5	0x10	/* Aux1 */
#define LED6	0x20	/* yellow */
#define LED7	0x40	/* Aux3 */
#define Beeper	0x80

#define ALL_LEDS_ON		SwitchLedsOn(LedBlue|LedRed|LedGreen|LedYellow)
#define AUX_LEDS_ON		SwitchLedsOn(LedAUX1|LedAUX2|LedAUX3)

#define ALL_LEDS_OFF	SwitchLedsOff(LedBlue|LedRed|LedGreen|LedYellow)
#define AUX_LEDS_OFF	SwitchLedsOff(LedAUX1|LedAUX2|LedAUX3)

#define ARE_ALL_LEDS_OFF if((LedShadow&(LedBlue|LedRed|LedGreen|LedYellow))==0)

#define LedRed_ON		SwitchLedsOn(LedRed)
#define LedBlue_ON		SwitchLedsOn(LedBlue)
#define LedGreen_ON		SwitchLedsOn(LedGreen)
#define LedYellow_ON	SwitchLedsOn(LedYellow)
#define LedAUX1_ON		SwitchLedsOn(LedAUX1);
#define LedAUX2_ON		SwitchLedsOn(LedAUX2);
#define LedAUX3_ON		SwitchLedsOn(LedAUX3);
#define LedRed_OFF		SwitchLedsOff(LedRed)
#define LedBlue_OFF		SwitchLedsOff(LedBlue)
#define LedGreen_OFF	SwitchLedsOff(LedGreen)
#define LedYellow_OFF	SwitchLedsOff(LedYellow)
#define LedRed_TOG		if( (LedShadow&LedRed) == 0 ) SwitchLedsOn(LedRed); else SwitchLedsOff(LedRed)
#define LedBlue_TOG		if( (LedShadow&LedBlue) == 0 ) SwitchLedsOn(LedBlue); else SwitchLedsOff(LedBlue)
#define Beeper_OFF		SwitchLedsOff(Beeper)
#define Beeper_ON		SwitchLedsOn(Beeper)
#define Beeper_TOG		if( (LedShadow&Beeper) == 0 ) SwitchLedsOn(Beeper); else SwitchLedsOff(Beeper)

// compass sensor
#define COMPASS_ADDR	(0x42)
#define COMPASS_MAXDEV	30					/* maximum yaw compensation of compass heading */
#define COMPASS_MAX		240					/* means 360 degrees */
#define COMPASS_INVAL	(COMPASS_MAX+15)	/* 15*4 cycles to settle */
#define COMPASS_MIDDLE	10					/* yaw stick neutral dead zone */

// baro (altimeter) sensor
#define BARO_TEMP		0xee
#define BARO_PRESS		0xf4
#define BARO_ADDR		0xee
#define BARO_CTL		0xf4
#define BARO_ADC		0xf6
#define THR_HOVER		60
#define THR_WINDOW		5

#define	_Signal			Flags[0]			/*if no valid signal is received*/
#define _NewValues		Flags[2]			/*new RX channel values sampled*/	
#define _FirstTimeout	Flags[3]			/*is 1 after first 9ms TO expired*/	
#define _BaroReady		Flags[5]
#define _LowBatt		Flags[5]
#define	_UseLISL		Flags[6]	
#define	_UseCompass		Flags[7]	

#define _UseBaro		Flags2[0]	
#define _BaroAcqTemp	Flags2[1]	
#define _OutToggle		Flags2[2]			/*cam servos only every 2nd output pulse*/									
#define _UseCh7Trigger	Flags2[3]			/*1: don't use Ch7 0: use Ch7 as Cam Roll trim*/
#define _LostModel		Flags2[4]
#define _ThrChanging	Flags2[5]
#define _Armed			Flags2[6]							
#define _MotorsEnabled	Flags2[7]

// Mask Bits of ConfigParam
#define FlyCrossMode 	IsSet(ConfigParam,0)
#define FutabaMode		IsSet(ConfigParam,1)
#define IntegralTest	IsSet(ConfigParam,2)
#define DoubleRate		IsSet(ConfigParam,3)	
#define NegativePPM		IsSet(ConfigParam,4)
#define CompassTest		IsSet(ConfigParam,5)

// LISL-Register mapping
#define	LISL_WHOAMI		(0x0f)
#define	LISL_OFFSET_X	(0x16)
#define	LISL_OFFSET_Y	(0x17)
#define	LISL_OFFSET_Z	(0x18)
#define	LISL_GAIN_X		(0x19)
#define	LISL_GAIN_Y		(0x1A)
#define	LISL_GAIN_Z		(0x1B)
#define	LISL_CTRLREG_1	(0x20)
#define	LISL_CTRLREG_2	(0x21)
#define	LISL_CTRLREG_3	(0x22)
#define	LISL_STATUS		(0x27)
#define LISL_OUTX_L		(0x28)
#define LISL_OUTX_H		(0x29)
#define LISL_OUTY_L		(0x2A)
#define LISL_OUTY_H		(0x2B)
#define LISL_OUTZ_L		(0x2C)
#define LISL_OUTZ_H		(0x2D)
#define LISL_FF_CFG		(0x30)
#define LISL_FF_SRC		(0x31)
#define LISL_FF_ACK		(0x32)
#define LISL_FF_THS_L	(0x34)
#define LISL_FF_THS_H	(0x35)
#define LISL_FF_DUR		(0x36)
#define LISL_DD_CFG		(0x38)
#define LISL_INCR_ADDR	(0x40)
#define LISL_READ		(0x80)
