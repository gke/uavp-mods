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

// Accelerator 400KHz I2C or SPI

#include "uavx.h"

#pragma idata acc_names
const rom uint8 MPUDLPFMask[] = { MPU_RA_DLPF_BW_256, MPU_RA_DLPF_BW_188,
		MPU_RA_DLPF_BW_98, MPU_RA_DLPF_BW_42, MPU_RA_DLPF_BW_20,
		MPU_RA_DLPF_BW_10, MPU_RA_DLPF_BW_5 };

#ifdef TESTING
const uint16 InertialLPFHz[] = { 256, 188, 98, 42, 20, 10, 5 };
const rom uint8 * DHPFName[] = { "Reset/0Hz", "5Hz", "2.5Hz", "1.25Hz", "0.63Hz",
		"?", "?", "Hold" };
#endif
#pragma idata

void ShowAccType(void);
void AccFailure(void);
void ReadAccelerations(void);
void GetNeutralAccelerations(void);
void InitAccelerometers(void);

void ReadMPU6050Acc(void);
void InitMPU6050Acc(void);
boolean MPU6050AccActive(void);

void ReadBMA180Acc(void);
void InitBMA180Acc(void);
boolean BMA180AccActive(void);
void ShowBMA180State(void);

void SendCommand(int8);
uint8 ReadLISL(uint8);
uint8 ReadLISLNext(void);
void WriteLISL(uint8, uint8);
void InitLISLAcc(void);
boolean LISLAccActive(void);
void ReadLISLAcc(void);

int16 RawAcc[3];
uint8 AccType;
uint8 MPU6050_ID;
uint8 BMA180_ID;
uint8 AccConfidence;

uint8 MPU6050DLPF, MPU6050DHPF;

const rom uint8 * AccName[AccUnknown+1] = 
		{"LIS3L", "MPU6050","BMA180", "Not supported", "No response"};

void ShowAccType(void)
{
	TxString(AccName[AccType]);
} // ShowAccType

void AccFailure(void)
{
	if ( State == InFlight )
	{
		Stats[AccFailS]++;	
		F.AccFailure = true;
	}
} // AccFailure

void ReadAccelerations(void)
{
	// X/Forward FB Acc sense to simplify gyro comp code
	switch ( P[SensorHint] ) {
	#ifdef INC_MPU6050
	case FreeIMU:
	case UAVXArm32IMU:
		ReadMPU6050Acc();
		A[LR].AccADC = -SRS16(RawAcc[X], 1); 
		A[DU].AccADC = SRS16(RawAcc[Z], 1) + GRAVITY;
		A[FB].AccADC = SRS16(RawAcc[Y], 1);
		break;
	case GY86IMU:
		ReadMPU6050Acc();
		A[LR].AccADC = SRS16(RawAcc[Y], 1); 
		A[DU].AccADC = SRS16(RawAcc[Z], 1) + GRAVITY;
		A[FB].AccADC = SRS16(RawAcc[X], 1);
		break;
	#else
	case FreeIMU:
	case UAVXArm32IMU:
		A[LR].AccADC = 0; 
		A[DU].AccADC = -GRAVITY;
		A[FB].AccADC = 0;
		break;
	#endif // INC_MPU6050	
	case DrotekIMU:
	#ifdef INC_BMA180
		ReadBMA180Acc();
		A[LR].AccADC = -SRS16(RawAcc[Y], 4); 
		A[FB].AccADC = SRS16(RawAcc[X], 4);
		A[DU].AccADC = -SRS16(RawAcc[Z], 4);
	#endif // INC_BMA180
		break;
	default:
	#ifdef INC_LISL
		ReadLISLAcc();
		A[LR].AccADC = RawAcc[X];
		A[FB].AccADC = RawAcc[Z];
		A[DU].AccADC = RawAcc[Y];
	#endif // INC_LISL
		break;
	} // switch

} // ReadAccelerations

void InitAccelerometers(void)
{
	static uint8 a;

	AccConfidence = 100;

	for ( a = Roll; a<=(uint8)Yaw; a++)
		A[a].AccBias = A[a].AccADC = 0;
	A[DU].AccADC = GRAVITY;

	AccType = AccUnknown;
	F.AccelerationsValid = false;

	switch ( P[SensorHint]){
		case FreeIMU:
		case GY86IMU:
		case UAVXArm32IMU:
		#ifdef INC_MPU6050
		F.AccelerationsValid = true;
			AccType = MPU6050Acc;
		#else
			AccType = AccUnsupported;
		#endif // INC_MPU6050
			break;
		case DrotekIMU:
		#ifdef INC_BMA180
			if ( BMA180AccActive() ) {
				AccType = BMA180Acc;
				InitBMA180Acc();
			} else
				AccType = AccUnknown;
		#else
			AccType = AccUnsupported;
		#endif // INC_BMA_180
			break;
		default:
		#ifdef INC_LISL	
			if ( LISLAccActive() ) {
				AccType = LISLAcc;
				InitLISLAcc();
			} else
				AccType = AccUnknown;
		#else
			AccType = AccUnsupported;
		#endif // INC_LISL
			break;
	} // switch

	if( F.AccelerationsValid ) 
		ReadAccCalEE();
	else
		AccFailure();

	F.AccelerometersEnabled = true;

} // InitAccelerometers

#ifdef TESTING

void GetNeutralAccelerations(void)
{
	static uint8 i, a, ch;
	static int16 Temp[3], b;

	TxString("\r\nCalibrate accelerometers - ");
	TxString("Click CONTINUE to confirm you wish to calibrate or CANCEL\r\n");

	do {
		ch = PollRxChar();
	} while ((ch != 'x') && (ch != 'z'));

	if (ch == 'x') {
		Temp[LR] = Temp[FB] = Temp[DU] = 0;
		if ( F.AccelerationsValid` ) {
			for ( i = 16; i; i--) {
				ReadAccelerations();
				for ( a = LR; a<=(uint8)DU; a++ )
					Temp[a] += A[a].AccADC;
	
				Delay1mS(10);
			}	
		
			Temp[DU] -= 16 * GRAVITY;
			for ( a = LR; a<=(uint8)DU; a++ ) 
				A[a].AccBias = SRS32(Temp[a], 4);

			TxNextLine();
			TxString("\tL->R: \t");TxVal32(A[LR].AccBias,0,0);
			if ( Abs(A[LR].AccBias) > 100 ) 
				TxString(" >100 - not level?");
			TxNextLine();
			TxString("\tF->B: \t");TxVal32(A[FB].AccBias,0,0); 
			if ( Abs(A[FB].AccBias) > 100 ) 
				TxString(" >100 - not level?");
			TxNextLine();
			TxString("\tD->U: \t");TxVal32(A[DU].AccBias,0,0); 
			TxNextLine();

			WriteAccCalEE();	
		} else {
			TxString("\r\nAccelerometer read failure\r\n");
			A[LR].AccBias = A[FB].AccBias = A[DU].AccBias = 0;
		}
	} else
		TxString("\r\nCancelled");

} // GetNeutralAccelerations

void WriteAccCalEE(void) 
{
	uint8 a;

	for ( a = LR; a<=(uint8)DU; a++ ) 
		WriteEE(ACC_BIAS_ADDR_EE + a, A[a].AccBias);

} // WriteAccCalEE

#endif // TESTING

void ReadAccCalEE(void) 
{
	uint8 a;

	for ( a = LR; a<=(uint8)DU; a++ ) 
		A[a].AccBias = ReadEE(ACC_BIAS_ADDR_EE + a);

} // ReadAccCalEE

//________________________________________________________________________________________________

#ifdef INC_MPU6050

boolean MPU6050AccActive(void);


void ReadMPU6050Acc(void) 
{
	if ( !ReadI2Ci16vAtAddr(MPU6050_ID, MPU6050_ACC_XOUT_H, RawAcc, 3, true) ) 
		AccFailure();

} // ReadMPU6050Acc


void InitMPU6050Acc() {
	uint8 v;

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_PWR_MGMT_1, 1<< MPU_RA_PWR1_DEVICE_RESET_BIT);
	Delay1mS(5);

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_FIFO_EN, 0); // disable FIFOs

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_SMPLRT_DIV, 0); 
	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_PWR_MGMT_1, MPU_RA_CLOCK_PLL_XGYRO); // No sleeping, temperature sensor On, Z ref.

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_ACC_CONFIG, (MPU_RA_ACC_FS_16 << 3)| MPU6050_DHPF_1P25); //

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_GYRO_CONFIG, MPU_RA_GYRO_FS_2000 << 3);

	// Enable I2C master mode
	v = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_USER_CTRL);
	Clear(v, MPU_RA_USERCTRL_I2C_MST_EN_BIT);
	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_USER_CTRL,v);

	// Allow bypass access to slave I2C devices (Magnetometer)
	v = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_INT_PIN_CFG);
	Set(v, MPU_RA_INTCFG_I2C_BYPASS_EN_BIT);
	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_INT_PIN_CFG, v);

	WriteI2CByteAtAddr(MPU6050_ID, MPU_RA_CONFIG, MPUDLPFMask[P[GyroLPF]]);

	Delay1mS(10);

#ifdef TESTING
	MPU6050DLPF = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_CONFIG) & 0x07;
	MPU6050DHPF = ReadI2CByteAtAddr(MPU6050_ID, MPU_RA_ACC_CONFIG) & 0x07;
#endif // TESTING

//	DoMPU6050AccScale(); // determine acc scaling

} // InitMPU6050Acc

boolean MPU6050AccActive(void) 
{
	MPU6050_ID = MPU6050_0xD0_ID;
    F.AccelerationsValid = I2CResponse(MPU6050_ID);
	if (!F.AccelerationsValid)
	{
		MPU6050_ID = MPU6050_0xD2_ID;
		F.AccelerationsValid = I2CResponse(MPU6050_ID);
	}
    return( F.AccelerationsValid );
} // MPU6050AccActive

#endif // INC_MPU6050

//________________________________________________________________________________________________

#ifdef INC_BMA180

// Bosch BMA180 Acc

// 0 1g, 1 1.5g, 2 2g, 3 3g, 4 4g, 5 8g, 6 16g
// 0 19Hz, 1 20, 2 40, 3 75, 4 150, 5 300, 6 600, 7 1200Hz 

#define BMA180_RANGE	2
#define BMA180_BW 		1 // 4

#define BMA180_Version 0x01
#define BMA180_ACCXLSB 0x02
#define BMA180_TEMPERATURE 0x08

#define BMA180_RESET 	0x10
#define BMA180_STATREG1 0x09
#define BMA180_STATREG2 0x0A
#define BMA180_STATREG3 0x0B
#define BMA180_STATREG4 0x0C
#define BMA180_CTRLREG0 0x0D
#define BMA180_CTRLREG1 0x0E
#define BMA180_CTRLREG2 0x0F

#define BMA180_BWTCS 0x20		// bandwidth
#define BMA180_CTRLREG3 0x21

#define BMA180_HILOWNFO 0x25
#define BMA180_LOWDUR 0x26
#define BMA180_LOWTH 0x29

#define BMA180_tco_y 0x2F
#define BMA180_tco_z 0x30

#define BMA180_OLSB1 0x35		// setting range

boolean BMA180AccActive(void);

void ReadBMA180Acc(void) 
{
	if ( !ReadI2Ci16vAtAddr(BMA180_ID, BMA180_ACCXLSB, RawAcc, 3, false) )
		AccFailure();

} // ReadBMA180Acc

void InitBMA180Acc() {
	
	uint8 i, bw, range, ee_w;

	// if connected correctly, ID register should be 3
//	if(read(ID) != 3)
//		return -1;

	WriteI2CByteAtAddr(BMA180_ID, BMA180_RESET, 0); 

	ee_w = ReadI2CByteAtAddr(BMA180_ID, BMA180_CTRLREG0);
	ee_w |= 0x10;
	WriteI2CByteAtAddr(BMA180_ID, BMA180_CTRLREG0, ee_w); // write enable registers

	bw = ReadI2CByteAtAddr(BMA180_ID, BMA180_BWTCS);
	bw &= ~0xF0;
	bw |= BMA180_BW << 4;
	WriteI2CByteAtAddr(BMA180_ID, BMA180_BWTCS, bw); // set BW	
		
	range = ReadI2CByteAtAddr(BMA180_ID, BMA180_OLSB1);
	range &= ~0x0E;
	range |= BMA180_RANGE<<1;
	WriteI2CByteAtAddr(BMA180_ID, BMA180_OLSB1, range); // set range

} // InitBMA180Acc

boolean BMA180AccActive(void) {

	BMA180_ID = BMA180_ID_0x80;
    if ( I2CResponse(BMA180_ID) )
		F.AccelerationsValid = true;
	else {
		BMA180_ID = BMA180_ID_0x82;
		if ( I2CResponse(BMA180_ID) )
			F.AccelerationsValid = true;
		else
			F.AccelerationsValid = false;
	}

    return( F.AccelerationsValid );

} // BMA180AccActive

#ifdef TESTING

void ShowBMA180State(void)
{
	#ifdef FULL_ACC_TEST

	static uint8 bw, range;
	bw = ReadI2CByteAtAddr(BMA180_ID, BMA180_BWTCS);		
	range = ReadI2CByteAtAddr(BMA180_ID, BMA180_OLSB1);
	TxString("\r\n\tBW:\t");
	TxVal32((bw>>4) & 0x0f,0,HT);
	TxString("Range:\t");
	TxVal32((range>>1) & 0x07,0,0);
	TxNextLine();

	#endif // FULL_ACC_TEST
} // ShowBMA180State

#endif // TESTING

#endif // INC_BMA180

//________________________________________________________________________________________________

// LISL Acc

#ifdef INC_LISL

#define SPI_HI_DELAY Delay10TCYx(2)
#define SPI_LO_DELAY Delay10TCYx(2)

// LISL-Register mapping

#define	LISL_WHOAMI		(uint8)(0x0f)
#define	LISL_OFFSET_X	(uint8)(0x16)
#define	LISL_OFFSET_Y	(uint8)(0x17)
#define	LISL_OFFSET_Z	(uint8)(0x18)
#define	LISL_GAIN_X		(uint8)(0x19)
#define	LISL_GAIN_Y		(uint8)(0x1A)
#define	LISL_GAIN_Z		(uint8)(0x1B)
#define	LISL_CTRLREG_1	(uint8)(0x20)
#define	LISL_CTRLREG_2	(uint8)(0x21)
#define	LISL_CTRLREG_3	(uint8)(0x22)
#define	LISL_STATUS		(uint8)(0x27)
#define LISL_OUTX_L		(uint8)(0x28)
#define LISL_OUTX_H		(uint8)(0x29)
#define LISL_OUTY_L		(uint8)(0x2A)
#define LISL_OUTY_H		(uint8)(0x2B)
#define LISL_OUTZ_L		(uint8)(0x2C)
#define LISL_OUTZ_H		(uint8)(0x2D)
#define LISL_FF_CFG		(uint8)(0x30)
#define LISL_FF_SRC		(uint8)(0x31)
#define LISL_FF_ACK		(uint8)(0x32)
#define LISL_FF_THS_L	(uint8)(0x34)
#define LISL_FF_THS_H	(uint8)(0x35)
#define LISL_FF_DUR		(uint8)(0x36)
#define LISL_DD_CFG		(uint8)(0x38)
#define LISL_INCR_ADDR	(uint8)(0x40)
#define LISL_READ		(uint8)(0x80)

void SendCommand(int8 c)
{
	static uint8 s;

	SPI_IO = WR_SPI;	
	SPI_CS = SEL_LISL;	
	for( s = 8; s; s-- ) {
		SPI_SCL = 0;
		if( c & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		c <<= 1;
		SPI_LO_DELAY;
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
} // SendCommand

uint8 ReadLISL(uint8 c)
{
	static uint8 d;

	SPI_SDA = 1; // very important!! really!! LIS3L likes it
	SendCommand(c);
	SPI_IO = RD_SPI;	// SDA is input
	d = ReadLISLNext();
	
	if( (c & LISL_INCR_ADDR) == (uint8)0 )
		SPI_CS = DSEL_LISL;
	return(d);
} // ReadLISL

uint8 ReadLISLNext(void)
{
	static uint8 s;
	static uint8 d;

	for( s = 8; s; s-- ) {
		SPI_SCL = 0;
		SPI_LO_DELAY;
		d <<= 1;
		if( SPI_SDA == (uint8)1 )
			d |= 1;	
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	return(d);
} // ReadLISLNext

void WriteLISL(uint8 d, uint8 c)
{
	static uint8 s;

	SendCommand(c);
	for( s = 8; s; s-- ) {
		SPI_SCL = 0;
		if( d & 0x80 )
			SPI_SDA = 1;
		else
			SPI_SDA = 0;
		d <<= 1;
		SPI_LO_DELAY;
		SPI_SCL = 1;
		SPI_HI_DELAY;
	}
	SPI_CS = DSEL_LISL;
	SPI_IO = RD_SPI;	// IO is input (to allow RS232 reception)
} // WriteLISL

void InitLISLAcc(void)
{
	static int8 r;

	Delay1mS(500);

	F.AccelerationsValid = false;

	r = ReadLISL(LISL_WHOAMI + LISL_READ);
	if( r == 0x3A ) { // a LIS03L sensor is there!
		WriteLISL(0b11000111, LISL_CTRLREG_1); // on always, 40Hz sampling rate,  10Hz LP cutoff, enable all axes
		WriteLISL(0b00000000, LISL_CTRLREG_3);
		WriteLISL(0b01000000, LISL_FF_CFG); // latch, no interrupts; 
		WriteLISL(0b00000000, LISL_FF_THS_L);
		WriteLISL(0b11111100, LISL_FF_THS_H); // -0,5g threshold
		WriteLISL(255, LISL_FF_DUR);
		WriteLISL(0b00000000, LISL_DD_CFG);
		F.AccelerationsValid = true;
	} else
		AccFailure();
} // InitLISLAcc

boolean LISLAccActive(void)
{
	SPI_CS = DSEL_LISL;
	WriteLISL(0b01001010, LISL_CTRLREG_2); // enable 3-wire, BDU=1, +/-2g

	F.AccelerationsValid = ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a;

	return ( F.AccelerationsValid );
} // LISLAccActive

void ReadLISLAcc()
{
	static charint16x4u L;

//	while( (ReadLISL(LISL_STATUS + LISL_READ) & 0x08) == (uint8)0 );

	F.AccelerationsValid = ReadLISL(LISL_WHOAMI + LISL_READ) == (uint8)0x3a; // Acc still there?
	if ( F.AccelerationsValid ) {
		L.c[0] = ReadLISL(LISL_OUTX_L + LISL_INCR_ADDR + LISL_READ);
		L.c[1] = ReadLISLNext();
		L.c[2] = ReadLISLNext();
		L.c[3] = ReadLISLNext();
		L.c[4] = ReadLISLNext();
		L.c[5] = ReadLISLNext();
		SPI_CS = DSEL_LISL;	// end transmission

		RawAcc[X] = L.i16[X];
		RawAcc[Y] = L.i16[Y];
		RawAcc[Z] = L.i16[Z];
	} else
		AccFailure();
} // ReadLISLAcc

#endif // INC_LISL
