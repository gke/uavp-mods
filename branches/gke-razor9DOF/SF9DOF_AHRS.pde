
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

// Axis definition: 
// X axis pointing forward (to the FTDI connector)
// Y axis pointing to the right 
// and Z axis pointing down.
// Pitch : positive up
// Roll : positive right
// Yaw : positive clockwise

/* 
 Hardware version - v13
 	
 ATMega328@3.3V w/ external 8MHz resonator
 High Fuse DA
 Low Fuse FF
 	
 ADXL345: Accelerometer
 HMC5843: Magnetometer
 LY530:	Yaw Gyro
 LPR530:	Pitch and Roll Gyro
 
 Programmer : 3.3v FTDI
 Arduino IDE : Select board  "Arduino Duemilanove w/ATmega328"
 This code works also on ATMega168 Hardware
 */

#include <Wire.h>
#include <EEPROM.h>

#define EEPROMBase 128  // keep clear of location zero

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
#define GRAVITY 256

// LPR530 & LY530 Sensitivity (from datasheet) => (3.3mv at 3v)at 3.3v: 3mV/º/s, 3.22mV/ADC step => 0.93
// Tested values : 0.92
#define GyroToDegreesSec 0.92 // deg/sec
#define GyroToRadianSec 0.016057 // radian/sec 

#define Kp_RollPitch 0.02
#define Ki_RollPitch 0.00002
#define Kp_Yaw 1.2
#define Ki_Yaw 0.00002

#define OUTPUTMODE 1
#define PRINT_EULER 1
#define PRINT_ANALOGS 0
#define PRINT_DCM 0

#define PRINT_READABLE 1
#define USING_UAVX 0
#define FREE_RUNNING 1
//_____________________________________________________________________

const byte Map[3] = {
  1,2,0}; // Map the ADC channels gyro x,y,z
const int GyroSign[3] = {
  -1,1,-1 };
const int AccSign[3] = {
  1,1,1};
const int MagSign[3] = {
  -1,-1,-1};  

float   G_Dt = 0.01; // DCM integration time 100Hz if possible

long    ClockmSp;
long    PeriodmS;
long    ClockmS;
byte    CompassInterval = 0;

int     Gyro[3], GyroADC[3], GyroNeutral[3];
int     Acc[3], AccADC[3], AccNeutral[3];
char    AccNeutralEE[3];   

int     Mag[3], MagADC[3];
float   MagHeading;

// Euler angles
float   RollAngle, PitchAngle, YawAngle;
float   RollAngleP, PitchAngleP, YawAngleP;
float   RollRate, PitchRate, YawRate;

float   RollPitchError[3] = {
  0,0,0}; 
float   YawError[3] = {
  0,0,0};

byte i;

void Initialise()
{
  static byte i, c;

#if FREE_RUNNING == 1

  AccNeutralEE[0] = 0;
  AccNeutralEE[1] = 0;
  AccNeutralEE[2] = 0;

#else

  for ( i = 0 ; i < 3 ; i++)
    AccNeutralEE[i] = char(EEPROM.read(EEPROMBase+i)); // probably 0xff or -1 if uninitialised OK
    
#endif // FREE_RUNNING

  for( i = 0; i < 32; i++ )
  {
    GetGyroRaw();
    GetAccelerometer();
    for( c = 0; c < 3; c++ )
    {
      GyroNeutral[c] += GyroADC[c];
      AccNeutral[c] += AccADC[c];
    }
    delay(20);
  }

  for( c = 0; c < 3; c++ )
  {
    GyroNeutral[c] >>= 5;
    AccNeutral[c] >>= 5;
  }

 // AccNeutral[3] -= GRAVITY * AccSign[3];

  RollAngleP = PitchAngleP = YawAngleP = 0.0;

  ClockmSp = millis();

} // Initialise

void setup()
{
  Serial.begin(115200);
  ADCReference(DEFAULT); 

  InitADCBuffers();
  InitADC();
  InitI2C();
  InitAccelerometers();
  GetAccelerometer();
  InitMagnetometer();

  Initialise();
} // setup

char ch;

void loop()
{

#if FREE_RUNNING == 1
  ClockmS = millis();
  PeriodmS = ClockmS - ClockmSp;
  if  ( PeriodmS >= 20 )
  {
    ClockmSp = ClockmS;
    G_Dt = (float)PeriodmS / 1000.0;

    GetGyroRaw(); 
    GetAccelerometer();   

    if ( ++CompassInterval > 5) // compass 1/5 rate
    {
      CompassInterval = 0;
      GetMagnetometer();
      ComputeHeading();
    }

    MUpdate(); 
    Normalize();
    DriftCorrection();
    EulerAngles(); 
    
    SendAttitude(); 
  }

#else

  if ( Serial.available() > 1 )
  {  
    ch = Serial.read();
    if ( ch == '$' )
    {
      ch = char(Serial.read());
      switch ( ch ) {
      case '?': 
        ClockmS = millis();
        PeriodmS = ClockmS - ClockmSp;

        ClockmSp = ClockmS;
        G_Dt = (float)PeriodmS / 1000.0;

        GetGyroRaw(); 
        GetAccelerometer();   

        if ( ++CompassInterval > 5) // compass 1/5 rate
        {
          CompassInterval = 0;
          GetMagnetometer();
          ComputeHeading();
        }

        MUpdate(); 
        Normalize();
        DriftCorrection();
        EulerAngles(); 
        
        SendAttitude(); 

        break; 
      case '!':
        Initialise();

        break;
      case 'N':
        while ( Serial.available() < 3 );
        for ( i = 0; i < 3; i++ )
        {
          AccNeutralEE[i] = char(Serial.read());
          EEPROM.write(EEPROMBase+i, AccNeutral[i]);
        }

        break;
      default:;
      } // switch
    }
  }

#endif // FREE_RUNNING
} // Main

