
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
	
// Useful Constants
#define NUL 	0
#define SOH 	1
#define EOT 	4
#define ACK	6
#define HT 	9
#define LF 	10
#define CR 	13
#define NAK 	21
#define ESC 	27

#define MILLIPI 		3142 
#define CENTIPI 		314 
#define HALFMILLIPI 		1571 
#define QUARTERMILLIPI		785
#define SIXTHMILLIPI		524
#define TWOMILLIPI 		6284

#define MILLIRAD 		18 
#define CENTIRAD 		2

// ADXL345 Sensitivity(from datasheet) => 4mg/LSB   1G => 1000mg/4mg = 256 steps
// Tested value : 248
#define GRAVITY 248  //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => (3.3mv at 3v)at 3.3v: 3mV/º/s, 3.22mV/ADC step => 0.93
// Tested values : 0.92
#define Gyro_Gain_X 0.92 //X axis Gyro gain
#define Gyro_Gain_Y 0.92 //Y axis Gyro gain
#define Gyro_Gain_Z 0.92 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_RollPitch 0.02
#define Ki_RollPitch 0.00002
#define Kp_Yaw 1.2
#define Ki_Yaw 0.00002

#define ADC_WARM_CYCLES 50
#define STATUS_LED 13 

// Debug

#define OUTPUTMODE 1
#define PRINT_EULER 1
#define PRINT_ANALOGS 0
#define PRINT_DCM 0
//_____________________________________________________________________

const byte Map[3] = {1,2,0};  			// Map the ADC channels gyro x,y,z
const int SensorSign[9] = {-1,1,-1,1,1,1,-1,-1,-1};  //Correct directions x,y,z - gyros, accels, magnetometer

float   G_Dt = 0.01;    			// DCM integration time 100Hz if possible

long    ClockmSp;
long    PeriodmS;
long    ClockmS;
byte    CompassInterval = 0;				//general purpuse timer

float   Sensor[6] = {0,0,0,0,0,0}; 				// gyros filtered data
float   SensorNeutral[6] = {0,0,0,0,0,0};       // sensor neutral values
int     Acc[3];          			// accelerometers data

int     AccX, AccY, AccZ;
int     MagX, MagY, MagZ;
float   MagHeading;

// Euler angles
float   Roll, Pitch, Yaw;

float   RollPitchError[3] = {0,0,0}; 
float   YawError[3] = {0,0,0};

void setup()
{
  static byte i, c;

  Serial.begin(115200);
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  Serial.print("Start");
  Serial.println();
  
  ADCReference(DEFAULT); 
  InitADCBuffers();
  InitADC();
  InitI2C();
  InitAccelerometers();
  GetAccelerometer();
  InitMagnetometer();  
 
  for( i = 0; i < 32; i++ )
  {
    GetGyroRaw();
    GetAccelerometer();
    for( c = 0; c < 6; c++ )
      SensorNeutral[c] += Sensor[c];
    delay(20);
  }
      
  for( c = 0; c < 6; c++ )
    SensorNeutral[c] /= 32.0;
      
  SensorNeutral[5] -= GRAVITY * (float)SensorSign[5];
  
  // need to capture acc neutrals at setup as for UAVX
  SensorNeutral[3] = -13.31;
  SensorNeutral[4] = 0.81;
  SensorNeutral[5] = -18.06;
    
  Serial.println("Offset:");
  for( c = 0; c < 6; c++ )
    Serial.println(SensorNeutral[c]);
    
  delay(2000);
  digitalWrite(STATUS_LED, HIGH);
      
  ClockmSp = millis();
  delay(20);
}

void loop()
{
  ClockmS = millis();
  PeriodmS = ClockmS - ClockmSp;
  if( PeriodmS >= 10 )  // Main loop runs at 50Hz
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
 
    PrintData();  
  }

} // Main
