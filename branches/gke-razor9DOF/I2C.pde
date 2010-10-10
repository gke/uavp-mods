// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

// I2C code for ADXL345 accelerometer and HMC5843 magnetometer 

#define I2C_ACC_ID 0x53
#define I2C_COMPASS_ID 0x1E //0x3C //0x3D;  //(0x42>>1);

void InitI2C(void)
{
  Wire.begin();
} // InitI2C

void InitAccelerometers()
{
  delay(100);  // let accelerometers settle

  Wire.beginTransmission(I2C_ACC_ID);
  Wire.send(0x2D);  // power register
  Wire.send(0x08);  // measurement mode
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(I2C_ACC_ID);
  Wire.send(0x31);  // Data format register
  Wire.send(0x08);  // set to full resolution
  Wire.endTransmission();
  delay(5);

  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
  Wire.beginTransmission(I2C_ACC_ID);
  Wire.send(0x2C);  // Rate
  Wire.send(0x09);  // set to 50Hz, normal operation
  Wire.endTransmission();
  delay(5);
} // InitAccelerometers

void GetAccelerometer()
{
  static int i;
  static byte buff[6];

  Wire.beginTransmission(I2C_ACC_ID); 
  Wire.send(0x32);     
  Wire.endTransmission(); 

  Wire.beginTransmission(I2C_ACC_ID); 
  Wire.requestFrom(I2C_ACC_ID, 6);    // request 6 bytes from device

  i = 0; 
  while(Wire.available()) 
    buff[i++] = Wire.receive();  

  Wire.endTransmission(); 

  if ( i==6 ) 
  {
    Acc[1] = ( (int)buff[1] << 8) | buff[0];    // Y axis (internal sensor x axis)
    Acc[0] = ( (int)buff[3] << 8) | buff[2];    // X axis (internal sensor y axis)
    Acc[2] = ( (int)buff[5] << 8) | buff[4];    // Z axis
    Sensor[3] = Acc[0];
    Sensor[4] = Acc[1];
    Sensor[5] = Acc[2];
    AccX = SensorSign[3] * (Acc[0] - SensorNeutral[3]);
    AccY = SensorSign[4] * (Acc[1] - SensorNeutral[4]);
    AccZ = SensorSign[5] * (Acc[2] - SensorNeutral[5]);
  }
  else
    Serial.println("!ERR: Acc data");
} // GetAccelerometer

//_____________________________________________________________________

void InitMagnetometer()
{
  delay(100); // let magnetometer settle
  Wire.beginTransmission(I2C_COMPASS_ID);
  Wire.send(0x02); 
  Wire.send(0x00);   // continuous mode (default to 10Hz)
  Wire.endTransmission(); 
} // InitMagnetometer

void GetMagnetometer(void)
{
  static int i;
  static byte buff[6];

  Wire.beginTransmission(I2C_COMPASS_ID); 
  Wire.send(0x03);
  Wire.endTransmission(); 

  Wire.beginTransmission(I2C_COMPASS_ID); 
  Wire.requestFrom(I2C_COMPASS_ID, 6);    // 6 bytes

  i = 0;
  while(Wire.available())
    buff[i++] = Wire.receive();  

  Wire.endTransmission(); //end transmission

    if ( i==6 )  
  {
    // MSB byte first, then LSB, X,Y,Z
    MagX = SensorSign[6] * (((int)buff[2]) << 8) | buff[3];    // X axis (internal sensor y axis)
    MagY = SensorSign[7] * (((int)buff[0]) << 8) | buff[1];    // Y axis (internal sensor x axis)
    MagZ = SensorSign[8] * (((int)buff[4]) << 8) | buff[5];    // Z axis
  }
  else
    Serial.println("!ERR: Mag data");

} // GetMagnetometer


