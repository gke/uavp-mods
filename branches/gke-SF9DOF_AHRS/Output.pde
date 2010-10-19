
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.1gke
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K. Egan 2010

#if USING_UAVX == 1

byte TxCheckSum;

#if PRINT_UAVX_READABLE == 1

void TxByte(char b)
{
  Serial.print(b);
  Serial.print(',');
} // TxByte

void TxWord(int w)
{
  Serial.print(w);
  Serial.print(','); 
} // TxWord

#else

void TxByte(char b)
{
  Serial.write(b);
  TxCheckSum ^= b;
} // TxByte

void TxWord(int w)
{
  TxByte(lowByte(w));
  TxByte(highByte(w));
} // TxWord

#endif // PRINT_UAVX_READABLE

void SendAttitude(void)
{
  static byte i;

  TxByte(byte('$')); // sentinel not included in checksum
  TxCheckSum = 0;
  TxByte(UAVXRazorPacketTag);
  TxByte(23);

  // angles in milliradian
  TxWord((int)(Roll * 1000.0 ));
  TxWord((int)(Pitch * 1000.0 ));
  TxWord((int)(Yaw * 1000.0 )); 

  TxWord((int)(Omega_V[0] * 1000.0 ));
  TxWord((int)(Omega_V[1] * 1000.0 ));
  TxWord((int)(Omega_V[2] * 1000.0 ));
 
  TxWord((int)(Acc_V[0] * AccToMilliG));
  TxWord((int)(Acc_V[1] * AccToMilliG));
  TxWord((int)(Acc_V[2] * AccToMilliG));
  
  TxWord((int)(MagHeading * 1000.0));

  TxByte(TxCheckSum);

#if PRINT_UAVX_READABLE == 1
  Serial.println();
#endif // PRINT_UAVX_READABLE

} // SendAttitude

#else

void SendAttitude(void)
{ 
  static byte i,j;

  Serial.print("!");

#if PRINT_EULER == 1

  Serial.print("ANG:");
  Serial.print(Roll * 57.2957795131);
  Serial.print(",");
  Serial.print(Pitch * 57.2957795131);
  Serial.print(",");
  Serial.print(Yaw * 57.2957795131);
  Serial.print(",");
 
#endif // PRINT_EULER

#if PRINT_ANALOGS == 1

  Serial.print("AN:");
  for ( i = 0; i < 3 ; i++ )
  {
    Serial.print(GyroADC[Map[i]]); 
    Serial.print(",");
  }
  for ( i = 0; i < 3 ; i++ )
  {
    Serial.print(AccADC[i]);
    Serial.print (",");
  }
  for ( i = 0; i < 3 ; i++ )
  { 
    Serial.print(Mag[i]); 
    Serial.print (","); 
  }

#endif // PRINT_ANALOGS

#if PRINT_DCM == 1

  Serial.print ("DCM:");
  for ( i = 0; i < 3 ; i++ )
    for ( j = 0; j < 3 ; j++ )
    {
      Serial.print(DCM_Matrix[i][j]*10000000);
      Serial.print (",");
    }

#endif // PRINT_DCM
  Serial.println();    

} // SendAttitude

#endif // USING_UAVX


