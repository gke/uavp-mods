
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

#if USING_UAVX == 1

byte TxCheckSum;

#if PRINT_READABLE == 1

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
  static byte h,l;

  h = highByte(w);
  l = lowByte(w);

  TxCheckSum ^= h;
  TxCheckSum ^= l;

  Serial.write(highByte(w));
  Serial.write(lowByte(w));
} // TxWord

#endif // PRINT_READABLE

void SendAttitude(void)
{
  static byte i;

  TxCheckSum = 0;
  Serial.write('$'); // sentinel not included in checksum

  // rescale angles and rates to milliradian
  TxWord((int)(RollAngle * 1000.0));
  TxWord((int)(PitchAngle * 1000.0));
  TxWord((int)(YawAngle * 1000.0));

  RollRate = (RollAngleP - RollAngle);
  PitchRate = (PitchAngleP - PitchAngle);
  YawRate = (YawAngleP - YawAngle);

  TxWord((int)(RollRate * 1000.0));
  TxWord((int)(PitchRate * 1000.0));
  TxWord((int)(YawRate * 1000.0));

  RollAngleP = RollAngle;
  PitchAngleP = PitchAngle;
  YawAngleP = YawAngle;

  // 1G is 256
  for ( i = 0 ; i < 3 ; i++ )
    TxWord(Acc[i]); 

  for ( i = 0 ; i < 3 ; i++ )
    TxWord(AccNeutral[i]); 

  TxWord((int)(MagHeading * 1000.0));

  TxByte(TxCheckSum);

#if PRINT_READABLE == 1
  Serial.println();
#endif // PRINT_READABLE

} // SendAttitude

#else

void SendAttitude(void)
{ 
  static byte i,j;

  Serial.print("!");

#if PRINT_EULER == 1
  Serial.print("ANG:");
  Serial.print(RollAngle*57.2957795131);
  Serial.print(",");
  Serial.print(PitchAngle*57.2957795131);
  Serial.print(",");
  Serial.print(YawAngle*57.2957795131);
  Serial.print(",");
#endif

#if PRINT_ANALOGS==1

  Serial.print("AN:");
  for ( i = 0; i < 3 ; i++ )
  {
    Serial.print(GyroADC[0]); 
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

#endif

#if PRINT_DCM == 1

  Serial.print ("DCM:");
  for ( i = 0; i < 3 ; i++ )
    for ( j = 0; j < 3 ; j++ )
    {
      Serial.print(DCM_Matrix[i][j]*10000000);
      Serial.print (",");
    }

#endif
  Serial.println();    

} // SendAttitude

#endif // USING_UAVX


