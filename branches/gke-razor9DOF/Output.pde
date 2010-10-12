
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

byte TxCheckSum;

void TxByte(char b)
{
  Serial.write(b);
  TxCheckSum ^= b;
} // TxByte

void TxWord(word w)
{
  static byte h,l;

  h = highByte(w);
  l = lowByte(w);

  TxCheckSum ^= h;
  TxCheckSum ^= l;

  Serial.write(highByte(w));
  Serial.write(lowByte(w));
} // TxWord

void SendAttitude(void)
{
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
  Serial.println();

} // SendAttitude



