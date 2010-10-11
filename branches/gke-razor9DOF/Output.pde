
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

byte TxCheckSum = 0;

void TxByte(byte b)
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

  // 1G is 256 - UAVX is scaled to 1024
  TxWord((int)AccX); // FBAcc
  TxWord((int)AccY); // LRAcc
  TxWord((int)AccZ); // DUAcc

  TxWord((int)(MagHeading * 1000.0));

  TxWord((int)SensorNeutral[3]); // NeutralFB
  TxWord((int)SensorNeutral[3]); // NeutralLR
  TxWord((int)SensorNeutral[5]); // NeutralDU

  TxByte(TxCheckSum);
  Serial.println();

} // SendAttitude

char PollRxChar(void)
{
  if ( Serial.available() < 1 )
    return (0);
  else
    return(Serial.read());
} // PollRxChar

signed char RxNumS(void)
{
  // UAVPSet sends sign and 2 digits
  byte ch;
  signed char n;
  byte Neg;
  n = 0;

  Neg = false;
  do
    ch = PollRxChar();
  while( ((ch < '0') || (ch > '9')) && (ch != '-') );
  if( ch == '-' )
  {
    Neg = true;
    do
      ch = PollRxChar();
    while( (ch < '0') || (ch > '9') );
  }
  n = (ch - '0') * 10;

  do
    ch = PollRxChar();
  while( (ch < '0') || (ch > '9') );
  n += ch - '0';
  if( Neg )
    n = -n;
  return(n);
} // RxNumS




