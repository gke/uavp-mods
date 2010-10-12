
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

// use oversampling and averaging method to increase the ADC resolution
// do not bother with storing Gyro values as floats

byte MuxSel = 0;
byte ADCRef;
word ADCBuff[6];
byte ADCCount[6];

void InitADCBuffers(void)
{
  static byte c;

  for ( c = 0; c < 3; c++ )
  {
    noInterrupts();
    ADCBuff[c] = ADCCount[c] = 0;
    interrupts();
  }
} // InitADCBuffers 

void GetGyroRaw(void)
{
  word s;    
  byte c, samples, m;    

  for ( c = 0 ; c < 3 ; c++ )
  {
    m = Map[c];
    noInterrupts();
    s = ADCBuff[m];              
    samples = ADCCount[m];
    interrupts(); 

    if ( samples > 0 ) // Check for divide by zero  
      GyroADC[c] = ( s + samples ) / samples;
    Gyro[c] = GyroSign[c] * (GyroADC[c] - GyroNeutral[c]);      
  }
  
  InitADCBuffers();

} // GetGyroRaw

void ADCReference(byte m)
{ // why a call?
  ADCRef = m;
} // ADCReference

void InitADC(void)
{
  ADCSRA |= ( 1 << ADIE )|( 1 << ADEN );
  ADCSRA |= ( 1 << ADSC );
  delay(100);  // let ADC cycle a few times
} // InitADC

// cycle through gyro ADC channels at maximum conversion rate 
ISR(ADC_vect)
{
  volatile byte low, high;

  low = ADCL;
  high = ADCH;

  if ( ADCCount[MuxSel] < 63) 
  {
    ADCBuff[MuxSel] += word( high << 8 ) | low; // accumulate samples
    ADCCount[MuxSel]++;
  }

  MuxSel = (MuxSel + 1) & 3; 
  ADMUX = ( ADCRef << 6) | MuxSel;
  ADCSRA |= ( 1 << ADSC );   // start the conversion

} // ISR


