
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

void PrintData(void)
{   
   static byte i, j, c;
   
  // Serial.print("!");

  #if PRINT_EULER == 1
    Serial.print(ToDeg(Roll));
    Serial.print(",");
    Serial.print(ToDeg(Pitch));
    Serial.print(",");
    Serial.print(ToDeg(Yaw));
  #endif      
  #if PRINT_ANALOGS == 1
    Serial.print(ClockmS);
    Serial.print(",");
    for ( c = 0; c < 6; c++ )
    {
      Serial.print(Sensor[c] - SensorNeutral[c]); 
      Serial.print(",");
    }
    Serial.print(MagX);
    Serial.print (",");
    Serial.print(MagY);
    Serial.print (",");
    Serial.print(MagZ);  
  #endif
  #if PRINT_DCM == 1
    Serial.print (",DCM:");
    for ( i = 0; i < 3; i++ )
      for ( j = 0; j < 3; j++ )
      {
        Serial.print(DCM_M[i][j]);
        Serial.print (",");
      }
  #endif
  Serial.println();    
      
} // PrintData

long convert_to_dec(float x)
{
  return x*10000000;
}

