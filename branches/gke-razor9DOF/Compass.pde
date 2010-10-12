
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.0
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K.  Egan 2010

// Local magnetic declination not included
// http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp

void ComputeHeading(void)
{
  static float MX, MY;
  static float Cos_Roll, Sin_Roll, Cos_Pitch, Sin_Pitch;
    
  Cos_Roll = cos(RollAngle);
  Sin_Roll = sin(RollAngle);
  Cos_Pitch = cos(PitchAngle);
  Sin_Pitch = sin(PitchAngle);
  
  // Tilt compensated Magnetic field X:
  MX = (float)Mag[0] * Cos_Pitch + (float)Mag[1] * Sin_Roll * Sin_Pitch + (float)Mag[2] * Cos_Roll * Sin_Pitch;
  	
  // Tilt compensated Magnetic field Y:
  MY = (float)Mag[1] * Cos_Roll - (float)Mag[2] * Sin_Roll;
  
  // Magnetic Heading
  MagHeading = atan2( -MY, MX );

} // ComputeHeading
