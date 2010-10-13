
// Sparkfun 9DOF Razor IMU AHRS
// 9 Degree of Freedom Attitude and Heading Reference System
// Firmware v1.1gke
//
// Released under Creative Commons License
// Based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel
// Substantially rewritten by Prof. G.K. Egan 2010


float   Accel_V[3] = {
  0,0,0}; 		
float   Gyro_V[3] = {
  0,0,0};
float   Omega_V[3] = {
  0,0,0}; // corrected gyro data
float   Omega_P[3] = {
  0,0,0}; // proportional correction
float   Omega_I[3] = {
  0,0,0}; // integral correction
float   Omega[3] = {
  0,0,0};
float   DCM_M[3][3] = {
  {
    1,0,0      }
  ,{
    0,1,0      }
  ,{
    0,0,1      }
}; 
float   Update_M[3][3] = {
  {
    0,1,2        }
  ,{
    3,4,5        }
  ,{
    6,7,8        }
}; //Gyros here
float   Temp_M[3][3] = {
  {
    0,0,0        }
  ,{ 
    0,0,0        }
  ,{
    0,0,0        }
};

void Normalize(void)
{
  static float Error = 0.0;
  static float Temp[3][3];
  static float Renorm = 0.0;

  Error= -VDot(&DCM_M[0][0], &DCM_M[1][0]) * 0.5; 		//eq.19

  VScale(&Temp[0][0], &DCM_M[1][0], Error); 			//eq.19
  VScale(&Temp[1][0], &DCM_M[0][0], Error); 			//eq.19

  VAdd(&Temp[0][0], &Temp[0][0], &DCM_M[0][0]);			//eq.19
  VAdd(&Temp[1][0], &Temp[1][0], &DCM_M[1][0]);			//eq.19

  VCross(&Temp[2][0],&Temp[0][0], &Temp[1][0]);                 // c= a * b eq.20

  Renorm = 0.5 * (3.0 - VDot(&Temp[0][0], &Temp[0][0])); 	//eq.21
  VScale(&DCM_M[0][0], &Temp[0][0], Renorm);

  Renorm = 0.5 * (3.0 - VDot(&Temp[1][0], &Temp[1][0])); 	//eq.21
  VScale(&DCM_M[1][0], &Temp[1][0], Renorm);

  Renorm = 0.5 * (3.0 - VDot(&Temp[2][0], &Temp[2][0])); 	//eq.21
  VScale(&DCM_M[2][0], &Temp[2][0], Renorm);
} // Normalize

void DriftCorrection(void)
{
  static float MagHeadingX, MagHeadingY;
  static float ErrorCourse; 
  static float Scaled_Omega_P[3], Scaled_Omega_I[3];
  static float Accel_Magnitude, Accel_Weight;

  // Roll and Pitch

  Accel_Magnitude = sqrt(Accel_V[0]*Accel_V[0] + Accel_V[1]*Accel_V[1] + Accel_V[2]*Accel_V[2]);
  Accel_Magnitude = Accel_Magnitude / (float)GRAVITY; // Scale to gravity.

  // dynamic weighting of accelerometer info (reliability filter)
  // weight for accelerometer info ( < 0.5G = 0.0, 1G = 1.0 , > 1.5G = 0.0) 
  Accel_Weight = constrain(1 - 2 * abs(1 - Accel_Magnitude), 0, 1); 

  VCross(&RollPitchError[0], &Accel_V[0], &DCM_M[2][0]); //adjust the ground of reference
  VScale(&Omega_P[0], &RollPitchError[0], Kp_RollPitch * Accel_Weight);

  VScale(&Scaled_Omega_I[0], &RollPitchError[0], Ki_RollPitch * Accel_Weight);
  VAdd(Omega_I,Omega_I, Scaled_Omega_I);     

  // Yaw - drift correction based on compass magnetic heading

  MagHeadingX = cos(MagHeading);
  MagHeadingY = sin(MagHeading);
  ErrorCourse = (DCM_M[0][0] * MagHeadingY) - (DCM_M[1][0] * MagHeadingX);  
  VScale(YawError,&DCM_M[2][0], ErrorCourse); 

  VScale(&Scaled_Omega_P[0], &YawError[0], Kp_Yaw); 
  VAdd(Omega_P, Omega_P, Scaled_Omega_P);

  VScale(&Scaled_Omega_I[0], &YawError[0], Ki_Yaw); 
  VAdd(Omega_I,Omega_I, Scaled_Omega_I); 
} // DriftCorrection

void AccelAdjust(void)
{
  float GPS_3D = 0.0;

  // Is this code wrong? 
  //Accel_V[1] += GPS_3D * Omega[2] * (GRAVITY/9.81); //  AccY = GPS_Velocity*GyroZ
  //Accel_V[2] -= GPS_3D * Omega[1] * (GRAVITY/9.81); //  AccZ = GPS_Velocity*GyroY 
} // AccelAdjust

void MUpdate(void)
{
  static byte i, j, k;
  static float op[3];

  for ( i = 0; i < 3 ; i++)
  {
    Gyro_V[i] = (float)Gyro[i] * GyroToRadianSec; 
    Accel_V[i] = (float)Acc[i];
  }

  VAdd(&Omega[0], &Gyro_V[0], &Omega_I[0]);
  VAdd(&Omega_V[0], &Omega[0], &Omega_P[0]);

  AccelAdjust(); // remove centrifugal accelerations. 

#if OUTPUTMODE==1         
  Update_M[0][0] = 0;
  Update_M[0][1] = -G_Dt * Omega_V[2];	//-z
  Update_M[0][2] = G_Dt * Omega_V[1];	//y
  Update_M[1][0] = G_Dt * Omega_V[2];	//z
  Update_M[1][1] = 0;
  Update_M[1][2] = -G_Dt * Omega_V[0];	//-x
  Update_M[2][0] = -G_Dt * Omega_V[1];	//-y
  Update_M[2][1] = G_Dt * Omega_V[0];	//x
  Update_M[2][2] = 0;
#else                    // Uncorrected data (no drift correction)
  Update_M[0][0] = 0;
  Update_M[0][1] = -G_Dt * Gyro_V[2];	//-z
  Update_M[0][2] = G_Dt * Gyro_V[1];	//y
  Update_M[1][0] = G_Dt * Gyro_V[2];	//z
  Update_M[1][1] = 0;
  Update_M[1][2] = -G_Dt * Gyro_V[0];
  Update_M[2][0] = -G_Dt * Gyro_V[1];
  Update_M[2][1] = G_Dt * Gyro_V[0];
  Update_M[2][2] = 0;
#endif

  for( i = 0; i < 3; i++ )
    for ( j = 0; j < 3; j++ )
    {
      for ( k = 0; k < 3; k++ )
        op[k] = DCM_M[i][k] * Update_M[k][j];

      Temp_M[i][j] = op[0] + op[1] + op[2];
    }

  for ( i = 0; i < 3; i++ )
    for (j = 0; j < 3; j++ )
      DCM_M[i][j] += Temp_M[i][j];

} // MUpdate

void EulerAngles(void)
{
  Pitch = -asin( DCM_M[2][0] );
  Roll = atan2( DCM_M[2][1], DCM_M[2][2] );
  Yaw = atan2( DCM_M[1][0], DCM_M[0][0] );
} // EulerAngles


