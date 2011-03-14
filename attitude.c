// ===============================================================================================
// =                              UAVXArm Quadrocopter Controller                                =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVXArm.

//    UAVXArm is free software: you can redistribute it and/or modify it under the terms of the GNU
//    General Public License as published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.

//    UAVXArm is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/

#include "UAVXArm.h"

// Reference frame is positive X forward, Y left, Z down, Roll right, Pitch up, Yaw CW.
// CAUTION: Because of the coordinate frame the LR Acc sense must be negated for roll compensation.

void AdaptiveYawLPFreq(void);
void DoLegacyYawComp(uint8);
void NormaliseAccelerations(void);
void AttitudeTest(void);

real32 AccMagnitude;
real32 EstAngle[3][MaxAttitudeScheme];
real32 EstRate[3][MaxAttitudeScheme];
real32 YawFilterLPFreq;
real32 dT, dTOn2, dTR, dTmS;
uint32 uSp;
uint8 AttitudeMethod = Wolferl; //Integrator, Wolferl MadgwickIMU PremerlaniDCM MadgwickAHRS, MultiWii;

void AdaptiveYawLPFreq(void) { // Filter LP freq is decreased with reduced yaw stick deflection

    YawFilterLPFreq = ( MAX_YAW_FREQ * abs(DesiredYaw) / RC_NEUTRAL );
    YawFilterLPFreq = Limit(YawFilterLPFreq, 0.5, MAX_YAW_FREQ);

} // AdaptiveYawFilterA

void DoLegacyYawComp(uint8 S) {

#define COMPASS_MIDDLE          10                 // yaw stick neutral dead zone
#define DRIFT_COMP_YAW_RATE     QUARTERPI          // Radians/Sec

    static int16 Temp;
    static real32 HE;

    // Yaw Angle here is meant to be interpreted as the Heading Error

    Rate[Yaw] = Gyro[Yaw];

    Temp = DesiredYaw - Trim[Yaw];
    if ( F.CompassValid )  // CW+
        if ( abs(Temp) > COMPASS_MIDDLE ) {
            DesiredHeading = Heading; // acquire new heading
            Angle[Yaw] = 0.0;
        } else {
            HE = MakePi(DesiredHeading - Heading);
            HE = Limit(HE, -SIXTHPI, SIXTHPI); // 30 deg limit
            HE = HE * K[CompassKp];
            Rate[Yaw] = -Limit(HE, -DRIFT_COMP_YAW_RATE, DRIFT_COMP_YAW_RATE);
        }
    else {
        DesiredHeading = Heading;
        Angle[Yaw] = 0.0;
    }

    Angle[Yaw] += Rate[Yaw] * dT;
    Angle[Yaw] = Limit(Angle[Yaw], -K[YawIntLimit], K[YawIntLimit]);

} // DoLegacyYawComp

void NormaliseAccelerations(void) {

    const real32 MIN_ACC_MAGNITUDE = 0.7; // below this the accelerometers are deemed unreliable - falling?

    static real32 ReNorm;

    AccMagnitude = sqrt(Sqr(Acc[BF]) + Sqr(Acc[LR]) + Sqr(Acc[UD]));
    F.AccMagnitudeOK = AccMagnitude > MIN_ACC_MAGNITUDE;
    if ( F.AccMagnitudeOK ) {
        ReNorm = 1.0 / AccMagnitude;
        Acc[BF] *= ReNorm;
        Acc[LR] *= ReNorm;
        Acc[UD] *= ReNorm;
    } else {
        Acc[LR] = Acc[BF]  = 0.0;
        Acc[UD] = 1.0;
    }
} // NormaliseAccelerations

void GetAttitude(void) {

    static uint32 Now;
    static uint8 i;

    if ( GyroType == IRSensors )
        GetIRAttitude();
    else {
        GetGyroRates();
        GetAccelerations();
    }

    Now = uSClock();
    dT = ( Now - uSp) * 0.000001;
    dTOn2 = 0.5 * dT;
    dTR = 1.0 / dT;
    uSp = Now;

    GetHeading(); // only updated every 50mS but read continuously anyway

    if ( GyroType == IRSensors ) {

        for ( i = 0; i < (uint8)2; i++ ) {
            Rate[i] = ( Angle[i] - Anglep[i] ) * dT;
            Anglep[i] = Angle[i];
        }

        Rate[Yaw] = 0.0; // need Yaw gyro!

    } else {
        DebugPin = true;

        NormaliseAccelerations(); // rudimentary check for free fall etc

        // Wolferl
        DoWolferl();

#ifdef INC_ALL_SCHEMES

        // Complementary
        DoCF();

        // Kalman
        DoKalman();

        //Premerlani DCM
        DoDCM();

        // MultiWii
        DoMultiWii();

        // Madgwick IMU BROKEN
        DoMadgwickIMU(Gyro[Roll], Gyro[Pitch], Gyro[Yaw], Acc[BF], -Acc[LR], Acc[UD]);

        // Madgwick AHRS BROKEN
        DoMadgwickAHRS(Gyro[Roll], Gyro[Pitch], Gyro[Yaw], Acc[BF], -Acc[LR], Acc[UD],Mag[BF].V, Mag[LR].V, Mag[UD].V);

        // Integrator - REFERENCE/FALLBACK
        DoIntegrator();

#endif // INC_ALL_SCHEMES

        Angle[Roll] = EstAngle[Roll][AttitudeMethod];
        Angle[Pitch] = EstAngle[Pitch][AttitudeMethod];

        DebugPin = false;
    }

    F.NearLevel = Max(fabs(Angle[Roll]), fabs(Angle[Pitch])) < NAV_RTH_LOCKOUT;

} // GetAttitude

//____________________________________________________________________________________________

// Integrator

void DoIntegrator(void) {

    static uint8 g;

    for ( g = 0; g < (uint8)3; g++ ) {
        Rate[g] = Gyro[g];
        EstAngle[g][Integrator] += Rate[g] * dT;
        EstAngle[g][Integrator] = DecayX(EstAngle[g][Integrator], 0.03 * dT);
    }

} // DoIntegrator

//____________________________________________________________________________________________

// Original simple accelerometer compensation of gyros developed for UAVP by Wolfgang Mahringer
// and adapted for UAVXArm

void DoWolferl(void) { // NO YAW ESTIMATE

    const real32 WKp = 0.13; // 0.1

    static real32 Grav[2], Dyn[2], Correction[2];
    static real32 CompStep;

    Rate[Roll] = Gyro[Roll];
    Rate[Pitch] = Gyro[Pitch];

    if ( F.AccMagnitudeOK ) {

        CompStep = WKp * dT;

        // Roll

        Grav[LR] = -sin(EstAngle[Roll][Wolferl]); // original used approximation for small angles

#ifdef ENABLE_DYNAMIC_MASS_COMP_ROLL
        Dyn[LR] = Rate[Roll];   // lateral acceleration due to rate - do later:).
#else
        Dyn[LR] = 0.0;
#endif

        Correction[LR] = -Acc[LR] + Grav[LR] + Dyn[LR]; // Acc is reversed
        Correction[LR] = Limit(Correction[LR], -CompStep, CompStep);

        EstAngle[Roll][Wolferl] += Rate[Roll] * dT;
        EstAngle[Roll][Wolferl] = Limit(EstAngle[Roll][Wolferl], -ATTITUDE_ANGLE_LIMIT, ATTITUDE_ANGLE_LIMIT);
        EstAngle[Roll][Wolferl] += Correction[LR];

        // Pitch

        Grav[BF] = -sin(EstAngle[Pitch][Wolferl]);

#ifdef ENABLE_DYNAMIC_MASS_COMP_PITCH
        Dyn[BF] = Rate[Pitch];
#else
        Dyn[BF] = 0.0;
#endif

        Correction[BF] = Acc[BF] + Grav[BF] + Dyn[BF];
        Correction[BF] = Limit(Correction[BF], -CompStep, CompStep);

        EstAngle[Pitch][Wolferl] += Rate[Pitch] * dT;
        EstAngle[Pitch][Wolferl] = Limit(EstAngle[Pitch][Wolferl], -ATTITUDE_ANGLE_LIMIT, ATTITUDE_ANGLE_LIMIT);
        EstAngle[Pitch][Wolferl] += Correction[BF];
    } else {
        EstAngle[Roll][Wolferl] += Rate[Roll] * dT;
        EstAngle[Pitch][Wolferl] += Rate[Pitch] * dT;
    }

} // DoWolferl

//____________________________________________________________________________________________

// The DCM formulation used here is due to W. Premerlani and P. Bizard in a paper entitled:
// Direction Cosine Matrix IMU: Theory, Draft 17 June 2009. This paper draws upon the original
// work by R. Mahony et al. - Thanks Rob!

// SEEMS TO BE A FAIRLY LARGE PHASE DELAY

void DCMNormalise(void);
void DCMDriftCorrection(void);
void DCMMotionCompensation(void);
void DCMUpdate(void);
void DCMEulerAngles(void);

// requires rescaling for the much faster PID cycle in UAVXArm

real32   RollPitchError[3] = {0,0,0};
real32   OmegaV[3] = {0,0,0}; // corrected gyro data
real32   OmegaP[3] = {0,0,0}; // proportional correction
real32   OmegaI[3] = {0,0,0}; // integral correction
real32   Omega[3] = {0,0,0};
real32   DCM[3][3] = {{1,0,0 },{0,1,0} ,{0,0,1}};
real32   U[3][3] = {{0,1,2},{ 3,4,5} ,{6,7,8}};
real32   TempM[3][3] = {{0,0,0},{0,0,0},{ 0,0,0}};

void DCMNormalise(void) {

    static real32 Error = 0;
    static real32 Renorm = 0.0;
    static boolean Problem;
    static uint8 r;

    Error = -VDot(&DCM[0][0], &DCM[1][0]) * 0.5;        //eq.19

    VScale(&TempM[0][0], &DCM[1][0], Error);            //eq.19
    VScale(&TempM[1][0], &DCM[0][0], Error);            //eq.19

    VAdd(&TempM[0][0], &TempM[0][0], &DCM[0][0]);       //eq.19
    VAdd(&TempM[1][0], &TempM[1][0], &DCM[1][0]);       //eq.19

    VCross(&TempM[2][0],&TempM[0][0], &TempM[1][0]);    // c= a * b eq.20

    Problem = false;
    for ( r = 0; r < (uint8)3; r++ ) {
        Renorm = VDot(&TempM[r][0], &TempM[r][0]);
        if ( (Renorm <  1.5625) && (Renorm > 0.64) )
            Renorm = 0.5 * (3.0 - Renorm);               //eq.21
        else
            if ( (Renorm < 100.0) && (Renorm > 0.01) )
                Renorm = 1.0 / sqrt( Renorm );
            else
                Problem = true;

        VScale(&DCM[r][0], &TempM[r][0], Renorm);
    }

    if ( Problem ) { // Divergent - force to initial conditions and hope!
        DCM[0][0] = 1.0;
        DCM[0][1] = 0.0;
        DCM[0][2] = 0.0;
        DCM[1][0] = 0.0;
        DCM[1][1] = 1.0;
        DCM[1][2] = 0.0;
        DCM[2][0] = 0.0;
        DCM[2][1] = 0.0;
        DCM[2][2] = 1.0;
    }

} // DCMNormalise

void DCMMotionCompensation(void) {
    // compensation for rate of change of velocity LR/BF needs GPS velocity but
    // updates probably too slow?
    Acc[LR ] += 0.0;
    Acc[BF] += 0.0;
} // DCMMotionCompensation

void DCMDriftCorrection(void) {

    static real32 ScaledOmegaI[3];

    //DON'T USE  #define USE_DCM_YAW_COMP
#ifdef USE_DCM_YAW_COMP
    static real32 ScaledOmegaP[3];
    static real32 YawError[3];
    static real32 ErrorCourse;
#endif // USE_DCM_YAW_COMP

    VCross(&RollPitchError[0], &Acc[0], &DCM[2][0]); //adjust the reference ground
    VScale(&OmegaP[0], &RollPitchError[0], Kp_RollPitch);

    VScale(&ScaledOmegaI[0], &RollPitchError[0], Ki_RollPitch);
    VAdd(&OmegaI[0], &OmegaI[0], &ScaledOmegaI[0]);

#ifdef USE_DCM_YAW_COMP
    if ( F.HeadingUpdated ) {
        // Yaw - drift correction based on compass/magnetometer heading
        HeadingCos = cos( Heading );
        HeadingSin = sin( Heading );
        ErrorCourse = ( U[0][0] * HeadingSin ) - ( U[1][0] * HeadingCos );
        VScale(YawError, &U[2][0], ErrorCourse );

        VScale(&ScaledOmegaP[0], &YawError[0], Kp_Yaw );
        VAdd(&OmegaP[0], &OmegaP[0], &ScaledOmegaP[0]);

        VScale(&ScaledOmegaI[0], &YawError[0], Ki_Yaw );
        VAdd(&OmegaI[0], &OmegaI[0], &ScaledOmegaI[0]);
    }
#endif // USE_DCM_YAW_COMP

} // DCMDriftCorrection

void DCMUpdate(void) {

    static uint8 i, j, k;
    static real32 op[3];

    VAdd(&Omega[0], &Gyro[0], &OmegaI[0]);
    VAdd(&OmegaV[0], &Omega[0], &OmegaP[0]);

//   MotionCompensation();

    U[0][0] =  0.0;
    U[0][1] = -dT * OmegaV[2];   //-z
    U[0][2] =  dT * OmegaV[1];   // y
    U[1][0] =  dT * OmegaV[2];   // z
    U[1][1] =  0.0;
    U[1][2] = -dT * OmegaV[0];   //-x
    U[2][0] = -dT * OmegaV[1];   //-y
    U[2][1] =  dT * OmegaV[0];   // x
    U[2][2] =  0.0;

    for ( i = 0; i < (uint8)3; i++ )
        for ( j = 0; j < (uint8)3; j++ ) {
            for ( k = 0; k < (uint8)3; k++ )
                op[k] = DCM[i][k] * U[k][j];

            TempM[i][j] = op[0] + op[1] + op[2];
        }

    for ( i = 0; i < (uint8)3; i++ )
        for (j = 0; j < (uint8)3; j++ )
            DCM[i][j] += TempM[i][j];

} // DCMUpdate

void DCMEulerAngles(void) {

    static uint8 g;

    for ( g = 0; g < (uint8)3; g++ )
        Rate[g] = Gyro[g];

    EstAngle[Pitch][PremerlaniDCM] = asin(DCM[2][0]);
    EstAngle[Roll][PremerlaniDCM]= -atan2(DCM[2][1], DCM[2][2]);
    EstAngle[Yaw][PremerlaniDCM] = atan2(DCM[1][0], DCM[0][0]);

} // DCMEulerAngles

void DoDCM(void) {
    DCMUpdate();
    DCMNormalise();
    DCMDriftCorrection();
    DCMEulerAngles();
} // DoDCM

//___________________________________________________________________________________

// IMU.c
// S.O.H. Madgwick
// 25th September 2010

// Description:

// Quaternion implementation of the 'DCM filter' [Mahony et al.].

// User must define 'dTOn2' as the (sample period / 2), and the filter gains 'MKp' and 'MKi'.

// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.

// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer
// units are irrelevant as the vector is normalised.

#include <math.h>

void IMUupdate(real32 gx, real32 gy, real32 gz, real32 ax, real32 ay, real32 az);

#define MKp 2.0           // proportional gain governs rate of convergence to accelerometer/magnetometer
#define MKi 0.25          // integral gain governs rate of convergence of gyroscope biases // 0.005

real32 q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;  // quaternion elements representing the estimated orientation
real32 exInt = 0.0, eyInt = 0.0, ezInt = 0.0;   // scaled integral error

void DoMadgwickIMU(real32 gx, real32 gy, real32 gz, real32 ax, real32 ay, real32 az) {

    static uint8 g;
    static real32 ReNorm;
    static real32 vx, vy, vz;
    static real32 ex, ey, ez;
    //  static real32 aMag;

//swap z and y?
    if ( F.AccMagnitudeOK ) {

        // estimated direction of gravity
        vx = 2.0*(q1*q3 - q0*q2);
        vy = 2.0*(q0*q1 + q2*q3);
        vz = Sqr(q0) - Sqr(q1) - Sqr(q2) + Sqr(q3);

        // error is sum of cross product between reference direction of field and direction measured by sensor
        ex = (ay*vz - az*vy);
        ey = (az*vx - ax*vz);
        ez = (ax*vy - ay*vx);

        // integral error scaled integral gain
        exInt += ex*MKi*dT; // added dT
        eyInt += ey*MKi*dT;
        ezInt += ez*MKi*dT;

        // adjusted gyroscope measurements
        gx += MKp*ex + exInt;
        gy += MKp*ey + eyInt;
        gz += MKp*ez + ezInt;

        // integrate quaternion rate and normalise
        q0 += (-q1*gx - q2*gy - q3*gz)*dTOn2;
        q1 += (q0*gx + q2*gz - q3*gy)*dTOn2;
        q2 += (q0*gy - q1*gz + q3*gx)*dTOn2;
        q3 += (q0*gz + q1*gy - q2*gx)*dTOn2;

        // normalise quaternion
        ReNorm = 1.0  /sqrt(Sqr(q0) + Sqr(q1) + Sqr(q2) + Sqr(q3));
        q0 *= ReNorm;
        q1 *= ReNorm;
        q2 *= ReNorm;
        q3 *= ReNorm;

        MadgwickEulerAngles(MadgwickIMU);

    } else
        for ( g = 0; g <(uint8)3; g++) {
            EstRate[g][MadgwickIMU] = Gyro[g];
            EstAngle[g][MadgwickIMU] += EstRate[g][MadgwickIMU] * dT;
        }

}  // DoMadgwickIMU

//_________________________________________________________________________________

#define gyroMeasError 3.14159265358979f * (75.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

// AHRS.c
// S.O.H. Madgwick
// 25th August 2010

// Description:

// Quaternion implementation of the 'DCM filter' [Mahoney et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// a only.

// User must define 'dTOn2' as the (sample period / 2), and the filter gains 'MKp' and 'MKi'.

// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.

// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.

void DoMadgwickAHRS(real32 gx, real32 gy, real32 gz, real32 ax, real32 ay, real32 az, real32 mx, real32 my, real32 mz) {

    static uint8 g;
    static real32 ReNorm;
    static real32 hx, hy, hz, bx2, bz2, mx2, my2, mz2;
    static real32 vx, vy, vz, wx, wy, wz;
    static real32 ex, ey, ez;
    static real32 q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    if ( F.AccMagnitudeOK ) {

        // auxiliary variables to reduce number of repeated operations
        q0q0 = q0*q0;
        q0q1 = q0*q1;
        q0q2 = q0*q2;
        q0q3 = q0*q3;
        q1q1 = q1*q1;
        q1q2 = q1*q2;
        q1q3 = q1*q3;
        q2q2 = q2*q2;
        q2q3 = q2*q3;
        q3q3 = q3*q3;

        ReNorm = 1.0/sqrt(Sqr(mx) + Sqr(my) + Sqr(mz));
        mx *= ReNorm;
        my *= ReNorm;
        mz *= ReNorm;
        mx2 = 2.0 * mx;
        my2 = 2.0 * my;
        mz2 = 2.0 * mz;

        // compute reference direction of flux
        hx = mx2*(0.5 - q2q2 - q3q3) + my2*(q1q2 - q0q3) + mz2*(q1q3 + q0q2);
        hy = mx2*(q1q2 + q0q3) + my2*(0.5 - q1q1 - q3q3) + mz2*(q2q3 - q0q1);
        hz = mx2*(q1q3 - q0q2) + my2*(q2q3 + q0q1) + mz2*(0.5 - q1q1 - q2q2);
        bx2 = 2.0*sqrt(Sqr(hx) + Sqr(hy));
        bz2 = 2.0*hz;

        // estimated direction of gravity and flux (v and w)
        vx = 2.0*(q1q3 - q0q2);
        vy = 2.0*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        wx = bx2*(0.5 - q2q2 - q3q3) + bz2*(q1q3 - q0q2);
        wy = bx2*(q1q2 - q0q3) + bz2*(q0q1 + q2q3);
        wz = bx2*(q0q2 + q1q3) + bz2*(0.5 - q1q1 - q2q2);

        // error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        // integral error scaled integral gain
        exInt += ex*MKi*dT; // dT
        eyInt += ey*MKi*dT;
        ezInt += ez*MKi*dT;

        // adjusted gyroscope measurements
        gx += MKp*ex + exInt;
        gy += MKp*ey + eyInt;
        gz += MKp*ez + ezInt;

        // integrate quaternion rate and normalise
        q0 += (-q1*gx - q2*gy - q3*gz)*dTOn2;
        q1 += (q0*gx + q2*gz - q3*gy)*dTOn2;
        q2 += (q0*gy - q1*gz + q3*gx)*dTOn2;
        q3 += (q0*gz + q1*gy - q2*gx)*dTOn2;

        // normalise quaternion
        ReNorm = 1.0 / sqrt(Sqr(q0) + Sqr(q1) + Sqr(q2) + Sqr(q3));
        q0 *= ReNorm;
        q1 *= ReNorm;
        q2 *= ReNorm;
        q3 *= ReNorm;

        MadgwickEulerAngles(MadgwickAHRS);

    } else
        for ( g = 0; g <(uint8)3; g++) {
            EstRate[g][MadgwickAHRS] = Gyro[g];
            EstAngle[g][MadgwickAHRS] += EstRate[g][MadgwickAHRS] * dT;
        }

} // DoMadgwickAHRS

void  MadgwickEulerAngles(uint8 S) {

    EstAngle[Roll][S] = atan2(2.0*q2*q3 - 2.0*q0*q1 , 2.0*Sqr(q0) + 2.0*Sqr(q3) - 1.0);
    EstAngle[Pitch][S] = asin(2.0*q1*q2 - 2.0*q0*q2);
    // EstAngle[Yaw][S] = -atan2(2.0*q1*q2 - 2.0*q0*q3 ,  2.0*Sqr(q0) + 2.0*Sqr(q1) - 1.0);
} // MadgwickEulerAngles

//_________________________________________________________________________________

// Complementary Filter originally authored by RoyLB
// http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286

const real32 TauCF = 1.1;

real32 AngleCF[2] = {0,0};
real32 F0[2] = {0,0};
real32 F1[2] = {0,0};
real32 F2[2] = {0,0};

real32 CF(uint8 a, real32 NewAngle, real32 NewRate) {

    if ( F.AccMagnitudeOK ) {
        F0[a] = (NewAngle - AngleCF[a]) * Sqr(TauCF);
        F2[a] += F0[a] * dT;
        F1[a] = F2[a] + (NewAngle - AngleCF[a]) * 2.0 * TauCF + NewRate;
        AngleCF[a] = (F1[a] * dT) + AngleCF[a];
    } else
        AngleCF[a] += NewRate * dT;

    return ( AngleCF[a] ); // This is actually the current angle, but is stored for the next iteration
} // CF

void DoCF(void) { // NO YAW ANGLE ESTIMATE

    EstAngle[Roll][Complementary] = CF(Roll, asin(-Acc[Roll]), Gyro[Roll]);
    EstAngle[Pitch][Complementary] = CF(Pitch, asin(-Acc[Pitch]), Gyro[Pitch]);
    EstRate[Roll][Complementary] = Gyro[Roll];
    EstRate[Pitch][Complementary] = Gyro[Pitch];

} // DoCF

//_________________________________________________________________________________

// Kalman Filter originally authored by Tom Pycke
// http://tom.pycke.be/mav/71/kalman-filtering-of-imu-data

real32 AngleKF[2] = {0,0};
real32 BiasKF[2] = {0,0};
real32 P00[2] = {0,0};
real32 P01[2] = {0,0};
real32 P10[2] = {0,0};
real32 P11[2] = {0,0};

real32 KalmanFilter(uint8 a, real32 NewAngle, real32 NewRate) {

    // Q is a 2x2 matrix that represents the process covariance noise.
    // In this case, it indicates how much we trust the accelerometer
    // relative to the gyros.
    const real32 AngleQ = 0.001;
    const real32 GyroQ = 0.003;

    // R represents the measurement covariance noise. In this case,
    // it is a 1x1 matrix that says that we expect 0.3 rad jitter
    // from the accelerometer.
    const real32 AngleR = 0.03; // Radian noise

    static real32 y, S;
    static real32 K0, K1;

    AngleKF[a] += (NewRate - BiasKF[a]) * dT;
    P00[a] -=  (( P10[a] + P01[a] ) + AngleQ ) * dT;
    P01[a] -=  P11[a] * dT;
    P10[a] -=  P11[a] * dT;
    P11[a] +=  GyroQ * dT;

    y = NewAngle - AngleKF[a];
    S = 1.0 / ( P00[a] + AngleR );
    K0 = P00[a] * S;
    K1 = P10[a] * S;

    AngleKF[a] +=  K0 * y;
    BiasKF[a]  +=  K1 * y;
    P00[a] -= K0 * P00[a];
    P01[a] -= K0 * P01[a];
    P10[a] -= K1 * P00[a];
    P11[a] -= K1 * P01[a];

    return ( AngleKF[a] );

}  // KalmanFilter

void DoKalman(void) { // NO YAW ANGLE ESTIMATE
    EstAngle[Roll][Kalman] = KalmanFilter(Roll, asin(-Acc[LR]) * RADDEG, Gyro[Roll] * RADDEG) * DEGRAD;
    EstAngle[Pitch][Kalman]  = KalmanFilter(Pitch, asin(Acc[BF]) * RADDEG, Gyro[Pitch] * RADDEG) * DEGRAD;
} // DoKalman


//_________________________________________________________________________________

// MultWii
// Original code by Alex at: http://radio-commande.com/international/triwiicopter-design/

// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// with this algorithm, we can get absolute angles for a stable mode integration

void DoMultiWii(void) { // V1.6  NO YAW ANGLE ESTIMATE

    const real32 acc_25deg = sin(25.0 * PI / 180.0);

    static real32 RxEst = 0.0;      // init acc in stable mode
    static real32 RyEst = 0.0;
    static real32 RzEst = 1.0;
    static real32 Axz,Ayz;        //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
    real32 RxGyro, RyGyro, RzGyro;  //R obtained from last estimated value and gyro movement
    real32 wGyro = 100.0;         // gyro weight/smoothing factor
    static real32 invW = 1.0 / (1.0 + 100.0);
    real32 a,b;
    static boolean small_angle = true;

// real32 magX;
// real32 magY;
// real32 cos_roll = 1.0;
// real32 sin_roll = 0.0;
// real32 cos_pitch = 1.0;
// real32 sin_pitch = 0.0;

    if ( Acc[Yaw] > 0.0 ) { //we want to be sure we are not flying inverted

        a = Gyro[Roll] * dT;
        b = Gyro[Pitch] * dT;

        // a very nice trigonometric approximation:
        // under 25deg, the error of this approximation is less than 1 deg:
        //   sin(x) =~= x =~= arcsin(x)
        //   angle_axis = arcsin(ACC_axis/ACC_1G) =~= ACC_axis/ACC_1G
        // the angle calculation is much faster in this case

        if ( (fabs(Acc[LR]) > acc_25deg) && (fabs(Acc[BF]) > acc_25deg) ) {
            Axz += a;
            Ayz += b;
            Axz = ( -Acc[LR] + Axz * wGyro ) * invW; // =~= sin_roll
            Ayz = ( Acc[BF] + Ayz * wGyro ) * invW; // =~= sin_pitch

            small_angle = true;
        } else {

            if ( !F.AccMagnitudeOK ) {
                Axz +=  a;
                Ayz +=  b;
            } else if ( !small_angle ) {
                Axz = atan2(RxEst,RzEst) + a;  // convert ADC value for to physical units
                Ayz = atan2(RyEst,RzEst) + b;  // and get updated angle according to gyro movement
            }

            //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
            RxGyro = sin(Axz) / sqrt( 1.0 + Sqr(cos(Axz)) * Sqr(tan(Ayz)) );
            RyGyro = sin(Ayz) / sqrt( 1.0 + Sqr(cos(Ayz)) * Sqr(tan(Axz)) );
            RzGyro = sqrt(abs(1 - Sqr(RxGyro) - Sqr(RyGyro)));

            //combine Accelerometer and gyro readings
            RxEst = ( Acc[Roll] + wGyro* RxGyro ) * invW;
            RyEst = ( Acc[Pitch] + wGyro* RyGyro ) * invW;
            RzEst = ( Acc[Yaw] + wGyro* RzGyro ) * invW;

            small_angle = false;
        }
    }

    EstAngle[Roll][MultiWii] =  Axz;
    EstAngle[Pitch][MultiWii] =  Ayz;

} // DoMultiWii

//_________________________________________________________________________________

void AttitudeTest(void) {

    TxString("\r\nAttitude Test\r\n");

    GetAttitude();

    TxString("\r\ndT \t");
    TxVal32(dT * 1000.0, 3, 0);
    TxString(" Sec.\r\n\r\n");

    if ( GyroType == IRSensors ) {

        TxString("IR Sensors:\r\n");
        TxString("\tRoll \t");
        TxVal32(IR[Roll] * 100.0, 2, HT);
        TxNextLine();
        TxString("\tPitch\t");
        TxVal32(IR[Pitch] * 100.0, 2, HT);
        TxNextLine();
        TxString("\tZ    \t");
        TxVal32(IR[Yaw] * 100.0, 2, HT);
        TxNextLine();
        TxString("\tMin/Max\t");
        TxVal32(IRMin * 100.0, 2, HT);
        TxVal32(IRMax * 100.0, 2, HT);
        TxString("\tSwing\t");
        TxVal32(IRSwing * 100.0, 2, HT);
        TxNextLine();

    } else {

        TxString("Gyro, Compensated, Max Delta(Deg./Sec.):\r\n");
        TxString("\tRoll \t");
        TxVal32(Gyro[Roll] * MILLIANGLE, 3, HT);
        TxVal32(Rate[Roll] * MILLIANGLE, 3, HT);
        TxVal32(GyroNoise[Roll] * MILLIANGLE,3, 0);
        TxNextLine();
        TxString("\tPitch\t");
        TxVal32(Gyro[Pitch] * MILLIANGLE, 3, HT);
        TxVal32(Rate[Pitch] * MILLIANGLE, 3, HT);
        TxVal32(GyroNoise[Pitch] * MILLIANGLE,3, 0);
        TxNextLine();
        TxString("\tYaw  \t");
        TxVal32(Gyro[Yaw] * MILLIANGLE, 3, HT);
        TxVal32(Rate[Yaw] * MILLIANGLE, 3, HT);
        TxVal32(GyroNoise[Yaw] * MILLIANGLE, 3, 0);
        TxNextLine();

        TxString("Accelerations , peak change(G):\r\n");
        TxString("\tB->F \t");
        TxVal32(Acc[BF] * 1000.0, 3, HT);
        TxVal32( AccNoise[BF] * 1000.0, 3, 0);
        TxNextLine();
        TxString("\tL->R \t");
        TxVal32(Acc[LR] * 1000.0, 3, HT);
        TxVal32( AccNoise[LR] * 1000.0, 3, 0);
        TxNextLine();
        TxString("\tU->D \t");
        TxVal32(Acc[UD] * 1000.0, 3, HT);
        TxVal32( AccNoise[UD] * 1000.0, 3, 0);
        TxNextLine();
    }

    if ( CompassType != HMC6352 ) {
        TxString("Magnetic:\r\n");
        TxString("\tX    \t");
        TxVal32(Mag[Roll].V, 0, 0);
        TxNextLine();
        TxString("\tY    \t");
        TxVal32(Mag[Pitch].V, 0, 0);
        TxNextLine();
        TxString("\tZ    \t");
        TxVal32(Mag[Yaw].V, 0, 0);
        TxNextLine();
    }

    TxString("Heading: \t");
    TxVal32(Make2Pi(Heading) * MILLIANGLE, 3, 0);
    TxNextLine();

} // AttitudeTest


