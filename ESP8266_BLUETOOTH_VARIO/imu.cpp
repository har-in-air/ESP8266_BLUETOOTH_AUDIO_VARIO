#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "imu.h"
#include "util.h"

//#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKpDef (2.0f * 5.0f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f)	// 2 * integral gain

volatile float twoKp = twoKpDef; // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef; // 2 * integral gain (Ki)
// quaternion of sensor frame relative to auxiliary frame
volatile float Q0 = 1.0f;
volatile float Q1 = 0.0f;
volatile float Q2 = 0.0f;
volatile float Q3 = 0.0f;					
volatile float integralFBx = 0.0f;
volatile float integralFBy = 0.0f;
volatile float integralFBz = 0.0f;	// integral error terms scaled by Ki


void imu_mahonyAHRS_update9DOF(int bUseAccel, int bUseMag, float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float invNorm;
  	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use 6dof  algorithm if magnetometer measurement invalid
	if(!bUseMag) {
		imu_mahonyAHRS_update6DOF(bUseAccel,dt, gx, gy, gz, ax, ay, az);
		return;
		}

	if(bUseAccel) {
		// Normalise accelerometer measurement
		invNorm = 1.0f/sqrt(ax * ax + ay * ay + az * az);
		ax *= invNorm;
		ay *= invNorm;
		az *= invNorm;     

		// Normalise magnetometer measurement
		invNorm = 1.0f/sqrt(mx * mx + my * my + mz * mz);
		mx *= invNorm;
		my *= invNorm;
		mz *= invNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = Q0 * Q0;
        q0q1 = Q0 * Q1;
        q0q2 = Q0 * Q2;
        q0q3 = Q0 * Q3;
        q1q1 = Q1 * Q1;
        q1q2 = Q1 * Q2;
        q1q3 = Q1 * Q3;
        q2q2 = Q2 * Q2;
        q2q3 = Q2 * Q3;
        q3q3 = Q3 * Q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
			}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
			}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
		}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = Q0;
	qb = Q1;
	qc = Q2;
	Q0 += (-qb * gx - qc * gy - Q3 * gz);
	Q1 += (qa * gx + qc * gz - Q3 * gy);
	Q2 += (qa * gy - qb * gz + Q3 * gx);
	Q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	invNorm = 1.0f/sqrt(Q0 * Q0 + Q1 * Q1 + Q2 * Q2 + Q3 * Q3);
	Q0 *= invNorm;
	Q1 *= invNorm;
	Q2 *= invNorm;
	Q3 *= invNorm;
	}


void imu_mahonyAHRS_update6DOF(int bUseAccel, float dt, float gx, float gy, float gz, float ax, float ay, float az) {
	float invNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid )
	if(bUseAccel) {
		// Normalise accelerometer measurement
		invNorm = 1.0f/sqrt(ax * ax + ay * ay + az * az);
		ax *= invNorm;
		ay *= invNorm;
		az *= invNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = Q1 * Q3 - Q0 * Q2;
		halfvy = Q0 * Q1 + Q2 * Q3;
		halfvz = Q0 * Q0 - 0.5f + Q3 * Q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
			}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
			}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
		}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * dt);		// pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = Q0;
	qb = Q1;
	qc = Q2;
	Q0 += (-qb * gx - qc * gy - Q3 * gz);
	Q1 += (qa * gx + qc * gz - Q3 * gy);
	Q2 += (qa * gy - qb * gz + Q3 * gx);
	Q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	invNorm = 1.0f/sqrt(Q0 * Q0 + Q1 * Q1 + Q2 * Q2 + Q3 * Q3);
	Q0 *= invNorm;
	Q1 *= invNorm;
	Q2 *= invNorm;
	Q3 *= invNorm;
	}

// HN
void imu_quaternion_to_yaw_pitch_roll(float q0, float q1, float q2, float q3, float* pYawDeg, float* pPitchDeg, float* pRollDeg) {
    float invNorm = 1.0f/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= invNorm;
    q1 *= invNorm;
    q2 *= invNorm;
    q3 *= invNorm;

    *pYawDeg   = _180_DIV_PI * atan2(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
    *pPitchDeg = _180_DIV_PI * -asin(2.0f * (q1*q3 - q0*q2));
    *pRollDeg  = _180_DIV_PI * atan2(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);
    }

float imu_gravity_compensated_accel(float ax, float ay, float az, float q0, float q1, float q2, float q3) {
    float acc = 2.0*(q1*q3 - q0*q2)*ax + 2.0f*(q0*q1 + q2*q3)*ay + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*az - 1000.0f;
    acc *= 0.98f; // in cm/s/s, assuming ax, ay, az are in milli-Gs
	  return acc;
    }

