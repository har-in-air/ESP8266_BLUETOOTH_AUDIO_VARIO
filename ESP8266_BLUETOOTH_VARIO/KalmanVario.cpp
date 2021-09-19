#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "util.h"
#include "KalmanVario.h"



// Tracks the position z and velocity v of an object moving in a straight line,
// (here assumed to be vertical) that is perturbed by random accelerations.
// sensor measurement of z is assumed to have constant measurement noise 
// variance zMeasVariance.
// This can be calculated offline for the specific sensor.
// zInitial can be determined by averaging a few samples of the altitude measurement.
// zAccelBiasVariance can be set low as it is not expected to drift much
// vInitial can be set to 0 if not known


void KalmanVario::Config(float zMeasVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial) {
#ifdef KF_DEBUG
	Serial.printf("zMeasVariance %d\r\nzAccelVariance %d\r\n", (int)zMeasVariance, (int)zAccelVariance);
#endif	
	zAccelVariance_ = zAccelVariance;
    zAccelBiasVariance_ = zAccelBiasVariance;
	zMeasVariance_ = zMeasVariance;

	z_ = zInitial;
	v_ = vInitial;
	aBias_ = 0.0f; // initial estimate for bias is 0
	
	Pzz_ = 400.0f;
	Pzv_ = 0.0f;
	Pza_ = 0.0f;
	
	Pvz_ = Pzv_;
	Pvv_ = 400.0f;
	Pva_ = 0.0f;
	
	Paz_ = Pza_;
	Pav_ = Pva_;
	Paa_ = 400.0f;
	}

// gravity-compensated earth-z accel in cm/s/s,  and elapsed time dt in second
void KalmanVario::Predict(float a, float dt) {
	// Predicted (a priori) state vector estimate x_k- = F * x_k-1+
	float accel_true = a - aBias_; // true acceleration = measured acceleration minus acceleration sensor bias
	z_ = z_ + (v_ * dt);
	v_ = v_ + (accel_true * dt);
	
    // Predict State Covariance matrix
	float t00,t01,t02;
    float t10,t11,t12;
    float t20,t21,t22;
	
    float dt2div2 = dt*dt*0.5f;
    float dt3div2 = dt2div2*dt;
    float dt4div4 = dt2div2*dt2div2;
	
	t00 = Pzz_ + dt*Pvz_ - dt2div2*Paz_;
	t01 = Pzv_ + dt*Pvv_ - dt2div2*Pav_;
	t02 = Pza_ + dt*Pva_ - dt2div2*Paa_;

	t10 = Pvz_ - dt*Paz_;
	t11 = Pvv_ - dt*Pav_;
	t12 = Pva_ - dt*Paa_;

	t20 = Paz_;
	t21 = Pav_;
	t22 = Paa_;
	
	Pzz_ = t00 + dt*t01 - dt2div2*t02;
	Pzv_ = t01 - dt*t02;
	Pza_ = t02;
	
	Pvz_ = Pzv_;
	Pvv_ = t11 - dt*t12;
	Pva_ = t12;
	
	Paz_ = Pza_;
	Pav_ = Pva_;
	Paa_ = t22;
	
	// add Q_k

    Pzz_ += dt4div4*zAccelVariance_;
    Pzv_ += dt3div2*zAccelVariance_;

    Pvz_ += dt3div2*zAccelVariance_;
    Pvv_ += dt*dt*zAccelVariance_;

    Paa_ += zAccelBiasVariance_;
	}

// Updates state [Z, V] given a sensor measurement of z, acceleration a, 
// and the time in seconds dt since the last measurement. 

void KalmanVario::Update(float z, float dt, float* pZ, float* pV) {
	// Error
	float innov = z - z_; 
	float sInv = 1.0f / (Pzz_ + zMeasVariance_);  

    // Kalman gains
	float kz = Pzz_ * sInv;  
	float kv = Pvz_ * sInv;
	float ka = Paz_ * sInv;

	// Update state 
	z_ += kz * innov;
	v_ += kv * innov;
	aBias_ += ka * innov;	

	// Update state covariance matrix
	Paz_ -= ka * Pzz_;
	Pav_ -= ka * Pzv_;
	Paa_ -= ka * Pza_;
	
	Pvz_ -= kv * Pzz_;
	Pvv_ -= kv * Pzv_;
	Pva_ -= kv * Pza_;
	
	Pzz_ -= kz * Pzz_;
	Pzv_ -= kz * Pzv_;
	Pza_ -= kz * Pza_;

	*pZ = z_;
	*pV = v_;
	}

