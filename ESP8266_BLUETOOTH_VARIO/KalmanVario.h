#ifndef KALMAN_VARIO_H_
#define KALMAN_VARIO_H_

class KalmanVario {
public :
	KalmanVario(){};
	void Config(float zMeasVariance, float zAccelVariance, float zAccelBiasVariance, float zInitial, float vInitial);
	void Predict(float a, float dt);
	void Update(float z, float* pZ, float* pV);


private :
	// State being tracked
	float z_;  // position
	float v_;  // velocity
	float aBias_;  // acceleration bias

	// 3x3 State Covariance matrix
	float Pzz_;
	float Pzv_;
	float Pza_;
	
	float Pvz_;
	float Pvv_;
	float Pva_;
	
	float Paz_;
	float Pav_;
	float Paa_;
	
	// configuration parameters
	float zAccelBiasVariance_; // can be set low as the acceleration bias will not drift much
	float zAccelVariance_;  // environmental acceleration variance, depends on conditions
	float zMeasVariance_; //  z measurement noise variance, calculate this offline for the z sensor

};

#endif

