#ifndef NVD_H_
#define NVD_H_

#include "config.h"

#if (CFG_BLUETOOTH == true)    
    #define NVD_SIZE_BYTES 90
#else
    #define NVD_SIZE_BYTES 88
#endif

typedef struct {
	char ssid[30];
	char password[30];
	} WIFI_CRED_t;

typedef struct {
	int16_t  axBias;
	int16_t  ayBias;
	int16_t  azBias;
	int16_t  gxBias;
	int16_t  gyBias;
	int16_t  gzBias;
	} CALIB_PARAMS_t;	

typedef struct {
	int16_t  climbThresholdCps;
	int16_t  zeroThresholdCps;
	int16_t  sinkThresholdCps;
	int16_t  crossoverCps;
	} VARIO_PARAMS_t;

typedef struct  {
	int16_t  accelVariance; // environmental acceleration disturbance variance, divided by 1000
	int16_t  zMeasVariance; // z measurement noise variance
	} KALMAN_FILTER_PARAMS_t;

typedef struct  {
	int16_t  sleepTimeoutMinutes;
#if (CFG_BLUETOOTH == true)    
	int16_t  bluetoothEnable;
#endif
	} MISC_PARAMS_t;

typedef struct {
	WIFI_CRED_t cred;
	VARIO_PARAMS_t vario;
	KALMAN_FILTER_PARAMS_t kf;
	MISC_PARAMS_t misc;
	} CONFIG_PARAMS_t;

typedef struct {
	CALIB_PARAMS_t  calib;
	CONFIG_PARAMS_t cfg;
	uint16_t checkSum;
	} NVD_PARAMS_t;

typedef union  {
	NVD_PARAMS_t par;
	uint8_t	   buf[NVD_SIZE_BYTES];
	} NVD_t;

extern NVD_t Nvd;

void nvd_init(void);
void nvd_set_defaults();
void nvd_save_calib_params(CALIB_PARAMS_t &calib);
void nvd_save_config_params(CONFIG_PARAMS_t &cfg);


#endif
