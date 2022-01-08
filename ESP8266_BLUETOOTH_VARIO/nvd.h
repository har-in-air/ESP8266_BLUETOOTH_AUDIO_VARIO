#ifndef NVD_H_
#define NVD_H_

#include "config.h"

#if (CFG_BLUETOOTH == true)    
    #define NVD_SIZE_BYTES 30
#else
    #define NVD_SIZE_BYTES 28
#endif

typedef struct CALIB_PARAMS_ {
	int16_t  axBias;
	int16_t  ayBias;
	int16_t  azBias;
	int16_t  gxBias;
	int16_t  gyBias;
	int16_t  gzBias;
	} CALIB_PARAMS;	

typedef struct VARIO_PARAMS_ {
	int16_t  climbThresholdCps;
	int16_t  zeroThresholdCps;
	int16_t  sinkThresholdCps;
	int16_t  crossoverCps;
	} VARIO_PARAMS;

typedef struct KALMAN_FILTER_PARAMS_ {
	int16_t  accelVariance; // environmental acceleration disturbance variance, divided by 1000
	int16_t  zMeasVariance; // z measurement noise variance
	} KALMAN_FILTER_PARAMS;

typedef struct MISC_PARAMS_ {
	int16_t  sleepTimeoutMinutes;
#if (CFG_BLUETOOTH == true)    
	int16_t  bluetoothEnable;
#endif
	} MISC_PARAMS;

typedef struct CONFIG_PARAMS_ {
	VARIO_PARAMS vario;
	KALMAN_FILTER_PARAMS kf;
	MISC_PARAMS misc;
	} CONFIG_PARAMS;

typedef struct NVD_PARAMS_ {
	CALIB_PARAMS  calib;
	CONFIG_PARAMS cfg;
	uint16_t checkSum;
	} NVD_PARAMS;

typedef union NVD_ {
	NVD_PARAMS par;
	uint8_t	   buf[NVD_SIZE_BYTES];
	} NVD;

extern NVD Nvd;

void nvd_init(void);
void nvd_set_defaults();
void nvd_save_calib_params(CALIB_PARAMS &calib);
void nvd_save_config_params(CONFIG_PARAMS &cfg);


#endif
