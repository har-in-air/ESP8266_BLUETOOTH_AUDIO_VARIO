#ifndef NVD_H_
#define NVD_H_

#define NVD_SIZE_BYTES	40


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

typedef struct ALARM_PARAMS_ {
	int16_t  accelThresholdmG;
	int16_t  gyroThresholdDps;
	int16_t  durationSecs;
} ALARM_PARAMS;

typedef struct KALMAN_FILTER_PARAMS_ {
	int16_t  accelVariance; // environmental acceleration disturbance variance, divided by 1000
	int16_t  zMeasVariance; // z measurement noise variance
} KALMAN_FILTER_PARAMS;

typedef struct MISC_PARAMS_ {
	int16_t  sleepTimeoutMinutes;
	int16_t  gyroOffsetLimit1000DPS;
  int16_t  bluetoothRateHz;
  int16_t  appMode;
} MISC_PARAMS;

typedef struct NVD_PARAMS_ {
	CALIB_PARAMS calib;
	VARIO_PARAMS vario;
	ALARM_PARAMS alarm;
	KALMAN_FILTER_PARAMS kf;
	MISC_PARAMS misc;
	uint16_t checkSum;
} NVD_PARAMS;

typedef union NVD_ {
	NVD_PARAMS params;
	uint8_t	   buf[NVD_SIZE_BYTES];
} NVD;

extern NVD nvd;


void nvd_Init(void);
void nvd_Commit(void);
uint16_t nvd_CheckSum(void);
void nvd_SaveCalibrationParams(int16_t axb, int16_t ayb, int16_t azb, int16_t gxb, int16_t gyb, int16_t gzb);
void nvd_SaveConfigurationParams(VARIO_PARAMS* pVario, ALARM_PARAMS* pAlarm, 
	KALMAN_FILTER_PARAMS* pKF, MISC_PARAMS* pMisc );


#endif

