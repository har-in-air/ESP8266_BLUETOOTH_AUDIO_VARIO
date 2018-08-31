#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "nvd.h"
#include "util.h"

// saves and retrieves non-volatile data (NVD) to / from ESP8266 flash. 

NVD nvd;


void nvd_Init(void)   {
	EEPROM.begin(NVD_SIZE_BYTES);
	for (int inx = 0; inx < NVD_SIZE_BYTES; inx++) {
		nvd.buf[inx] = EEPROM.read(inx);
		}
	uint16_t checkSum = nvd_CheckSum();	
#ifdef NVD_DEBUG	
  Serial.printf("Sizeof(NVD) = %d bytes\r\n", sizeof(NVD));
	Serial.printf("Calculated checkSum = 0x%04x\r\n", checkSum);	
	Serial.printf("Saved checkSum = ~0x%04x\r\n", ~nvd.params.checkSum&0xFFFF);
#endif
	
  if ((checkSum ^ nvd.params.checkSum) == 0xFFFF) {
#ifdef NVD_DEBUG	
		Serial.printf("NVD checkSum OK\r\n\r\n");
		Serial.println("ACCEL & GYRO CALIBRATION");
		Serial.printf("axBias = %d\r\n", nvd.params.calib.axBias);
		Serial.printf("ayBias = %d\r\n", nvd.params.calib.ayBias);
		Serial.printf("azBias = %d\r\n", nvd.params.calib.azBias);
		Serial.printf("gxBias = %d\r\n", nvd.params.calib.gxBias);
		Serial.printf("gyBias = %d\r\n", nvd.params.calib.gyBias);
		Serial.printf("gzBias = %d\r\n", nvd.params.calib.gzBias);
    Serial.println("VARIO CONFIGURATION");
    Serial.printf("climbThresholdCps = %d\r\n", nvd.params.vario.climbThresholdCps);
    Serial.printf("zeroThresholdCps = %d\r\n", nvd.params.vario.zeroThresholdCps);
    Serial.printf("sinkThresholdCps = %d\r\n", nvd.params.vario.sinkThresholdCps);
    Serial.printf("crossoverCps = %d\r\n", nvd.params.vario.crossoverCps);
    
    Serial.println("KALMAN FILTER CONFIGURATION");
    Serial.printf("accelVariance = %d\r\n", nvd.params.kf.accelVariance);
    Serial.printf("zMeasVariance = %d\r\n", nvd.params.kf.zMeasVariance);
    
    Serial.println("ALARM CONFIGURATION");
    Serial.printf("accelThresholdmG = %d\r\n", nvd.params.alarm.accelThresholdmG);
    Serial.printf("gyroThresholddps = %d\r\n", nvd.params.alarm.gyroThresholdDps);
    Serial.printf("durationSecs = %d\r\n", nvd.params.alarm.durationSecs);
    
    Serial.println("MISCELLANEOUS CONFIGURATION");
    Serial.printf("gyroOffsetLimit1000DPS = %d\r\n", nvd.params.misc.gyroOffsetLimit1000DPS);
    Serial.printf("sleepTimeoutMinutes = %d\r\n", nvd.params.misc.sleepTimeoutMinutes);
    Serial.printf("bluetoothRateHz = %d\r\n", nvd.params.misc.bluetoothRateHz);
    Serial.printf("appMode = %d\r\n", nvd.params.misc.appMode);
#endif
		}
   else  {
#ifdef NVD_DEBUG	
		Serial.println("ERROR!! NVD BAD CHECKSUM, SETTING DEFAULTS");
#endif
		nvd.params.calib.axBias = 0;
		nvd.params.calib.ayBias = 0;
		nvd.params.calib.azBias = 0;
		nvd.params.calib.gxBias = 0;
		nvd.params.calib.gyBias = 0;
		nvd.params.calib.gzBias = 0;

		nvd.params.vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
		nvd.params.vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
		nvd.params.vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT;
		nvd.params.vario.crossoverCps = VARIO_CROSSOVER_CPS_DEFAULT;

		nvd.params.alarm.accelThresholdmG = MOTION_ALARM_ACCEL_THRESHOLD_DEFAULT;
		nvd.params.alarm.gyroThresholdDps = MOTION_ALARM_GYRO_THRESHOLD_DEFAULT;
		nvd.params.alarm.durationSecs = MOTION_ALARM_DURATION_SECS_DEFAULT;
		
		nvd.params.kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
		nvd.params.kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

		nvd.params.misc.gyroOffsetLimit1000DPS = GYRO_OFFSET_LIMIT_1000DPS_DEFAULT;
		nvd.params.misc.sleepTimeoutMinutes = SLEEP_TIMEOUT_MINUTES_DEFAULT;
    nvd.params.misc.bluetoothRateHz = BLUETOOTH_RATE_DEFAULT;
    nvd.params.misc.appMode = APP_MODE_DEFAULT;

    nvd_Commit();
		}
   EEPROM.end();
   }

   
uint16_t nvd_CheckSum(void) {
	uint16_t checkSum = 0;
	for (int inx = 0; inx < NVD_SIZE_BYTES-2; inx++) {
		checkSum += (uint16_t)nvd.buf[inx]; 
		}
	return checkSum;
	}
	
void nvd_Commit(void) {
	uint16_t checkSum = nvd_CheckSum();
	nvd.params.checkSum = ~checkSum;
	for  (int inx = 0; inx < NVD_SIZE_BYTES; inx++){
		EEPROM.write(inx, nvd.buf[inx]);
		}
	EEPROM.commit();
	}
	

void nvd_SaveCalibrationParams(int16_t axb, int16_t ayb, int16_t azb, int16_t gxb, int16_t gyb, int16_t gzb) {
	EEPROM.begin(NVD_SIZE_BYTES);
	nvd.params.calib.axBias = axb;
	nvd.params.calib.ayBias = ayb;
	nvd.params.calib.azBias = azb;
	nvd.params.calib.gxBias = gxb;
	nvd.params.calib.gyBias = gyb;
	nvd.params.calib.gzBias = gzb;
	nvd_Commit();
	EEPROM.end();
	}	
	
void nvd_SaveConfigurationParams(VARIO_PARAMS* pVario, ALARM_PARAMS* pAlarm, 
								KALMAN_FILTER_PARAMS* pKF, MISC_PARAMS* pMisc ) {
	EEPROM.begin(NVD_SIZE_BYTES);
	memcpy(&nvd.params.vario, pVario, sizeof(VARIO_PARAMS));
	memcpy(&nvd.params.alarm, pAlarm, sizeof(ALARM_PARAMS));
	memcpy(&nvd.params.kf, pKF, sizeof(KALMAN_FILTER_PARAMS));
	memcpy(&nvd.params.misc, pMisc, sizeof(MISC_PARAMS));
	nvd_Commit();
	EEPROM.end();
	}	
	


