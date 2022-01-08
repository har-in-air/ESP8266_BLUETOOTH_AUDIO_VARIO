#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "nvd.h"
#include "util.h"

// saves and retrieves non-volatile data (NVD) to / from ESP8266 flash. 

NVD Nvd;

// crc16  :  https://www.embeddedrelated.com/showcode/295.php
#define CRC16_DNP	0x3D65		// DNP, IEC 870, M-BUS, wM-BUS, ...
#define CRC16_CCITT	0x1021		// X.25, V.41, HDLC FCS, Bluetooth, ...
#define POLYNOM		CRC16_DNP

static uint16_t crc16(uint16_t crcValue, uint8_t newByte);
static uint16_t nvd_checksum(void);
static void nvd_commit(void);

static uint16_t crc16(uint16_t crcValue, uint8_t newByte) {
	for (int bitIndex = 0; bitIndex < 8; bitIndex++) {
		if (((crcValue & 0x8000) >> 8) ^ (newByte & 0x80)){
			crcValue = (crcValue << 1)  ^ POLYNOM;
			}
		else{
			crcValue = (crcValue << 1);
			}
		newByte <<= 1;
		}
	return crcValue;
	}


static uint16_t nvd_checksum(void){
	//uint16_t crc = 0xFFFF; // CCITT
	uint16_t crc = 0x0000; // DNP
	int nBytes = sizeof(NVD_PARAMS)-2;
	for (int inx = 0; inx < nBytes; inx++) {
		crc = crc16(crc, Nvd.buf[inx]); 
		}
	//return crc; // CCITT
	return (~crc); // DNP
	}


void nvd_init(void)   {
	EEPROM.begin(NVD_SIZE_BYTES);
	for (int inx = 0; inx < NVD_SIZE_BYTES; inx++) {
		Nvd.buf[inx] = EEPROM.read(inx);
		}
	uint16_t checkSum = nvd_checksum();	
#ifdef NVD_DEBUG	
    dbg_printf(("Sizeof(NVD_PARAMS) = %d bytes\r\n", sizeof(NVD_PARAMS)));
	dbg_printf(("Calculated checkSum = 0x%04x\r\n", checkSum));	
	dbg_printf(("Saved checkSum = ~0x%04x\r\n", ~Nvd.par.checkSum&0xFFFF));
#endif
	bool badCalibData = ((Nvd.par.calib.axBias == -1) && (Nvd.par.calib.ayBias == -1) && (Nvd.par.calib.azBias == -1)) ? true : false; 
#if (CFG_BLUETOOTH == true)    
    bool badBluetoothData = ((Nvd.par.cfg.misc.bluetoothEnable == 0) || (Nvd.par.cfg.misc.bluetoothEnable == 1)) ? false : true;
    bool badData = (badCalibData || badBluetoothData);
#else
    bool badData = badCalibData;
#endif

  if ((badData == false) && (checkSum ^ Nvd.par.checkSum) == 0xFFFF) {
#ifdef NVD_DEBUG	
	dbg_println(("NVD checkSum OK\r\n"));
	dbg_println(("ACCEL & GYRO Calibration Values"));
	dbg_printf(("axBias = %d\r\n", Nvd.par.calib.axBias));
	dbg_printf(("ayBias = %d\r\n", Nvd.par.calib.ayBias));
	dbg_printf(("azBias = %d\r\n", Nvd.par.calib.azBias));
	dbg_printf(("gxBias = %d\r\n", Nvd.par.calib.gxBias));
	dbg_printf(("gyBias = %d\r\n", Nvd.par.calib.gyBias));
	dbg_printf(("gzBias = %d\r\n", Nvd.par.calib.gzBias));
    
    dbg_println(("VARIO"));
    dbg_printf(("climbThresholdCps = %d\r\n", Nvd.par.cfg.vario.climbThresholdCps));
    dbg_printf(("zeroThresholdCps = %d\r\n", Nvd.par.cfg.vario.zeroThresholdCps));
    dbg_printf(("sinkThresholdCps = %d\r\n", Nvd.par.cfg.vario.sinkThresholdCps));
    dbg_printf(("crossoverCps = %d\r\n", Nvd.par.cfg.vario.crossoverCps));
    
    dbg_println(("KALMAN FILTER"));
    dbg_printf(("accelVariance = %d\r\n", Nvd.par.cfg.kf.accelVariance));
    dbg_printf(("zMeasVariance = %d\r\n", Nvd.par.cfg.kf.zMeasVariance));
        
    dbg_println(("MISCELLANEOUS"));
    dbg_printf(("sleepTimeoutMinutes = %d\r\n", Nvd.par.cfg.misc.sleepTimeoutMinutes));
#if (CFG_BLUETOOTH == true)    
    dbg_printf(("bluetoothEnable = %d\r\n", Nvd.par.cfg.misc.bluetoothEnable));
#endif
#endif
		}
   else  {
		#ifdef NVD_DEBUG	
		dbg_println(("ERROR!! NVD BAD CHECKSUM, SETTING DEFAULTS"));
		#endif
		nvd_set_defaults();
		nvd_commit();
		}
   EEPROM.end();
   }


void nvd_set_defaults() {
    Nvd.par.calib.axBias = 0;
    Nvd.par.calib.ayBias = 0;
    Nvd.par.calib.azBias = 0;
    Nvd.par.calib.gxBias = 0;
    Nvd.par.calib.gyBias = 0;
    Nvd.par.calib.gzBias = 0;

    Nvd.par.cfg.vario.climbThresholdCps = VARIO_CLIMB_THRESHOLD_CPS_DEFAULT;
    Nvd.par.cfg.vario.zeroThresholdCps = VARIO_ZERO_THRESHOLD_CPS_DEFAULT;
    Nvd.par.cfg.vario.sinkThresholdCps = VARIO_SINK_THRESHOLD_CPS_DEFAULT;
    Nvd.par.cfg.vario.crossoverCps = VARIO_CROSSOVER_CPS_DEFAULT;

    Nvd.par.cfg.kf.accelVariance = KF_ACCEL_VARIANCE_DEFAULT;
    Nvd.par.cfg.kf.zMeasVariance = KF_ZMEAS_VARIANCE_DEFAULT;

    Nvd.par.cfg.misc.sleepTimeoutMinutes = SLEEP_TIMEOUT_MINUTES_DEFAULT;
#if (CFG_BLUETOOTH == true)    
    Nvd.par.cfg.misc.bluetoothEnable = BLUETOOTH_DEFAULT;
#endif    
	}

	
static void nvd_commit(void) {
	uint16_t checkSum = nvd_checksum();
	Nvd.par.checkSum = ~checkSum;
	int nBytes = sizeof(NVD_PARAMS);
	for  (int inx = 0; inx < nBytes; inx++){
		EEPROM.write(inx, Nvd.buf[inx]);
		}
	EEPROM.commit();
	}
	

void nvd_save_calib_params(CALIB_PARAMS &calib) {
	EEPROM.begin(sizeof(NVD_PARAMS));
	Nvd.par.calib.axBias = calib.axBias;
	Nvd.par.calib.ayBias = calib.ayBias;
	Nvd.par.calib.azBias = calib.azBias;
	Nvd.par.calib.gxBias = calib.gxBias;
	Nvd.par.calib.gyBias = calib.gyBias;
	Nvd.par.calib.gzBias = calib.gzBias;
	nvd_commit();
	EEPROM.end();
	}	
	
void nvd_save_config_params(CONFIG_PARAMS &cfg) {
	EEPROM.begin(sizeof(NVD_PARAMS));
	memcpy(&Nvd.par.cfg.vario, &cfg.vario, sizeof(VARIO_PARAMS));
	memcpy(&Nvd.par.cfg.kf, &cfg.kf, sizeof(KALMAN_FILTER_PARAMS));
	memcpy(&Nvd.par.cfg.misc, &cfg.misc, sizeof(MISC_PARAMS));
	nvd_commit();
	EEPROM.end();
	}	
	

