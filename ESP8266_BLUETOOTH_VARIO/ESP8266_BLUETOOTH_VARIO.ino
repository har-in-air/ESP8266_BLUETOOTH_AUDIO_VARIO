#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include "config.h"
#include "board.h"
#include "util.h"
#include "MahonyAHRS.h"
#include "MPU9250.h"
#include "MS5611.h"
#include "kalmanfilter4.h"
#include "VarioAudio.h"
#include "cct.h"
#include "adc.h"
#include "nvd.h"
#include "audio.h"
#include "ringbuf.h"
#include "wificonfig.h"
#include "ui.h"
#include "btserial.h"

int      appMode;

uint32_t timePreviousUs;
uint32_t timeNowUs;
float 	 imuTimeDeltaUSecs; // time between imu samples, in microseconds
float 	 kfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

float accelmG[3]; // in milli-Gs
float gyroDps[3];  // in degrees/second
float kfAltitudeCm = 0.0f;
float kfClimbrateCps  = 0.0f; // filtered climbrate in cm/s
int32_t audioCps; // rounded to nearest cm/s

// pinPGCC (GPIO0) has an external 10K pullup resistor to VCC, pressing the button 
// will ground the pin.
// This button has three different functions : program, configure, and calibrate
// 1. (Pgm)Power on the unit with pgmconfcal button pressed. Or with power on, keep 
//    pgmconfcal pressed and momentarily press the reset button.
//    This will put the ESP8266/ESP8285 into programming mode, and you can flash 
//    the application code from the Arduino IDE.
// 2. (Conf)After normal power on, immediately press pgmconfcal and keep it pressed. After a 
//    few seconds, you will hear a low tone for about 5 seconds.
//    You can release the button as soon as you hear this tone, the unit will now
//    be in web configuration mode. 
// 3. (Cal)After normal power on, wait until you hear the battery voltage feedback beeps and
//    then the countdown to gyroscope calibration. If you press the pgmconfcal button
//    during the gyro calibration countdown, the unit will start accelerometer calibration first. 
//    Accelerometer calibration is only required if the accel calibration values in 
//    flash were never written, or were corrupted.

volatile int drdyCounter;
volatile boolean drdyFlag;
volatile int sleepCounter;
volatile int baroCounter;
volatile int sleepTimeoutSecs;

int LEDPwmLkp[4] = {LANTERN_DIM, LANTERN_LOW, LANTERN_MID, LANTERN_HI};
int LanternState;

boolean bWebConfigure = false;

MPU9250 imu;
MS5611 baro;
VarioAudio vario;

const char* FirmwareRevision = "0.97";

// handles data ready interrupt from MPU9250
void IRAM_ATTR drdy_InterruptHandler() {
	drdyFlag = true;
	drdyCounter++;
	}	

// setup time markers for imu, baro and kalman filter
void time_Init() {
	timeNowUs = timePreviousUs = micros();
	}

inline void time_Update(){
	timeNowUs = micros();
	imuTimeDeltaUSecs = timeNowUs > timePreviousUs ? (float)(timeNowUs - timePreviousUs) : 2000.0f; // if rollover use expected time difference
	timePreviousUs = timeNowUs;
	}


void setupVario() {
	dbg_println(("Vario mode"));
	if (nvd.par.cfg.misc.bluetoothEnable) {
		dbg_println(("Enabling Bluetooth"));
		digitalWrite(pinHM11Pwr, 1);
		delay(300);
		Serial1.begin(9600);
		delay(20);
		Serial1.print("AT+NAMEEspVario");
		}
	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(400000); // set i2c clock frequency AFTER Wire.begin()
	delay(100);
	dbg_println(("\r\nChecking communication with MS5611"));
	if (!baro.ReadPROM()) {
		dbg_println(("Bad CRC read from MS5611 calibration PROM"));
		Serial.flush();
		indicateFaultMS5611(); 
		goToSleep();   // switch off and then on to fix this
		}
	dbg_println(("MS5611 OK"));
  
	dbg_println(("\r\nChecking communication with MPU9250"));
	if (!imu.CheckID()) {
		dbg_println(("Error reading mpu9250 WHO_AM_I register"));
		Serial.flush();
		indicateFaultMPU9250();
		goToSleep();   // switch off and then on to fix this
		}
	dbg_println(("MPU9250 OK"));
    
	drdyCounter = 0;
	drdyFlag = false;
	// interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
	// pinDRDYInt (GPIO15) which already has an external 10K pull-down resistor (required for normal ESP8266 boot mode)
	pinMode(pinDRDYInt, INPUT); 
	attachInterrupt(digitalPinToInterrupt(pinDRDYInt), drdy_InterruptHandler, RISING);

	// configure MPU9250 to start generating gyro and accel data  
	imu.ConfigAccelGyro();
	calibrate_accel_gyro();
	delay(100);  
	  
	dbg_println(("\r\nMS5611 config"));
	baro.Reset();
	baro.GetCalibrationCoefficients(); // load MS5611 factory programmed calibration data
	baro.AveragedSample(4); // get an estimate of starting altitude
	baro.InitializeSampleStateMachine(); // start the pressure & temperature sampling cycle

	dbg_println(("\r\nKalmanFilter config"));
	// initialize kalman filter with barometer estimated altitude, and climbrate = 0.0
	kalmanFilter4_configure((float)nvd.par.cfg.kf.zMeasVariance, 1000.0f*(float)nvd.par.cfg.kf.accelVariance, true, baro.zCmAvg_, 0.0f, 0.0f);

	vario.Config();  
	time_Init();
	kfTimeDeltaUSecs = 0.0f;
	baroCounter = 0;
	sleepTimeoutSecs = 0;
	ringbuf_init(); 
	sleepCounter = 0;
	if (nvd.par.cfg.misc.bluetoothEnable ){
		dbg_println(("\r\nStarting vario with bluetooth LK8EX1 messages @ 10Hz\r\n"));
		}
	else {
		dbg_println(("\r\nStarting vario with bluetooth disabled\r\n"));  
		}
	}

   
void setupLantern() {
   dbg_println(("Lantern mode"));
   LanternState = 0;
   analogWrite(pinLED, LEDPwmLkp[LanternState]);
   }


void setup() {
	pinMode(pinLED, OUTPUT);
	digitalWrite(pinLED, 0); // LED off
	pinMode(pinHM11Pwr, OUTPUT); // enable/disable power to HM-11 Bluetooth module
	digitalWrite(pinHM11Pwr, 0);  // disable
	pinMode(pinL9110Pwr, OUTPUT); // enable/disable power to L9110 push-pull driver
	digitalWrite(pinL9110Pwr, 0); // disable
	pinMode(pinPGCC, INPUT); //  Program/Configure/Calibrate Button

	wificonfig_wifiOff(); // turn off radio to save power

#ifdef TOP_DEBUG    
	Serial.begin(115200);
#endif
  
	dbg_printf(("\r\n\r\nESP8266 BLUETOOTH VARIO compiled on %s at %s\r\n", __DATE__, __TIME__));
	dbg_printf(("Firmware Revision %s\r\n", FirmwareRevision));
	dbg_println(("\r\nChecking non-volatile data (calibration and configuration)"));  
	nvd_Init();

	if (!SPIFFS.begin()){
		dbg_println(("Error mounting SPIFFS"));
		return;
		}   

	audio_Config(pinAudio); 

	bWebConfigure = false;
	dbg_println(("To start web configuration mode, press and hold the pgmconfcal button"));
	dbg_println(("until you hear a low-frequency tone start. Then release the button"));
	for (int cnt = 0; cnt < 4; cnt++) {
		dbg_println((4-cnt));
		delay(1000);
		if (digitalRead(pinPGCC) == 0) {
			bWebConfigure = true;
			break;
			}
		}
	if (bWebConfigure == true) {
		dbg_println(("Web configuration mode selected"));
		// 3 second long tone with low frequency to indicate unit is now in web server configuration mode.
		// After you are done with web configuration, switch off the vario as the wifi radio
		// consumes a lot of power.
		audio_GenerateTone(200, 3000);
		wifi_access_point_init(); 
		}
  	else {
    	battery_IndicateVoltage();
    	switch (appMode) {
			case APP_MODE_VARIO :
			default :
			setupVario();
			break;

			case APP_MODE_LANTERN :
			setupLantern();
			break;
			}
		}
	btn_init();	
	}


void vario_loop() {
	if (drdyFlag == true) {
		// 500Hz ODR => 2mS sample interval
		drdyFlag = false;
		time_Update();
		#ifdef CCT_DEBUG    
		cct_SetMarker(); // set marker for estimating the time taken to read and process the data
		#endif    
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
		imu.GetAccelGyroData(accelmG, gyroDps); 

		// We arbitrarily decide that with the power bank lying flat with the pcb on top,
		// the CJMCU-117 board silkscreen -Y points "forward" or "north"  (the side with the HM-11), 
		// silkscreen -X points "right" or "east", and silkscreen -Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in our AHRS algorithm implementation.
		// The required mapping from sensor samples to NED frame for our specific board orientation is : 
		// gxned = gx, gyned = gy, gzned = -gz (clockwise rotations about the axis must result in +ve readings on the axis)
		// axned = ay, ayned = ax, azned = az (when the axis points down, axis reading must be +ve)
		// The AHRS algorithm expects rotation rates in radians/second
		// Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
		float accelMagnitudeSquared = accelmG[0]*accelmG[0] + accelmG[1]*accelmG[1] + accelmG[2]*accelmG[2];
		int bUseAccel = ((accelMagnitudeSquared > 562500.0f) && (accelMagnitudeSquared < 1562500.0f)) ? 1 : 0;
		float dtIMU = imuTimeDeltaUSecs/1000000.0f;
		imu_MahonyAHRSupdate6DOF(bUseAccel, dtIMU, DEG_TO_RAD*gyroDps[0], DEG_TO_RAD*gyroDps[1], -DEG_TO_RAD*gyroDps[2], accelmG[1], accelmG[0], accelmG[2]);
		float gravityCompensatedAccel = imu_GravityCompensatedAccel(accelmG[1], accelmG[0], accelmG[2], q0, q1, q2, q3);
		ringbuf_addSample(gravityCompensatedAccel);  

		baroCounter++;
		kfTimeDeltaUSecs += imuTimeDeltaUSecs;
		if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			baroCounter = 0;    // alternating between pressure and temperature samples
			// one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = baro.SampleStateMachine(); 
			if ( zMeasurementAvailable ) { 
				// average earth-z acceleration over the 20mS interval between z samples
				// is used in the kf algorithm update phase
				float zAccelAverage = ringbuf_averageNewestSamples(10); 
				float dtKF = kfTimeDeltaUSecs/1000000.0f;
				kalmanFilter4_predict(dtKF);
				kalmanFilter4_update(baro.zCmSample_, zAccelAverage, (float*)&kfAltitudeCm, (float*)&kfClimbrateCps);
				// reset time elapsed between kalman filter algorithm updates
				kfTimeDeltaUSecs = 0.0f;
				audioCps =  kfClimbrateCps >= 0.0f ? (int32_t)(kfClimbrateCps+0.5f) : (int32_t)(kfClimbrateCps-0.5f);
				vario.Beep(audioCps);                
				if (ABS(audioCps) > SLEEP_THRESHOLD_CPS) { 
					// reset sleep timeout watchdog if there is significant vertical motion
					sleepTimeoutSecs = 0;
					}
				else
				if (sleepTimeoutSecs >= (nvd.par.cfg.misc.sleepTimeoutMinutes*60)) {
					dbg_println(("Timed out with no significant climb/sink, put MPU9250 and ESP8266 to sleep to minimize current draw"));
					Serial.flush();
					indicateSleep(); 
					goToSleep();
					}   
				}
			}
			
	#ifdef CCT_DEBUG      
		uint32_t elapsedUs =  cct_ElapsedTimeUs(); // calculate time  taken to read and process the data, must be less than 2mS
	#endif
		if (drdyCounter >= 50) {
			drdyCounter = 0; // 0.1 second elapsed
			if (nvd.par.cfg.misc.bluetoothEnable) {
				int adcVal = analogRead(A0);
				float bv = adc_battery_voltage(adcVal);
				int altM =  kfAltitudeCm > 0.0f ? (int)((kfAltitudeCm+50.0f)/100.0f) :(int)((kfAltitudeCm-50.0f)/100.0f);
				btserial_transmitSentence(altM, audioCps, bv);
				}
			sleepCounter++;
			if (sleepCounter >= 10) {
				sleepCounter = 0;
				sleepTimeoutSecs++;
				#ifdef IMU_DEBUG
				float yaw, pitch, roll;
				imu_Quaternion2YawPitchRoll(q0,q1,q2,q3, &yaw, &pitch, &roll);
				// Pitch is positive for clockwise rotation about the +Y axis
				// Roll is positive for clockwise rotation about the +X axis
				// Yaw is positive for clockwise rotation about the +Z axis
				// Magnetometer isn't used, so yaw is initialized to 0 for the "forward" direction of the case on power up.
				dbg_printf(("\r\nY = %d P = %d R = %d\r\n", (int)yaw, (int)pitch, (int)roll));
				dbg_printf(("ba = %d ka = %d kv = %d\r\n",(int)baro.zCmSample_, (int)kfAltitudeCm, (int)kfClimbrateCps));
				#endif     
				#ifdef CCT_DEBUG      
				dbg_printf(("Elapsed %dus\r\n", (int)elapsedUs)); // ~ 700 uS, so plenty of headroom
				#endif
				}
			}
		}	

	if (BtnPGCCLongPress == true) {
		appMode = APP_MODE_LANTERN;
		setupLantern();
		delay(500);
		btn_clear();    
		}  
	}


void lantern_loop() {
	int count;
	if (BtnPGCCPressed) {
		btn_clear();
		LanternState++;
		if (LanternState > 4) {
			LanternState = 0;
			}
		if (LanternState < 4) {
			analogWrite(pinLED, LEDPwmLkp[LanternState]);
			}
		}
	if (LanternState == 4) {
	// flash S.O.S. pattern
		count = 3;
		while (count--) {
			if (BtnPGCCPressed) return;
			analogWrite(pinLED, 1023);
			delay(50);
			analogWrite(pinLED, 0);
			delay(400);
			}
		count = 12;
		while (count--) {
			if (BtnPGCCPressed) return;
			delay(50);        
			}
		count = 3;
		while (count--) {
			if (BtnPGCCPressed) return;
			analogWrite(pinLED, 1023);
			delay(1000);
			analogWrite(pinLED, 0);
			delay(400);
			}
		count = 12;
		while (count--) {
			if (BtnPGCCPressed) return;
			delay(50);        
			}
		count = 3;
		while (count--) {
			if (BtnPGCCPressed) return;
			analogWrite(pinLED, 1023);
			delay(50);
			analogWrite(pinLED, 0);
			delay(400);
			}
		count = 80;
		while (count--) {
			if (BtnPGCCPressed) return;
			delay(50);
			}    
		}
	}
  

void loop(){
	if (bWebConfigure == true) {
		// nothing to do here
		}
	else { 
		switch (appMode) {
			case APP_MODE_VARIO :
			default :
			vario_loop();
			break;

			case APP_MODE_LANTERN :
			lantern_loop();
			break;      
			}
		} 
	}
