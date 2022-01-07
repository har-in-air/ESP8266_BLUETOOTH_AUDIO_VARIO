#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include "config.h"
#include "util.h"
#include "imu.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "kalmanfilter4.h"
#include "vario_audio.h"
#include "cct.h"
#include "adc.h"
#include "nvd.h"
#include "audio.h"
#include "ringbuf.h"
#include "wificfg.h"
#include "ui.h"
#if (CFG_BLUETOOTH == true)
#include "btserial.h"
#endif

int      AppMode;

uint32_t TimePreviousUs;
uint32_t TimeNowUs;
float 	 ImuTimeDeltaUSecs; // time between imu samples, in microseconds
float 	 KfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

float AccelmG[3]; // in milli-Gs
float GyroDps[3];  // in degrees/second
float KfAltitudeCm = 0.0f;
float KfClimbrateCps  = 0.0f; // filtered climbrate in cm/s
int32_t AudioCps; // filtered climbrate, rounded to nearest cm/s

// pinPGCC (GPIO9) has an external 10K pullup resistor to VCC
// pressing the button  will ground the pin.
// This button has three different functions : program, configure, and calibrate (PGCC)
// 1. (Program)
//    Power on the unit with PGCC button pressed. Or with power on, keep 
//    PGCC pressed and momentarily press the reset button.
//    This will put the ESP8266 into programming mode, and you can flash 
//    the application code from the Arduino IDE.
// 2. (WiFi Configuration)
//    After normal power on, immediately press PGCC and keep it pressed. 
//    Wait until you hear a low tone, then release. The unit will now be in WiFi configuration
//    configuration mode. 
// 3. (Calibrate)
//    After normal power on, wait until you hear the battery voltage feedback beeps and
//    then the countdown to gyroscope calibration. If you press the PGCC button
//    during the gyro calibration countdown, the unit will start accelerometer calibration first. 
//    Accelerometer re-calibration is required if the acceleration calibration values in 
//    flash were never written, or if the entire flash has been erased.

volatile int DrdyCounter;
volatile boolean DrdyFlag;
volatile int SleepCounter;
volatile int BaroCounter;
volatile int SleepTimeoutSecs;

#if (CFG_LANTERN == true)
int LEDPwmLkp[4] = {LANTERN_DIM, LANTERN_LOW, LANTERN_MID, LANTERN_HI};
int LanternState;
#endif

boolean bWebConfigure = false;

MPU9250    Mpu9250;
MS5611     Ms5611;

const char* FirmwareRevision = "1.20";

// handles data ready interrupt from MPU9250 (every 2ms)
void IRAM_ATTR drdy_interrupt_handler() {
	DrdyFlag = true;
	DrdyCounter++;
	}	

// setup time markers for Mpu9250, Ms5611 and kalman filter
void time_init() {
	TimeNowUs = TimePreviousUs = micros();
	}

inline void time_update(){
	TimeNowUs = micros();
	ImuTimeDeltaUSecs = TimeNowUs > TimePreviousUs ? (float)(TimeNowUs - TimePreviousUs) : 2000.0f; // if rollover use expected time difference
	TimePreviousUs = TimeNowUs;
	}


void setup_vario() {
	dbg_println(("Vario mode"));
 	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(400000); // set i2c clock frequency to 400kHz, AFTER Wire.begin()
	delay(100);
	dbg_println(("\r\nChecking communication with MS5611"));
	if (!Ms5611.read_prom()) {
		dbg_println(("Bad CRC read from MS5611 calibration PROM"));
		Serial.flush();
		ui_indicate_fault_MS5611(); 
		ui_go_to_sleep();   // switch off and then on to fix this
		}
	dbg_println(("MS5611 OK"));
  
	dbg_println(("\r\nChecking communication with MPU9250"));
	if (!Mpu9250.check_id()) {
		dbg_println(("Error reading Mpu9250 WHO_AM_I register"));
		Serial.flush();
		ui_indicate_fault_MPU9250();
		ui_go_to_sleep();   // switch off and then on to fix this
		}
	dbg_println(("MPU9250 OK"));
    
	DrdyCounter = 0;
	DrdyFlag = false;
	// interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
	// pinDRDYInt (GPIO15) which already has an external 10K pull-down resistor (required for normal ESP8266 boot mode)
	pinMode(pinDRDYInt, INPUT); 
	attachInterrupt(digitalPinToInterrupt(pinDRDYInt), drdy_interrupt_handler, RISING);

	// configure MPU9250 to start generating gyro and accel data  
	Mpu9250.config_accel_gyro();

	// calibrate gyro (and accel if required)
	ui_calibrate_accel_gyro();
	delay(50);  
	  
	dbg_println(("\r\nMS5611 config"));
	Ms5611.reset();
	Ms5611.get_calib_coefficients(); // load MS5611 factory programmed calibration data
	Ms5611.averaged_sample(4); // get an estimate of starting altitude
	Ms5611.init_sample_state_machine(); // start the pressure & temperature sampling cycle

	dbg_println(("\r\nKalmanFilter config"));
	// initialize kalman filter with Ms5611meter estimated altitude, estimated initial climbrate = 0.0
	kalmanFilter4_configure((float)Nvd.par.cfg.kf.zMeasVariance, 1000.0f*(float)Nvd.par.cfg.kf.accelVariance, true, Ms5611.altitudeCmAvg, 0.0f, 0.0f);

	vaudio_config();  
	time_init();
	KfTimeDeltaUSecs = 0.0f;
	BaroCounter = 0;
	SleepTimeoutSecs = 0;
	ringbuf_init(); 
	SleepCounter = 0;
#if (CFG_BLUETOOTH == true)    
	if (Nvd.par.cfg.misc.bluetoothEnable == 1){
		dbg_println(("\r\nStarting Vario with bluetooth LK8EX1 messages @ 10Hz\r\n"));
		}
	else {
		dbg_println(("\r\nStarting Vario with bluetooth disabled\r\n"));  
		}
#else 
	dbg_println(("\r\nStarting Vario\r\n"));  
#endif
	}

#if (CFG_LANTERN == true)   
void setup_lantern() {
    pinMode(pinLED, OUTPUT);
	digitalWrite(pinLED, 0); // LED off
    dbg_println(("Lantern mode"));
    LanternState = 0;
    analogWrite(pinLED, LEDPwmLkp[LanternState]);
    }
#endif

#if (CFG_BLUETOOTH == true)   
void config_bluetooth() {
	pinMode(pinHM11Pwr, OUTPUT); // enable/disable power to HM-11 Bluetooth module
	digitalWrite(pinHM11Pwr, 0);  // disable
	if (Nvd.par.cfg.misc.bluetoothEnable) {
		dbg_println(("Enabling Bluetooth"));
		digitalWrite(pinHM11Pwr, 1);
		delay(300);
		Serial1.begin(9600);
		delay(20);
		// bluetooth device name = "EspVario"
		Serial1.print("AT+NAMEEspVario");
		}
   }
#endif        

void setup() {
	pinMode(pinPGCC, INPUT); //  Program/Configure/Calibrate Button
	wificfg_wifi_off(); // turn off radio to save power

#ifdef TOP_DEBUG    
	Serial.begin(115200);
#endif
  
	dbg_printf(("\r\n\r\nESP8266 BLUETOOTH VARIO compiled on %s at %s\r\n", __DATE__, __TIME__));
	dbg_printf(("Firmware Revision %s\r\n", FirmwareRevision));
	dbg_println(("\r\nChecking non-volatile data (calibration and configuration)"));  
	nvd_init();

	if (!LittleFS.begin()){
		dbg_println(("Error mounting LittleFS"));
		return;
		}   

	audio_config(pinAudio); 

	bWebConfigure = false;
	dbg_println(("To start web configuration mode, press and hold the PGCC button"));
	dbg_println(("until you hear a low-frequency tone. Then release the button"));
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
		audio_generate_tone(200, 3000);
		wificfg_ap_server_init(); 
		}
  	else {
    	ui_indicate_battery_voltage();
#if (CFG_BLUETOOTH == true)   
        config_bluetooth();
#endif        
    	switch (AppMode) {
			case APP_MODE_VARIO :
			default :
			setup_vario();
			break;
#if (CFG_LANTERN == true)
			case APP_MODE_LANTERN :
			setup_lantern();
			break;
#endif            
			}
		}
	ui_btn_init();	
	}


void vario_loop() {
	if (DrdyFlag == true) {
		// 500Hz ODR => 2mS sample interval
		DrdyFlag = false;
		time_update();
		#ifdef CCT_DEBUG    
		cct_set_marker(); // set marker for estimating the time taken to read and process the data (needs to be < 2mS !!)
		#endif    
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
		Mpu9250.get_accel_gyro_data(AccelmG, GyroDps); 

		// We arbitrarily decide that with the power bank lying flat with the pcb on top,
		// the CJMCU-117 board silkscreen Y points "forward" or "north"  (the side with the HM-11), 
		// silkscreen X points "right" or "east", and silkscreen Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in our AHRS algorithm implementation.
		// The required mapping from sensor samples to NED frame for our specific board orientation is : 
		// gxned = gx, gyned = gy, gzned = -gz (clockwise rotations about the axis must result in +ve readings on the axis)
		// axned = ay, ayned = ax, azned = az (when the axis points down, axis reading must be +ve)
		// The AHRS algorithm expects rotation rates in radians/second
		// Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
		float accelMagnitudeSquared = AccelmG[0]*AccelmG[0] + AccelmG[1]*AccelmG[1] + AccelmG[2]*AccelmG[2];
		int bUseAccel = ((accelMagnitudeSquared > 562500.0f) && (accelMagnitudeSquared < 1562500.0f)) ? 1 : 0;
        float dtIMU = ImuTimeDeltaUSecs/1000000.0f;
        float gxned = DEG_TO_RAD*GyroDps[0];
        float gyned = DEG_TO_RAD*GyroDps[0];
        float gzned = -DEG_TO_RAD*GyroDps[0];
        float axned = AccelmG[1];
        float ayned = AccelmG[0];
        float azned = AccelmG[2];
		imu_mahonyAHRS_update6DOF(bUseAccel, dtIMU, gxned, gyned, gzned, axned, ayned, azned);
		float gCompensatedAccel = imu_gravity_compensated_accel(axned, ayned, azned, Q0, Q1, Q2, Q3);
		ringbuf_add_sample(gCompensatedAccel);  

		BaroCounter++;
		KfTimeDeltaUSecs += ImuTimeDeltaUSecs;
		if (BaroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			BaroCounter = 0;    // alternating between pressure and temperature samples
			// one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = Ms5611.sample_state_machine(); 
			if ( zMeasurementAvailable ) { 
				// average earth-z acceleration over the 20mS interval between z samples
				// is used in the kf algorithm update phase
				float zAccelAverage = ringbuf_average_newest_samples(10); 
				float dtKF = KfTimeDeltaUSecs/1000000.0f;
				kalmanFilter4_predict(dtKF);
				kalmanFilter4_update(Ms5611.altitudeCm, zAccelAverage, (float*)&KfAltitudeCm, (float*)&KfClimbrateCps);
				// reset time elapsed between kalman filter algorithm updates
				KfTimeDeltaUSecs = 0.0f;
				AudioCps =  KfClimbrateCps >= 0.0f ? (int32_t)(KfClimbrateCps+0.5f) : (int32_t)(KfClimbrateCps-0.5f);
				vaudio_tick_handler(AudioCps);                
				if (ABS(AudioCps) > SLEEP_THRESHOLD_CPS) { 
					// reset sleep timeout watchdog if there is significant vertical motion
					SleepTimeoutSecs = 0;
					}
				else
				if (SleepTimeoutSecs >= (Nvd.par.cfg.misc.sleepTimeoutMinutes*60)) {
					dbg_println(("Timed out with no significant climb/sink, put MPU9250 and ESP8266 to sleep to minimize current draw"));
					Serial.flush();
					ui_indicate_sleep(); 
					ui_go_to_sleep();
					}   
				}
			}
			
	#ifdef CCT_DEBUG      
		uint32_t elapsedUs =  cct_get_elapsedUs(); // calculate time  taken to read and process the data, must be less than 2mS
	#endif
		if (DrdyCounter >= 50) {
			DrdyCounter = 0; // 0.1 second elapsed
#if (CFG_BLUETOOTH == true)
			if (Nvd.par.cfg.misc.bluetoothEnable == 1) {
				int adcVal = analogRead(A0);
				float bv = adc_battery_voltage(adcVal);
				int altM =  KfAltitudeCm > 0.0f ? (int)((KfAltitudeCm+50.0f)/100.0f) :(int)((KfAltitudeCm-50.0f)/100.0f);
				btserial_transmit_LK8EX1(altM, AudioCps, bv);
				}
#endif
			SleepCounter++;
			if (SleepCounter >= 10) {
				SleepCounter = 0;
				SleepTimeoutSecs++;
				#ifdef IMU_DEBUG
				float yaw, pitch, roll;
				imu_quaternion_to_yaw_pitch_roll(Q0,Q1,Q2,Q3, &yaw, &pitch, &roll);
				// Pitch is positive for clockwise rotation about the +Y axis
				// Roll is positive for clockwise rotation about the +X axis
				// Yaw is positive for clockwise rotation about the +Z axis
				// Magnetometer isn't used, so yaw is initialized to 0 for the "forward" direction of the case on power up.
				dbg_printf(("\r\nY = %d P = %d R = %d\r\n", (int)yaw, (int)pitch, (int)roll));
				dbg_printf(("ba = %d ka = %d kv = %d\r\n",(int)Ms5611.altitudeCm, (int)KfAltitudeCm, (int)KfClimbrateCps));
				#endif     
				#ifdef CCT_DEBUG      
                // The raw IMU data rate is 500Hz, i.e. 2000uS between Data Ready Interrupts
                // We need to read the MPU9250 data, MS5611 data and finish all computations
                // and actions well within this interval.
                // Checked, < 620 uS @ 80MHz clock
				dbg_printf(("Elapsed %dus\r\n", (int)elapsedUs)); 
				#endif
				}
			}
		}	
#if (CFG_LANTERN == true)
	if (BtnPGCCLongPress == true) {
		AppMode = APP_MODE_LANTERN;
		setup_lantern();
		delay(500);
		ui_btn_clear();    
		}  
#endif        
	}

#if (CFG_LANTERN == true)
void lantern_loop() {
	int count;
	if (BtnPGCCPressed) {
		ui_btn_clear();
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
#endif


void loop(){
	if (bWebConfigure == true) {
		// nothing to do here
		}
	else { 
		switch (AppMode) {
			case APP_MODE_VARIO :
			default :
			vario_loop();
			break;

#if (CFG_LANTERN == true)
			case APP_MODE_LANTERN :
			lantern_loop();
			break;      
#endif            
			}
		} 
	}
