#include <Arduino.h>
#include <Wire.h>
#include <Ticker.h>
#include "board.h"
#include "config.h"
#include "util.h"
#include "MahonyAHRS.h"
#include "MPU9250.h"
#include "MS5611.h"
#include "kalmanfilter4.h"
#include "VarioAudio.h"
#include "cct.h"
#include "nvd.h"
#include "audio.h"
#include "ringbuf.h"
#include "wificonfig.h"

Ticker     Tickr;
int        appMode;

uint32_t 	timePreviousUs;
uint32_t 	timeNowUs;
float 		imuTimeDeltaUSecs; // time between imu samples, in microseconds
float 		kfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

float accelmG[3]; // in milli-Gs
float gyroDps[3];  // in degrees/second
float kfAltitudeCm = 0.0f;
float kfClimbrateCps  = 0.0f; // filtered climbrate in cm/s
int32_t audioCps; // rounded to nearest cm/s


// pinPgmConfCalBtn (GPIO0) has an external 10K pullup resistor to VCC, pressing the button 
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


int bluetoothCounter;
volatile int drdyCounter;
int sleepCounter;
volatile boolean drdyFlag;

int alarmCounter;
int bMotionAlarm = false;

volatile uint32_t BtnPGCCState;
volatile bool BtnPGCCPressed = false;
volatile bool BtnPGCCLongPress = false;

int LEDPwmLkp[4] = {LANTERN_DIM, LANTERN_LOW, LANTERN_MID, LANTERN_HI};
int LanternState;

int baroCounter;
int sleepTimeoutSecs;

boolean bWebConfigure;

MPU9250 imu;
MS5611 baro;
VarioAudio vario;

uint16_t FirmwareRevision = 0.94; // v0.94


void btn_clear() {
   BtnPGCCPressed  = false;
   BtnPGCCLongPress = false;
   }


void IRAM_ATTR btn_debounce() {
   BtnPGCCState = ((BtnPGCCState<<1) | ((uint32_t)BTN_PGCC()) );
   if ((BtnPGCCState | 0xFFFFFFF0) == 0xFFFFFFF8) {
     BtnPGCCPressed = true;
     }    
   if (BtnPGCCState == 0) BtnPGCCLongPress = true;
   }



int adc_sampleAverage(int nSamples) {
  int adcSample = 0;
  for (int inx = 0; inx < 4; inx++) {
    adcSample += analogRead(A0);
    delay(1);
    }
  adcSample /= 4;
  return adcSample;  
  }

// voltage divider with 12K and 3.3K to scale 4.2V down to < 1.0V for the ESP8266 ADC
// calibrate slope & intercept with two test points
#define V1  3.286f
#define V2  4.050f
#define ADC1  765.0f
#define ADC2  932.0f

float battery_voltage(int sample) {
  float slope = (V2 - V1)/(ADC2 - ADC1);
  float voltage = slope*(sample - ADC1) + V1;
  return voltage;
  }
  
void battery_IndicateVoltage() {
   int numBeeps;
   int adcSample = adc_sampleAverage(4);
   float fbv = battery_voltage(adcSample);
#ifdef MAIN_DEBUG   
   Serial.printf("\r\nBattery voltage = %.2fV\r\n", fbv );
#endif

   if (fbv >= 4.0f) numBeeps = 5;
   else
   if (fbv >= 3.9f) numBeeps = 4;
   else
   if (fbv >= 3.7f) numBeeps = 3;
   else
   if (fbv >= 3.6f) numBeeps = 2;
   else  numBeeps = 1;
   while (numBeeps--) {
      audio_GenerateTone(BATTERY_TONE_HZ, 300);
      delay(300);
      }
   }
  

static uint8_t btserial_nmeaChecksum(const char *szNMEA){
   const char* sz = &szNMEA[1]; // skip leading '$'
   uint8_t cksum = 0;

   while ((*sz) != 0 && (*sz != '*')) {
      cksum ^= (uint8_t) *sz;
      sz++;
      }
   return cksum;
   }

   
void btserial_transmitSentence() {
   char szmsg[40];
   int  adcSample = analogRead(A0);
   float batteryVoltage = battery_voltage(adcSample);
   sprintf(szmsg, "$LK8EX1,%d,99999,%d,99,%.1f*", (uint32_t)(baro.paSample_+0.5f), audioCps, batteryVoltage);
   uint8_t cksum = btserial_nmeaChecksum(szmsg);
   char szcksum[5];
   sprintf(szcksum,"%02X\r\n", cksum);
   strcat(szmsg, szcksum);
   Serial1.print(szmsg);
  }

	
// residual current draw in sleep mode is the sum of ESP8266 deep sleep mode current,
// MPU9250 sleep mode current, MS5611 standby current, quiescent current of voltage
// regulators, and miscellaneous current through resistive paths e.g. the
// ADC voltage divider.
void goToSleep() {
	audio_SetFrequency(0); // switch off pwm audio 
  digitalWrite(pinBTSw, 0); // switch off bluetooth power supply
	imu.Sleep(); // put MPU9250 in sleep mode
	ESP.deepSleep(0); // ESP8266 in sleep can only recover with a reset/power cycle
	}

	
// if imu calibration data in flash was never initialized or is corrupt, the accel 
// and gyro biases are set to 0, and this uncalibrated state is indicated with a 
// continuous sequence of alternating high and low beeps for 10 seconds.
// !! You must manually calibrate the accelerometer if you hear this alarm, it is
// !! required for normal vario operation.
void indicateUncalibratedAccelerometerGyro() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio_GenerateTone(UNCALIBRATED_TONE_HZ, 500); 
		audio_GenerateTone(UNCALIBRATED_TONE_HZ/2, 500);
		}
	}

	
// "no-activity" timeout sleep is indicated with a series of descending
// tones. If you do hear this, switch off the vario as there is still
// residual current draw from the circuit components in sleep mode
void indicateSleep() {
	audio_GenerateTone(2000,1000); 
	audio_GenerateTone(1000,1000);
	audio_GenerateTone(500, 1000);
	audio_GenerateTone(250, 1000);
	}

// problem with MS5611 calibration CRC, assume communication error or bad device. 
void indicateFaultMS5611() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio_GenerateTone(MS5611_ERROR_TONE_HZ, 1000); 
		delay(100);
		}
	}

// problem reading MPU9250 ID, assume communication error or bad device. 
void indicateFaultMPU9250() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio_GenerateTone(MPU9250_ERROR_TONE_HZ, 1000); 
		delay(100);
		}
	}


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
#ifdef MAIN_DEBUG    
  Serial.println("Vario mode");
#endif        
  if (nvd.params.misc.bluetoothRateHz != BLUETOOTH_RATE_DISABLED) {
    digitalWrite(pinBTSw, 1);
    delay(300);
    Serial1.begin(9600);
    delay(20);
    Serial1.print("AT+NAMEEspVario");
    }
  Wire.begin(pinSDA, pinSCL);
  Wire.setClock(400000); // set i2c clock frequency AFTER Wire.begin()

  delay(1000);
  
#ifdef MAIN_DEBUG   
    Serial.println("\r\nChecking communication with MS5611");
#endif
  if (!baro.ReadPROM()) {
#ifdef MAIN_DEBUG   
    Serial.println("Bad CRC read from MS5611 calibration PROM");
    Serial.flush();
#endif
    indicateFaultMS5611(); 
    goToSleep();   // switch off and then on to fix this
    }
#ifdef MAIN_DEBUG   
    Serial.println("MS5611 OK");
#endif
  
#ifdef MAIN_DEBUG   
    Serial.println("\r\nChecking communication with MPU9250");
#endif
  if (!imu.CheckID()) {
#ifdef MAIN_DEBUG   
    Serial.println("Error reading mpu9250 WHO_AM_I register");
    Serial.flush();
#endif
    indicateFaultMPU9250();
    goToSleep();   // switch off and then on to fix this
    }
#ifdef MAIN_DEBUG   
    Serial.println("MPU9250 OK");
#endif
  
  if ((nvd.params.calib.axBias == 0) && (nvd.params.calib.ayBias == 0) && (nvd.params.calib.azBias == 0)) {
    indicateUncalibratedAccelerometerGyro(); 
#ifdef MAIN_DEBUG   
    Serial.println("Error : uncalibrated accelerometer, manual calibration is now REQUIRED for vario operation !!");
    Serial.println("When you hear the gyro calibration countdown, press the pgmconfcal button to force");    
    Serial.println("accelerometer calibration as well as gyro calibration");    
#endif    
    }
    
  // load the accel & gyro calibration parameters from the non-volatile data structure
  imu.GetCalibrationParams(&nvd);
  
  drdyCounter = 0;
  drdyFlag = 0;
  // interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
  // pinDRDYInt (GPIO15) which already has an external 10K pull-down resistor (required for normal ESP8266 boot mode)
  pinMode(pinDRDYInt, INPUT); 
  attachInterrupt(digitalPinToInterrupt(pinDRDYInt), drdy_InterruptHandler, RISING);
    
  // configure MPU9250 to start generating gyro and accel data  
  imu.ConfigAccelGyro();
  
  // Vario will attempt to calibrate gyro each time on power up. If the vario is disturbed, it will
  // use the last saved gyro calibration values.
  // The software delays a few seconds so that the unit can be left undisturbed for gyro calibration.
  // This delay is indicated with a series of 10 short beeps. While it is beeping, if you press the
  // pgmconfcal button, the unit will calibrate the accelerometer first and then the gyro.
  // As soon as you hear the long confirmation tone, release the button and
  // put the unit in accelerometer calibration position resting undisturbed on a horizontal surface 
  // with the accelerometer +z axis pointing vertically downwards. You will have some time 
  // to do this, indicated by a series of beeps. After calibration, the unit will generate another 
  // tone, save the calibration parameters to flash, and continue with normal vario operation
  
  boolean bCalibrateAccelerometer = false;
#ifdef MAIN_DEBUG
  Serial.println("Counting down to gyro calibration");
  Serial.println("Press the pgmconfcal button if accelerometer calibration is also required");
#endif  
  for (int inx = 0; inx < 10; inx++) {
    delay(500); 
#ifdef MAIN_DEBUG
  Serial.println(10-inx);
#endif  
    audio_GenerateTone(CALIBRATING_TONE_HZ, 50); 
    if (digitalRead(pinPgmConfCalBtn) == 0) {
       bCalibrateAccelerometer = true;
       break;
      }
    }
  if (bCalibrateAccelerometer == true) {  
    // acknowledge calibration button press with long tone
    audio_GenerateTone(CALIBRATING_TONE_HZ, 3000);
#ifdef MAIN_DEBUG   
    Serial.println("Manual accelerometer calibration requested");
    Serial.println("Place vario on a level surface with accelerometer z axis vertical and leave it undisturbed");
    Serial.println("You have 10 seconds, counted down with rapid beeps");
#endif
    for (int inx = 0; inx < 50; inx++) {
      delay(200); 
#ifdef MAIN_DEBUG   
  Serial.println(50-inx);
#endif  
      audio_GenerateTone(CALIBRATING_TONE_HZ, 50);
      }
#ifdef MAIN_DEBUG   
    Serial.println("\r\nCalibrating accelerometer");
#endif
    imu.CalibrateAccel();
#ifdef MAIN_DEBUG   
    Serial.println("Accelerometer calibration done");
#endif
    nvd_SaveCalibrationParams(imu.axBias_,imu.ayBias_,imu.azBias_,imu.gxBias_,imu.gyBias_,imu.gzBias_);
    }

#ifdef MAIN_DEBUG   
    Serial.println("\r\nCalibrating gyro");
#endif
  // normal power-on operation flow, always attempt to calibrate gyro. If calibration isn't possible because 
  // the unit is continuously disturbed (e.g. you turned on the unit while already flying), indicate this and
  // use the last saved gyro biases. Otherwise, save the new gyro biases to flash memory
  if (imu.CalibrateGyro()) {
#ifdef MAIN_DEBUG   
    Serial.println("Gyro calibration OK");
#endif
    audio_GenerateTone(CALIBRATING_TONE_HZ, 1000);
    nvd_SaveCalibrationParams(imu.axBias_,imu.ayBias_,imu.azBias_,imu.gxBias_,imu.gyBias_,imu.gzBias_);
    }
  else { 
#ifdef MAIN_DEBUG   
    Serial.println("Gyro calibration failed");
#endif
    audio_GenerateTone(CALIBRATING_TONE_HZ, 1000);
    delay(500);
    audio_GenerateTone(CALIBRATING_TONE_HZ/2, 1000);
    }
    
  delay(1000);  
      
#ifdef MAIN_DEBUG   
    Serial.println("\r\nMS5611 config");
#endif
  baro.Reset();
  baro.GetCalibrationCoefficients(); // load MS5611 factory programmed calibration data
  baro.AveragedSample(4); // get an estimate of starting altitude
  baro.InitializeSampleStateMachine(); // start the pressure & temperature sampling cycle

#ifdef MAIN_DEBUG   
  Serial.println("\r\nKalmanFilter config");
#endif  
  // initialize kalman filter with barometer estimated altitude, and climbrate = 0.0
  //kf.Config((float)nvd.params.kf.zMeasVariance, 1000.0f*(float)nvd.params.kf.accelVariance, KF_ACCELBIAS_VARIANCE, baro.zCmAvg_, 0.0f);
  kalmanFilter4_configure((float)nvd.params.kf.zMeasVariance, 1000.0f*(float)nvd.params.kf.accelVariance, true, baro.zCmAvg_, 0.0f, 0.0f);

#ifdef MAIN_DEBUG   
  Serial.println("\r\nVario beeper config");
#endif
  vario.Config();
  
  time_Init();
  kfTimeDeltaUSecs = 0.0f;
  baroCounter = 0;
  sleepTimeoutSecs = 0;
  // circular buffer of latest z-earth acceleration samples, needed to ensure we use the acceleration samples that are
  // in sync with the pressure-derived altitude samples
  ringbuf_init(); 
  bluetoothCounter = 0;
  sleepCounter = 0;
#ifdef MAIN_DEBUG   
  Serial.println("\r\nStart vario");
#endif
  }


   
void setupLantern() {
#ifdef MAIN_DEBUG    
  Serial.println("Lantern mode");
#endif        
   LanternState = 0;
   analogWrite(pinLED, LEDPwmLkp[LanternState]);
   }


void setupMotionAlarm() {
#ifdef MAIN_DEBUG    
  Serial.println("Motion alarm mode");
#endif        
  Wire.begin(pinSDA, pinSCL);
  Wire.setClock(400000); // set i2c clock frequency AFTER Wire.begin()

  delay(1000);
    
#ifdef MAIN_DEBUG   
    Serial.println("\r\nChecking communication with MPU9250");
#endif
  if (!imu.CheckID()) {
#ifdef MAIN_DEBUG   
    Serial.println("Error reading mpu9250 WHO_AM_I register");
    Serial.flush();
#endif
    indicateFaultMPU9250();
    goToSleep();   // switch off and then on to fix this
    }
#ifdef MAIN_DEBUG   
    Serial.println("MPU9250 OK");
#endif
      
  // load the accel & gyro calibration parameters from the non-volatile data structure
  imu.GetCalibrationParams(&nvd);
  
  drdyCounter = 0;
  drdyFlag = 0;
  // interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
  // pinDRDYInt (GPIO15) which already has an external 10K pull-down resistor (required for normal ESP8266 boot mode)
  pinMode(pinDRDYInt, INPUT); 
  attachInterrupt(digitalPinToInterrupt(pinDRDYInt), drdy_InterruptHandler, RISING);

  // no need for accelerometer/gyro calibration checking, as we don't need that much precision  
  // configure MPU9250 to start generating gyro and accel data  
  imu.ConfigAccelGyro();
  
   bMotionAlarm = false;
   alarmCounter = 0;   
#ifdef MAIN_DEBUG
  Serial.println("Counting down to motion alarm arming");
#endif  
  for (int inx = 0; inx < 10; inx++) {
    delay(500); 
#ifdef MAIN_DEBUG
  Serial.println(10-inx);
#endif  
    audio_GenerateTone(ALARM_COUNTDOWN_HZ, 50); 
    }
     
#ifdef MAIN_DEBUG   
  Serial.println("\r\nMotion alarm is armed");
#endif
  }



void setup() {
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, 0);
  pinMode(pinBTSw, OUTPUT);
  digitalWrite(pinBTSw, 0);
  pinMode(pinAudioEn, OUTPUT);
  digitalWrite(pinAudioEn, 0);
  bWebConfigure = false;
  pinMode(pinPgmConfCalBtn, INPUT);
  wificonfig_wifiOff(); // turn off radio to save power

#ifdef MAIN_DEBUG    
  Serial.begin(115200);
  Serial.printf("\r\nESP8266 BLUETOOTH VARIO compiled on %s at %s\r\n", __DATE__, __TIME__);
  Serial.printf("Firmware Revision %d.%02d\r\n", FirmwareRevision/100, FirmwareRevision%100);
#endif
#ifdef MAIN_DEBUG    
  Serial.println("\r\nChecking non-volatile data (calibration and configuration)");  
#endif
   nvd_Init();
   appMode = nvd.params.misc.appMode;
   
   audio_Config(pinAudio); 
  
#ifdef MAIN_DEBUG    
  Serial.println("To start web configuration mode, press and hold the pgmconfcal button");
  Serial.println("until you hear a low-frequency tone start. Then release the button");
#endif
  for (int cnt = 0; cnt < 4; cnt++) {
#ifdef MAIN_DEBUG    
    Serial.println(4-cnt);
#endif
    delay(1000);
    }
  if (digitalRead(pinPgmConfCalBtn) == 0) {
    bWebConfigure = true;
#ifdef MAIN_DEBUG    
  Serial.println("Web configuration mode selected");
#endif
    // 5 second long tone with low frequency to indicate unit is now in web server configuration mode.
    // You can release the button as soon as the tone starts.
    // After you are done with web configuration, switch off the vario as the wifi radio
    // consumes a lot of power.
    audio_GenerateTone(200, 5000);
    }   

  if (bWebConfigure == true) { 
    setupConfig(); 
    }
  else {
#if 1
    battery_IndicateVoltage();
#else // to calibrate adc-voltage slope/intercept
    while (1){
      int adc = adc_sampleAverage(4);
      Serial.println(adc);
      delay(500);
      }
#endif
    switch (appMode) {
      case APP_MODE_VARIO :
      default :
      setupVario();
      break;

      case APP_MODE_LANTERN :
      setupLantern();
      break;

      case APP_MODE_ALARM :
      setupMotionAlarm();
      break;
      
      case APP_MODE_OTAU :
      setupOTAUpgrade();
      break;
    }
  }

  Tickr.attach_ms(25, btn_debounce);
  btn_clear();
}


void vario_loop() {
  // normal vario loop
  if ( drdyFlag == true ) { // 500Hz ODR => 2mS sample interval
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
    imu_MahonyAHRSupdate6DOF(bUseAccel,imuTimeDeltaUSecs/1000000.0f, DEG_TO_RAD*gyroDps[0], DEG_TO_RAD*gyroDps[1], -DEG_TO_RAD*gyroDps[2], accelmG[1], accelmG[0], accelmG[2]);

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
        // is used in the update stage
        float zAccelAverage = ringbuf_averageNewestSamples(10); 
        float dt = kfTimeDeltaUSecs/1000000.0f;
        kalmanFilter4_predict(dt);
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
        if (sleepTimeoutSecs >= (nvd.params.misc.sleepTimeoutMinutes*60)) {
#ifdef MAIN_DEBUG       
          Serial.println("Timed out with no significant climb/sink, put MPU9250 and ESP8266 to sleep to minimize current draw");
          Serial.flush();
#endif          
          indicateSleep(); 
          goToSleep();
          }   
        }
      }
#ifdef CCT_DEBUG      
    uint32_t elapsedUs =  cct_ElapsedTimeUs(); // calculate time  taken to read and process the data, must be less than 2mS
#endif
    if (drdyCounter >= 50) {
      drdyCounter = 0; // count off 1/10 second intervals
      if (nvd.params.misc.bluetoothRateHz != BLUETOOTH_RATE_DISABLED) {
        bluetoothCounter++;
        if (bluetoothCounter >= (10/nvd.params.misc.bluetoothRateHz)) {
          bluetoothCounter = 0;
          btserial_transmitSentence();
          }
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
        Serial.printf("\r\nY = %d P = %d R = %d\r\n", (int)yaw, (int)pitch, (int)roll);
        Serial.printf("ba = %d ka = %d kv = %d\r\n",(int)baro.zCmSample_, (int)kfAltitudeCm, (int)kfClimbrateCps);
#endif     
#ifdef CCT_DEBUG      
        Serial.printf("Elapsed %dus\r\n", (int)elapsedUs); // ~ 700 uS, so plenty of headroom
#endif
       }
    }
  if (BtnPGCCLongPress == true) {
    appMode = APP_MODE_LANTERN;
    setupLantern();
    delay(500);
    btn_clear();    
    }  
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
  

void alarm_loop() {  
  if ( drdyFlag == true ) { // 500Hz ODR => 2mS sample interval
    drdyFlag = false;
    // accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
    imu.GetAccelGyroData(accelmG, gyroDps); 

    float accelMagnitudeSquared = accelmG[0]*accelmG[0] + accelmG[1]*accelmG[1] + accelmG[2]*accelmG[2];
    if (!bMotionAlarm) {
      float accelThresholdLo = 1000.0f - nvd.params.alarm.accelThresholdmG;
      float accelThresholdHi = 1000.0f + nvd.params.alarm.accelThresholdmG;
      int bAccelAlarm = ((accelMagnitudeSquared < accelThresholdLo*accelThresholdLo) || (accelMagnitudeSquared > accelThresholdHi*accelThresholdHi)) ? 1 : 0;
      int bGyroAlarm = (abs(gyroDps[0]) > (float)nvd.params.alarm.gyroThresholdDps) || (abs(gyroDps[1]) > (float)nvd.params.alarm.gyroThresholdDps) || (abs(gyroDps[2]) > (float)nvd.params.alarm.gyroThresholdDps) ? 1 : 0;
      bMotionAlarm = bAccelAlarm || bGyroAlarm;
      if (bMotionAlarm) {
        audio_SetFrequency(MOTION_ALARM_FREQUENCY_HZ);
        alarmCounter = 0;
        }
      }

    if (drdyCounter >= 500) {
      drdyCounter = 0; // count off 1 second intervals
      if (bMotionAlarm) {
        alarmCounter++;
        audio_SetFrequency(alarmCounter&1 ? MOTION_ALARM_FREQUENCY_HZ*2 : MOTION_ALARM_FREQUENCY_HZ); 
        }
      if (alarmCounter > nvd.params.alarm.durationSecs) {
        bMotionAlarm = 0;
        alarmCounter = 0;
        audio_SetFrequency(0);
        }
      }
    }
}



void loop(){
  if (bWebConfigure == true) {
    wificonfig_HandleClient();
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
      
      case APP_MODE_ALARM :
      alarm_loop();
      break;
      
      case APP_MODE_OTAU :
      httpServer.handleClient();
      break;
		  }
    } 
    
	}
