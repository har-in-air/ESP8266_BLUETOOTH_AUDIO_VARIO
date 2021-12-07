#ifndef CONFIG_H_
#define CONFIG_H_

////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   	20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   	100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	10
#define VARIO_ZERO_THRESHOLD_CPS_MIN    	-20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    	20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	-250
#define VARIO_SINK_THRESHOLD_CPS_MIN    	-400
#define VARIO_SINK_THRESHOLD_CPS_MAX    	-100


// When generating climbtones, the vario allocates most of the speaker 
// frequency bandwidth to climbrates below this crossover threshold 
// so you have more frequency discrimination. So set the crossover threshold 
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT     400
#define VARIO_CROSSOVER_CPS_MIN         300
#define VARIO_CROSSOVER_CPS_MAX         800

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     100
#define KF_ACCEL_VARIANCE_MIN         50
#define KF_ACCEL_VARIANCE_MAX         150

#define KF_ZMEAS_VARIANCE_DEFAULT    200
#define KF_ZMEAS_VARIANCE_MIN        100
#define KF_ZMEAS_VARIANCE_MAX        500

// Sleep timeout. The vario will go into sleep mode
// if it does not detect climb or sink rates more than
// SLEEP_THRESHOLD_CPS, for the specified minutes.
// You can only exit  sleep mode by power cycling the unit.
#define SLEEP_TIMEOUT_MINUTES_DEFAULT   15
#define SLEEP_TIMEOUT_MINUTES_MIN       5
#define SLEEP_TIMEOUT_MINUTES_MAX       30

// audio feedback tones
#define BATTERY_TONE_HZ       400
#define CALIBRATING_TONE_HZ   800
#define UNCALIBRATED_TONE_HZ  2000
#define MPU9250_ERROR_TONE_HZ   200 
#define MS5611_ERROR_TONE_HZ    2500
#define ALARM_COUNTDOWN_HZ      400

// if you find that gyro calibration fails when you leave
// the unit undisturbed, possibly your unit has an MPU9250 device
// with a larger gyro bias. In that case try increasing this
// limit  until you find the calibration works consistently.

#define GYRO_OFFSET_LIMIT_1000DPS_DEFAULT   	50
#define GYRO_OFFSET_LIMIT_1000DPS_MIN       	25
#define GYRO_OFFSET_LIMIT_1000DPS_MAX		    200

#define BLUETOOTH_RATE_DISABLED   0
#define BLUETOOTH_RATE_5HZ        5
#define BLUETOOTH_RATE_10HZ       10

#define BLUETOOTH_RATE_DEFAULT  BLUETOOTH_RATE_DISABLED
#define BLUETOOTH_RATE_MIN  BLUETOOTH_RATE_DISABLED
#define BLUETOOTH_RATE_MAX  BLUETOOTH_RATE_10HZ


#define APP_MODE_VARIO    0
#define APP_MODE_LANTERN  1
#define APP_MODE_ALARM    2
#define APP_MODE_OTAU     3

#define APP_MODE_DEFAULT  APP_MODE_VARIO
#define APP_MODE_MIN      APP_MODE_VARIO
#define APP_MODE_MAX      APP_MODE_OTAU

///////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// Three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates 
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

// uncomment this if you want the current beep/tone to be interrupted and
// a new tone generated when there is a 'significant' change in climb/sink rate
// this will give you faster apparent vario response, but could also be 
// confusing/irritating if you are in choppy air
//#define VARIO_INTERRUPT_BEEPS

// this is the 'significant change' threshold that is used when 
// VARIO_INTERRUPT_BEEPS is enabled
#define VARIO_DISCRIMINATION_THRESHOLD_CPS    25

// This is set low as the residual acceleration bias after calibration
// is expected to have little variation/drift
#define KF_ACCELBIAS_VARIANCE   0.005f

// KF4 Acceleration Update variance default
#define KF_ACCEL_UPDATE_VARIANCE   50.0f


#define SLEEP_THRESHOLD_CPS    50

// print debug information to the serial port for different code modules

// these #defines can be left uncommented after debugging, as the enclosed
// debug prints do not appear in the critical run-time loop
#define MAIN_DEBUG
#define KF_DEBUG
#define VARIO_DEBUG
#define NVD_DEBUG
#define MPU9250_DEBUG
#define MS5611_DEBUG
#define WEBCFG_DEBUG

// !! ensure these #defines are commented out after debugging, as the 
// enclosed debug prints are in the critical run-time loop.
//#define IMU_DEBUG
//#define CCT_DEBUG


#define LANTERN_DIM   20
#define LANTERN_LOW   50
#define LANTERN_MID   200
#define LANTERN_HI   1023


#define MOTION_ALARM_ACCEL_THRESHOLD_MIN      300
#define MOTION_ALARM_ACCEL_THRESHOLD_DEFAULT  500
#define MOTION_ALARM_ACCEL_THRESHOLD_MAX      900


#define MOTION_ALARM_GYRO_THRESHOLD_MIN       10.0f   // 10deg/sec
#define MOTION_ALARM_GYRO_THRESHOLD_DEFAULT   20.0f   // 10deg/sec
#define MOTION_ALARM_GYRO_THRESHOLD_MAX       50.0f   // 10deg/sec

#define MOTION_ALARM_FREQUENCY_HZ 2000  // set this to the resonant frequency of speaker for loudest volume

#define MOTION_ALARM_DURATION_SECS_MIN      10
#define MOTION_ALARM_DURATION_SECS_DEFAULT  30
#define MOTION_ALARM_DURATION_SECS_MAX      60

#endif
