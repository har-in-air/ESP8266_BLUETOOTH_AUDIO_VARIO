#ifndef UI_H_
#define UI_H_

extern volatile bool BtnPGCCPressed;
extern volatile bool BtnPGCCLongPress;

void indicateUncalibratedAccelerometerGyro();
void indicateSleep();
void indicateFaultMS5611();
void indicateFaultMPU9250();
void battery_IndicateVoltage();
void calibrate_accelerometer(CALIB_PARAMS &calib);
void calibrate_gyro(CALIB_PARAMS &calib);
void calibrate_accel_gyro();
void goToSleep();
void btn_init();
void btn_clear();

#endif
