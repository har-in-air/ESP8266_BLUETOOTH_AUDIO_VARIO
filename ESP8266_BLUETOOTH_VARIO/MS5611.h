#ifndef MS5611_H_
#define MS5611_H_

//#define MS5611_TEST

// max conversion time with OSR=4096 is  9.04mS
#define MS5611_SAMPLE_PERIOD_MS         10

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22


#define MS5611_I2C_ADDRESS 0x77

#define MS5611_RESET      	0x1E
#define MS5611_CONVERT_D1 	0x40
#define MS5611_CONVERT_D2 	0x50
#define MS5611_ADC_READ   	0x00

#define MS5611_ADC_4096 	0x08

class MS5611 {
	
public :
MS5611();
void TriggerPressureSample(void);
void TriggerTemperatureSample(void);
uint32_t  ReadSample(void);
void AveragedSample(int nSamples);
void CalculateTemperatureCx10(void);
float CalculatePressurePa(void);
void CalculateSensorNoisePa(void);
void Reset(void);

void GetCalibrationCoefficients(void);
float Pa2Cm(float pa);
void Test(int nSamples);
int  ReadPROM(void);
uint8_t CRC4(uint8_t prom[] );
int  SampleStateMachine(void);
void InitializeSampleStateMachine(void);

volatile float paSample_;
volatile float zCmSample_;
float zCmAvg_;
int  celsiusSample_;
volatile int sensorState;

private :
uint8_t prom_[16];
uint16_t cal_[6];
int64_t tref_;
int64_t offT1_;
int64_t sensT1_;
int32_t tempCx100_;
uint32_t D1_;
uint32_t D2_;
int64_t dT_;
};

#endif // MS5611_H_
