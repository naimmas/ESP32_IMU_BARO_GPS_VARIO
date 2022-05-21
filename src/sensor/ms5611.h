#ifndef MS5611_H_
#define MS5611_H_

#define MS5611_SAMPLE_PERIOD_MS         10

#define MS5611_READ_TEMPERATURE 		11
#define MS5611_READ_PRESSURE			22

#define MS5611_CMD_RESET      	0x1E
#define MS5611_CMD_CONVERT_D1 	0x40
#define MS5611_CMD_CONVERT_D2 	0x50
#define MS5611_CMD_ADC_READ   	0x00
#define MS5611_CMD_ADC_4096 	0x08

#define MS5611_ADDRESS                (0x77)
#define MS5611_CMD_READ_PROM          (0xA2)



void 	ms5611_averagedSample(int numSamples);	
void  	ms5611_calculateTemperatureC(void);
float  	ms5611_calculatePressurePa(void);
void 	ms5611_calculateSensorNoisePa(void);
int 	ms5611_config(void);
int 	ms5611_sampleStateMachine(void);
void 	ms5611_initializeSampleStateMachine(void);
float  	ms5611_pa2Cm(float pa);
void 	ms5611_measure_noise();	
uint8_t ms5611_CRC4(uint8_t prom[] );
void 	ms5611_readPROM(void);
void 	ms5611_getCalibrationParameters(void);
void 	ms5611_reset(void);

uint32_t ms5611_readSample(void);
void ms5611_triggerTemperatureSample();
void ms5611_triggerPressureSample();

uint32_t readRegister24(uint8_t reg);
uint16_t readRegister16(uint8_t reg);

extern	float ZCmAvg_MS5611;
extern	float ZCmSample_MS5611;
extern	float PaSample_MS5611;
extern	int   CelsiusSample_MS5611;

#endif // MS5611_H_
