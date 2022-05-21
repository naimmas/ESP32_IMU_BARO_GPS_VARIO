#include "common.h"
#include "config.h"
#include "drv/cct.h"
#include "sensor/ms5611.h"
#include <Wire.h>

float ZCmAvg_MS5611;
float ZCmSample_MS5611;
float PaSample_MS5611;
int   CelsiusSample_MS5611;

static uint16_t Cal_[6];
static int64_t  Tref_;
static int64_t  OffT1_;
static int64_t  SensT1_;	
static int32_t  TempCx100_;
static uint32_t D1_;
static uint32_t D2_;
static int64_t  DT_;	
static int      SensorState_;

static const char* TAG = "ms5611";


int ms5611_config(void) {
	MS5611_CS_HI();
	PaSample_MS5611 = 0.0f;
	ZCmSample_MS5611 = 0.0f;
	CelsiusSample_MS5611 = 0;
	ZCmAvg_MS5611 = 0.0f;
	//ms5611_reset();
	ms5611_readPROM();
	ms5611_getCalibrationParameters();
#if MS5611_MEASURE_NOISE	
	ms5611_measure_noise();
#endif	
	return 0;
	}


void ms5611_initializeSampleStateMachine(void) {
   ms5611_triggerTemperatureSample();
   SensorState_ = MS5611_READ_TEMPERATURE;
   }

int ms5611_sampleStateMachine(void) {
   	if (SensorState_ == MS5611_READ_TEMPERATURE) {
      	D2_ = ms5611_readSample();
      	ms5611_triggerPressureSample();
      	ms5611_calculateTemperatureC();
	   	SensorState_ = MS5611_READ_PRESSURE;
      	return 0;  // no altitude sample available
      	}
   	else
   	if (SensorState_ == MS5611_READ_PRESSURE) {
      	D1_ = ms5611_readSample();
      	PaSample_MS5611 = ms5611_calculatePressurePa();
	   	ZCmSample_MS5611 = ms5611_pa2Cm(PaSample_MS5611);
      	ms5611_triggerTemperatureSample();
      	SensorState_ = MS5611_READ_TEMPERATURE;
      	return 1; // new altitude sample is available
      	}
   	return 0;    
   	}


#if MS5611_MEASURE_NOISE
static float pa_to_zcm(float pa) {
    return 4430769.396f * (1.0f - pow(pa/101325.0f, 0.190284f));
    }

static float pa[NUM_TEST_SAMPLES];
static float zcm[NUM_TEST_SAMPLES];

void ms5611_measure_noise() {
    float paMean, zcmMean, zcmVariance, paVariance;
    paMean = 0.0f;
    zcmMean = 0.0f;
    paVariance = 0.0f;
    zcmVariance = 0.0f;
	// get dummy samples for a couple of seconds first
    ESP_LOGD(TAG,"MS5611 estimate sensor noise");
	ms5611_triggerTemperatureSample();
    for (int n = 0; n < 100; n++) {
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ms5611_readSample();
		ms5611_triggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ms5611_readSample();
    	ms5611_triggerTemperatureSample();
		}
    for (int n = 0; n < NUM_TEST_SAMPLES; n++) {
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ms5611_readSample();
		ms5611_calculateTemperatureC();
		ms5611_triggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ms5611_readSample();
		pa[n] = ms5611_calculatePressurePa();
    	ms5611_triggerTemperatureSample();
        //z[n] =  ms5611_pa2Cm(pa[n]);
        zcm[n] =  pa_to_zcm(pa[n]);
        paMean += pa[n];
        zcmMean += zcm[n];
        }
    paMean /= NUM_TEST_SAMPLES;
    zcmMean /= NUM_TEST_SAMPLES;
    ESP_LOGD(TAG,"pa_mean = %f Pa,  z_mean = %f cm",paMean, zcmMean);
    for (int n = 0; n < NUM_TEST_SAMPLES; n++) {
        //ESP_LOGD(TAG, "%f %f",pa[n]-paMean,zcm[n]-zcmMean);		
		printf("%f %f\n",pa[n],zcm[n]);		
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zcmVariance += (zcm[n]-zcmMean)*(zcm[n]-zcmMean);
       }
    paVariance /= (NUM_TEST_SAMPLES-1);
    zcmVariance /= (NUM_TEST_SAMPLES-1);
    ESP_LOGD(TAG,"pa_variance %f Pa^2  z_variance %f cm^2",paVariance, zcmVariance);    
	}
#endif


void ms5611_averagedSample(int numSamples) {
	int32_t tc,tAccum,n;
   	float pa,pAccum;
	pAccum = 0.0f;
   	tAccum = 0;
	n = numSamples;
   	while (n--) {
		ms5611_triggerTemperatureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D2_ = ms5611_readSample();
		ms5611_calculateTemperatureC();
		ms5611_triggerPressureSample();
		delayMs(MS5611_SAMPLE_PERIOD_MS);
		D1_ = ms5611_readSample();
		pa = ms5611_calculatePressurePa();
		pAccum += pa;
		tAccum += TempCx100_;
		}
	tc = tAccum/numSamples;
	CelsiusSample_MS5611 = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	PaSample_MS5611 = (pAccum + numSamples/2) / numSamples;
	ZCmAvg_MS5611 = ZCmSample_MS5611 = ms5611_pa2Cm(PaSample_MS5611);
	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.txt"

float ms5611_pa2Cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)paf;

   	if (pa > PA_INIT) {
      	zf = (float)(PZTable[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(PZTable[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = PZTable[inx];
         	z2 = PZTable[inx+1];
         	zf = (float)z1 + ((((float)pa1)-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}

uint32_t readRawTemperature(void)
{
    Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_CONVERT_D2 | MS5611_CMD_ADC_4096);
    #else
	Wire.send(MS5611_CMD_CONVERT_D2 | MS5611_CMD_ADC_4096);
    #endif

    Wire.endTransmission();

    delay(MS5611_SAMPLE_PERIOD_MS);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t readRawPressure(void)
{
    Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_CONVERT_D1 | MS5611_CMD_ADC_4096);
    #else
	Wire.send(MS5611_CMD_CONVERT_D1 | S5611_CMD_ADC_4096);
    #endif

    Wire.endTransmission();

    delay(MS5611_SAMPLE_PERIOD_MS);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t readRegister24(uint8_t reg)
{
    uint32_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(!Wire.available()) {};
#if ARDUINO >= 100
        uint8_t vxa = Wire.read();
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vxa = Wire.receive();
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
#endif
    Wire.endTransmission();

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}

void ms5611_calculateTemperatureC(void) {
	DT_ = (int64_t)D2_ - Tref_;
	TempCx100_ = 2000 + ((DT_*((int32_t)Cal_[5]))>>23);
   	CelsiusSample_MS5611 = (TempCx100_ >= 0? (TempCx100_+50)/100 : (TempCx100_-50)/100);
	}


float ms5611_calculatePressurePa(void) {
   float pa;
   int64_t offset, sens,offset2,sens2,t2;
   offset = OffT1_ + ((((int64_t)Cal_[3])*(int64_t)DT_)>>7);
   sens = SensT1_ + ((((int64_t)Cal_[2])*(int64_t)DT_)>>8);
   if (TempCx100_ < 2000) {
      t2 = ((DT_*DT_)>>31); 
      offset2 = (5*(TempCx100_-2000)*(TempCx100_-2000))/2;
      sens2 = offset2/2;
      } 
   else {
      t2 = 0;
      sens2 = 0;
      offset2 = 0;
      }
   TempCx100_ -= t2;
   offset -= offset2;
   sens -= sens2;
   pa = ((float)((int64_t)D1_ * sens)/2097152.0f - (float)(offset)) / 32768.0f;
   return pa;
   }



void ms5611_reset(void) {
	Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_RESET);
    #else
	Wire.send(MS5611_CMD_RESET);
    #endif

    Wire.endTransmission();
	delay(4);
   }

   	
void ms5611_getCalibrationParameters(void)  {
   ESP_LOGI(TAG,"Calib Coeffs : %d %d %d %d %d %d",Cal_[0],Cal_[1],Cal_[2],Cal_[3],Cal_[4],Cal_[5]);
   Tref_ = ((int64_t)Cal_[4])<<8;
   OffT1_ = ((int64_t)Cal_[1])<<16;
   SensT1_ = ((int64_t)Cal_[0])<<15;		
   }
   

void ms5611_readPROM(void)    {
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	Cal_[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
   }
	
uint16_t readRegister16(uint8_t reg)
{
    uint16_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}
	
uint8_t ms5611_CRC4(uint8_t prom[] ) {
   int cnt, nbit; 
   uint16_t crcRemainder; 
   uint8_t crcSave = prom[15]; // crc byte in PROM
   ESP_LOGD(TAG,"PROM CRC = 0x%x", prom[15] & 0x0F);
   crcRemainder = 0x0000;
   prom[15] = 0; //CRC byte is replaced by 0

   for (cnt = 0; cnt < 16; cnt++)  {
		crcRemainder ^= (uint16_t) prom[cnt];
		for (nbit = 8; nbit > 0; nbit--) {
			if (crcRemainder & (0x8000)) {
				crcRemainder = (crcRemainder << 1) ^ 0x3000; 
				}
			else {
				crcRemainder = (crcRemainder << 1);
				}
			}
		}
   crcRemainder= (0x000F & (crcRemainder >> 12)); // final 4-bit reminder is CRC code
   prom[15] = crcSave; // restore the crc byte
   ESP_LOGD(TAG, "Calculated CRC = 0x%x",  crcRemainder ^ 0x0);
   return (uint8_t)(crcRemainder ^ 0x0);
   } 


