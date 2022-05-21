#include "common.h"
#include <FS.h>
#include "LITTLEFS.h"
#include <WiFi.h>              
#include "common.h"
#include "config.h"
#include "drv/cct.h"
#include "drv/wdog.h"
#include "sensor/mpu9250.h"
#include "sensor/gps.h"
#include "sensor/madc.h"
#include "sensor/ringbuf.h"
#include "sensor/imu.h"
#include "sensor/ms5611.h"
#include "sensor/kalmanfilter4d.h"
#include "nv/flashlog.h"
#include "nv/calib.h"
#include "nv/options.h"
#include "wifi/async_server.h"
#include "ui/ui.h"
#include "ui/route.h"

static const char* TAG = "main";

volatile int LedState = 0;
volatile float KFAltitudeCm, KFClimbrateCps,DisplayClimbrateCps;
volatile float YawDeg, PitchDeg, RollDeg;
volatile SemaphoreHandle_t DrdySemaphore;
volatile bool DrdyFlag = false;

int BacklitCounter;
bool IsGpsInitComplete = false;
bool IsServer = false; 
char *buffer;

static void pinConfig();
static void vario_taskConfig();
static void btserial_task(void *pvParameter);
static void ui_task(void *pvParameter);
static void gps_task(void *pvParameter);
static void vario_task(void *pvParameter);
static void main_task(void* pvParameter);
static void IRAM_ATTR drdyHandler(void);

void pinConfig() {	
    // using NS8002 amp module, external 100K pullup resistor to 5V. Pull down to enable.

    pinMode(pinLED, OUTPUT);
    LED_OFF();

    pinMode(pinDRDYINT, INPUT); // external 10K pullup
    }


static void ui_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting ui task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    int counter = 0;
    NAV_PVT navpvt;
    TRACK  track;
    IsGpsFixStable = false;
    IsGpsTrackActive = false;
    EndGpsTrack = false;

    while(1) {
		if (IsGpsNavUpdated) {
			IsGpsNavUpdated = false;
            counter++;
            if (counter >= 5) {
                counter = 0;
                // GPS update interval = 0.1s => display update interval =  0.5s
                memcpy(&navpvt, (void*)&NavPvt, sizeof(NAV_PVT)); 
                ui_updateFlightDisplay(&navpvt,&track);
                }
			}
        if (IsGpsTrackActive && EndGpsTrack) {
            IsGpsTrackActive = false;
            sprintf(buffer,  "%4d/%02d/%02d %02d:%02d", track.year, track.month, track.day, track.hour, track.minute);
            Serial.println(buffer);
            sprintf(buffer, "Duration %02dhr %02dmin", track.elapsedHours, track.elapsedMinutes);
            Serial.println(buffer);
            sprintf(buffer, "Alt St %4dm Mx %4dm", track.startAltm, track.maxAltm);
            Serial.println(buffer);
            sprintf(buffer, "Max Climb +%.1fm/s", track.maxClimbrateCps/100.0f);
            Serial.println(buffer);
            sprintf(buffer, "Max Sink  %.1fm/s", track.maxSinkrateCps/100.0f);
            Serial.println(buffer);
            ui_saveFlightLogSummary(&navpvt, &track);
            while(1) delayMs(100);
            }

		delayMs(5);
		}
    vTaskDelete(NULL);
	}	


static void gps_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting gps task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    if (!gps_config()) {
        ESP_LOGE(TAG, "error configuring gps");
		Serial.println("GPS init error");
        while (1) delayMs(100);
        }
    IsGpsInitComplete = true;
    while(1) {
        gps_stateMachine();
        delayMs(5);
        }
    vTaskDelete(NULL);
    }  


static void IRAM_ATTR drdyHandler(void) {
	DrdyFlag = true;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(DrdySemaphore, &xHigherPriorityTaskWoken);
    if( xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(); // this wakes up vario_task immediately instead of on next FreeRTOS tick
		}
	//LED_TOGGLE();
	}


static void vario_taskConfig() {
    ESP_LOGI(TAG, "vario config");
    if (mpu9250_config() < 0) {
        ESP_LOGE(TAG, "error MPU9250 config");
		Serial.println("MPU9250 config failed");
        while (1) {delayMs(100);};
        }
    // if calib.txt not found, enforce accel and mag calibration and write new calib.txt
    bool isAccelMagCalibRequired = !IsCalibrated;
    int counter = 300;
    while ((!isAccelMagCalibRequired) && counter--) {

   	    sprintf(buffer, "Gyro calib in %ds",(counter+50)/100);
           Serial.println(buffer);
            // manually force accel+mag calibration by pressing BTN0 during gyro calib countdown
           
            //isAccelMagCalibRequired = true; 
            //break;

        delayMs(10);	
		}
    if (isAccelMagCalibRequired) {
        counter = 8;
        while (counter--) {
            sprintf(buffer, "Accel calib in %ds",counter+1);
            Serial.println(buffer);
            delayMs(1000);	
            }
        Serial.println("Calibrating Accel...");
        if (mpu9250_calibrateAccel() < 0) {
 	        Serial.println("Accel calib failed");
            while (1) {delayMs(100);};
            }
        delayMs(1000);
        counter = 5;
        while (counter--) {
            sprintf(buffer, "Mag calib in %ds",counter+1);
            Serial.println(buffer);
            delayMs(1000);	
            }
        Serial.println("Calibrating Mag...");
        if (mpu9250_calibrateMag()  < 0 ) {
 	        Serial.println("Mag calib failed");
            }
        }
    if (isAccelMagCalibRequired) {
        counter = 3;
        while (counter--) {
            sprintf(buffer, "Gyro calib in %ds",counter+1);
            Serial.println(buffer);
            }
        }
    Serial.println("Calibrating Gyro...");
    if (mpu9250_calibrateGyro() < 0) {
 	    Serial.println("Gyro calib fail");
        delayMs(1000);
        }
   // mpu9250_dump_noise_samples();

#if USE_MS5611
    if (ms5611_config() < 0) {
        ESP_LOGE(TAG, "error MS5611 config");
        Serial.println("MS5611 config fail");
        while (1) {delayMs(100);}
        }	
    ms5611_averagedSample(20);
    ESP_LOGD(TAG,"MS5611 Altitude %dm Temperature %dC", (int)(ZCmAvg_MS5611/100.0f), (int)CelsiusSample_MS5611);
    float zcm = ZCmAvg_MS5611;
    ms5611_initializeSampleStateMachine();
#endif

    // KF4D algorithm to fuse gravity-compensated acceleration and pressure altitude to estimate
    // altitude and climb/sink rate
    kalmanFilter4d_configure(1000.0f*(float)opt.kf.accelVariance, KF_ADAPT, zcm, 0.0f, 0.0f);

    sprintf(buffer, "Baro Altitude %dm", (int)(zcm/100.0f));
    Serial.println(buffer);
    // switch to high clock frequency for sensor readout & flash writes
    ringbuf_init(); 
    }


static void vario_task(void *pvParameter) {
    float gxdps, gydps, gzdps, axmG, aymG, azmG, mx, my, mz;
    float gxNEDdps, gyNEDdps, gzNEDdps, axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED;

    ESP_LOGI(TAG, "Starting vario task on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    uint32_t clockPrevious, clockNow; // time markers for imu, baro and kalman filter
    float 	imuTimeDeltaUSecs; // time between imu samples, in microseconds
    float 	kfTimeDeltaUSecs = 0.0f; // time between kalman filter updates, in microseconds

    clockNow = clockPrevious = XTHAL_GET_CCOUNT();
    int drdyCounter = 0;   
    int baroCounter = 0;
    DrdySemaphore = xSemaphoreCreateBinary();
    attachInterrupt(pinDRDYINT, drdyHandler, RISING);

    while (1) {
        xSemaphoreTake(DrdySemaphore, portMAX_DELAY); // wait for data ready interrupt from MPU9250 (500Hz)
        clockNow = XTHAL_GET_CCOUNT();
#ifdef IMU_DEBUG
        uint32_t marker =  cct_setMarker();
        LED_ON();
#endif
        imuTimeDeltaUSecs = cct_intervalUs(clockPrevious, clockNow); // time in us since last sample
        clockPrevious = clockNow;
        drdyCounter++;
        baroCounter++;
        mpu9250_getGyroAccelMagData( &gxdps, &gydps, &gzdps, &axmG, &aymG, &azmG, &mx, &my, &mz);
        // translate from sensor axes to AHRS NED (north-east-down) right handed coordinate frame      
        axNEDmG = -aymG;
        ayNEDmG = -axmG;
        azNEDmG = azmG;
        gxNEDdps = gydps;
        gyNEDdps = gxdps;
        gzNEDdps = -gzdps;
        mxNED =  mx;
        myNED =  my;
        mzNED =  mz;
        // Use accelerometer data for determining the orientation quaternion only when accel 
        // vector magnitude is in [0.75g, 1.25g] window.
        float asqd = axNEDmG*axNEDmG + ayNEDmG*ayNEDmG + azNEDmG*azNEDmG;
        int useAccel = ((asqd > 562500.0f) && (asqd < 1562500.0f)) ? 1 : 0;	
        int useMag = true;
        imu_mahonyAHRSupdate9DOF(useAccel, useMag,((float)imuTimeDeltaUSecs)/1000000.0f, DEG2RAD(gxNEDdps), DEG2RAD(gyNEDdps), DEG2RAD(gzNEDdps), axNEDmG, ayNEDmG, azNEDmG, mxNED, myNED, mzNED);
        imu_quaternion2YawPitchRoll(q0,q1,q2,q3, (float*)&YawDeg, (float*)&PitchDeg, (float*)&RollDeg);
        float gravityCompensatedAccel = imu_gravityCompensatedAccel(axNEDmG, ayNEDmG, azNEDmG, q0, q1, q2, q3);
        ringbuf_addSample(gravityCompensatedAccel);
        kfTimeDeltaUSecs += imuTimeDeltaUSecs;

        if (baroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
            baroCounter = 0;     // alternating between pressure and temperature samples
            int zMeasurementAvailable = ms5611_sampleStateMachine(); 
            // one altitude sample is calculated for a pair of pressure & temperature samples
			if ( zMeasurementAvailable ) { 
                // KF4 uses the acceleration data in the update phase
                float zAccelAverage = ringbuf_averageNewestSamples(10); 
                kalmanFilter4d_predict(kfTimeDeltaUSecs/1000000.0f);
                kalmanFilter4d_update(ZCmSample_MS5611, zAccelAverage, (float*)&KFAltitudeCm, (float*)&KFClimbrateCps);
                kfTimeDeltaUSecs = 0.0f;
                // LCD display shows damped climbrate
                DisplayClimbrateCps = (DisplayClimbrateCps*(float)opt.vario.varioDisplayIIR + KFClimbrateCps*(100.0f - (float)opt.vario.varioDisplayIIR))/100.0f; 
			    if ((opt.misc.logType == LOGTYPE_IBG) && FlashLogMutex) {
			        if ( xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {
                        FlashLogIBGRecord.hdr.baroFlags = 1;
				        FlashLogIBGRecord.baro.heightMSLcm = ZCmSample_MS5611;
					    xSemaphoreGive( FlashLogMutex );
					    }
				    }
                }    
            } 
        if ((opt.misc.logType == LOGTYPE_IBG) && FlashLogMutex) {
		    if (xSemaphoreTake( FlashLogMutex, portMAX_DELAY )) {      
                FlashLogIBGRecord.hdr.magic = FLASHLOG_IBG_MAGIC;
			  	FlashLogIBGRecord.imu.gxNEDdps = gxNEDdps;
			    FlashLogIBGRecord.imu.gyNEDdps = gyNEDdps;
			    FlashLogIBGRecord.imu.gzNEDdps = gzNEDdps;
			    FlashLogIBGRecord.imu.axNEDmG = axNEDmG;
			    FlashLogIBGRecord.imu.ayNEDmG = ayNEDmG;
			    FlashLogIBGRecord.imu.azNEDmG = azNEDmG;
			    FlashLogIBGRecord.imu.mxNED = mxNED;
			    FlashLogIBGRecord.imu.myNED = myNED;
			    FlashLogIBGRecord.imu.mzNED = mzNED;
                // worst case imu+baro+gps record = 80bytes,
                //  ~130uS intra-page, ~210uS across page boundary
                if (IsLoggingIBG) {
                    // out of memory, indicate in UI that logging has stopped
   	                if (flashlog_writeIBGRecord(&FlashLogIBGRecord) < 0) {
                        IsLoggingIBG = false;
                        } 
                    memset(&FlashLogIBGRecord, 0, sizeof(FLASHLOG_IBG_RECORD));
                    }
				xSemaphoreGive( FlashLogMutex );
				}			
	        }  
#ifdef IMU_DEBUG
        uint32_t eus = cct_elapsedUs(marker);
		LED_OFF(); // scope the led on-time to ensure worst case < 2mS
#endif
        if (drdyCounter >= 500) {
            drdyCounter = 0;
#ifdef IMU_DEBUG
            //ESP_LOGD(TAG,"%dus",eus); // need to ensure time elapsed is less than 2mS worst case
			//ESP_LOGD(TAG,"\r\nY = %d P = %d R = %d", (int)YawDeg, (int)PitchDeg, (int)RollDeg);
			//ESP_LOGD(TAG,"ba = %d ka = %d v = %d",(int)ZCmSample, (int)KFAltitudeCm, (int)KFClimbrateCps);

            // ESP_LOGD(TAG,"ax %.1f ay %.1f az %.1f", axmG, aymG, azmG);
            // ESP_LOGD(TAG,"gx %.1f gy %.1f gz %.1f", gxdps, gydps, gzdps);
            // ESP_LOGD(TAG,"mx %.1f my %.1f mz %.1f", mx, my, mz);
            // ESP_LOGD(TAG,"ax %.1f ay %.1f az %.1f", axNEDmG, ayNEDmG, azNEDmG);
            // ESP_LOGD(TAG,"gx %.1f gy %.1f gz %.1f", gxNEDdps, gyNEDdps, gzNEDdps);
            // ESP_LOGD(TAG,"mx %.1f my %.1f mz %.1f", mxNED, myNED, mzNED);

            // ESP_LOGD(TAG,"baro alt %dcm", (int)ZCmSample);
            //lcd_clear_frame();
            //lcd_printf(0,0,"a %d %d %d", (int)axmG, (int)aymG, (int)azmG);
            //lcd_printf(1,0,"g %d %d %d", (int)gxdps, (int)gydps, (int)gzdps);
            //lcd_printf(2,0,"m %d %d %d", (int)mx, (int)my, (int)mz);
            //lcd_printf(0,0,"a %d %d %d", (int)axNEDmG, (int)ayNEDmG, (int)azNEDmG);
            //lcd_printf(1,0,"g %d %d %d", (int)gxNEDdps, (int)gyNEDdps, (int)gzNEDdps);
            //lcd_printf(2,0,"m %d %d %d", (int)mxNED, (int)myNED, (int)mzNED);
            //lcd_send_frame();
#endif
            }
        }
    vTaskDelete(NULL);
    }


static void main_task(void* pvParameter) {
    pinConfig();
    // turn off radio to save power
    WiFi.mode(WIFI_OFF);
    btStop();
    ESP_LOGI(TAG, "Firmware compiled on %s at %s", __DATE__, __TIME__);
    ESP_LOGD(TAG, "Max task priority = %d", configMAX_PRIORITIES-1);
    ESP_LOGD(TAG, "Setup and loop running on core %d with priority %d", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    
    ESP_LOGD(TAG, "Mounting LittleFS ...");
    // do NOT format, partition is built and flashed using PlatformIO Build FileSystem Image + Upload FileSystem Image    
    if (!LITTLEFS.begin(false)) { 
	ESP_LOGE(TAG, "Cannot mount LittleFS, Rebooting");
	delay(1000);
	ESP.restart();
	}    
    //littlefs_directory_listing();
    // read calibration parameters from calib.txt
    calib_init();
    // set default configuration parameters, then override them with configuration parameters read from options.txt
    // so you only have to specify the parameters you want to modify, in the file options.txt
    opt_init();
    // configure DAC sine-wave tone generation
 
    // HSPI bus used for 128x64 LCD display
    
    BacklitCounter = opt.misc.backlitSecs*40;
    adc_init();
    SupplyVoltageV = adc_battery_voltage();
    ESP_LOGD(TAG, "Power = %.1fV", SupplyVoltageV);

    sprintf(buffer, "%s %s", __DATE__, __TIME__);
    Serial.println(buffer);

    sprintf(buffer,"Power : %.1fV", SupplyVoltageV);
    Serial.println(buffer);


    // VSPI bus used for MPU9250, MS5611 and 128Mbit spi flash
    // start with low clock frequency for sensor configuration
    if (flashlog_init() < 0) {
	    ESP_LOGE(TAG, "Spi flash log error");
	    Serial.println("Flash Error");		
	    while (1) {delayMs(100);}
	    }

    //lcd_printlnf(true,3,"Data log : %d %% used", DATALOG_PERCENT_USED()); 
    
    delayMs(2000);
    IsServer = 1;
    ESP_LOGI(TAG,"Press btn0 within 3 seconds to start WiFi AP and web server");
    if (IsServer) { // Wifi Configuration mode
        Serial.println("WiFi Access Point :");
        Serial.println(" \"ESP32GpsVario\"");
        Serial.println("Web Page :");
        Serial.println(" http://esp32.local");
        
        ESP_LOGI(TAG, "Wifi access point ESP32GpsVario starting...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32GpsVario");
        IPAddress myIP = WiFi.softAPIP();
        Serial.println(myIP);
        ESP_LOGI(TAG,"WiFi Access Point IP address: %s", myIP.toString().c_str());
        server_init();
        }
    else { // GPS Vario mode
        ui_screenInit();
        ui_displayOptions();
        while (ui_optionsEventHandler() == 0) {
            delayMs(30);
            }
        if (rte_selectRoute()) {
            int32_t rteDistance = rte_totalDistance();

           sprintf(buffer,"Route %.2fkm", ((float)rteDistance)/1000.0f);
           Serial.println(buffer);
           delayMs(1000);
            IsRouteActive = true;
            }
        vario_taskConfig();   	
        // vario task on core 1 needs to complete all processing within 2mS IMU data sampling period,
        // given highest priority on core 1
	    xTaskCreatePinnedToCore(&vario_task, "variotask", 4096, NULL, configMAX_PRIORITIES-1, NULL, CORE_1);
        IsGpsInitComplete = false;
        // gps task on core 0 given max priority
	    xTaskCreatePinnedToCore(&gps_task, "gpstask", 2048, NULL, configMAX_PRIORITIES-1, NULL, CORE_0);
        while(!IsGpsInitComplete){
            delayMs(10);
            }
        // ui_task on core 0 given lower priority than gps_task
	    xTaskCreatePinnedToCore(&ui_task, "uitask", 4096, NULL, configMAX_PRIORITIES-3, NULL, CORE_0);
      }
    while (1) {
        loop();
        }
    }


// loop runs on core 1 with default priority = 1
void loop() {
    if (IsServer) {
        ESP_LOGV(TAG, "Loop priority = %d", uxTaskPriorityGet(NULL));
        delayMs(500); // delay() is required to yield to other tasks
        return;
        }
    delayMs(25);
    }

// workaround for not having access to 'make menuconfig' to configure the stack size for the
// setup and loop task is to create a new main task with desired stack size, and then delete setup 
// task. 
// Core 0 : ui, GPS, Bluetooth tasks
// Core 1 : setup() + loop(), wifi config / vario task
void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(&main_task, "main_task", 16384, NULL, configMAX_PRIORITIES-4, NULL, CORE_1);
    vTaskDelete(NULL);
    }
