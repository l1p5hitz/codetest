#ifndef __COMMONUSE_H
#define __COMMONUSE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "UserData.h"


#ifdef COMMONUSE_INIT
#ifdef CONFIG_SECURE_SIGNED_ON_UPDATE
	#pragma message("Secure Boot feature = enable")
#else
	#pragma message("Secure Boot feature = disable")
#endif
#if CONFIG_SECURE_FLASH_ENC_ENABLED
	#pragma message("Flash encryption feature = enable")
#else
	#pragma message("Flash encryption feature = disable")
#endif
#if (defined CONFIG_SECURE_SIGNED_ON_UPDATE) && (CONFIG_SECURE_FLASH_ENC_ENABLED)
	#pragma message("This is secure release (with cyber security)")
#elif (!defined CONFIG_SECURE_SIGNED_ON_UPDATE) && (!CONFIG_SECURE_FLASH_ENC_ENABLED)
	#pragma message("This is legacy release (without cyber security), version must be 0.XX.XX !!!")
#endif
#endif //COMMONUSE_INIT

#if (defined CONFIG_SECURE_SIGNED_ON_UPDATE) && (CONFIG_SECURE_FLASH_ENC_ENABLED)
	#define BROAN_SECURE_RELEASE
#elif (!defined CONFIG_SECURE_SIGNED_ON_UPDATE) && (!CONFIG_SECURE_FLASH_ENC_ENABLED)
	#define BROAN_LEGACY_RELEASE
#else
	#error
#endif



#define MSTR(string) 	STR(string)
#define STR(string)		#string

#define ESP_FW_inByte_Major 2
#define ESP_FW_inByte_Minor 1
#define ESP_FW_inByte_Sub 6
#define ESP_FW_print_Sub_2 "a"
#define ESP_FW_print_Date "16/05/2022"
#define ESP_FW_string MSTR(ESP_FW_inByte_Major) "." MSTR(ESP_FW_inByte_Minor) "." MSTR(ESP_FW_inByte_Sub)

//GPIO configuration
#define STM_BOOT0_PIN 4
#define STM_RST_PIN 5

#define STM_RST_SET 1
#define STM_RST_RESET 0

/* for 031 smart plug only*/
//Output1 = Class II
//Output2 = 120V
#define OUTPUT1_PIN		16
#define OUTPUT2_PIN		17

#define BUTTON1_PIN		19
#define BUTTON2_PIN		18

#define WRITING_STM_FLAG_UP 1
#define WRITE_STM_FLAG_DOWN 0


#define DEBUG_DRIVER_
//#define DEBUG_UART_

#ifdef DEBUG_DRIVER_
  #define DEBUG_DRIVER printf
#else
  #define DEBUG_DRIVER printNull
#endif

#ifdef DEBUG_UART_
  #define DEBUG_UART printf
#else
  #define DEBUG_UART printNull
#endif
    
#define SINGLE_PRESS_TIME 1200//499
#define LONG_PRESS_TIME 1200//500
#define LONG_PRESS_MAX_TIME 3600
#define EXTENDED_PRESS_TIME 5000//1000

#define PWM_MAX 100
#define LED_RUN_FREQ_2p0HZ 2500
#define LED_RUN_FREQ_1p0HZ 5000
#define LED_RUN_FREQ_0p5HZ 10000


//#define TH_TESTING //for TH testing only

//#define DEBUG_REQUEST_BY_D4 //requested by Driven-4 for debugging

#define DEFAULT_DEVICECONFIG_VAL	2
#define DEFAULT_IAQLEDINTENSITY_VAL	5 //range from 1 to 10

#define USE_STM32_CT_BOOTLOADER //new CT 2nd bootloader, ESP32 version must be 2.X.X or above
#ifdef USE_STM32_CT_BOOTLOADER
	#define ALLOW_UPDATE_BOOTLOADER
	#if ESP_FW_inByte_Major < 2
		#error "ESP32 version must be 2.X.X or above"
	#endif
#endif

#if 0
typedef struct
{
  uint32_t co2;
  uint32_t temperature;
  uint32_t humidity;
}SCD30outRawData_;

typedef struct
{
  uint32_t co2;
  uint32_t IAQ;
  uint32_t TVOC;
  uint32_t eCO2;
  uint32_t r_mox;
}ZMOD4410outRawData_;

typedef struct
{
  uint32_t temperature;
  uint32_t humidity;
}Si7021outRawData_;

typedef struct
{
  SCD30outRawData_ SCD30outRawData;
  uint32_t PM2p5_32bit;
  ZMOD4410outRawData_ ZMOD4410outRawData;
  Si7021outRawData_ Si7021outRawData;
}SensorsOutRawData_;
#endif

typedef struct
{
	//float Temperature;
	//float Humidity;
	float RawTemp;
	float RawHumidity;
	float RawNTC1;
	float RawNTC2;
	float RawNTC3; 
}TempSensorRawData_t;

uint8_t Suspend_BleTask;
uint8_t Suspend_ButtonTask;
uint8_t Suspend_SensorTask;
uint8_t Suspend_Uart;
uint8_t Suspend_MsgTask;
volatile uint8_t STM_CT_BOOTLOADER_VER;
bool STM_CT_BOOTLOADER_VER_bValid;
volatile uint8_t STM_FW_VER_Major;
volatile uint8_t STM_FW_VER_Minor;
volatile uint8_t STM_FW_VER_Sub;
bool STM_FW_VER_bValid;
uint8_t STM_FW_MODEL;
bool STM_FW_MODEL_bValid;
#ifdef BROAN_SECURE_RELEASE
bool STM_FW_SIGN_bValid;
bool STM_FW_SIGN_bFail;
uint8_t STM_FW_SIGN_ARR[64];
bool STM_FW_SIGN_bOTAValid;
#endif
bool STM_bPrepareUpdateBootloader;



float BLE_send_Temperature;
float BLE_send_Humidity;
float BLE_send_TVOC;
float BLE_send_CO2;
float BLE_send_eCO2;
float BLE_send_PM2p5;


enum {
	//LEDMODE_DEFAULT_NO_WIFI_BLE = 1,
	LEDMODE_BT_PAIRING_MODE = 2,
	//LEDMODE_BT_PAIR_OK = 3,
	LEDMODE_WIFI_CONNECT_OK = 4,
	//LEDMODE_WIFI_CONNECTED_NOTACTIVE = 5,
	//LEDMODE_WIFI_CONNECTED_ACTIVE = 6,
	LEDMODE_WIFI_DISCONNECTED = 7,
	//LEDMODE_MANUAL_ON = 8,
};
extern uint8_t switchLedMode_031;

//system event
typedef enum {
	SYS_EVENT_WIFI_RESET = 0,
	SYS_EVENT_FACTORY_RESET,
	SYS_EVENT_REBOOT,
	SYS_EVENT_STM32_START,
	NUM_OF_SYS_EVENT,
} sys_event_t;
void system_event_run(sys_event_t event);

//char UpHttpData[100];
//char UpHttpPath[50];


typedef struct {
	size_t offset;
	size_t size;
	uint8_t* data;
} UD_info_t;
int StoreUserData_Multi(UD_info_t* multi_data, uint8_t count);
int StoreUserData(size_t offset, uint8_t *data, size_t size);
void DumpUserData(void);


const char* appCommon_GetVersionString(void);
uint8_t appCommon_GetModel(void);
const char* appCommon_GetModelString(void);
const char* appCommon_GetDeviceID(void);
void appCommon_SetHwVersion(uint8_t version);
uint8_t appCommon_GetHwVersion(void);


void appCommon_SetTestMode(uint8_t value);
uint8_t appCommon_GetTestMode(void);


void appCommon_SetTempSensorRawData(TempSensorRawData_t *pData);
void appCommon_GetTempSensorRawData(TempSensorRawData_t *pData);


uint32_t appCommon_TimeDiff(uint32_t time1, uint32_t time0);
uint32_t appCommon_GetTime_ms(void);
uint32_t appCommon_GetTime_sec(void);
void appCommon_TaskDelay_ms(uint32_t ms);

//flash encryption
void appCommon_CheckFlashEncrypted(void);
bool appCommon_IsFlashEncrypted(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
