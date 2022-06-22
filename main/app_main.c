#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/apps/sntp.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "UART_OS.h"
#include "cJSON.h"
#include "utils.h"

#include "CommonUse.h"

#include "driver/ledc.h"//2019.11.25

// #include "ZMOD4410/continuous.h"

// #include "driver/i2c.h"

#include "Usart_BootMode.h"
#include "BLE_gatts_demo.h"
#include "ota.h"
#include "CertData.h"
#include "app_storage.h"
#include "app_wifi.h"
#include "app_mqtt.h"

#include "app_main.h"

#define APP_TAG		"APP"
#define WIFI_TAG	"WIFI"
#define NTP_TAG		"NTP"
#define MQTT_TAG	"MQTT"
#define UDATA_TAG	"UDATA"

uint8_t wifi_isSet = 0;

#define MAX_WIFI  2
char WIFI_SSID[MAX_WIFI][UD_WIFI_SSID_FIELD_LEN] = {"dummy_SSID", "dummy_SSID"};
char WIFI_PASS[MAX_WIFI][UD_WIFI_PW_FIELD_LEN] = {"password", "password"};

char strftime_buf[64];

esp_mqtt_client_handle_t client = NULL;
char MQTT_BROKER_URL[MQTT_BROKER_URL_LEN] = "mqtts://iotqa.broan-nutone.com:61714";
char MQTT_BROKER_URL_TestMode[MQTT_BROKER_URL_LEN] = "mqtts://iotqa.broan-nutone.com:61714";
//char MQTT_BROKER_URL[100] = "mqtt://13.235.135.44:1883";	//Broan MQTT
//char MQTT_BROKER_URL[100] = "mqtt://192.168.3.2:1883";
//#define MQTT_BROKER_URL "mqtt://13.235.100.251:1883"
// #define MQTT_BROKER_URL "mqtt://13.235.135.44:1883"//

char cPayload[1024];
float lastSensorSentVal[MAX_SENSOR];
char HW_ID[UD_DEV_ID_FIELD_LEN] = "SS123456";
#define TOPIC_HEADER "Broan-"

extern void BLE_Task(void *pvParameters);

//{"CfmRate":12,"ActiveMode":45,"Temperature":32.33,"Humidity":60.5,"TVOC":2.3,"CO2":23.33,"FanRealState":0}
/*
#define USER_ID		"0000"
#define DEVICE_ID	"0000"
#define PTOPIC_SENSOR0 "Broan/" USER_ID "/" DEVICE_ID "/Sensors/Temperature"
#define PTOPIC_SENSOR1 "Broan/" USER_ID "/" DEVICE_ID "/Sensors/Humidity"
#define PTOPIC_SENSOR2 "Broan/" USER_ID "/" DEVICE_ID "/Sensors/VoC"
#define PTOPIC_SENSOR3 "Broan/" USER_ID "/" DEVICE_ID "/Sensors/CO2"
#define PTOPIC_SENSOR4 "Broan/" USER_ID "/" DEVICE_ID "/Sensors/PM2.5"
#define PTOPIC_CONFIG0 "Broan/" USER_ID "/" DEVICE_ID "/Configuration/Firmware"

#define STOPIC_SYSTEM0 "Broan/" USER_ID "/" DEVICE_ID "/Configuration/Connected Products"
#define STOPIC_SYSTEM1 "Broan/" USER_ID "/" DEVICE_ID "/Configuration/Behavior"
#define STOPIC_SYSTEM2 "Broan/" USER_ID "/" DEVICE_ID "/Configuration/User Settings"
#define STOPIC_SYSTEM3 "Broan/" USER_ID "/" DEVICE_ID "/Configuration/Server"
*/


//struct tm sensorLastUpdateTime[MAX_SENSOR];
//struct tm configLastUpdateTime[MAX_CONFIG];


//button mode since use IO 18 and 19 which collision with I2C 
// #define BUTTON_ONLY	//for BRNC031
unsigned long But1PressedCnt = 0;
unsigned long But2PressedCnt = 0;
unsigned long DualBtnCnt = 0;
uint8_t Btn1Status_curr = 1;
uint8_t Btn1Status_B4 = 1;
uint8_t Btn2Status_curr = 1;
uint8_t Btn2Status_B4 = 1;
uint8_t Btn1_detectFall_Cnt = 0;
uint8_t Btn2_detectFall_Cnt = 0;
uint8_t DualBtn_Detected = 0;

unsigned char firmwareVersion = 0x11;
// int getNumber = 0;//2019.12.11
//011
uint8_t IAQindex = 0;
uint8_t IAQcolor = 200;
uint8_t LEDanimation = 0;
uint8_t Bttn1AppState = 0;

//021
uint8_t FanAlgoState = 0;
uint8_t Bttn2AppState = 0;
uint8_t FanAppState = 0;
uint8_t SwitchConfig = DEFAULT_DEVICECONFIG_VAL;


uint8_t IAQLEDIntensity = DEFAULT_IAQLEDINTENSITY_VAL;

uint8_t STM_bootloader_test = 0;

//uint8_t WifiBT_Statu2STM_Cnt = 0;

static bool bForcedMQTTSendFWver = false;
static bool Debug_bMQTTRaw = false;
bool Debug_bAllowUpdateSTMBootLoader = false;
static bool Debug_bSTMBootLoaderTest = false;

TaskHandle_t xHandle_BleTask;
TaskHandle_t xHandle_ButtonTask;
TaskHandle_t xHandle_SensorTask;
TaskHandle_t xHandle_Uart;

void output_1_SetLevel(bool bOn);
void output_1_ToggleLevel(void);
void output_2_SetLevel(bool bOn);
void output_2_ToggleLevel(void);

void LED_1_On(void);
void LED_1_Off(void);
void LED_1_FadeUp(uint16_t FadeTime);//ms
void LED_1_FadeDown(uint16_t FadeTime);//ms
void LED_2_On(void);
void LED_2_Off(void);
void LED_2_FadeUp(uint16_t FadeTime);//ms
void LED_2_FadeDown(uint16_t FadeTime);//ms
//--------------------------------------------


static void statusResponseRecord_addRecord(void);
static void statusResponseRecord_clearAll(void);


static void mqtt_setPublishIgnore(void);
static bool mqtt_isPublishIgnore(void);
static void mqtt_setForcedStausUpdate(bool bUpdate);
static bool mqtt_isForcedStausUpdate(void);


bool isJSONValidNumber(cJSON* messageType)
{
	bool ret = true;
	char* p;
	if (!cJSON_GetStringValue(messageType)) {
		return false;
	}
	if(*(messageType->valuestring)=='\0') {
		return false;
	}
	p = messageType->valuestring;
	while (*p != '\0') {
		if (!isdigit(*p) && *p!='.') {
			ret = false;
			break;
		}
		p++;
	}
	return ret;
}

bool isJSONValidString(cJSON* messageType)
{
	if (!cJSON_GetStringValue(messageType)) {
		return false;
	}
	if(*(messageType->valuestring)=='\0') {
		return false;
	}
	return true;
}


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    //esp_mqtt_client_handle_t thisClient = event->client;
	cJSON* root;
    //int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
			mqtt_SetConnected(true);
			
            //msg_id = esp_mqtt_client_publish(thisClient, "/topic/qos1", "data_3", 0, 1, 0);
            ////ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_subscribe(thisClient, "/topic/qos0", 0);
            ////ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_subscribe(thisClient, "/topic/qos1", 1);
            ////ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);

            //msg_id = esp_mqtt_client_unsubscribe(thisClient, "/topic/qos1");
            ////ESP_LOGI(MQTT_TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
			
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
			mqtt_SetConnected(false);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            //ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(thisClient, "/topic/qos0", "data", 0, 0, 0);
            ////ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
			
        case MQTT_EVENT_UNSUBSCRIBED:
            //ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
			
        case MQTT_EVENT_PUBLISHED:
            //ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
			
        case MQTT_EVENT_DATA:
			ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
			ESP_LOGI(MQTT_TAG, "TOPIC=%.*s", event->topic_len, event->topic);
			ESP_LOGI(MQTT_TAG, "DATA=%.*s", event->data_len, event->data);
			
			if (strncmp(event->topic, TOPIC_HEADER, strlen(TOPIC_HEADER)) != 0)
			{
				//prefix not match
				break;
			}
			root = cJSON_Parse(event->data);
			if (root == NULL)
			{
				//not JASON format
				break;
			}

			//cJSON* root = cJSON_Parse("{\"DeviceId\":\"SS123456\",\"NewData\":\"True\",\"FanRealState\":\"1\"}");
			//printf("....%s\n", event->data);
			//printf("....%s\n", "{\"DeviceId\":\"SS123456\",\"NewData\":\"True\",\"FanRealState\":\"1\"}");
			if (strncmp(&event->topic[event->topic_len-2], "CC", 2) == 0)
			{
				bool bControlCmd = false;
				//------------------------------------------2019.12.11
				cJSON* messageType;
				int message_value;
				switch(firmwareVersion)
				{
					case 0x11://Room Sesor
						/*
							{"DeviceId":"RS20X0123",
							"IAQindex":"32",
							"IAQcolor":"1", 
							"LEDanimation":"0",
							"NewAppCMD":"1",
							"Bttn1AppState":"0",
							"TimeStamp":"119|9|29|00|00|00"}
						*/
						{
							switchCommand = 0;//clear
							//---------------------
							messageType = cJSON_GetObjectItem(root, "IAQindex");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 100)
								{
									IAQindex = message_value;
									//printf("IAQindex = %d\n", IAQindex);
									switchCommand |= (1UL << SW_CMD_IAQ_INDEX);
								}
							}
							//---------------------
							messageType = cJSON_GetObjectItem(root, "IAQcolor");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 2)
								{
									IAQcolor = message_value;
									//printf("IAQcolor = %d\n", IAQcolor);
									switchCommand |= (1UL << SW_CMD_IAQ_COLOR);
								}
							}
							//---------------------
							messageType = cJSON_GetObjectItem(root, "LEDanimation");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 4)
								{
									LEDanimation = message_value;
									// printf("LEDanimation = %d\n", LEDanimation);
									switchCommand |= (1UL << SW_CMD_LED_ANIMATION);
								}
							}
							//---------------------
							uint8_t NewAppCMD = 0;
							messageType = cJSON_GetObjectItem(root, "NewAppCMD");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 1)
								{
									NewAppCMD = message_value;
								}
							}
							if (NewAppCMD == 1)
							{
								messageType = cJSON_GetObjectItem(root, "Bttn1AppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= 1)
									{
										Bttn1AppState = message_value;
										// printf("Bttn1AppState = %d\n", Bttn1AppState);
										switchCommand |= (1UL << SW_CMD_BTTN1_APP_STATE);
										bControlCmd = true;
									}
								}
							}
							// printf("switchCommand = 0x%lx\n", switchCommand);
							//--------------------------2019.12.17
							if(switchCommand)
								switchCommand_Repeat = SW_CMD_REPEAT_CNT;
							//--------------------------
						}break;
					
					case 0x21://Wall Control
						/*
							{"DeviceId":"WC20X0123",
							"IAQindex":"32",
							"IAQcolor":"1",
							"LEDanimation":"0",
							"FanAlgoState":"1", 
							"NewAppCMD":"0", 
							"FanAppState":"0", 
							"Bttn1AppState":"0", 
							"Bttn2AppState":"0", 
							"TimeStamp":"119|9|29|00|00|00""}
						*/
						{
							switchCommand = 0;//clear
							//---------------------
							messageType = cJSON_GetObjectItem(root, "IAQindex");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 100)
								{
									IAQindex = message_value;
									// printf("IAQindex = %d\n", IAQindex);
									switchCommand |= (1UL << SW_CMD_IAQ_INDEX);
								}
							}
							//---------------------
							messageType = cJSON_GetObjectItem(root, "IAQcolor");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 2)
								{
									IAQcolor = message_value;
									// printf("IAQcolor = %d\n", IAQcolor);
									switchCommand |= (1UL << SW_CMD_IAQ_COLOR);
								}
							}
							//---------------------
							messageType = cJSON_GetObjectItem(root, "LEDanimation");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 4)
								{
									LEDanimation = message_value;
									// printf("LEDanimation = %d\n", LEDanimation);
									switchCommand |= (1UL << SW_CMD_LED_ANIMATION);
								}
							}
							//---------------------
							uint8_t NewAppCMD = 0;
							messageType = cJSON_GetObjectItem(root, "NewAppCMD");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 1)
								{
									NewAppCMD = message_value;
								}
							}
							if (NewAppCMD == 1)
							{
								messageType = cJSON_GetObjectItem(root, "FanAppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= UINT8_MAX)
									{
										FanAppState = message_value;
										//printf("FanAppState = %d\n", FanAppState);
										switchCommand |= (1UL << SW_CMD_FAN_APP_STATE);
										bControlCmd = true;
									}
								}
								//---------------------
								messageType = cJSON_GetObjectItem(root, "Bttn1AppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= 1)
									{
										Bttn1AppState = message_value;
										// printf("Bttn1AppState = %d\n", Bttn1AppState);
										switchCommand |= (1UL << SW_CMD_BTTN1_APP_STATE);
										bControlCmd = true;
									}
								}
								//---------------------
								messageType = cJSON_GetObjectItem(root, "Bttn2AppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= 1)
									{
										Bttn2AppState = message_value;
										// printf("Bttn2AppState = %d\n", Bttn2AppState);
										switchCommand |= (1UL << SW_CMD_BTTN2_APP_STATE);
										bControlCmd = true;
									}
								}
							}
							else //no NewAppCMD
							{
								messageType = cJSON_GetObjectItem(root, "FanAlgoState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= UINT8_MAX)
									{
										FanAlgoState = message_value;
										//printf("FanAlgoState = %d\n", FanAlgoState);
										switchCommand |= (1UL << SW_CMD_FAN_ALGO_STATE);
										//bControlCmd = true;
									}
								}
							}
							// printf("switchCommand = 0x%lx\n", switchCommand);							
							//--------------------------2019.12.17
							if(switchCommand)
								switchCommand_Repeat = SW_CMD_REPEAT_CNT;
							//--------------------------
						}break;
					
					case 0x31://Smart Plug ,no UART .
						/*
							{"DeviceId":"SP20X0123",
							"FanAlgoState":"1", 
							"NewAppCMD":"1", 
							"FanAppState":"0", 
							"Bttn1AppState":"0", 
							"Bttn2AppState":"0", 
							"TimeStamp":"119|9|29|00|00|00"}
						*/
						{
							uint8_t NewAppCMD = 0;
							messageType = cJSON_GetObjectItem(root, "NewAppCMD");
							if (isJSONValidNumber(messageType))
							{
								message_value = atoi(messageType->valuestring);
								if (message_value >= 0 && message_value <= 1)
								{
									NewAppCMD = message_value;
								}
							}
							if (NewAppCMD == 1)
							{
								bool bChangeOutput = false;
								messageType = cJSON_GetObjectItem(root, "FanAppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= UINT8_MAX)
									{
										FanAppState = message_value;
										printf("FanAppState = %d\n", FanAppState);
										if (SwitchConfig == 1 || SwitchConfig == 2 || SwitchConfig == 4 || SwitchConfig == 5)
										{
											if (FanAppState == 0 || FanAppState == 1) {
												output_1_SetLevel(FanAppState);
												bChangeOutput = true;
											}
										}
										else if (SwitchConfig == 3)
										{
											if (FanAppState == 0 || FanAppState == 1) {
												output_2_SetLevel(FanAppState);
												bChangeOutput = true;
											}
										}
										bControlCmd = true;
									}
								}
								messageType = cJSON_GetObjectItem(root, "Bttn1AppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= 1)
									{
										Bttn1AppState = message_value;
										printf("Bttn1AppState = %d\n", Bttn1AppState);
										//no function for Bttn1AppState...
										//bControlCmd = true;
									}
								}
								messageType = cJSON_GetObjectItem(root, "Bttn2AppState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= 1)
									{
										Bttn2AppState = message_value;
										printf("Bttn2AppState = %d\n", Bttn2AppState);
										if (SwitchConfig == 2 || SwitchConfig == 5)
										{
											if (Bttn2AppState == 0 || Bttn2AppState == 1) {
												output_2_SetLevel(Bttn2AppState);
												bChangeOutput = true;
											}
										}
										bControlCmd = true;
									}
								}
								if (bChangeOutput)
								{
									device_updateResult_031();
								}
							}
							else //no NewAppCMD
							{
								bool bChangeOutput = false;
								messageType = cJSON_GetObjectItem(root, "FanAlgoState");
								if (isJSONValidNumber(messageType))
								{
									message_value = atoi(messageType->valuestring);
									if (message_value >= 0 && message_value <= UINT8_MAX)
									{
										uint8_t state = message_value;
										printf("FanAlgoState = %d\n", state);
										if (SwitchConfig == 1 || SwitchConfig == 2 || SwitchConfig == 4 || SwitchConfig == 5)
										{
											if (state == 0 || state == 1) {
												if(!MOverride) {
													output_1_SetLevel(state);
												}
												bChangeOutput = true;
											}
										}
										else if (SwitchConfig == 3)
										{
											if (state == 0 || state == 1) {
												if(!MOverride) {
													output_2_SetLevel(state);
												}
												bChangeOutput = true;
											}
										}
										if (bChangeOutput) {
											FanAlgoState = state;
											device_updateResult_031();
										}
										//bControlCmd = true;
									}
								}
							}
							//update status immediately
							mqtt_setForcedStausUpdate(true);
						}break;
					
					default: //previous settings
						{
						}break;
				}
				if (bControlCmd) {
					statusResponseRecord_addRecord();
					mqtt_setPublishIgnore();
				}
				//------------------------------------------
			}
			else if (strncmp(&event->topic[event->topic_len-4], "Util", 4) == 0)
			{
				cJSON* messageType;
				int message_value;
				messageType = cJSON_GetObjectItem(root, "DeviceConfig");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value >= 1 && message_value <= UINT8_MAX)
					{
						uint8_t value = message_value;
						if (value != SwitchConfig) {
							SwitchConfig = value;
							storage_WriteDeviceConfig(SwitchConfig);

							if (firmwareVersion == 0x31) {
								//reset all outputs
								output_1_SetLevel(0);
								output_2_SetLevel(0);
								FanAlgoState = 0;
								device_updateResult_031();
							} else {
								FanAlgoState = 0;
							}
						}
						printf("DeviceConfig = %d\n", SwitchConfig);
						switchCommand |= (1UL << SW_CMD_SW_COFIG);
						switchCommand_Repeat = SW_CMD_REPEAT_CNT;
					}
				}
				messageType = cJSON_GetObjectItem(root, "ResetWifi");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value == 1)
					{
						system_event_run(SYS_EVENT_WIFI_RESET);
					}
				}
				messageType = cJSON_GetObjectItem(root, "RebootDevice");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value == 1)
					{
						system_event_run(SYS_EVENT_REBOOT);
					}
				}
				messageType = cJSON_GetObjectItem(root, "LEDIntensity");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value >= 1 && message_value <= 10)
					{
						if (IAQLEDIntensity != message_value)
						{
							IAQLEDIntensity = message_value;
							storage_WriteIAQLEDIntensity(IAQLEDIntensity);
						}
						switchCommand |= (1UL << SW_CMD_IAQLED_INTENSITY);
						switchCommand_Repeat = SW_CMD_REPEAT_CNT;
					}
				}
				messageType = cJSON_GetObjectItem(root, "InitiateFRC");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value == 1)
					{
						FRCSetvalue = 400;
						switchAppCmd |= (1UL << SW_APPCMD_CO2_FRC_SET);
					}
				}
				

				//internal use
				messageType = cJSON_GetObjectItem(root, "RequestFWver");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value == 1)
					{
						bForcedMQTTSendFWver = true;
					}
				}
				messageType = cJSON_GetObjectItem(root, "DebugInfo");
				if (isJSONValidNumber(messageType))
				{
					message_value = atoi(messageType->valuestring);
					if (message_value == 1)
					{
						Debug_bMQTTRaw = true;
					}
				}
				messageType = cJSON_GetObjectItem(root, "SN_SET");
				if (isJSONValidString(messageType))
				{
					//message_value = atoi(messageType->valuestring);
					if (appCommon_GetTestMode()) {
						if (strlen(messageType->valuestring) <= 32){
							storage_WriteSN(messageType->valuestring);
						}
					}
				}
				messageType = cJSON_GetObjectItem(root, "ServerTestMode");
				if (isJSONValidString(messageType))
				{
					bool bChangeTestMode = false;
					uint8_t value = 0;
					if (strcmp(messageType->valuestring, "4LV7IO0ctuNoL9ri5CMFMA==") == 0)
					{
						//enable test mode
						if (!appCommon_GetTestMode()) {
							bChangeTestMode = true;
							value = 1;
						}
					}
					else if (strcmp(messageType->valuestring, "ilX7I5+QZKgBViJ5gz9O2g==") == 0)
					{
						//disable test mode
						if (appCommon_GetTestMode()) {
							bChangeTestMode = true;
							value = 0;
						}
					}
					if (bChangeTestMode) {
						appCommon_SetTestMode(value);
						StoreUserData(PKI_TESTMODE_ADDR, &value, 1);
						//test mode is changed, re-connect correct MQTT broker
						mqtt_SetReconnectAction(true);
					}
				}
#ifdef ALLOW_UPDATE_BOOTLOADER
				if(firmwareVersion != 0x31)
				{
					messageType = cJSON_GetObjectItem(root, "AllowUpdateSTMBootLoader");
					if (isJSONValidNumber(messageType))
					{
						message_value = atoi(messageType->valuestring);
						if (message_value == 1)
						{
							Debug_bAllowUpdateSTMBootLoader = true;
						}
					}
					messageType = cJSON_GetObjectItem(root, "STMBootLoaderTest");
					if (isJSONValidString(messageType))
					{
						if (strcmp(messageType->valuestring, "qUO9ca/xWLfvGFP7bgr1Bw==") == 0)
						{
							Debug_bSTMBootLoaderTest = true;
						}
					}
				}
#endif
			}
			else if (strncmp(&event->topic[event->topic_len-7], "OTA_old", 7) == 0)
			{
				uint32_t STM32_version = 0x0;
				char* STM32_url = NULL;
				uint32_t ESP32_version = 0x0;
				char* ESP32_url = NULL;
				char* appKey = NULL;
				bool bRunOTA = false;
				cJSON* messageType;

				messageType = cJSON_GetObjectItem(root, "newSTM32FWver");
				if (isJSONValidString(messageType))
				{
					char * pch;
					uint32_t value = 0;
					pch = strtok (messageType->valuestring, " .");
					if (pch) {

						value += (atoi(pch) & 0xff) << 16;
						pch = strtok(NULL, " .");
						if (pch) {
							value += (atoi(pch) & 0xff) << 8;
							pch = strtok(NULL, " .");
							if (pch) {
								value += atoi(pch) & 0xff;
								STM32_version = value;
							}
						}
					}
				}
				messageType = cJSON_GetObjectItem(root, "STM32URL");
				if (isJSONValidString(messageType))
				{
					STM32_url = malloc(strlen(messageType->valuestring)+1);
					if (STM32_url) {
						strcpy(STM32_url, messageType->valuestring);
					}
					bRunOTA = true;
				}
				
				messageType = cJSON_GetObjectItem(root, "newESP32FWver");
				if (isJSONValidString(messageType))
				{
					char * pch;
					uint32_t value = 0;
					pch = strtok (messageType->valuestring, " .");
					if (pch) {

						value += (atoi(pch) & 0xff) << 16;
						pch = strtok(NULL, " .");
						if (pch) {
							value += (atoi(pch) & 0xff) << 8;
							pch = strtok(NULL, " .");
							if (pch) {
								value += atoi(pch) & 0xff;
								ESP32_version = value;
							}
						}
					}
				}
				messageType = cJSON_GetObjectItem(root, "ESP32URL");
				if (isJSONValidString(messageType))
				{
					ESP32_url = malloc(strlen(messageType->valuestring)+1);
					if (ESP32_url) {
						strcpy(ESP32_url, messageType->valuestring);
					}
					bRunOTA = true;
				}
				messageType = cJSON_GetObjectItem(root, "appKey");
				if (isJSONValidString(messageType))
				{
					appKey = malloc(strlen(messageType->valuestring)+1);
					if (appKey) {
						strcpy(appKey, messageType->valuestring);
					}
				}

				if (bRunOTA) {
					ota_task_run(STM32_version, STM32_url, ESP32_version, ESP32_url, 0, NULL, appKey, NULL);
				}
				if (STM32_url)
					free(STM32_url);
				if (ESP32_url)
					free(ESP32_url);
				if (appKey)
					free(appKey);
			}
			else if (strncmp(&event->topic[event->topic_len-3], "OTA", 3) == 0)
			{
				uint32_t FW_version = 0x0;
				char* FW_url = NULL;
				char* FW_appKey = NULL;
				char* FW_deploymentId = NULL;
				cJSON* messageType;

#ifdef ALLOW_UPDATE_BOOTLOADER
				messageType = cJSON_GetObjectItem(root, "AllowUpdateSTMBootLoader");
				if (isJSONValidNumber(messageType))
				{
					int message_value = atoi(messageType->valuestring);
					if (message_value == 1)
					{
						Debug_bAllowUpdateSTMBootLoader = true;
					}
				}
#endif
				messageType = cJSON_GetObjectItem(root, "NewFWver");
				if (isJSONValidString(messageType))
				{
					char * pch;
					uint32_t value = 0;
					pch = strtok (messageType->valuestring, " .");
					if (pch) {
						value += (atoi(pch) & 0xff) << 16;
						pch = strtok(NULL, " .");
						if (pch) {
							value += (atoi(pch) & 0xff) << 8;
							pch = strtok(NULL, " .");
							if (pch) {
								value += atoi(pch) & 0xff;
								FW_version = value;
							}
						}
					}
				}
				messageType = cJSON_GetObjectItem(root, "DeploymentId");
				if (isJSONValidString(messageType))
				{
					FW_deploymentId = malloc(strlen(messageType->valuestring)+1);
					if (FW_deploymentId) {
						strcpy(FW_deploymentId, messageType->valuestring);
					}
				}
				messageType = cJSON_GetObjectItem(root, "FWURL");
				if (isJSONValidString(messageType))
				{
					FW_url = malloc(strlen(messageType->valuestring)+1);
					if (FW_url) {
						strcpy(FW_url, messageType->valuestring);
					}
				}
				messageType = cJSON_GetObjectItem(root, "appKey");
				if (isJSONValidString(messageType))
				{
					FW_appKey = malloc(strlen(messageType->valuestring)+1);
					if (FW_appKey) {
						strcpy(FW_appKey, messageType->valuestring);
					}
				}

				messageType = cJSON_GetObjectItem(root, "FWtype");
				if (isJSONValidString(messageType))
				{
					//STM32
					if (strcmp(messageType->valuestring, "STM32") == 0)
					{
						ota_task_run(FW_version, FW_url, 0, NULL, 0, NULL, FW_appKey, FW_deploymentId);
					}
					//ESP32
					else if (strcmp(messageType->valuestring, "ESP32") == 0)
					{
						ota_task_run(0, NULL, FW_version, FW_url, 0, NULL, FW_appKey, FW_deploymentId);
					}
					//CERT
					else if (strcmp(messageType->valuestring, "CERT") == 0)
					{
						ota_task_run(0, NULL, 0, NULL, FW_version, FW_url, FW_appKey, FW_deploymentId);
					}
				}

				if (FW_url)
					free(FW_url);
				if (FW_appKey)
					free(FW_appKey);
				if (FW_deploymentId)
					free(FW_deploymentId);
			}
			cJSON_Delete(root);
            break;
			
        case MQTT_EVENT_ERROR:
            //ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
            break;
			
		case MQTT_EVENT_BEFORE_CONNECT:
			//ESP_LOGI(MQTT_TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
			
        default:
            //ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static bool bWiFiReconnectSkipOnce = false;
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	switch(event_id) {
		case WIFI_EVENT_STA_START:
			wifi_SetConnectionState(WIFI_STATE_INIT);
			ESP_LOGI(WIFI_TAG, "STA start to connect");
	        esp_wifi_connect();
			break;
		case WIFI_EVENT_STA_CONNECTED:
			wifi_SetConnectionState(WIFI_STATE_CONNECTING);
	        ESP_LOGI(WIFI_TAG, "STA connected");
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
		{
			wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
			wifi_SetConnectionState(WIFI_STATE_DISCONNECTED);
			ESP_LOGI(WIFI_TAG, "STA disconnected %d", event->reason);
			/* This is a workaround as ESP32 WiFi libs don't currently
	           auto-reassociate. */
			if (bWiFiReconnectSkipOnce) {
				bWiFiReconnectSkipOnce = false;
			} else {
				esp_wifi_connect();
			}
			break;
		}
		default:
			ESP_LOGW(WIFI_TAG, "unhandled wifi event: %d", event_id);
			break;
	}
}

static void  ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	switch(event_id) {
		case IP_EVENT_STA_GOT_IP:
		{
			ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
			wifi_SetConnectionState(WIFI_STATE_CONNECTED);
			ESP_LOGI(WIFI_TAG, "STA got ip: " IPSTR, IP2STR(&event->ip_info.ip));
			break;
		}
		default:
			ESP_LOGW(WIFI_TAG, "unhandled ip event: %d", event_id);
			break;
	}
}

void initialise_wifi(void)
{
	int i, j;
    ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    //ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
	
	
	for (i = 0; i < MAX_WIFI; i++)
	{
		wifi_config_t wifi_config = {
			.sta = {
				.ssid = {0},
				.password = {0},
			},
		};
		
		memcpy(wifi_config.sta.ssid, WIFI_SSID[0], sizeof(wifi_config.sta.ssid)); 
		memcpy(wifi_config.sta.password, WIFI_PASS[0], sizeof(wifi_config.sta.password)); 
		
		ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
		ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
		ESP_LOGI(WIFI_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
		ESP_LOGI(WIFI_TAG, "Setting WiFi configuration PW ...");
		
		esp_wifi_get_mac(WIFI_IF_STA, myMacAddr);
		ESP_LOGI(WIFI_TAG, "Setting Mac Addr = [%X] [%X] [%X] [%X] [%X] [%X] \r\n", myMacAddr[0],myMacAddr[1],myMacAddr[2],myMacAddr[3],myMacAddr[4],myMacAddr[5]);

		ESP_ERROR_CHECK( esp_wifi_start() );
		
		for (j = 0; j < 50; j ++)
		{
			vTaskDelay(100 / portTICK_RATE_MS);
			if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
				return;
			}
		}
	}
	
	///* Wait for WiFI to show as connected */
	//xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

static bool bSNTPInited = false;
static bool bSNTPSyncOK = false;
static void sntp_sync_time_cb(struct timeval *tv)
{
	ESP_LOGI(NTP_TAG, "Time sync OK!!!");
	bSNTPSyncOK = true;
}

static void initialize_sntp(void)
{
    ESP_LOGI(NTP_TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
	//sntp_setservername(0, "0.us.pool.ntp.org");
	//sntp_setservername(1, "1.us.pool.ntp.org");
	//sntp_setservername(2, "2.us.pool.ntp.org");
	sntp_set_time_sync_notification_cb(sntp_sync_time_cb);
    sntp_init();
	ESP_LOGI(NTP_TAG, "SNTP sync interval: %ds", sntp_get_sync_interval()/1000);
	
	// wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
	if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED)
	{
		int retry = 0;
		const int retry_count = 10;
		while(timeinfo.tm_year < (2019 - 1900) && ++retry < retry_count) 
		{
			ESP_LOGI(NTP_TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			time(&now);
			localtime_r(&now, &timeinfo);
		}
	}
	// Set timezone to HK Standard Time
    //setenv("TZ", "CST-8", 1);
    setenv("TZ", "UTC", 1);
    tzset();
    localtime_r(&now, &timeinfo);

    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    //ESP_LOGI(NTP_TAG, "The current date/time in Hong Kong is: %s", strftime_buf);
	ESP_LOGI(NTP_TAG, "The current date/time in UTC is: %s", strftime_buf);
	bSNTPInited = true;
}


static void wifi_callback(wifi_state_t state, wifi_state_t state_prev)
{
	if (state == WIFI_STATE_DISABLED)
	{

	}
	else if (state == WIFI_STATE_INIT)
	{

	}
	else if (state == WIFI_STATE_CONNECTING) //connected, but not yet got IP address
	{
		
	}
	else if (state == WIFI_STATE_CONNECTED)
	{
		if (!BLE_IsEnabled()) {
			//...
		}

		//check if SNTP is running
		if (bSNTPInited) {
			if (!bSNTPSyncOK && sntp_enabled()) {
				sntp_stop();
			}
			if (!sntp_enabled()) {
				sntp_init();
			}
		}
	}
	else if (state == WIFI_STATE_DISCONNECTED)
	{
		if ((wifi_isSet == 1) && !BLE_IsEnabled()) {
			//...
		}
#ifdef DEBUG_REQUEST_BY_D4
		Call_BT_enable = 1;
#endif
	}
}

static void mqtt_callback(bool bConnect)
{
	if(bConnect)
	{
		if (!BLE_IsEnabled()) {
			uart_setSwitchLedCmd(SW_LEDCMD_WIFI_CONNECT_OK);
			switchLedMode_031 = LEDMODE_WIFI_CONNECT_OK;
		}
	}
	else
	{
		if (!BLE_IsEnabled()) {
			uart_setSwitchLedCmd(SW_LEDCMD_WIFI_DISCONNECTED);
			switchLedMode_031 = LEDMODE_WIFI_DISCONNECTED;
		}
		if (firmwareVersion == 0x31) {
			FanAlgoState = 0;
			device_updateResult_031();
		} else {
			FanAlgoState = 0;
		}
	}
}

static void BLE_callback(BLE_state_t state, BLE_state_t state_prev)
{
	if (state == BLE_STATE_DISABLED)
	{
		BLE_SetDebugMode(false);
		switchAppCmd |= (1UL << SW_APPCMD_BT_STATUS);

		if (mqtt_IsConnected()) {
			uart_setSwitchLedCmd(SW_LEDCMD_WIFI_CONNECT_OK);
			switchLedMode_031 = LEDMODE_WIFI_CONNECT_OK;
		} else {
			uart_setSwitchLedCmd(SW_LEDCMD_WIFI_DISCONNECTED);
			switchLedMode_031 = LEDMODE_WIFI_DISCONNECTED;
			if (wifi_GetConnectionState() != WIFI_STATE_CONNECTED) {
				//re-connect WiFi
				bWiFiReconnectSkipOnce = false;
				esp_err_t err = wifi_StationJoin(WIFI_SSID[0], WIFI_PASS[0]);
				if (err != ESP_OK) {
					ESP_LOGW(APP_TAG, "Fail to connect WiFi when exit BT mode");
				}
			}
		}
	}
	else if (state == BLE_STATE_INIT)
	{
		switchAppCmd |= (1UL << SW_APPCMD_BT_STATUS);
		uart_setSwitchLedCmd(SW_LEDCMD_BT_PAIRING);
		switchLedMode_031 = LEDMODE_BT_PAIRING_MODE;

		if (firmwareVersion == 0x31) {
			//reset all outputs
			output_1_SetLevel(0);
			output_2_SetLevel(0);
			FanAlgoState = 0;
			device_updateResult_031(); //MOverride is set to 0
		} else {
			FanAlgoState = 0;
		}

		if(BLE_IsDebugMode()) {
			//don't disconnect WiFi if it is BLE debug mode
		} else {
			//disconnect WiFi
			if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
				esp_err_t err = esp_wifi_disconnect();
				if (err != ESP_OK) {
					ESP_LOGW(APP_TAG, "Fail to disconnect WiFi when enter BT mode");
				}
			}
			bWiFiReconnectSkipOnce = true;
		}
	}
	else if (state == BLE_STATE_CONNECTED)
	{
	}
	else if (state == BLE_STATE_DISCONNECTED)
	{
	}
}

static uint8_t Bttn1RealState_backup = 0;
static uint8_t Bttn2RealState_backup = 0;
static uint8_t FanRealState_backup = 0;
static uint8_t FanAlgoState_backup = 0;

static bool STM32SystemStartCmd = false;
static uint32_t STM32SystemStartCmd_time = 0;
static void STM32SystemStartCmd_Set(void)
{
	STM32SystemStartCmd_time = utils_time_ms_get();
	STM32SystemStartCmd = true;
}

static void STM32SystemStartCmd_Check(void)
{
	if (STM32SystemStartCmd == false) {
		return;
	}

	if (utils_time_diff(utils_time_ms_get(), STM32SystemStartCmd_time) > 1200) {
		ESP_LOGI(APP_TAG, "STM32SystemStartCmd");
		if (BLE_IsEnabled())
		{
			//no commands to send
		}
		else
		{
			//restore Algo/Button state
			Bttn1RealState = Bttn1RealState_backup;
			Bttn2RealState = Bttn2RealState_backup;
			FanRealState = FanRealState_backup;
			FanAlgoState = FanAlgoState_backup;

			if (mqtt_IsConnected())
			{
				//SW_CMD_IAQ_COLOR (if IAQcolor !=200)
				//SW_CMD_BTTN1_APP_STATE
				//SW_CMD_BTTN2_APP_STATE
				//SW_CMD_FAN_APP_STATE
				//SW_CMD_FAN_ALGO_STATE
				if (IAQcolor != 200) {
					switchCommand |= (1UL << SW_CMD_IAQ_COLOR);
				}
				if (firmwareVersion == 0x11) //Room Sesor
				{
					Bttn1AppState = Bttn1RealState;
					switchCommand |= (1UL << SW_CMD_BTTN1_APP_STATE);
					switchCommand_Repeat = SW_CMD_REPEAT_CNT;
				}
				else if (firmwareVersion == 0x21) //Wall Control
				{
					Bttn1AppState = Bttn1RealState;
					switchCommand |= (1UL << SW_CMD_BTTN1_APP_STATE);
					Bttn2AppState = Bttn2RealState;
					switchCommand |= (1UL << SW_CMD_BTTN2_APP_STATE);
					FanAppState = FanRealState;
					switchCommand |= (1UL << SW_CMD_FAN_APP_STATE);
					//FanAlgoState
					switchCommand |= (1UL << SW_CMD_FAN_ALGO_STATE);
					switchCommand_Repeat = SW_CMD_REPEAT_CNT;
				}
			}
			else
			{
				//SW_CMD_BTTN1_APP_STATE
				//SW_CMD_BTTN2_APP_STATE
				//SW_CMD_FAN_APP_STATE
				if (firmwareVersion == 0x11) //Room Sesor
				{
					Bttn1AppState = Bttn1RealState;
					switchCommand |= (1UL << SW_CMD_BTTN1_APP_STATE);
					switchCommand_Repeat = SW_CMD_REPEAT_CNT;
				}
				else if (firmwareVersion == 0x21) //Wall Control
				{
					Bttn1AppState = Bttn1RealState;
					switchCommand |= (1UL << SW_CMD_BTTN1_APP_STATE);
					Bttn2AppState = Bttn2RealState;
					switchCommand |= (1UL << SW_CMD_BTTN2_APP_STATE);
					FanAppState = FanRealState;
					switchCommand |= (1UL << SW_CMD_FAN_APP_STATE);
					switchCommand_Repeat = SW_CMD_REPEAT_CNT;
				}
			}
		}
		STM32SystemStartCmd = false;
	}
}

void system_event_run(sys_event_t event)
{
	switch(event)
	{
		case SYS_EVENT_WIFI_RESET:
		{
			UD_info_t UD_wifi[2];
			memset(WIFI_SSID[0], 0, sizeof(WIFI_SSID[0]));
			memset(WIFI_PASS[0], 0, sizeof(WIFI_PASS[0]));
			UD_wifi[0].offset = PKI_WIFI_SSID_ADDR;
			UD_wifi[0].size = sizeof(WIFI_SSID[0]);
			UD_wifi[0].data = (uint8_t *)WIFI_SSID[0];
			UD_wifi[1].offset = PKI_WIFI_PW_ADDR;
			UD_wifi[1].size = sizeof(WIFI_PASS[0]);
			UD_wifi[1].data = (uint8_t *)WIFI_PASS[0];
			StoreUserData_Multi(UD_wifi, 2);
			ESP_LOGW(APP_TAG, "WiFi reset!");
			esp_restart();
			break;
		}
		case SYS_EVENT_FACTORY_RESET: // internal use during factory production
		{
			//reset WiFi, TestMode
			UD_info_t UD_info[3];
			uint8_t uTestMode = 0;
			memset(WIFI_SSID[0], 0, sizeof(WIFI_SSID[0]));
			memset(WIFI_PASS[0], 0, sizeof(WIFI_PASS[0]));
			appCommon_SetTestMode(uTestMode);
			UD_info[0].offset = PKI_WIFI_SSID_ADDR;
			UD_info[0].size = sizeof(WIFI_SSID[0]);
			UD_info[0].data = (uint8_t *)WIFI_SSID[0];
			UD_info[1].offset = PKI_WIFI_PW_ADDR;
			UD_info[1].size = sizeof(WIFI_PASS[0]);
			UD_info[1].data = (uint8_t *)WIFI_PASS[0];
			UD_info[2].offset = PKI_TESTMODE_ADDR;
			UD_info[2].size = sizeof(uTestMode);
			UD_info[2].data = &uTestMode;
			StoreUserData_Multi(UD_info, 3);

			//reset DeviceConfig
			SwitchConfig = DEFAULT_DEVICECONFIG_VAL;
			storage_WriteDeviceConfig(SwitchConfig);
			//reset IAQLED intensity
			IAQLEDIntensity = DEFAULT_IAQLEDINTENSITY_VAL;
			storage_WriteIAQLEDIntensity(IAQLEDIntensity);

			ESP_LOGW(APP_TAG, "Factory reset!");
			esp_restart();
			break;
		}
		case SYS_EVENT_REBOOT:
		{
			ESP_LOGW(APP_TAG, "Reboot device!");
			esp_restart();
			break;
		}
		case SYS_EVENT_STM32_START: //STM32 system start
		{
			ESP_LOGW(APP_TAG, "STM32 start!");
			//send dummy command first after STM32 bootup, avoid 1st command is missing
			uart_setSwitchDummyCmd();

			//send HW version
			switchAppCmd |= (1UL << SW_APPCMD_HW_VER);
			//send deviceConfig
			switchCommand |= (1UL << SW_CMD_SW_COFIG);
			//send IAQLED intensity
			switchCommand |= (1UL << SW_CMD_IAQLED_INTENSITY);
			switchCommand_Repeat = SW_CMD_REPEAT_CNT;

			//send BLE and MQTT status
			switchAppCmd |= (1UL << SW_APPCMD_BT_STATUS);
			if (BLE_IsEnabled()) {
				uart_setSwitchLedCmd(SW_LEDCMD_BT_PAIRING);
			} else {
				if (mqtt_IsConnected()) {
					uart_setSwitchLedCmd(SW_LEDCMD_WIFI_CONNECT_OK);
				} else {
					uart_setSwitchLedCmd(SW_LEDCMD_WIFI_DISCONNECTED);
				}
			}

			//if first time normal power on, don't need to restore Algo/Button state
			static bool bFirstTime = true;
			if (bFirstTime) {
				bFirstTime = false;
			} else {
				//backup Algo/Button state before STM32SystemStartCmd_Set()
				Bttn1RealState_backup = Bttn1RealState;
				Bttn2RealState_backup = Bttn2RealState;
				FanRealState_backup = FanRealState;
				FanAlgoState_backup = FanAlgoState;
				STM32SystemStartCmd_Set();
			}
			break;
		}
		default:
			break;
	}
}


void write_empty_user_data(user_data_t *pNewUserData, uint8_t HW_version, char* device_id)
{
	if((pNewUserData == NULL) || (device_id == NULL)) {
		return;
	}
	memset(pNewUserData, 0 , sizeof(user_data_t));
	memcpy(pNewUserData->hdr.marker, UD_FILE_MARKER, UD_FILE_MARKER_LEN);
	pNewUserData->hdr.version = UD_HEADER_VERSION_CURR;
	pNewUserData->hdr.HW_version = HW_version;
	memcpy(pNewUserData->device_id, device_id, sizeof(pNewUserData->device_id));
	StoreUserData(0, (uint8_t *)pNewUserData, sizeof(user_data_t));
}

void check_user_data(void)
{
	if (strncmp(HW_ID, "SS", 2) == 0)
	{
		//recover HW_ID
		if (storage_RecoverDeviceID(HW_ID))
		{
			if (strncmp(HW_ID, "RS", 2) == 0) {
				firmwareVersion = 0x11;
				printf("brnc011 \n");
			} else if (strncmp(HW_ID, "WC", 2) == 0) {
				firmwareVersion = 0x21;
				printf("brnc021 \n");
			} else if (strncmp(HW_ID, "SP", 2) == 0) {
				firmwareVersion = 0x31;
				printf("brnc031 \n");
			}
		}
	}
}

int get_user_data(void)
{
	int ret = 0;
	const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "pki");
	if (!partition) 
	{
		ESP_LOGE(UDATA_TAG, "No user data partition");
		return -1;
	}

	uint8_t *buffer = malloc(sizeof(user_data_t));
	user_data_t *pUserData = malloc(sizeof(user_data_t));
	if (!buffer || !pUserData)
	{
		ret = -2;
	}
	else
	{
		memset(buffer, 0, sizeof(user_data_t));
		memset(pUserData, 0, sizeof(user_data_t));
		if (ESP_OK != esp_partition_read(partition, 0, buffer, sizeof(user_data_t))) {
			ret = -3;
		} else if (memcmp(buffer, UD_FILE_MARKER, UD_FILE_MARKER_LEN)) {
			ESP_LOGE(UDATA_TAG, "No user partition header");
			if (storage_RecoverDeviceID(HW_ID)) {
				uint8_t HW_version = 0;
				storage_RecoverHwVersion(&HW_version);
				//write new user data
				write_empty_user_data(pUserData, HW_version, HW_ID);
				ESP_LOGW(UDATA_TAG, "write new user data");
				appCommon_SetHwVersion(HW_version);
				if (strncmp(HW_ID, "RS", 2) == 0) {
					firmwareVersion = 0x11;
					printf("brnc011 \n");
				} else if (strncmp(HW_ID, "WC", 2) == 0) {
					firmwareVersion = 0x21;
					printf("brnc021 \n");
				} else if (strncmp(HW_ID, "SP", 2) == 0) {
					firmwareVersion = 0x31;
					printf("brnc031 \n");
				}
			} else {
				ret = -1;
			}
		} else {
			//verify and convert old format user data
			uint16_t UD_version = ((user_data_header *)buffer)->version;
			
			if (UD_version > UD_HEADER_VERSION_CURR) {
				memcpy(pUserData, buffer, sizeof(user_data_t));
				//new version --> v5 (current version)
				ESP_LOGW(UDATA_TAG, "convert new version to v%d (current version)", UD_HEADER_VERSION_CURR);
				pUserData->hdr.version = UD_HEADER_VERSION_CURR;
				pUserData->hdr.HW_version = ((user_data_header *)buffer)->HW_version;
				memcpy(pUserData->hdr.reserved, ((user_data_header *)buffer)->reserved, sizeof(pUserData->hdr.reserved));
				StoreUserData(0, (uint8_t *)pUserData, sizeof(user_data_t));
			} else if (UD_version == UD_HEADER_VERSION_CURR) {
				memcpy(pUserData, buffer, sizeof(user_data_t));
			} else { //old version
				if (UD_version <= UD_HEADER_VERSION_V3) {
					//v3 --> v5 (current version)
					ESP_LOGW(UDATA_TAG, "convert v3 to v%d (current version)", UD_HEADER_VERSION_CURR);
					user_data_t_v3 *pUserData_v3 = malloc(sizeof(user_data_t_v3));
					if (pUserData_v3) {
						memcpy(pUserData_v3, buffer, sizeof(user_data_t_v3));

						//create new version user data
						memcpy(pUserData->hdr.marker, UD_FILE_MARKER, UD_FILE_MARKER_LEN);
						pUserData->hdr.version = UD_HEADER_VERSION_CURR;
						pUserData->hdr.HW_version = pUserData_v3->hdr.HW_version;
						memcpy(pUserData->hdr.reserved, pUserData_v3->hdr.reserved, sizeof(pUserData_v3->hdr.reserved));
						memcpy(pUserData->device_id, pUserData_v3->device_id, sizeof(pUserData_v3->device_id));
						memcpy(pUserData->wifi_ssid, pUserData_v3->wifi_ssid, sizeof(pUserData_v3->wifi_ssid));
						memcpy(pUserData->wifi_password, pUserData_v3->wifi_password, sizeof(pUserData_v3->wifi_password));
						memcpy(pUserData->mqtt_broker_url, pUserData_v3->mqtt_broker_url, sizeof(pUserData_v3->mqtt_broker_url));
						memcpy(pUserData->bt_dev_name, pUserData_v3->bt_dev_name, sizeof(pUserData_v3->bt_dev_name));
						pUserData->test_mode = 0;
						pUserData->mqtt_broker_url_testMode[0] = '\0';
						StoreUserData(0, (uint8_t *)pUserData, sizeof(user_data_t));

						free(pUserData_v3);
					}
				} else if (UD_version == UD_HEADER_VERSION_V4) {
					//v4 --> v5 (current version)
					ESP_LOGW(UDATA_TAG, "convert v4 to v%d (current version)", UD_HEADER_VERSION_CURR);
					user_data_t_v4 *pUserData_v4 = malloc(sizeof(user_data_t_v4));
					if (pUserData_v4) {
						memcpy(pUserData_v4, buffer, sizeof(user_data_t_v4));

						//create new version user data
						memcpy(pUserData->hdr.marker, UD_FILE_MARKER, UD_FILE_MARKER_LEN);
						pUserData->hdr.version = UD_HEADER_VERSION_CURR;
						pUserData->hdr.HW_version = pUserData_v4->hdr.HW_version;
						memcpy(pUserData->hdr.reserved, pUserData_v4->hdr.reserved, sizeof(pUserData_v4->hdr.reserved));
						memcpy(pUserData->device_id, pUserData_v4->device_id, sizeof(pUserData_v4->device_id));
						memcpy(pUserData->wifi_ssid, pUserData_v4->wifi_ssid, sizeof(pUserData_v4->wifi_ssid));
						memcpy(pUserData->wifi_password, pUserData_v4->wifi_password, sizeof(pUserData_v4->wifi_password));
						memcpy(pUserData->mqtt_broker_url, pUserData_v4->mqtt_broker_url, sizeof(pUserData_v4->mqtt_broker_url));
						memcpy(pUserData->bt_dev_name, pUserData_v4->bt_dev_name, sizeof(pUserData_v4->bt_dev_name));
						pUserData->test_mode = 0;
						pUserData->mqtt_broker_url_testMode[0] = '\0';
						StoreUserData(0, (uint8_t *)pUserData, sizeof(user_data_t));

						free(pUserData_v4);
					}
				} else { //unknown newer version
					ESP_LOGW(UDATA_TAG, "convert unknown old version to v%d (current version)", UD_HEADER_VERSION_CURR);
					//create new version user data, copy device_id only
					memcpy(pUserData->hdr.marker, UD_FILE_MARKER, UD_FILE_MARKER_LEN);
					pUserData->hdr.version = UD_HEADER_VERSION_CURR;
					pUserData->hdr.HW_version = ((user_data_header *)buffer)->HW_version;
					memcpy(pUserData->hdr.reserved, ((user_data_header *)buffer)->reserved, sizeof(pUserData->hdr.reserved));
					memcpy(pUserData->device_id, ((user_data_t *)buffer)->device_id, sizeof(pUserData->device_id));
					StoreUserData(0, (uint8_t *)pUserData, sizeof(user_data_t));
				}
			}

			//backup important data to nvs
			storage_BackupDeviceID(pUserData->device_id);
			storage_BackupHwVersion(pUserData->hdr.HW_version);

			appCommon_SetHwVersion(pUserData->hdr.HW_version);
			memcpy(HW_ID, pUserData->device_id, sizeof(pUserData->device_id));
			if ((pUserData->wifi_ssid[0] != '\0') && (pUserData->wifi_password[0] != '\0')) {
				memset(WIFI_SSID[0], 0, sizeof(WIFI_SSID[0]));
				memset(WIFI_PASS[0], 0, sizeof(WIFI_PASS[0]));
				memcpy(WIFI_SSID[0], pUserData->wifi_ssid, sizeof(pUserData->wifi_ssid));
				memcpy(WIFI_PASS[0], pUserData->wifi_password, sizeof(pUserData->wifi_password));
				wifi_isSet = 1;
			} else {
				wifi_isSet = 0;
			}
			if (pUserData->mqtt_broker_url[0] != '\0') {
				if (!strncmp(pUserData->mqtt_broker_url, "mqtt", 4)) {
					sprintf(MQTT_BROKER_URL, "%.*s", sizeof(pUserData->mqtt_broker_url), pUserData->mqtt_broker_url); 
				} else {
					sprintf(MQTT_BROKER_URL, "mqtt://%.*s", sizeof(pUserData->mqtt_broker_url), pUserData->mqtt_broker_url);
				}
			}
			if (pUserData->mqtt_broker_url_testMode[0] != '\0') {
				if (!strncmp(pUserData->mqtt_broker_url_testMode, "mqtt", 4)) {
					sprintf(MQTT_BROKER_URL_TestMode, "%.*s", sizeof(pUserData->mqtt_broker_url_testMode), pUserData->mqtt_broker_url_testMode); 
				} else {
					sprintf(MQTT_BROKER_URL_TestMode, "mqtt://%.*s", sizeof(pUserData->mqtt_broker_url_testMode), pUserData->mqtt_broker_url_testMode);
				}
			}
			if (pUserData->test_mode == 0x01)
			{
				appCommon_SetTestMode(0x01);
			}
			else
			{
				appCommon_SetTestMode(0x00);
			}
			//memcpy(BT_Dev_name, pUserData->bt_dev_name, sizeof(pUserData->bt_dev_name));
			ESP_LOGI(UDATA_TAG, "User data is found !! (%s|%s|%s)", HW_ID, WIFI_SSID[0], WIFI_PASS[0]);
			ESP_LOGI(UDATA_TAG, "MQTT = %s", MQTT_BROKER_URL);
			if (appCommon_GetTestMode()) {
				ESP_LOGI(UDATA_TAG, "MQTT(test mode) = %s", MQTT_BROKER_URL_TestMode);
			}


// 4.	Room Sensor 1	(Living room):	EX20190003
// 5.	Room Sensor 2	(Living room):	EX20190004
// 6.	Room Sensor 3	(Bed room):		EX20190005

// 1.	Wall Control 1	(Range hood):	EX20190001
// 2.	Wall control 2	(Bath fan):		EX20190002
// 3.	Wall control 3	(Bath fan):		EX20190007

// 7.	Smart Plug (ERV): EX20190006

			//--------------------------------------2019.12.18
			if(HW_ID[0] == 'E' && HW_ID[1] == 'X')
			{
				switch(HW_ID[9])
				{
					case '3':
					case '4':
					case '5':
							firmwareVersion = 0x11;
							printf("For EX brnc011 \n");
							break;
					case '1':
					case '2':
					case '7':
							firmwareVersion = 0x21;
							printf("For EX brnc021 \n");
							break;
					case '6':
							firmwareVersion = 0x31;
							printf("For EX brnc031 \n");
							break;
					default:
							firmwareVersion = 0x31;
							printf("the EX ID not correctly, set brnc011 as the default model \n");
							break;
				}
			}
			else
			//--------------------------------------
			{
				//--------------------------------------2019.12.11
				if(HW_ID[0] == 'R' && HW_ID[1] == 'S')
				{
					firmwareVersion = 0x11;
					printf("For brnc011 \n");
				}
				else if(HW_ID[0] == 'W' && HW_ID[1] == 'C')
				{
					firmwareVersion = 0x21;
					printf("For brnc021 \n");
				}
				else if(HW_ID[0] == 'S' && HW_ID[1] == 'P')
				{
					firmwareVersion = 0x31;
					printf("For brnc031 \n");
				}
				else
				{
					firmwareVersion = 0x11;
					printf("the user ID is wrong, use default model brnc011 \n");
				}
				//--------------------------------------		
			}
		}
	}

	if (buffer) {
		free(buffer);
	}
	if (pUserData) {
		free(pUserData);
	}
	return ret;
}


#define CERT_BROAN_IOTDEV_INIT
#include "cert_broan_iotdev.h"
#define CERT_BROAN_IOTQA_INIT
#include "cert_broan_iotqa.h"
static const char* broan_iotdev_url = "mqtts://iotdev.broan-nutone.com:61714";
static const char* broan_iotqa_url = "mqtts://iotqa.broan-nutone.com:61714";

//encoded size= ceil(n/3)*4
static const unsigned char iotdev_username_encoded[64] =
{
	0x59, 0x6d, 0x35, 0x6b, 0x5a, 0x58, 0x5a, 0x70,
	0x59, 0x32, 0x55, 0x77, 0x4d, 0x51, 0x3d, 0x3d,
	0x00,
};
static const unsigned char iotdev_password_encoded[64] =
{
	0x54, 0x54, 0x6c, 0x33, 0x4d, 0x6a, 0x77, 0x6d,
	0x63, 0x6e, 0x41, 0x32, 0x57, 0x47, 0x31, 0x4e,
	0x4e, 0x48, 0x64, 0x32, 0x64, 0x77, 0x3d, 0x3d,
	0x00,
};
static unsigned char iotdev_username[32] = {0};
static unsigned char iotdev_password[32] = {0};

#include "mbedtls/base64.h"
void mqtt_app_start(void)
{	
	esp_mqtt_client_config_t mqtt_cfg = {0};
	cert_info_t* pCertInfo = NULL;

	if (appCommon_GetTestMode()) {
		mqtt_cfg.uri = MQTT_BROKER_URL_TestMode;
	} else {
		mqtt_cfg.uri = MQTT_BROKER_URL;
	}

	//verify cert info
	{
		cert_info_t* pTempInfo = certData_GetInfo();
		if (pTempInfo && (strncmp(mqtt_cfg.uri, "mqtts", strlen("mqtts"))==0)) {
			if (pTempInfo->hostname == NULL) {
				pCertInfo = pTempInfo;
			} else if (pTempInfo->hostname[0] == '*') {
				pCertInfo = pTempInfo;
			} else {
				if (certData_CompareHostname(mqtt_cfg.uri, pTempInfo->hostname)) {
					pCertInfo = pTempInfo;
				}
			}
		}
	}

	//SSL certificate
	if (pCertInfo)
	{
		if (pCertInfo->ca_cert_len) {
			mqtt_cfg.cert_pem = pCertInfo->ca_cert;
			mqtt_cfg.cert_len = pCertInfo->ca_cert_len;
		}
		if (pCertInfo->client_cert_len) {
			mqtt_cfg.client_cert_pem = pCertInfo->client_cert;
			mqtt_cfg.client_cert_len = pCertInfo->client_cert_len;
		}
		if (pCertInfo->client_key_len) {
			mqtt_cfg.client_key_pem = pCertInfo->client_key;
			mqtt_cfg.client_key_len = pCertInfo->client_key_len;
		}
		if (pCertInfo->username) {
			mqtt_cfg.username = pCertInfo->username;
		}
		if (pCertInfo->password) {
			mqtt_cfg.password = pCertInfo->password;
		}
		ESP_LOGI(MQTT_TAG, "Use CertData (%s)", (pCertInfo->hostname)?pCertInfo->hostname:"<null>");
	}
	else if (!strncmp(mqtt_cfg.uri, broan_iotdev_url, strlen(broan_iotdev_url)))
	{
		mqtt_cfg.cert_pem = (const char *)broan_iotdev_ca_cert;
		mqtt_cfg.client_cert_pem = broan_iotdev_client_cert;
		mqtt_cfg.client_key_pem = broan_iotdev_client_key;

		//user credential
		size_t outlen;
		int ret1, ret2;
		memset(iotdev_username, 0, sizeof(iotdev_username));
		memset(iotdev_password, 0, sizeof(iotdev_password));
		ret1 = mbedtls_base64_decode(iotdev_username, sizeof(iotdev_username), &outlen, iotdev_username_encoded, strlen((char *)iotdev_username_encoded));
		ret2 = mbedtls_base64_decode(iotdev_password, sizeof(iotdev_password), &outlen, iotdev_password_encoded, strlen((char *)iotdev_password_encoded));
		if (ret1 >= 0 && ret2 >= 0) {
			mqtt_cfg.username = (char *)iotdev_username;
			mqtt_cfg.password = (char *)iotdev_password;
		}
	}
	else if (!strncmp(mqtt_cfg.uri, broan_iotqa_url, strlen(broan_iotqa_url)))
	{
		mqtt_cfg.cert_pem = (const char *)broan_iotqa_ca_cert;
		mqtt_cfg.client_cert_pem = broan_iotqa_client_cert;
		mqtt_cfg.client_key_pem = broan_iotqa_client_key;
	}

	if (client)
	{
		esp_mqtt_client_destroy(client);
	}
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void mqtt_app_stop(void)
{
	if (client)
	{
		if(mqtt_IsConnected()) {
			mqtt_SetConnected(false);
		}
		esp_mqtt_client_stop(client);
		esp_mqtt_client_destroy(client);
		client = NULL;
	}
}

static uint32_t healthDataTime = 11;
void SensorTask(void *param)
{
    while (1)
	{
		//request sensor data every 2s
		if (uart_isSendCmdReady()) {
			uart_sendCmd(UART_SEND_TYPE_SENSOR_DATA);
		}
		for (uint8_t i = 0; i < 20; i++) {
			//send health data every 11s
			if (uart_isSendCmdReady()) {
				uint32_t now_time = utils_time_sec_get();
				if (now_time > healthDataTime) {
					uart_sendCmd(UART_SEND_TYPE_HEALTH_DATA);
					healthDataTime = now_time + 11;
				}
			}

			if (uart_isSendCmdReady()) {
				uart_sendCmd(UART_SEND_TYPE_GENERAL);
			}
			vTaskDelay(100 / portTICK_PERIOD_MS);

			while (Suspend_SensorTask) {
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
		}

		uart_sensorCmdTimeoutCheck();
	}
}

int8_t getWifiRSSI()
{
    wifi_ap_record_t info;
    uint8_t rssi = 0;
    if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
        rssi = info.rssi;
#if 0 //check it later for factory production
		if (WifiBT_Statu2STM_Cnt < 10) {
			ESP_LOGE(WIFI_TAG, "set SW_LEDCMD_WIFI_CONNECTED_XXX for factory test");
			if (mqtt_IsConnected()) {
				uart_setSwitchLedCmd(SW_LEDCMD_WIFI_CONNECTED_ACTIVE);
			} else {
				uart_setSwitchLedCmd(SW_LEDCMD_WIFI_CONNECT_OK);
			}
			WifiBT_Statu2STM_Cnt++;
		}
#endif
    }else{
		
	}
    return rssi;
}


#define MAX_STATUS_RESPONSE_RECORD	(10)
typedef struct {
	struct {
		bool bSendRespond;
		uint32_t receive_time_ms;
	} record[MAX_STATUS_RESPONSE_RECORD];
} statusResponseRecord_t;
static statusResponseRecord_t statusResponseRecord = {0};

static void statusResponseRecord_addRecord(void)
{
	for(uint8_t i = 0; i < MAX_STATUS_RESPONSE_RECORD; i++) {
		if(!statusResponseRecord.record[i].bSendRespond) {
			statusResponseRecord.record[i].receive_time_ms = utils_time_ms_get();
			statusResponseRecord.record[i].bSendRespond = true;
			break;
		}
	}
}

static void statusResponseRecord_clearAll(void)
{
	memset(&statusResponseRecord, 0, sizeof(statusResponseRecord));
}

static void statusResponseRecord_check(void)
{
	uint8_t i;
	bool bCheck = false;

	//don't check if ForcedStausUpdate is enabled
	if (mqtt_isForcedStausUpdate()) {
		return;
	}

	for(i = 0; i < MAX_STATUS_RESPONSE_RECORD; i++) {
		if(statusResponseRecord.record[i].bSendRespond) {
			bCheck = true;
			break;
		}
	}
	if (bCheck) {
		uint32_t now_ms = utils_time_ms_get();
		for(i = 0; i < MAX_STATUS_RESPONSE_RECORD; i++) {
			if(statusResponseRecord.record[i].bSendRespond) {
				if(utils_time_diff(now_ms, statusResponseRecord.record[i].receive_time_ms) > 1000) {
					mqtt_setForcedStausUpdate(true);
					statusResponseRecord.record[i].bSendRespond = false;
					statusResponseRecord.record[i].receive_time_ms = 0;
					break;
				}
			}
		}
	}
}


static uint32_t mqtt_uPublishIgnoreTime_last = 0;
static bool mqtt_bPublishIgnore = false;
static void mqtt_setPublishIgnore(void)
{
	mqtt_uPublishIgnoreTime_last = appCommon_GetTime_sec();
	mqtt_bPublishIgnore = true;
}

static bool mqtt_isPublishIgnore(void)
{
	if(mqtt_bPublishIgnore) {
		//ignore MQTT publish 2 seconds
		uint32_t now_time = appCommon_GetTime_sec();
		if (appCommon_TimeDiff(now_time, mqtt_uPublishIgnoreTime_last) >= 2) {
			mqtt_bPublishIgnore = false;
			return mqtt_bPublishIgnore;
		} else {
			return true;
		}
	} else {
		return false;
	}
}

static bool mqtt_bForcedStausUpdate = false;
static void mqtt_setForcedStausUpdate(bool bUpdate)
{
	mqtt_bForcedStausUpdate = bUpdate;
}

static bool mqtt_isForcedStausUpdate(void)
{
	return mqtt_bForcedStausUpdate;
}

static int mqtt_publish_FWver(void)
{
	int ret;
	esp_err_t err;
	time_t now = 0;
	struct tm timeinfo = { 0 };
	char topic_string[32];
	char FWver_string[32] = {0};
	size_t DeploymentId_len = 0;
	char* DeploymentId_ESP32 = NULL;
	char* DeploymentId_STM32 = NULL;
	char* DeploymentId_CERT = NULL;

	err = storage_ReadESP32OtaDeploymentId(NULL, &DeploymentId_len, 0);
	if ((err == ESP_OK) && (DeploymentId_len > 1)) {
		DeploymentId_ESP32 = malloc(DeploymentId_len);
		if (DeploymentId_ESP32) {
			memset(DeploymentId_ESP32, 0, DeploymentId_len);
			storage_ReadESP32OtaDeploymentId(DeploymentId_ESP32, &DeploymentId_len, DeploymentId_len);
		}
	}
	err = storage_ReadSTM32OtaDeploymentId(NULL, &DeploymentId_len, 0);
	if ((err == ESP_OK) && (DeploymentId_len > 1)) {
		DeploymentId_STM32 = malloc(DeploymentId_len);
		if (DeploymentId_STM32) {
			memset(DeploymentId_STM32, 0, DeploymentId_len);
			storage_ReadSTM32OtaDeploymentId(DeploymentId_STM32, &DeploymentId_len, DeploymentId_len);
		}
	}
	err = storage_ReadCertOtaDeploymentId(NULL, &DeploymentId_len, 0);
	if ((err == ESP_OK) && (DeploymentId_len > 1)) {
		DeploymentId_CERT = malloc(DeploymentId_len);
		if (DeploymentId_CERT) {
			memset(DeploymentId_CERT, 0, DeploymentId_len);
			storage_ReadCertOtaDeploymentId(DeploymentId_CERT, &DeploymentId_len, DeploymentId_len);
		}
	}

	time(&now);
	localtime_r(&now, &timeinfo);
	snprintf(topic_string, sizeof(topic_string), "%s%s%s", TOPIC_HEADER, HW_ID, "FW");
	//ESP32
	snprintf(FWver_string, sizeof(FWver_string), "%d.%d.%d", ESP_FW_inByte_Major, ESP_FW_inByte_Minor, ESP_FW_inByte_Sub);
	sprintf(cPayload, "{\"DeviceId\":\"%s\",\"FWtype\":\"%s\",\"CurrentFWver\":\"%s\",\"DeploymentId\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
		HW_ID, "ESP32", FWver_string, (DeploymentId_ESP32)?DeploymentId_ESP32:"",
		timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
	ret = esp_mqtt_client_publish(client, topic_string, cPayload, 0, 1, 0);
	ESP_LOGI(MQTT_TAG, "publish ESP32 FWver %s", (ret < 0)?"FAIL":"OK");
	//STM32
	if (firmwareVersion != 0x31)
	{
		vTaskDelay(200 / portTICK_PERIOD_MS);
		if (STM_FW_VER_bValid) {
			snprintf(FWver_string, sizeof(FWver_string), "%d.%d.%d", STM_FW_VER_Major, STM_FW_VER_Minor, STM_FW_VER_Sub);
		} else {
			snprintf(FWver_string, sizeof(FWver_string), "0.0.0");
		}
		sprintf(cPayload, "{\"DeviceId\":\"%s\",\"FWtype\":\"%s\",\"CurrentFWver\":\"%s\",\"BootloaderVersion\":\"%s\",\"DeploymentId\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
			HW_ID, "STM32", FWver_string, (STM_CT_BOOTLOADER_VER_bValid)?"b":"a", (DeploymentId_STM32)?DeploymentId_STM32:"",
			timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		ret = esp_mqtt_client_publish(client, topic_string, cPayload, 0, 1, 0);
		ESP_LOGI(MQTT_TAG, "publish STM32 FWver %s", (ret < 0)?"FAIL":"OK");
	}
	vTaskDelay(200 / portTICK_PERIOD_MS);
	//CERT
	cert_info_t* pCertInfo = certData_GetInfo();
	if (pCertInfo) {
		snprintf(FWver_string, sizeof(FWver_string), "%d.%d.%d", pCertInfo->version_major, pCertInfo->version_minor, pCertInfo->version_sub);
	} else {
		snprintf(FWver_string, sizeof(FWver_string), "0.0.0");
	}
	sprintf(cPayload, "{\"DeviceId\":\"%s\",\"FWtype\":\"%s\",\"CurrentFWver\":\"%s\",\"DeploymentId\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
		HW_ID, "CERT", FWver_string, (DeploymentId_CERT)?DeploymentId_CERT:"",
		timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
	ret = esp_mqtt_client_publish(client, topic_string, cPayload, 0, 1, 0);
	ESP_LOGI(MQTT_TAG, "publish CERT FWver %s", (ret < 0)?"FAIL":"OK");

	if (DeploymentId_ESP32) {
		free(DeploymentId_ESP32);
	}
	if (DeploymentId_STM32) {
		free(DeploymentId_STM32);
	}
	if (DeploymentId_CERT) {
		free(DeploymentId_CERT);
	}
	return ret;
}

static void mqtt_publish_status(void)
{
	time_t now = 0;
	struct tm timeinfo = { 0 };
	char topic_string[32];

	sprintf(topic_string, "%s%s", TOPIC_HEADER, HW_ID);
	time(&now);
	localtime_r(&now, &timeinfo);
	if (firmwareVersion == 0x11) //Room Sesor
	{
		sprintf(cPayload, "{\"DeviceId\":\"%s\",\"Bttn1RealState\":\"%u\",\"MOverride\":\"%s\",\"DeviceConfig\":\"%d\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
				HW_ID,
				Bttn1RealState,
				MOverride == 1 ? "true" : "false", SwitchConfig,
				timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		esp_mqtt_client_publish(client, topic_string, cPayload, 0, 1, 0);
	}
	else if (firmwareVersion == 0x21) //Wall Control
	{
		sprintf(cPayload, "{\"DeviceId\":\"%s\",\"FanRealState\":\"%u\",\"Bttn1RealState\":\"%u\",\"Bttn2RealState\":\"%u\",\"MOverride\":\"%s\",\"DeviceConfig\":\"%d\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
				HW_ID,
				FanRealState, Bttn1RealState, Bttn2RealState,
				MOverride == 1 ? "true" : "false", SwitchConfig,
				timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		esp_mqtt_client_publish(client, topic_string, cPayload, 0, 1, 0);
	}
	else if (firmwareVersion == 0x31) //Smart Plug
	{
		sprintf(cPayload, "{\"DeviceId\":\"%s\",\"FanRealState\":\"%u\",\"Bttn1RealState\":\"%u\",\"Bttn2RealState\":\"%u\",\"MOverride\":\"%s\",\"DeviceConfig\":\"%d\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}", 
				HW_ID,
				FanRealState, Bttn1RealState, Bttn2RealState,
				MOverride == 1 ? "true" : "false", SwitchConfig,
				timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
		esp_mqtt_client_publish(client, topic_string, cPayload, 0, 1, 0);
	}
	ESP_LOGI(MQTT_TAG, "publish_status");
}

void MessageTask(void *param)
{
	int msg_id;
	// int8_t wifiSSID;
	time_t now = 0;
    struct tm timeinfo = { 0 };
	char PTOPIC_SENSOR[32];
	char topic_string[32];
	sprintf(PTOPIC_SENSOR, "%s%s", TOPIC_HEADER, HW_ID);
	uint8_t iTry= 0;
	int res;
	
	while (1)
	{
		if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED){
			printf("WiFi RSSI = %d dBm\n", getWifiRSSI());
		}
		while (!mqtt_IsConnected() || wifi_GetConnectionState() != WIFI_STATE_CONNECTED)
		{
			if (BLE_IsConnectedAndEnabledNotify()) {
				//****************************** patrick flag flag
				if (wifi_GetConnectionState() != WIFI_STATE_CONNECTED
					&& wifi_GetConnectionState() != WIFI_STATE_CONNECTING
					&& BLE_ProvAckMaskIsSet(WAIT_WIFI_ACK_BIT)){
					printf("Now Disconnect and Reconnect Wifi.......\r\n");

					res = esp_wifi_disconnect();
					if (res == ESP_OK)
						printf("Wifi Disconnected\r\n");
					else
						printf("Wifi Disconnect failed, res = %d\r\n", res);
					res = esp_wifi_connect();
					if (res == ESP_OK){
						iTry = 0;
						printf("Wifi esp_wifi_connect OK, try time = %d\r\n", iTry);
					}else{
						iTry++;
						printf("Wifi esp_wifi_connect failed, res = %d, try time = %d\r\n", res, iTry);
					}
				}
			}

			if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
				//switchLedMode_031 = LEDMODE_WIFI_CONNECTED_NOTACTIVE;
				printf("WiFi RSSI = %d dBm\n", getWifiRSSI());
			}
			
			
			//ESP_LOGI(MQTT_TAG, "waiting for connection ...wifiState = %d, mqtt_is_connected =%d", wifiState, mqtt_is_connected);
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			if (!mqtt_IsConnected() || wifi_GetConnectionState() != WIFI_STATE_CONNECTED) {
				vTaskDelay(2000 / portTICK_PERIOD_MS);
			}
			if (mqtt_IsConnected()) {
				//send FWver again if MQTT connection is resumed
				bForcedMQTTSendFWver = true;
			}
		}
		/*
		esp_mqtt_client_subscribe(client, STOPIC_SYSTEM0, 1);
		esp_mqtt_client_subscribe(client, STOPIC_SYSTEM1, 1);
		esp_mqtt_client_subscribe(client, STOPIC_SYSTEM2, 1);
		esp_mqtt_client_subscribe(client, STOPIC_SYSTEM3, 1);
				
		// once you listen, it will dequeue the message and server side cannot get it
		esp_mqtt_client_subscribe(client, PTOPIC_SENSOR0, 0);
		esp_mqtt_client_subscribe(client, PTOPIC_SENSOR1, 0);
		esp_mqtt_client_subscribe(client, PTOPIC_SENSOR2, 0);
		esp_mqtt_client_subscribe(client, PTOPIC_SENSOR3, 0);
		esp_mqtt_client_subscribe(client, PTOPIC_SENSOR4, 0);
		esp_mqtt_client_subscribe(client, PTOPIC_CONFIG0, 0);
		*/
		
		//clear status response record
		statusResponseRecord_clearAll();

		printf("MQTT subscribe\n");
		//subscribe control topic
		snprintf(topic_string, sizeof(topic_string), "%s%s%s", TOPIC_HEADER, HW_ID, "CC");
		esp_mqtt_client_subscribe(client, topic_string, 1);
		//subscribe OTA topic
		snprintf(topic_string, sizeof(topic_string), "%s%s%s", TOPIC_HEADER, HW_ID, "OTA");
		esp_mqtt_client_subscribe(client, topic_string, 1);
		snprintf(topic_string, sizeof(topic_string), "%s%s%s", TOPIC_HEADER, HW_ID, "OTA_old");
		esp_mqtt_client_subscribe(client, topic_string, 1);
		//subscribe Util topic
		snprintf(topic_string, sizeof(topic_string), "%s%s%s", TOPIC_HEADER, HW_ID, "Util");
		esp_mqtt_client_subscribe(client, topic_string, 1);
		
		while (mqtt_IsConnected())
		{
			time(&now);
			localtime_r(&now, &timeinfo);
			//strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

			{
				//send FW version periodically
				static uint32_t publish_FWver_time = 0;
				uint32_t now_time = utils_time_sec_get();
				if (bForcedMQTTSendFWver || (now_time > publish_FWver_time)) {
					int ret;
					ret = mqtt_publish_FWver();
					if (ret < 0) {
						//send again after 30s
						publish_FWver_time = now_time + 30;
					} else {
						//next sending time is after 22~24 hours
						publish_FWver_time = now_time + 22*60*60 + (esp_random()%(2*60*60));
					}
					bForcedMQTTSendFWver = false;
				}
			}

			/*
			sprintf(cPayload, "{\"value\":\"%d\",\"time\":\"%d|%d|%d|%d|%d|%d\"}", 
					sensorVal[0], sensorLastUpdateTime[0].tm_year, sensorLastUpdateTime[0].tm_mon, sensorLastUpdateTime[0].tm_mday, sensorLastUpdateTime[0].tm_hour, sensorLastUpdateTime[0].tm_min, sensorLastUpdateTime[0].tm_sec);
			msg_id = esp_mqtt_client_publish(client, PTOPIC_SENSOR0, cPayload, 0, 0, 0);
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);
					
			sprintf(cPayload, "{\"value\":\"%d\",\"time\":\"%d|%d|%d|%d|%d|%d\"}", 
					sensorVal[1], sensorLastUpdateTime[1].tm_year, sensorLastUpdateTime[1].tm_mon, sensorLastUpdateTime[1].tm_mday, sensorLastUpdateTime[1].tm_hour, sensorLastUpdateTime[1].tm_min, sensorLastUpdateTime[1].tm_sec);
			msg_id = esp_mqtt_client_publish(client, PTOPIC_SENSOR1, cPayload, 0, 0, 0);
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);

			sprintf(cPayload, "{\"value\":\"%d\",\"time\":\"%d|%d|%d|%d|%d|%d\"}", 
					sensorVal[2], sensorLastUpdateTime[2].tm_year, sensorLastUpdateTime[2].tm_mon, sensorLastUpdateTime[2].tm_mday, sensorLastUpdateTime[2].tm_hour, sensorLastUpdateTime[2].tm_min, sensorLastUpdateTime[2].tm_sec);
			msg_id = esp_mqtt_client_publish(client, PTOPIC_SENSOR2, cPayload, 0, 0, 0);
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);

			sprintf(cPayload, "{\"value\":\"%d\",\"time\":\"%d|%d|%d|%d|%d|%d\"}", 
					sensorVal[3], sensorLastUpdateTime[3].tm_year, sensorLastUpdateTime[3].tm_mon, sensorLastUpdateTime[3].tm_mday, sensorLastUpdateTime[3].tm_hour, sensorLastUpdateTime[3].tm_min, sensorLastUpdateTime[3].tm_sec);
			msg_id = esp_mqtt_client_publish(client, PTOPIC_SENSOR3, cPayload, 0, 0, 0);
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);
			
			sprintf(cPayload, "{\"value\":\"%d\",\"time\":\"%d|%d|%d|%d|%d|%d\"}", 
					sensorVal[4], sensorLastUpdateTime[4].tm_year, sensorLastUpdateTime[4].tm_mon, sensorLastUpdateTime[4].tm_mday, sensorLastUpdateTime[4].tm_hour, sensorLastUpdateTime[4].tm_min, sensorLastUpdateTime[4].tm_sec);
			msg_id = esp_mqtt_client_publish(client, PTOPIC_SENSOR4, cPayload, 0, 0, 0);
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);
			
			sprintf(cPayload, "{\"value\":\"%d\",\"time\":\"%d|%d|%d|%d|%d|%d\"}", 
					configVal[0], configLastUpdateTime[0].tm_year, configLastUpdateTime[0].tm_mon, configLastUpdateTime[0].tm_mday, configLastUpdateTime[0].tm_hour, configLastUpdateTime[0].tm_min, configLastUpdateTime[0].tm_sec);
			msg_id = esp_mqtt_client_publish(client, PTOPIC_CONFIG0, cPayload, 0, 0, 0);
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);
			*/
			//------------------------------------------2019.12.11
			if (uart_getSenorDataFlag()) {
				if (isNewData)
					isNewDataFalseCnt = 0;
				else 
					isNewDataFalseCnt++;
				
				if (isNewDataFalseCnt >= 10)
				{
					isNewDataFalseCnt = 0;
				#ifdef BROAN_SECURE_RELEASE
					if (!STM_FW_SIGN_bFail)
				#else
					if (1)
				#endif
					{
						ESP_LOGE(APP_TAG, "fail to get sensor data, reset STM!!!");
#ifdef USE_STM32_CT_BOOTLOADER
						Suspend_SensorTask = 1;
						Suspend_Uart = 1;
						vTaskDelay(400/portTICK_RATE_MS);
						uart_flushData();
#endif
						gpio_set_level(STM_RST_PIN, 0);
						vTaskDelay(100/portTICK_RATE_MS);
						gpio_set_level(STM_RST_PIN, 1);
#ifdef USE_STM32_CT_BOOTLOADER
						vTaskDelay(200/portTICK_RATE_MS);
						BootLoader_GoCmd();
						vTaskDelay(200/portTICK_RATE_MS);
						Suspend_SensorTask = 0;
						Suspend_Uart = 0;
#endif

						//don't request sensor data for 8*2=16 seconds
						uart_skipToSendSensorDataCmd(8);

						//Update: avid WiFi disconnection
						////disconnect WiFi to make sure that all necessary commands send to STM32 again
						//if(wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
						//	esp_wifi_disconnect();
						//}
					}
				}
			}

			int8_t rssi = getWifiRSSI();
			printf("WiFi RSSI = %d dBm\n", rssi);

			switch(firmwareVersion)
			{
				default:
				case 0x11://Room Sesor
					/*
						{"DeviceId":"RS19XXXXXXX","NewData":"true",
						"Temperature":"0,00", "Humidity":"0.00","TVOC":"0.00","CO2":"0,00","PM2.5":"0.00", 
						"SlopeTmp":"0.00","SlopeRH":"0.00","SlopeTVOC":"0.00","SlopeCO2":"0.00", "SlopePM2.5":"0.00",
						"Bttn1RealState":"0",
						"MOverride":"false", "TimeStamp":"119|9|29|14|47|26"}
					*/
					{
					/*
					sprintf(cPayload, "{\"DeviceId\":\"%s\",\"NewData\":\"%s\",\"Temperature\":\"%0.2f\",\"Humidity\":\"%0.2f\",\"TVOC\":\"%0.2f\",\"CO2\":\"%0.2f\",\"PM2.5\":\"%0.2f\",\"SCD30_temp\":\"%0.2f\",\"SCD30_RH\":\"%0.2f\",\"NTC1\":\"%0.2f\",\"NTC2\":\"%0.2f\",\"SlopePM2.5\":\"%0.2f\",\"Bttn1RealState\":\"%u\",\"MOverride\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}", 
							HW_ID, isNewData == 1 ? "true" : "false",
							sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3], sensorVal[4],
							sensorVal[5],
							sensorVal[6],
							sensorVal[7],
							sensorVal[8],
							(float)(sensorVal[4]-lastSensorSentVal[4])/5.0f,
							Bttn1RealState, 
							MOverride == 1 ? "true" : "false", timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
											
					*/

					
					sprintf(cPayload, "{\"DeviceId\":\"%s\",\"NewData\":\"%s\",\"Temperature\":\"%0.2f\",\"Humidity\":\"%0.2f\",\"TVOC\":\"%0.2f\",\"CO2\":\"%0.2f\",\"PM2.5\":\"%0.2f\",\"SlopeTmp\":\"%0.2f\",\"SlopeRH\":\"%0.2f\",\"SlopeTVOC\":\"%0.2f\",\"SlopeCO2\":\"%0.2f\",\"SlopePM2.5\":\"%0.2f\",", 
							HW_ID, isNewData == 1 ? "true" : "false",
							sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3], sensorVal[4],
							(float)(sensorVal[0]-lastSensorSentVal[0])/5.0f, 
							(float)(sensorVal[1]-lastSensorSentVal[1])/5.0f, 
							(float)(sensorVal[2]-lastSensorSentVal[2])/5.0f, 
							(float)(sensorVal[3]-lastSensorSentVal[3])/5.0f, 
							(float)(sensorVal[4]-lastSensorSentVal[4])/5.0f);
					if (appCommon_GetTestMode() || Debug_bMQTTRaw)
					{
						TempSensorRawData_t TempRawData = {0};
						appCommon_GetTempSensorRawData(&TempRawData);
						sprintf(cPayload + strlen(cPayload), "\"RawTemp\":\"%0.2f\",\"RawHumidity\":\"%0.2f\",\"RawNTC1\":\"%0.2f\",\"RawNTC2\":\"%0.2f\",\"RawNTC3\":\"%0.2f\",",
								TempRawData.RawTemp, TempRawData.RawHumidity, TempRawData.RawNTC1, TempRawData.RawNTC2, TempRawData.RawNTC3);
						if (Debug_bMQTTRaw) {
							Debug_bMQTTRaw = false;
						}
					}
					sprintf(cPayload + strlen(cPayload), "\"Bttn1RealState\":\"%u\",\"MOverride\":\"%s\",\"DeviceConfig\":\"%d\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
							Bttn1RealState, 
							MOverride == 1 ? "true" : "false", SwitchConfig,
							timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
					
					}break;
				
				case 0x21://Wall Control
					/*
					sprintf(cPayload, "{\"DeviceId\":\"%s\",\"NewData\":\"%s\",\"Temperature\":\"%0.2f\",\"Humidity\":\"%0.2f\",\"TVOC\":\"%0.2f\",\"CO2\":\"%0.2f\",\"SCD30_temp\":\"%0.2f\",\"SCD30_RH\":\"%0.2f\",\"NTC1\":\"%0.2f\",\"NTC2\":\"%0.2f\",\"FanRealState\":\"%ud\",\"Bttn1RealState\":\"%ud\",\"Bttn2RealState\":\"%ud\",\"MOverride\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}", 
							HW_ID, isNewData == 1 ? "true" : "false",
							sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3],
							sensorVal[5],
							sensorVal[6],
							sensorVal[7],
							sensorVal[8],
							FanRealState, Bttn1RealState, Bttn2RealState, 
							MOverride == 1 ? "true" : "false", timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
						
					*/
					{
					
					sprintf(cPayload, "{\"DeviceId\":\"%s\",\"NewData\":\"%s\",\"Temperature\":\"%0.2f\",\"Humidity\":\"%0.2f\",\"TVOC\":\"%0.2f\",\"CO2\":\"%0.2f\",\"SlopeTmp\":\"%0.2f\",\"SlopeRH\":\"%0.2f\",\"SlopeTVOC\":\"%0.2f\",\"SlopeCO2\":\"%0.2f\",", 
							HW_ID, isNewData == 1 ? "true" : "false",
							sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3],
							(float)(sensorVal[0]-lastSensorSentVal[0])/5.0f, 
							(float)(sensorVal[1]-lastSensorSentVal[1])/5.0f, 
							(float)(sensorVal[2]-lastSensorSentVal[2])/5.0f, 
							(float)(sensorVal[3]-lastSensorSentVal[3])/5.0f);
					if (appCommon_GetTestMode() || Debug_bMQTTRaw)
					{
						TempSensorRawData_t TempRawData = {0};
						appCommon_GetTempSensorRawData(&TempRawData);
						sprintf(cPayload + strlen(cPayload), "\"RawTemp\":\"%0.2f\",\"RawHumidity\":\"%0.2f\",\"RawNTC1\":\"%0.2f\",\"RawNTC2\":\"%0.2f\",\"RawNTC3\":\"%0.2f\",",
								TempRawData.RawTemp, TempRawData.RawHumidity, TempRawData.RawNTC1, TempRawData.RawNTC2, TempRawData.RawNTC3);
						if (Debug_bMQTTRaw) {
							Debug_bMQTTRaw = false;
						}
					}
					sprintf(cPayload + strlen(cPayload), "\"FanRealState\":\"%u\",\"Bttn1RealState\":\"%u\",\"Bttn2RealState\":\"%u\",\"MOverride\":\"%s\",\"DeviceConfig\":\"%d\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}",
							FanRealState, Bttn1RealState, Bttn2RealState, 
							MOverride == 1 ? "true" : "false", SwitchConfig,
							timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
					
					/*
					sprintf(cPayload, "{\"DeviceId\":\"%s\",\"NewData\":\"%s\",\"Temperature\":\"%0.2f\",\"Humidity\":\"%0.2f\",\"TVOC\":\"%0.2f\",\"CO2\":\"%0.2f\",\"SCD30_temp\":\"%0.2f\",\"SCD30_RH\":\"%0.2f\",\"NTC1\":\"%0.2f\",\"NTC2\":\"%0.2f\",\"FanRealState\":\"%u\",\"Bttn1RealState\":\"%u\",\"Bttn2RealState\":\"%u\",\"MOverride\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}", 
							HW_ID, isNewData == 1 ? "true" : "false",
							sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3],
							sensorVal[5],
							sensorVal[6],
							sensorVal[7],
							sensorVal[8],
							FanRealState, Bttn1RealState, Bttn2RealState, 
							MOverride == 1 ? "true" : "false", timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
					*/
					
						
						
					}break;
				
				case 0x31://Smart Plug
					/*
						{"DeviceId":"SP19XXXXXXX","NewData":"true","
						FanRealState":"0", 
						"MOverride":"false", "TimeStamp":"119|9|29|14|47|26"}
					*/
					{
					sprintf(cPayload, "{\"DeviceId\":\"%s\",\"FanRealState\":\"%u\",\"Bttn1RealState\":\"%u\",\"Bttn2RealState\":\"%u\",\"MOverride\":\"%s\",\"DeviceConfig\":\"%d\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}", 
							HW_ID,
							FanRealState, Bttn1RealState, Bttn2RealState,
							MOverride == 1 ? "true" : "false", SwitchConfig,
							timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
					if (appCommon_GetTestMode() || Debug_bMQTTRaw) {
						if (Debug_bMQTTRaw) {
							Debug_bMQTTRaw = false;
						}
					}
					
					}break;
			}
			//------------------------------------------
			/* sprintf(cPayload, "{\"DeviceId\":\"%s\",\"NewData\":\"%s\",\"Temperature\":\"%0.2f\",\"Humidity\":\"%0.2f\",\"TVOC\":\"%0.2f\",\"CO2\":\"%0.2f\",\"PM2.5\":\"%0.2f\",\"SlopeTmp\":\"%0.2f\",\"SlopeRH\":\"%0.2f\",\"SlopeTVOC\":\"%0.2f\",\"SlopeCO2\":\"%0.2f\",\"SlopePM2.5\":\"%0.2f\",\"FanRealState\":\"%d\",\"MOverride\":\"%s\",\"TimeStamp\":\"%d|%d|%d|%d|%d|%d\"}", 
					HW_ID, isNewData == 1 ? "true" : "false",
					sensorVal[0], sensorVal[1], sensorVal[2], sensorVal[3], sensorVal[4],
					(float)(sensorVal[0]-lastSensorSentVal[0])/5.0f, 
					(float)(sensorVal[1]-lastSensorSentVal[1])/5.0f, 
					(float)(sensorVal[2]-lastSensorSentVal[2])/5.0f, 
					(float)(sensorVal[3]-lastSensorSentVal[3])/5.0f, 
					(float)(sensorVal[4]-lastSensorSentVal[4])/5.0f,
					FanRealState, "false",
					timeinfo.tm_year, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
					 */
			//------------------------------------------
			isNewData = 0;
			if (mqtt_isPublishIgnore()) {
				ESP_LOGW(MQTT_TAG, "Ignore publish!");
			} else {
				msg_id = esp_mqtt_client_publish(client, PTOPIC_SENSOR, cPayload, 0, 0, 0);
			}
			lastSensorSentVal[0] = sensorVal[0];//Temperature
			lastSensorSentVal[1] = sensorVal[1];//Humidity
			lastSensorSentVal[2] = sensorVal[2];//TVOC
			lastSensorSentVal[3] = sensorVal[3];//CO2
			lastSensorSentVal[4] = sensorVal[4];//PM2.5
			
			//ESP_LOGI(MQTT_TAG, "%s msg_id %d", cPayload, msg_id);
			
			//-------------------
			
			uint32_t i;
			for (i=0; i<50; i++) {
				vTaskDelay(100 / portTICK_PERIOD_MS);
				if (mqtt_IsConnected()) {
					statusResponseRecord_check();
					if (mqtt_isForcedStausUpdate()) {
						mqtt_publish_status();
						mqtt_setForcedStausUpdate(false);
					}
				}
				if (Suspend_MsgTask) {
					break;
				}
			}
			if (mqtt_IsReconnectAction()) {
				if (mqtt_IsConnected()) {
					mqtt_app_stop();
					mqtt_app_start();
				}
				mqtt_SetReconnectAction(false);
			}
			if (Suspend_MsgTask) {
				ESP_LOGI(MQTT_TAG, "stop MsgTask");
				mqtt_app_stop();
				while (Suspend_MsgTask) {
					vTaskDelay(100 / portTICK_PERIOD_MS);
				}
				ESP_LOGI(MQTT_TAG, "start MsgTask");
				bForcedMQTTSendFWver = true;
				mqtt_app_start();
			}

		}
	}
}


//DeviceConfig settings
typedef struct {
	uint8_t Output1;
	uint8_t Output2;
} DeviceConfigOutput_t;

typedef struct {
	const DeviceConfigOutput_t *pOutput;
	size_t size;
	bool bFanAlgoCmd;
} DeviceConfigOutputTable_t;

#define DEVICECONFIG_TABLE_031_CNT		6
// DeviceConfig=1, ERV/HRV Only
static const DeviceConfigOutput_t DeviceConfigOutput_1_031[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_DISABLE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_DISABLE},
};
// DeviceConfig=2, ERV/HRV + 120V Device
static const DeviceConfigOutput_t DeviceConfigOutput_2_031[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_IGNORE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_IGNORE},
};
// DeviceConfig=3, Supply Fan Only
static const DeviceConfigOutput_t DeviceConfigOutput_3_031[2] =
{
	{DEVICECONFIG_OUTPUT_DISABLE,	DEVICECONFIG_OUTPUT_OFF},
	{DEVICECONFIG_OUTPUT_DISABLE,	DEVICECONFIG_OUTPUT_ON},
};
// DeviceConfig=4, MUAD Only
static const DeviceConfigOutput_t DeviceConfigOutput_4_031[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_DISABLE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_DISABLE},
};
// DeviceConfig=5, MUAD + 120V Device
static const DeviceConfigOutput_t DeviceConfigOutput_5_031[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_IGNORE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_IGNORE},
};
// DeviceConfig=6, 120V Device Only (No FanAlgoState)
static const DeviceConfigOutput_t DeviceConfigOutput_6_031[1] =
{
	{DEVICECONFIG_OUTPUT_DISABLE,	DEVICECONFIG_OUTPUT_IGNORE},
};

static DeviceConfigOutputTable_t DeviceConfigTable_031[DEVICECONFIG_TABLE_031_CNT+1] =
{
	{NULL, 0, false},
	{DeviceConfigOutput_1_031, sizeof(DeviceConfigOutput_1_031)/sizeof(DeviceConfigOutput_1_031[0]), true},
	{DeviceConfigOutput_2_031, sizeof(DeviceConfigOutput_2_031)/sizeof(DeviceConfigOutput_2_031[0]), true},
	{DeviceConfigOutput_3_031, sizeof(DeviceConfigOutput_3_031)/sizeof(DeviceConfigOutput_3_031[0]), true},
	{DeviceConfigOutput_4_031, sizeof(DeviceConfigOutput_4_031)/sizeof(DeviceConfigOutput_4_031[0]), true},
	{DeviceConfigOutput_5_031, sizeof(DeviceConfigOutput_5_031)/sizeof(DeviceConfigOutput_5_031[0]), true},
	{DeviceConfigOutput_6_031, sizeof(DeviceConfigOutput_6_031)/sizeof(DeviceConfigOutput_6_031[0]), false},
};

bool device_isValidDeviceConfig_031(uint8_t _DeviceConfig)
{
	if (_DeviceConfig == 0) {
		return false;
	}
	if(_DeviceConfig > DEVICECONFIG_TABLE_031_CNT) {
		return false;
	}
	return true;
}

static const DeviceConfigOutput_t* device_getDeviceConfigOutput_031(uint8_t _DeviceConfig, uint8_t _FanAlgoState)
{
	if (_DeviceConfig == 0) {
		return NULL;
	}
	if(_DeviceConfig > DEVICECONFIG_TABLE_031_CNT) {
		return NULL;
	}

	if(DeviceConfigTable_031[_DeviceConfig].bFanAlgoCmd) {
		if (_FanAlgoState >= DeviceConfigTable_031[_DeviceConfig].size) {
			//FanAlgoState exceeds
			return NULL;
		}
		return &DeviceConfigTable_031[_DeviceConfig].pOutput[_FanAlgoState];
	} else {
		//no FanAlgo command
		return &DeviceConfigTable_031[_DeviceConfig].pOutput[0];
	}
}

static uint8_t device_getOutput1Config_internal_031(uint8_t _DeviceConfig, uint8_t _FanAlgoState)
{
	const DeviceConfigOutput_t* pOutput;
	pOutput = device_getDeviceConfigOutput_031(_DeviceConfig, _FanAlgoState);
	if (!pOutput) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	if (pOutput->Output1 >= NUM_OF_DEVICECONFIG_OUTPUT) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	return pOutput->Output1;
}

static uint8_t device_getOutput2Config_internal_031(uint8_t _DeviceConfig, uint8_t _FanAlgoState)
{
	const DeviceConfigOutput_t* pOutput;
	pOutput = device_getDeviceConfigOutput_031(_DeviceConfig, _FanAlgoState);
	if (!pOutput) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	if (pOutput->Output2 >= NUM_OF_DEVICECONFIG_OUTPUT) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	return pOutput->Output2;
}

//input = DeviceConfig, Bttn1RealState, Bttn2RealState
//output = FanRealState
uint8_t device_calcFanRealState_031(uint8_t _DeviceConfig, uint8_t _Bttn1RealState, uint8_t _Bttn2RealState)
{
	//check DeviceConfig
	if(!device_isValidDeviceConfig_031(_DeviceConfig)) {
		return 0;
	}
	//check FanAlgo command
	if(!DeviceConfigTable_031[_DeviceConfig].bFanAlgoCmd) {
		return 0;
	}

	uint8_t i;
	const DeviceConfigOutput_t *pOutput;
	uint8_t state = 0;
	for(i = 0; i < DeviceConfigTable_031[_DeviceConfig].size; i++) {
		bool bResult_Output1 = false;
		bool bResult_Output2 = false;
		pOutput = &DeviceConfigTable_031[_DeviceConfig].pOutput[i];
		if (!pOutput) {
			continue;
		}
		if((pOutput->Output1 == DEVICECONFIG_OUTPUT_DISABLE)
			||(pOutput->Output1 == DEVICECONFIG_OUTPUT_IGNORE))
		{
			bResult_Output1 = true;
		}
		if((pOutput->Output2 == DEVICECONFIG_OUTPUT_DISABLE)
			||(pOutput->Output2 == DEVICECONFIG_OUTPUT_IGNORE))
		{
			bResult_Output2 = true;
		}
		if(!bResult_Output1)
		{
			if(pOutput->Output1 == DEVICECONFIG_OUTPUT_OFF && _Bttn1RealState == 0) {
				bResult_Output1 = true;
			}
			if(pOutput->Output1 == DEVICECONFIG_OUTPUT_ON && _Bttn1RealState) {
				bResult_Output1 = true;
			}
		}
		if(!bResult_Output2)
		{
			if(pOutput->Output2 == DEVICECONFIG_OUTPUT_OFF && _Bttn2RealState == 0) {
				bResult_Output2 = true;
			}
			if(pOutput->Output2 == DEVICECONFIG_OUTPUT_ON && _Bttn2RealState) {
				bResult_Output2 = true;
			}
		}

		if(bResult_Output1 && bResult_Output2)
		{
			state = i;
			break;
		}
	}
	return state;
}

//input = FanAlgoState, FanRealState
//output = MOverride
uint8_t device_calcMOverride_031(uint8_t _DeviceConfig, uint8_t _FanAlgoState, uint8_t _FanRealState)
{
	//check DeviceConfig
	if(!device_isValidDeviceConfig_031(_DeviceConfig)) {
		return 0;
	}
	//check FanAlgo command
	if(!DeviceConfigTable_031[_DeviceConfig].bFanAlgoCmd) {
		return 0;
	}

	if(_FanAlgoState == _FanRealState) {
		return 0;
	} else {
		return 1;
	}
}

static bool bAlgoActive = false;
static uint8_t output1_config = DEVICECONFIG_OUTPUT_IGNORE;
static uint8_t output2_config = DEVICECONFIG_OUTPUT_IGNORE;
bool device_isAlgoActive_031(void)
{
	return bAlgoActive;
}

uint8_t device_getOutput1Config_031(void)
{
	return output1_config;
}

uint8_t device_getOutput2Config_031(void)
{
	return output2_config;
}

void device_updateResult_031(void)
{
	FanRealState = device_calcFanRealState_031(SwitchConfig, Bttn1RealState, Bttn2RealState);
	MOverride = device_calcMOverride_031(SwitchConfig, FanAlgoState, FanRealState);
	if ((FanAlgoState > 0) && (MOverride == 0)) {
		bAlgoActive = true;
	} else {
		bAlgoActive = false;
	}
	output1_config = device_getOutput1Config_internal_031(SwitchConfig, FanAlgoState);
	output2_config = device_getOutput2Config_internal_031(SwitchConfig, FanAlgoState);
}


void output_Init(void)
{
	gpio_set_direction(OUTPUT1_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(OUTPUT1_PIN, 0);
	gpio_set_direction(OUTPUT2_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(OUTPUT2_PIN, 0);
}

void output_1_SetLevel(bool bOn)
{
	if (bOn) {
		gpio_set_level(OUTPUT1_PIN, 1);
		Bttn1RealState = 1;
	} else {
		gpio_set_level(OUTPUT1_PIN, 0);
		Bttn1RealState = 0;
	}
}

void output_1_ToggleLevel(void)
{
	if (Bttn1RealState) {
		output_1_SetLevel(0);
	} else {
		output_1_SetLevel(1);
	}
}

void output_2_SetLevel(bool bOn)
{
	if (bOn) {
		gpio_set_level(OUTPUT2_PIN, 1);
		Bttn2RealState = 1;
	} else {
		gpio_set_level(OUTPUT2_PIN, 0);
		Bttn2RealState = 0;
	}
}

void output_2_ToggleLevel(void)
{
	if (Bttn2RealState) {
		output_2_SetLevel(0);
	} else {
		output_2_SetLevel(1);
	}
}


// #define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_TIMER_0          LEDC_TIMER_0
#define LEDC_HS_TIMER_1          LEDC_TIMER_1
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

#define LEDC_HS_CH0_GPIO       (26)//led1
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (33)//led2
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

// #define LEDC_LS_TIMER          LEDC_TIMER_1
// #define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

ledc_channel_config_t led_1_config = {
	.channel    = LEDC_HS_CH0_CHANNEL,
	.duty       = 0,
	.gpio_num   = LEDC_HS_CH0_GPIO,
	.speed_mode = LEDC_HS_MODE,
	.hpoint     = 0,
	.timer_sel  = LEDC_HS_TIMER_0
};
								
ledc_channel_config_t led_2_config = {
	.channel    = LEDC_HS_CH1_CHANNEL,
	.duty       = 0,
	.gpio_num   = LEDC_HS_CH1_GPIO,
	.speed_mode = LEDC_HS_MODE,
	.hpoint     = 0,
	// .timer_sel  = LEDC_HS_TIMER_1
	.timer_sel  = LEDC_HS_TIMER_0
};

enum {
	LED_STATE_OFF = 0,
	LED_STATE_ON,
	LED_STATE_FADEUP,
	LED_STATE_FADEDOWN,
	NUM_OF_LED_STATE
};
static uint8_t LED1_state = LED_STATE_OFF;
static uint8_t LED2_state = LED_STATE_OFF;
void LED_1_On(void)
{
	//printf("LED 1 set duty = 1 without fade\n");	
	ledc_stop(	led_1_config.speed_mode,
				led_1_config.channel,
				1
				);
	LED1_state = LED_STATE_ON;
}
void LED_1_Off(void)
{
	//printf("LED 1 set duty = 0 without fade\n");	
	ledc_stop(	led_1_config.speed_mode,
				led_1_config.channel,
				0
				);
	LED1_state = LED_STATE_OFF;
}
void LED_1_FadeUp(uint16_t FadeTime)
{
	// printf("LED 1 fade up to duty = %d\n", LEDC_TEST_DUTY);
	ledc_set_fade_with_time(	led_1_config.speed_mode,
								led_1_config.channel,
								LEDC_TEST_DUTY,
								FadeTime
								);
	ledc_fade_start(	led_1_config.speed_mode,
						led_1_config.channel,
						LEDC_FADE_NO_WAIT
						);			
	LED1_state = LED_STATE_FADEUP;
}
void LED_1_FadeDown(uint16_t FadeTime)
{
	// printf("LED 1 fade down to duty = 0\n");
	ledc_set_fade_with_time(	led_1_config.speed_mode,
								led_1_config.channel,
								0,
								FadeTime
								);
	ledc_fade_start(	led_1_config.speed_mode,
						led_1_config.channel,
						LEDC_FADE_NO_WAIT
						);
	LED1_state = LED_STATE_FADEDOWN;
}
void LED_2_On(void)
{
	//printf("LED 2 set duty = 1 without fade\n");	
	ledc_stop(	led_2_config.speed_mode,
				led_2_config.channel,
				1
				);
	LED2_state = LED_STATE_ON;
}
void LED_2_Off(void)
{
	//printf("LED 2 set duty = 0 without fade\n");	
	ledc_stop(	led_2_config.speed_mode,
				led_2_config.channel,
				0
				);
	LED2_state = LED_STATE_OFF;
}
void LED_2_FadeUp(uint16_t FadeTime)
{
	// printf("LED 2 fade up to duty = %d\n", LEDC_TEST_DUTY);
	ledc_set_fade_with_time(	led_2_config.speed_mode,
								led_2_config.channel,
								LEDC_TEST_DUTY,
								FadeTime
								);
	ledc_fade_start(	led_2_config.speed_mode,
						led_2_config.channel,
						LEDC_FADE_NO_WAIT
						);
	LED2_state = LED_STATE_FADEUP;
}
void LED_2_FadeDown(uint16_t FadeTime)
{
	// printf("LED 2 fade down to duty = 0\n");
	ledc_set_fade_with_time(	led_2_config.speed_mode,
								led_2_config.channel,
								0,
								FadeTime
								);
	ledc_fade_start(	led_2_config.speed_mode,
						led_2_config.channel,
						LEDC_FADE_NO_WAIT
						);
	LED2_state = LED_STATE_FADEDOWN;
}

uint8_t LED_1_GetState(void)
{
	return LED1_state;
}

uint8_t LED_2_GetState(void)
{
	return LED2_state;
}


#define BTN_ID_1		0x0
#define BTN_ID_2		0x1
#define BTN_ID_BOTH		0x2	//both 1 and 2
#define BTN_EVENT_SHORT_RELEASED	0
#define BTN_EVENT_LONG_PRESSED		1
#define BTN_EVENT_LONG_RELEASED		2
static bool bIgnoreBothBtnLongReleased = false;
static uint32_t button_ignore_time = 0;
static void Button_event(uint8_t buttonID, uint8_t event)
{
	if (bIgnoreBothBtnLongReleased) {
		if (buttonID == BTN_ID_BOTH && event == BTN_EVENT_LONG_RELEASED) {
			bIgnoreBothBtnLongReleased = false;
			return;
		}
	}
	if (button_ignore_time) {
		if(button_ignore_time > appCommon_GetTime_sec()) {
			return;
		} else {
			button_ignore_time = 0;
		}
	}
	if (BLE_IsEnabled()) {
		if (event == BTN_EVENT_SHORT_RELEASED || event == BTN_EVENT_LONG_RELEASED) {
			ESP_LOGI(APP_TAG, "Disable Bluetooth by any keys");
			Call_BT_disable = 1;
		}
		return;
	}

	if (buttonID == BTN_ID_1)
	{
		if (event == BTN_EVENT_SHORT_RELEASED || event == BTN_EVENT_LONG_RELEASED) {
			ESP_LOGI(APP_TAG, "Button 1 released !!");
			if (device_getOutput1Config_031() != DEVICECONFIG_OUTPUT_DISABLE) {
				output_1_ToggleLevel();
				device_updateResult_031();
			}
		}
	}
	else if (buttonID == BTN_ID_2)
	{
		if (event == BTN_EVENT_SHORT_RELEASED || event == BTN_EVENT_LONG_RELEASED) {
			ESP_LOGI(APP_TAG, "Button 2 released !!");
			if (device_getOutput2Config_031() != DEVICECONFIG_OUTPUT_DISABLE) {
				output_2_ToggleLevel();
				device_updateResult_031();
			}
		}
	}
	else if(buttonID == BTN_ID_BOTH)
	{
		if (event == BTN_EVENT_LONG_PRESSED) {
			ESP_LOGI(APP_TAG, "Both Button long pressed !!");
			if (!BLE_IsEnabled()) {
				switchLedMode_031 = LEDMODE_BT_PAIRING_MODE;
				BLE_Start();
				bIgnoreBothBtnLongReleased = true; //ignore long released event once
				button_ignore_time = appCommon_GetTime_sec() + 2;
			}
		} else if (event == BTN_EVENT_LONG_RELEASED) {
			ESP_LOGI(APP_TAG, "Both Button long released !!");
			
		}
	}
}

uint8_t switchLedMode_031 = 0;
void ButtonTask(void *param)
{
/*
	gpio_config_t output_conf, input_conf, input_pullHigh_conf;

	// Configure GPIO Number As Ouput Pin
	output_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	output_conf.mode = GPIO_MODE_OUTPUT;
	//output_conf.pin_bit_mask =   GPIO_SEL_0  | GPIO_SEL_2  | GPIO_SEL_4  | GPIO_SEL_5
	//							| GPIO_SEL_12 | GPIO_SEL_13 | GPIO_SEL_14 | GPIO_SEL_15 | GPIO_SEL_16 | GPIO_SEL_17 | GPIO_SEL_18 | GPIO_SEL_19
	//							| GPIO_SEL_21 | GPIO_SEL_22 | GPIO_SEL_23 | GPIO_SEL_25 | GPIO_SEL_26 | GPIO_SEL_27 | GPIO_SEL_32 | GPIO_SEL_33 
	//							| GPIO_SEL_34 | GPIO_SEL_35 | GPIO_SEL_35 | GPIO_SEL_39;
	output_conf.pin_bit_mask =   GPIO_SEL_25 | GPIO_SEL_33;
	output_conf.pull_down_en = 0;
	output_conf.pull_up_en = 0;
	gpio_config(&output_conf);

	//Configure GPIO Number As Input Pin
	input_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	input_conf.mode = GPIO_MODE_INPUT;
	input_conf.pin_bit_mask =   GPIO_SEL_8 | GPIO_SEL_9;
	input_conf.pull_down_en = 0;
	input_conf.pull_up_en = 0;
	gpio_config(&input_conf);
*/
	uint8_t switchLedMode_previous = -1;
	uint16_t counterLED = 0;
	//uint8_t Buffer_Bttn1RealState = 0;
	//uint8_t Buffer_Bttn2RealState = 0;
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        // .speed_mode = LEDC_HS_MODE,           // timer mode
        // .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER_0;
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for high speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER_1;
    ledc_timer_config(&ledc_timer);
	
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */

    // Set LED Controller with previously prepared configuration
	ledc_channel_config(&led_1_config);
	ledc_channel_config(&led_2_config);
	
    // Initialize fade service.
    ledc_fade_func_install(0);
	
	//------------------------------------------------------------2020.01.02
	
	LED_1_Off();
	LED_2_Off();
	Bttn1RealState = 0;
	Bttn2RealState = 0;	
	device_updateResult_031();
	
	//------------------------------------------------------------
	
	//----------------------Relay
	if(firmwareVersion == 0x31){
		output_Init();
		FanRealState = 0;
	}
	//----------------------
	
	gpio_set_pull_mode(BUTTON1_PIN, GPIO_PULLUP_ONLY);//button
	gpio_set_direction(BUTTON1_PIN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON2_PIN, GPIO_PULLUP_ONLY);//button
	gpio_set_direction(BUTTON2_PIN, GPIO_MODE_INPUT);	
	
	
	while (1)
	{
		//--------------------------------------------------//button 1
		Btn1Status_curr = gpio_get_level(BUTTON1_PIN);
		
		if (Btn1Status_curr == 0)
		{
			if (Btn1Status_B4 != Btn1Status_curr){	//detect falling
				Btn1_detectFall_Cnt = 1;
				Btn1Status_B4 = Btn1Status_curr;
			}
			if (Btn1_detectFall_Cnt == 1){
				But1PressedCnt ++;
			}
			if (DualBtn_Detected == 0) {
				if (But1PressedCnt == 500) {
					Button_event(BTN_ID_1, BTN_EVENT_LONG_PRESSED);
				}
			}
		}
		else
		{
			
			if (Btn1Status_B4 != Btn1Status_curr){	//detect rising
				Btn1_detectFall_Cnt = 0;
				Btn1Status_B4 = Btn1Status_curr;
				if (DualBtn_Detected == 0){
					if (But1PressedCnt >= 3 && But1PressedCnt < 500){		//detect short press
						Button_event(BTN_ID_1, BTN_EVENT_SHORT_RELEASED);
					}else if (But1PressedCnt >= 500){		//detect hold (long) press
						Button_event(BTN_ID_1, BTN_EVENT_LONG_RELEASED);
					}
					But1PressedCnt = 0;
				}
			}
		}
		//--------------------------------------------------//button 2
		Btn2Status_curr = gpio_get_level(BUTTON2_PIN);
		
		if (Btn2Status_curr == 0)
		{
			if (Btn2Status_B4 != Btn2Status_curr) {	//detect falling
				Btn2_detectFall_Cnt = 1;
				Btn2Status_B4 = Btn2Status_curr;
			}
			if (Btn2_detectFall_Cnt == 1) {
				But2PressedCnt ++;
			}
			if (DualBtn_Detected == 0) {
				if (But2PressedCnt == 500) {
					Button_event(BTN_ID_2, BTN_EVENT_LONG_PRESSED);
				}
			}
		}
		else
		{
			
			if (Btn2Status_B4 != Btn2Status_curr){	//detect rising
				Btn2_detectFall_Cnt = 0;
				Btn2Status_B4 = Btn2Status_curr;
				if (DualBtn_Detected == 0){
					if (But2PressedCnt >= 3 && But2PressedCnt < 500) {		//detect short press
						Button_event(BTN_ID_2, BTN_EVENT_SHORT_RELEASED);
					} else if (But2PressedCnt >= 500){		//detect hold (long) press
						Button_event(BTN_ID_2, BTN_EVENT_LONG_RELEASED);
					}
					But2PressedCnt = 0;
				}
			}
		}
		
		if (Btn1_detectFall_Cnt == 1 && Btn2_detectFall_Cnt == 1){
			DualBtn_Detected = 1;
			DualBtnCnt++;
			if (DualBtnCnt % 100 == 0){
				printf("Detect Dual press down %lu s \r\n", DualBtnCnt/100);
			}
			if (DualBtnCnt == 500) {
				Button_event(BTN_ID_BOTH, BTN_EVENT_LONG_PRESSED);
			}
		}else if (Btn1_detectFall_Cnt == 0 && Btn2_detectFall_Cnt == 0 && DualBtn_Detected == 1){
			if (DualBtnCnt >= 3 && DualBtnCnt < 500){
				Button_event(BTN_ID_BOTH, BTN_EVENT_SHORT_RELEASED);
			}else if (DualBtnCnt >= 500){		//detect hold (long) press
				Button_event(BTN_ID_BOTH, BTN_EVENT_LONG_RELEASED);
			}
			DualBtn_Detected = 0;
			DualBtnCnt = 0;
			But1PressedCnt = 0;
			But2PressedCnt = 0;
		}
		
		//--------------------------------------------------2019.12.31				
		// LED_1_On();		
		// LED_1_Off();	
		// LED_1_FadeUp(uint16_t FadeTime);
		// LED_1_FadeDown(uint16_t FadeTime);
		// LED_2_On();
		// LED_2_Off();
		// LED_2_FadeUp(uint16_t FadeTime);
		// LED_2_FadeDown(uint16_t FadeTime);

		if (switchLedMode_previous != switchLedMode_031) {
			ESP_LOGI(APP_TAG, "switchLedMode_031: %d", switchLedMode_031);
			counterLED = 0;
			LED_1_Off();
			LED_2_Off();
			switchLedMode_previous = switchLedMode_031;
		}
		switch(switchLedMode_031)
		{
//			case LEDMODE_DEFAULT_NO_WIFI_BLE://Default Mode, No Wi-Fi, No Bluetooth
//				// Button 1 Indicator	|| Button 2 Indicator
//				// 	Blink, .5Hz(T = 2S)	||	-
//				{
//					if(counterLED == 1)//
//					{
//						LED_1_On();	
//					}
//					else if(counterLED == 100)//
//					{
//						LED_1_Off();
//					}
//					else if(counterLED > 200)
//					{
//						counterLED = 0;
//					}
//					
//				}break;
				
			case LEDMODE_BT_PAIRING_MODE://Bluetooth Pairing Mode
				// Button 1 Indicator		|| Button 2 Indicator
				//Blink Alternatively, 2Hz	||Blink Alternatively, 2Hz(T = 0.5S)
				{
					if(counterLED == 1)//
					{	
						LED_1_On();	
						LED_2_Off();
					}
					else if(counterLED == 25)//
					{
						LED_1_Off();
						LED_2_On();
					}
					else if(counterLED > 50)
					{
						counterLED = 0;
					}
				}break;
				
//			case LEDMODE_BT_PAIR_OK://Bluetooth Pair Successful
//				// Button 1 Indicator			|| Button 2 Indicator
//				//Blink Alternatively, 0.5 Hz	|| Blink Alternatively, 0.5 Hz(T = 2S)
//				{
//					if(counterLED == 1)//
//					{	
//						LED_1_On();	
//						LED_2_Off();
//					}
//					else if(counterLED == 100)//
//					{
//						LED_1_Off();
//						LED_2_On();
//					}
//					else if(counterLED > 200)
//					{
//						counterLED = 0;
//					}
//					
//				}break;
//				
//			case LEDMODE_WIFI_CONNECT_OK://Wi-Fi Connect Successful
//				// Button 1 Indicator	|| Button 2 Indicator
//				//Blink 3x, 1 Hz		||	Blink 3x, 1 Hz(T = 1S)
//				{
//					if(counterLED == 1)//on
//					{	
//						LED_1_On();	
//						LED_2_On();
//					}
//					else if(counterLED == 15)//off 0.15s
//					{
//						LED_1_Off();
//						LED_2_Off();
//					}
//					else if(counterLED == 30)//on
//					{
//						LED_1_On();	
//						LED_2_On();
//					}
//					else if(counterLED == 45)//off
//					{
//						LED_1_Off();
//						LED_2_Off();
//					}
//					else if(counterLED == 60)//on
//					{
//						LED_1_On();	
//						LED_2_On();
//					}
//					else if(counterLED == 75)//off
//					{
//						LED_1_Off();
//						LED_2_Off();
//					}
//					else if(counterLED > 100)
//					{
//						counterLED = 0;
//					}
//					
//				}break;
//				
//			case LEDMODE_WIFI_CONNECTED_NOTACTIVE://Wi-Fi Connected, Not Active
//				// Button 1 Indicator	|| Button 2 Indicator
//				//		On, Solid		||	On, Solid
//				{					
//						LED_1_On();	
//						LED_2_On();					
//				}break;
//				
//			case LEDMODE_WIFI_CONNECTED_ACTIVE://Wi-Fi Connected, Active
//				// Button 1 Indicator		|| Button 2 Indicator
//				//Active Output, Fade 0.5 Hz	||	Active Output, Fade 0.5 Hz(T = 2S)
//				//Non-Active Output, Off	||	Non-Active Output, Off			
//				{
//					
//					
//				//--------------- Fade 0.5 Hz
//					if(counterLED == 1)//fade up 1s
//					{
//						if (Bttn1RealState)
//						{
//							LED_1_FadeUp(500);
//						}
//							
//						if (Bttn2RealState)
//						{
//							LED_2_FadeUp(500);
//						}						
//					}
//					else if(counterLED == 100)//fade down 1s
//					{
//						if (Bttn1RealState)
//						{
//							LED_1_FadeDown(500);
//						}
//							
//						if (Bttn2RealState)
//						{
//							LED_2_FadeDown(500);
//						}
//					}
//					else if(counterLED > 200)
//					{			
//						counterLED = 0;
//					}
//					
//				//--------------- Off
//					if(Buffer_Bttn1RealState != Bttn1RealState)
//					{
//						Buffer_Bttn1RealState = Bttn1RealState;
//						if(!Bttn1RealState)	//off
//						{
//							LED_1_Off();					
//						}
//					}
//					
//					if(Buffer_Bttn2RealState != Bttn2RealState)
//					{
//						Buffer_Bttn2RealState = Bttn2RealState;
//						if(!Bttn2RealState)	//off
//						{
//							LED_2_Off();					
//						}
//					}	
//					
//				}break;
//				
//			case LEDMODE_WIFI_DISCONNECTED://Wi-Fi Disconnected
//				// Button 1 Indicator				|| Button 2 Indicator
//				//On Solid, then blink 2x, 0.5 Hz	||On Solid, then blink 2x, 0.5 Hz(T = 2S)
//				{
//					if(counterLED == 1)//On Solid 1S
//					{	
//						LED_1_On();	
//						LED_2_On();
//					}
//					else if(counterLED == 100)//fade down 0.25s
//					{
//						LED_1_FadeDown(250);
//						LED_2_FadeDown(250);
//					}
//					else if(counterLED == 125)//fade up 0.25s
//					{
//						LED_1_FadeUp(250);
//						LED_2_FadeUp(250);
//					}
//					else if(counterLED == 150)//fade down 0.25s
//					{
//						LED_1_FadeDown(250);
//						LED_2_FadeDown(250);
//					}
//					else if(counterLED == 175)//fade up 0.25s
//					{
//						LED_1_FadeUp(250);
//						LED_2_FadeUp(250);
//					}
//					else if(counterLED > 200)
//					{			
//						counterLED = 0;
//					}
//					
//				}break;
//				
//			case LEDMODE_MANUAL_ON://Manual On
//				// Button 1 Indicator		|| Button 2 Indicator
//				//Active Output, Fade 1 Hz	||	Active Output, Fade 1 Hz(T = 1S)
//				//Non-Active Output, Off	||	Non-Active Output, Off		
//				{
//					
//				//--------------- Fade 1 Hz
//					if(counterLED == 1)//fade up 0.5
//					{
//						if (Bttn1RealState)
//						{
//							LED_1_FadeUp(500);
//						}
//							
//						if (Bttn2RealState)
//						{
//							LED_2_FadeUp(500);
//						}						
//					}
//					else if(counterLED == 50)//fade down 0.5
//					{
//						if (Bttn1RealState)
//						{
//							LED_1_FadeDown(500);
//						}
//							
//						if (Bttn2RealState)
//						{
//							LED_2_FadeDown(500);
//						}
//					}
//					else if(counterLED > 100)
//					{			
//						counterLED = 0;
//					}
//					
//				//--------------- Off
//					if(Buffer_Bttn1RealState != Bttn1RealState)
//					{
//						Buffer_Bttn1RealState = Bttn1RealState;
//						if(!Bttn1RealState)	//off
//						{
//							LED_1_Off();					
//						}
//					}
//					
//					if(Buffer_Bttn2RealState != Bttn2RealState)
//					{
//						Buffer_Bttn2RealState = Bttn2RealState;
//						if(!Bttn2RealState)	//off
//						{
//							LED_2_Off();					
//						}
//					}									
//					
//				}break;
			case LEDMODE_WIFI_CONNECT_OK:
				{
					// Button 1 Indicator			|| Button 2 Indicator
					// Active Output, Fade 0.5 Hz	|| Active Output, Fade 0.5 Hz(T = 2S)
					// Non-Active Output, Off		|| Non-Active Output, Off			
					bool LED1_bFade = false;
					bool LED2_bFade = false;
					if (Bttn1RealState)
					{
						if (device_isAlgoActive_031() && (device_getOutput1Config_031() == DEVICECONFIG_OUTPUT_ON)) {
							//LED1 fade
							if ((LED_1_GetState() != LED_STATE_FADEUP) && (LED_1_GetState() != LED_STATE_FADEDOWN)) {
								counterLED = 1;
							}
							LED1_bFade = true;
						} else {
							//LED1 solid
							if (LED_1_GetState() != LED_STATE_ON) {
								LED_1_On();
							}
						}
					}
					else
					{
						//LED1 off
						if (LED_1_GetState() != LED_STATE_OFF) {
							LED_1_Off();
						}
					}
					if (Bttn2RealState)
					{
						if (device_isAlgoActive_031() && (device_getOutput2Config_031() == DEVICECONFIG_OUTPUT_ON)) {
							//LED2 fade
							if ((LED_2_GetState() != LED_STATE_FADEUP) && (LED_2_GetState() != LED_STATE_FADEDOWN)) {
								counterLED = 1;
							}
							LED2_bFade = true;
						} else {
							//LED2 solid
							if (LED_2_GetState() != LED_STATE_ON) {
								LED_2_On();
							}
						}
					}
					else
					{
						//LED2 off
						if (LED_2_GetState() != LED_STATE_OFF) {
							LED_2_Off();
						}
					}

					//--------------- Fade 0.5 Hz
					if (LED1_bFade)
					{
						if (counterLED == 1)//fade up 1s
						{
							LED_1_FadeUp(500);
						}
						else if (counterLED == 100)//fade down 1s
						{
							LED_1_FadeDown(500);
						}
					}
					if (LED2_bFade)
					{
						if (counterLED == 1)//fade up 1s
						{
							LED_2_FadeUp(500);
						}
						else if (counterLED == 100)//fade down 1s
						{
							LED_2_FadeDown(500);
						}
					}
					if (LED1_bFade || LED2_bFade)
					{
						if (counterLED > 200) {
							counterLED = 0;
						}
					}
				}break;
			case LEDMODE_WIFI_DISCONNECTED:
				{
					if(Bttn1RealState) {
						if (LED_1_GetState() != LED_STATE_ON) {
							LED_1_On();
						}
					} else {
						if (LED_1_GetState() != LED_STATE_OFF) {
							LED_1_Off();
						}
					}
					if(Bttn2RealState) {
						if (LED_2_GetState() != LED_STATE_ON) {
							LED_2_On();
						}
					} else {
						if (LED_2_GetState() != LED_STATE_OFF) {
							LED_2_Off();
						}
					}
				}break;
			case 9://Error Codes
				{
				}break;
			default:
				{
				}break;
		}
		
		//--------------------------------------------------delay 10ms
		if(counterLED < 2000)	counterLED++;	//20s
		else 					counterLED = 0;		
		
		vTaskDelay(10/portTICK_RATE_MS);//10ms
		while (Suspend_ButtonTask) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
	}
}

/*
static void i2c_master_init()
{
    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode,
    		I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}
*/

static esp_err_t app_nvs_init(void)
{
	esp_err_t err;
	bool bNormalNVS = false;
#ifdef CONFIG_NVS_ENCRYPTION
	nvs_sec_cfg_t cfg;
	const esp_partition_t* key_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS, "nvs_key");
	if (appCommon_IsFlashEncrypted() && key_part) {
		//encrypted method
		err = nvs_flash_read_security_cfg(key_part, &cfg);
		if (err == ESP_ERR_NVS_KEYS_NOT_INITIALIZED) {
			err = nvs_flash_generate_keys(key_part, &cfg);
		}
		if (err == ESP_OK) {
			err = nvs_flash_secure_init(&cfg);
			if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
				// NVS partition was truncated and needs to be erased
				nvs_flash_erase();
				err = nvs_flash_secure_init(&cfg);
			}
		}

		if (err != ESP_OK) {
			bNormalNVS = true;
		}
	} else {
		bNormalNVS = true;
	}
#else
	bNormalNVS = true;
#endif

	if (bNormalNVS) {
		//normal method
		err = nvs_flash_init();
		if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
			// NVS partition was truncated and needs to be erased
			nvs_flash_erase();
			err = nvs_flash_init();
		}
	}
	return err;
}

void app_main()
{
	esp_err_t err;
	appCommon_CheckFlashEncrypted();
	err = app_nvs_init();

#if 1
	void uartCmd_task_init(void);
	uartCmd_task_init();
#endif
	gpio_set_pull_mode(25, GPIO_PULLUP_ONLY);
	gpio_set_direction(25, GPIO_MODE_INPUT);
	
	// gpio_set_pull_mode(35, GPIO_PULLDOWN_ONLY);
	// gpio_set_pull_mode(35, GPIO_FLOATING);
	gpio_set_pull_mode(35, GPIO_PULLUP_ONLY);
	gpio_set_direction(35, GPIO_MODE_INPUT);
	
	gpio_set_pull_mode(STM_BOOT0_PIN, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(STM_RST_PIN, GPIO_PULLUP_ONLY);
	gpio_set_level(STM_BOOT0_PIN, 0);
	gpio_set_direction(STM_BOOT0_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(STM_RST_PIN, 1);
	gpio_set_direction(STM_RST_PIN, GPIO_MODE_OUTPUT);
	
    ESP_LOGI(APP_TAG, "[APP] Startup..App verstion: (%d.%d.%d %s %s) ***********************************************************************************", ESP_FW_inByte_Major, ESP_FW_inByte_Minor, ESP_FW_inByte_Sub, ESP_FW_print_Sub_2, ESP_FW_print_Date);
    ESP_LOGI(APP_TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(APP_TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

	Suspend_BleTask = 0;
	Suspend_ButtonTask = 0;
	Suspend_SensorTask = 0;
	Suspend_Uart = 0;
	Suspend_MsgTask = 0;
	STM_CT_BOOTLOADER_VER = 0xff;
	STM_CT_BOOTLOADER_VER_bValid = false;
	STM_FW_VER_Major = 0xff;
	STM_FW_VER_Minor = 0xff;
	STM_FW_VER_Sub = 0xff;
	STM_FW_VER_bValid = false;
	STM_FW_MODEL = 0x00;
	STM_FW_MODEL_bValid = false;
#ifdef BROAN_SECURE_RELEASE
	STM_FW_SIGN_bValid = false;
	STM_FW_SIGN_bFail = false;
	memset(STM_FW_SIGN_ARR, 0, sizeof(STM_FW_SIGN_ARR));
	STM_FW_SIGN_bOTAValid = false;
#endif
	STM_bPrepareUpdateBootloader = false;

    switchLedMode_031 = LEDMODE_WIFI_DISCONNECTED;
	wifi_isSet = 0;
	
	//init. storage
	storage_Init();
	err = storage_ReadDeviceConfig(&SwitchConfig);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
		SwitchConfig = DEFAULT_DEVICECONFIG_VAL;
		storage_WriteDeviceConfig(SwitchConfig);
	}
	uint8_t dummy_value;
	err = storage_ReadSTM32OTAFlag(&dummy_value);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
		storage_WriteSTM32OTAFlag(WRITE_STM_FLAG_DOWN);
	}
	err = storage_ReadIAQLEDIntensity(&IAQLEDIntensity);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
		storage_WriteIAQLEDIntensity(DEFAULT_IAQLEDINTENSITY_VAL);
	}
#ifdef BROAN_SECURE_RELEASE
	size_t read_sign_len = 0;
	if(storage_ReadSTM32Sign(STM_FW_SIGN_ARR, &read_sign_len, sizeof(STM_FW_SIGN_ARR)) != ESP_OK) {
		STM_FW_SIGN_bValid = true;
	}
	if (read_sign_len < sizeof(STM_FW_SIGN_ARR)) {
		STM_FW_SIGN_bValid = true;
	}
#endif
	certData_Init();

	get_user_data();
	check_user_data();
	//i2c_master_init();
	//InitZMOD4410();

	//-------------------judge PCBA version.20191125
	
	gpio_set_pull_mode(25, GPIO_PULLUP_ONLY);
	gpio_set_direction(25, GPIO_MODE_INPUT);
	
	// gpio_set_pull_mode(35, GPIO_PULLDOWN_ONLY);
	// gpio_set_pull_mode(35, GPIO_FLOATING);	
	gpio_set_pull_mode(34, GPIO_PULLUP_ONLY);
	gpio_set_direction(34, GPIO_MODE_INPUT);
	
	// gpio_set_pull_mode(35, GPIO_PULLDOWN_ONLY);
	// gpio_set_pull_mode(35, GPIO_FLOATING);
	gpio_set_pull_mode(35, GPIO_PULLUP_ONLY);
	gpio_set_direction(35, GPIO_MODE_INPUT);
	
	/* 
	if (gpio_get_level(25) == 0)
	{
		// printf("25 = 0 \n");//brnc031
		printf("For brnc031 \n");//brnc031
		firmwareVersion = 0x31;
	}
	else 
	{
		printf("Pin25 = 1 \n");//defalut
	}
	
	if (gpio_get_level(34) == 0)
	{
		printf("34 = 0 \n");
	}
	else 
	{
		printf("34 = 1 \n");		
	}
	
	if (gpio_get_level(35) == 0)
	{
		printf("35 = 0 \n");
	}
	else 
	{
		printf("35 = 1 \n");		
	} */	
	
	//-------------------

	wifi_SetStateCallback(wifi_callback);
	mqtt_SetConnectionCallback(mqtt_callback);
	BLE_SetStateCallback(BLE_callback);


	if(firmwareVersion != 0x31)
	{
		UART_Init();		
	}
	gpio_set_level(STM_BOOT0_PIN, 0);
	gpio_set_level(STM_RST_PIN, 0);
	vTaskDelay(100/portTICK_RATE_MS);
	gpio_set_level(STM_RST_PIN, 1);
	vTaskDelay(200/portTICK_RATE_MS);

	if(firmwareVersion != 0x31)
	{
		//check if STM32 OTA is failed to complete
		if(ota_STM32_IsFlagSet())
		{
#ifdef USE_STM32_CT_BOOTLOADER
			BootLoader_GoCmd();
#endif
			ota_STM32_boot_check();
		}
		else
		{
#ifdef USE_STM32_CT_BOOTLOADER
			BootLoader_GoCmd();
#endif
		}
	}

	err = BLE_Init(false);
	if (err == ESP_OK) {
		ESP_LOGI(APP_TAG, "[APP] BLE_Init befre Free memory: %d bytes", esp_get_free_heap_size());
		xTaskCreate(&BLE_Task, "BLE_Task", 6*1024, NULL, 20, &xHandle_BleTask);
		ESP_LOGI(APP_TAG, "[APP] BLE_Init after Free memory: %d bytes", esp_get_free_heap_size());
	}

	if(firmwareVersion == 0x31)
	{	
		//xTaskCreate(&ButtonTask, "ButtonTask", 1024, NULL, 20, NULL);
		// xTaskCreate(&ButtonTask, "ButtonTask", 2048, NULL, 20, NULL);
		xTaskCreate(&ButtonTask, "ButtonTask", 4096, NULL, 20, NULL);
	}
	if(firmwareVersion != 0x31)
	{
		xTaskCreate(&SensorTask, "SensorTask", 4096, NULL, 20, xHandle_SensorTask);
		xTaskCreate(&uart_TxRxTask, "UartTask", 6144, NULL, 20, xHandle_Uart);

		//read STM32 version
		ESP_LOGI(APP_TAG, "read STM32 ver");
		uint8_t i, j;
		STM_FW_VER_bValid = false;
		for(i=0; i<4 && !STM_FW_VER_bValid; i++)
		{
			switchAppCmd |= (1UL << SW_APPCMD_REQ_STM_VER);
			for(j=0; j<20 && !STM_FW_VER_bValid; j++)
			{
				vTaskDelay(50/portTICK_RATE_MS);
			}
		}
		ESP_LOGI(APP_TAG, "STM32 ver: %d.%d.%d", STM_FW_VER_Major, STM_FW_VER_Minor, STM_FW_VER_Sub);

		//read STM32 model
		ESP_LOGI(APP_TAG, "read STM32 model");
		STM_FW_MODEL_bValid = false;
		for(i=0; i<4 && !STM_FW_MODEL_bValid; i++)
		{
			switchAppCmd |= (1UL << SW_APPCMD_REQ_STM_MODEL);
			for(j=0; j<20 && !STM_FW_MODEL_bValid; j++)
			{
				vTaskDelay(50/portTICK_RATE_MS);
			}
		}
		ESP_LOGI(APP_TAG, "STM32 model: 0x%02x", STM_FW_MODEL);

#ifdef BROAN_SECURE_RELEASE
		if (!STM_FW_SIGN_bValid)
		{
			//request signature
			switchAppCmd |= (1UL << SW_APPCMD_REQ_STM_SIGN);
		}
#endif

		//send HW version
		switchAppCmd |= (1UL << SW_APPCMD_HW_VER);
		//send DeviceConfig
		switchCommand |= (1UL << SW_CMD_SW_COFIG);
		switchCommand_Repeat = SW_CMD_REPEAT_CNT;
	}
	
    initialise_wifi();

	uint32_t start_time = appCommon_GetTime_sec();
	while(1) {
		appCommon_TaskDelay_ms(200);

		STM32SystemStartCmd_Check();

		uint32_t now_time = appCommon_GetTime_sec();
		//wait for 60s timeout
		if (appCommon_TimeDiff(now_time, start_time) >= 60) {
			break;
		}
		//check if BLE is enabled
		if ((appCommon_TimeDiff(now_time, start_time) >= 10) && BLE_IsEnabled()) {
			break;
		}
		//check if WiFi is connected
		if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
			break;
		}
	}

	initialize_sntp();
    mqtt_app_start();
	if(firmwareVersion != 0x31)
	{
		//enable to read sensor data from STM32
		uart_setSenorDataFlag(true);
	}
	xTaskCreate(&MessageTask, "MessageTask", 4096, NULL, 20, NULL);

	uint32_t loop_start_time = appCommon_GetTime_sec();
	while(1)
	{
		uint32_t now_time = appCommon_GetTime_sec();
#ifdef BROAN_SECURE_RELEASE
		if(firmwareVersion != 0x31)
		{
			//request signature
			if (!STM_FW_SIGN_bValid)
			{
				static uint32_t request_STM32FwSign_time = 0;
				if (now_time > request_STM32FwSign_time) {
					//send STM32 FW sign request every 12s
					request_STM32FwSign_time = now_time + 12;
					switchAppCmd |= (1UL << SW_APPCMD_REQ_STM_SIGN);
					if (!STM_FW_VER_bValid) {
						switchAppCmd |= (1UL << SW_APPCMD_REQ_STM_VER);
					}
				}
				if (now_time > (loop_start_time+60)) {
					if (!STM_FW_VER_bValid) {
						STM_FW_SIGN_bFail = true;
						//hold STM32 at reset state
						gpio_set_level(STM_RST_PIN, 0);
						ESP_LOGE(APP_TAG, "STM32 signature checking is failed!");
					}
					STM_FW_SIGN_bValid = true;
				}
			}
		}
#endif

#ifdef ALLOW_UPDATE_BOOTLOADER
		if(firmwareVersion != 0x31)
		{
			if(Debug_bSTMBootLoaderTest) {
				STM_Boot2OrgBootLoader();
				Debug_bSTMBootLoaderTest = false;
			}
		}
#endif

		STM32SystemStartCmd_Check();

		appCommon_TaskDelay_ms(200);
	}
}


