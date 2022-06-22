#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "uartcmd_list.h"

static const char *TAG = "uartCmd";

extern uint8_t Call_BT_enable;
extern uint8_t Call_BT_disable;
#include "esp_wifi.h"
#include "ota.h"
#include "CommonUse.h"
#include "UART_OS.h"
#include "app_storage.h"

//#define ENABLE_UARTCMD_DEBUG

void uartCmd_dummy(char *param_str)
{
	uint32_t data1=-1;
	uint32_t data2=-1;
	sscanf(param_str, "%d %d", &data1, &data2);
	printf("data:0x%x 0x%x\n", data1, data2);

	if(data1==-1/* || data2==-1*/)
	{
		return;
	}

	if(data1==1) {
		//...
	} else if(data1==2) {
		//...
	}
}

void uartCmd_help(char *param_str)
{
	uint16_t i;
	printf("Uart cmd list:\n");
	for(i=0; uartCmd_funcList[i].str!=NULL; i++)
	{
		printf("%s\n", uartCmd_funcList[i].str);
	}
}

#ifdef ENABLE_UARTCMD_DEBUG
void uartCmd_restart(char *param_str)
{
	ESP_LOGI(TAG, "Restarting");
	esp_restart();
}

//#include "ct_version.h"
void uartCmd_version(char *param_str)
{
	printf("version: %s\n", ESP_FW_string);
	printf("STM32 version: %d.%d.%d\n", STM_FW_VER_Major, STM_FW_VER_Minor, STM_FW_VER_Sub);
	if(STM_CT_BOOTLOADER_VER_bValid) {
		printf("STM32 2nd bootloader version: 0x%02x\n", STM_CT_BOOTLOADER_VER);
	} else {
		printf("No STM32 2nd bootloader\n");
	}
}

void uartCmd_loglevel(char *param_str)
{
	char str[32];
	uint32_t value;
	sscanf(param_str, "%s %d", str, &value);
	//1=ESP_LOG_ERROR
	//2=ESP_LOG_WARN
	//3=ESP_LOG_INFO
	//4=ESP_LOG_DEBUG
	//5=ESP_LOG_VERBOSE
	printf("loglevel: %s %d", str, value);
	esp_log_level_set(str, value);
}

void uartCmd_task(char *param_str)
{
#ifdef CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS
	#define DEFAULT_BUFFER_SIZE		(1024)
	char* pBuf = malloc(DEFAULT_BUFFER_SIZE);
	if(pBuf)
	{
		vTaskList(pBuf);
		printf("task name\tstatus\tprio\thighWM\tnum\tcore\n");
		printf("%s", pBuf);
		free(pBuf);
		printf("\n");
	}
#endif

	size_t free_dram = esp_get_free_heap_size();
	size_t free_iram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) - free_dram;
	printf("free memory=%d (DRAM/heap), %d (IRAM)\n", free_dram, free_iram);
}

void uartCmd_gpio(char *param_str)
{
	uint32_t pin=-1, value=-1;
	sscanf(param_str, "%d %d", &pin, &value);
	if(pin==-1 || pin>=GPIO_NUM_MAX)
		return;
	if(value==-1)//read
	{
		gpio_set_direction(pin, GPIO_MODE_INPUT);
		value = gpio_get_level(pin);
		printf("gpio[r] %d %d\n", pin, value);
	}
	else
	{
		gpio_set_direction(pin, GPIO_MODE_OUTPUT);
		gpio_set_level(pin, (value)?1:0);
		printf("gpio[w] %d %d\n", pin, (value)?1:0);
	}
}

void uartCmd_gpiopull(char *param_str)
{
	uint32_t pin=-1, pull_mode=-1;
	sscanf(param_str, "%d %d", &pin, &pull_mode);
	if(pin==-1 || pull_mode==-1)
		return;
	//Input-only GPIOs 34-39 do not pull
	//0=GPIO_PULLUP_ONLY
	//1=GPIO_PULLDOWN_ONLY
	//2=GPIO_PULLUP_PULLDOWN
	//3=GPIO_FLOATING
	gpio_set_pull_mode(pin, pull_mode);
	printf("gpiopull %d %d\n", pin, pull_mode);
}

#include "ota.h"
void uartCmd_otainfo(char *param_str)
{
	ota_info_print();
}

#include "CertData.h"
void uartCmd_certdata(char *param_str)
{
	cert_info_t* pInfo = certData_GetInfo();
	certData_DebugPrint(pInfo, true);
}
#endif //ENABLE_UARTCMD_DEBUG

void uartCmd_sn_set(char *param_str)
{
	char *pSerialNumber = NULL;
	if(param_str)
	{
		pSerialNumber = strtok(param_str, " ");
		if(pSerialNumber)
		{
			if (storage_WriteSN(pSerialNumber) == ESP_OK)
			{
				printf("sn_set OK\n");
				return;
			}
		}
	}
	printf("sn_set FAIL\n");
}

void uartCmd_sn_get(char *param_str)
{
	char serialNumber[64] = {0};
	size_t len;
	if(storage_ReadSN(serialNumber, &len, sizeof(serialNumber)) == ESP_OK)
	{
		printf("sn_get OK\n");
		printf("sn=%s\n", serialNumber);
	}
	else
	{
		printf("sn_get FAIL\n");
	}
}


#if 0
#include "UserData.h"
void uartCmd_flashurl(char *param_str)
{
	int len = strlen(param_str);
	StoreUserData(PKI_MQTT_BROKER_ADDR, (uint8_t *)param_str, (len < UD_MQTT_BROKER_FIELD_LEN)?(len+1):(UD_MQTT_BROKER_FIELD_LEN));
}

void uartCmd_flashurltest(char *param_str)
{
	int len = strlen(param_str);
	StoreUserData(PKI_MQTT_BROKER_TESTMODE_ADDR, (uint8_t *)param_str, (len < UD_MQTT_BROKER_TESTMODE_FIELD_LEN)?(len+1):(UD_MQTT_BROKER_TESTMODE_FIELD_LEN));
}

void uartCmd_flashwifi(char *param_str)
{
	char *pSSID = NULL;
	char *pPassword = NULL;
	if(!param_str)
		return;
	pSSID = strtok(param_str, " ");
	if(pSSID) {
		pPassword = strtok(NULL, " ");
	}
	if(pSSID && pPassword) {
		int SSID_len = strlen(pSSID);
		int password_len = strlen(pPassword);
		printf("set Wifi %s %s\n", pSSID, pPassword);
		UD_info_t UD_wifi[2];
		UD_wifi[0].offset = PKI_WIFI_SSID_ADDR;
		UD_wifi[0].size = (SSID_len < UD_WIFI_SSID_FIELD_LEN)?(SSID_len+1):(UD_WIFI_SSID_FIELD_LEN);
		UD_wifi[0].data = (uint8_t *)pSSID;
		UD_wifi[1].offset = PKI_WIFI_PW_ADDR;
		UD_wifi[1].size = (password_len < UD_WIFI_PW_FIELD_LEN)?(password_len+1):(UD_WIFI_PW_FIELD_LEN);
		UD_wifi[1].data = (uint8_t *)pPassword;
		StoreUserData_Multi(UD_wifi, 2);

		esp_err_t wifi_StationJoin(char* ssid, char* password);
		wifi_StationJoin(pSSID, pPassword);
	}
	
}
#endif

uartCmd_func_t uartCmd_funcList[] =
{
	{"dummy", uartCmd_dummy},
	{"help", uartCmd_help},
#ifdef ENABLE_UARTCMD_DEBUG
	{"restart", uartCmd_restart},
	{"version", uartCmd_version},
	{"loglevel", uartCmd_loglevel},
	{"task", uartCmd_task},
	{"gpio", uartCmd_gpio},
	{"gpiopull", uartCmd_gpiopull},
	{"otainfo", uartCmd_otainfo},
	{"certdata", uartCmd_certdata},
#endif //ENABLE_UARTCMD_DEBUG
	{"sn_set", uartCmd_sn_set},
	{"sn_get", uartCmd_sn_get},
#if 0
	{"flashurl", uartCmd_flashurl},
	{"flashurltest", uartCmd_flashurltest},
	{"flashwifi", uartCmd_flashwifi},
#endif
	{0, 0},
};

