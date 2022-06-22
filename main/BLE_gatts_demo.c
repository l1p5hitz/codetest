/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This demo showcases BLE GATT server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
* Client demo will enable gatt_server's notify after connection. The two devices will then exchange
* data.
*
****************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "uart_os.h"
#include "BLE_gatts_demo.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "driver/ledc.h"//2019.11.25

#include "CommonUse.h"
// #include "Usart_BootMode.h"
#include "ota.h"
#include "app_main.h"
#include "app_storage.h"
#include "app_wifi.h"
#include "app_mqtt.h"


#define BLE_DATA_DEBUG

#define GATTS_TAG "GATTS_DEMO"

#define OTA_TAG "OTA_TAG"

//uint8_t myMacAddr[6];
// uint8_t	BRNC_BT_Get_data = 0;
// uint8_t	BRNC_BT_Send_data = 0;

extern uint8_t wifi_isSet;
extern char WIFI_SSID[2][UD_WIFI_SSID_FIELD_LEN];
extern char WIFI_PASS[2][UD_WIFI_PW_FIELD_LEN];

extern unsigned int FRCSetvalue;

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static uint8_t BRNC_BT_CheckSum(uint8_t *value, uint8_t len);
static void BLE_SetState(BLE_state_t state);

#define BLE_DEVICEINFO_SERVICE
#ifdef BLE_DEVICEINFO_SERVICE
static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
//static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;


//Device information service
static const uint16_t device_info_svc_uuid = ESP_GATT_UUID_DEVICE_INFO_SVC;
static const uint16_t device_info_model_num_uuid = ESP_GATT_UUID_MODEL_NUMBER_STR;
static const uint16_t device_info_serial_num_uuid = ESP_GATT_UUID_SERIAL_NUMBER_STR;
static const uint16_t device_info_hw_version_uuid = ESP_GATT_UUID_HW_VERSION_STR;
static const uint16_t device_info_sw_version_uuid = ESP_GATT_UUID_SW_VERSION_STR;

typedef struct {
    esp_gatt_if_t gatts_if;
    uint16_t service_handle;
	uint16_t model_num_handle;
	uint16_t serial_num_handle;
	uint16_t hw_version_handle;
	uint16_t sw_version_handle;
} gatts_service_t;
static gatts_service_t device_info_svc = {0};

enum {
	DIS_IDX_SVC = 0,
	DIS_IDX_MODEL_NUM_CHAR,
	DIS_IDX_MODEL_NUM_CHAR_VAL,
	DIS_IDX_SERIAL_NUM_CHAR,
	DIS_IDX_SERIAL_NUM_CHAR_VAL,
	DIS_IDX_HW_VERSION_CHAR,
	DIS_IDX_HW_VERSION_CHAR_VAL,
	DIS_IDX_SW_VERSION_CHAR,
	DIS_IDX_SW_VERSION_CHAR_VAL,
	NUMBER_OF_DIS_IDX,
};

static const esp_gatts_attr_db_t gatt_device_info_db[NUMBER_OF_DIS_IDX] =
{
	/* Service Declaration */
	[DIS_IDX_SVC] =
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
		sizeof(device_info_svc_uuid), sizeof(device_info_svc_uuid), (uint8_t *)&device_info_svc_uuid}},
    /* Characteristic Declaration */
	[DIS_IDX_MODEL_NUM_CHAR] =
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
		1, 1, (uint8_t *)&char_prop_read}},
    /* Characteristic Value */
	[DIS_IDX_MODEL_NUM_CHAR_VAL] =
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&device_info_model_num_uuid, ESP_GATT_PERM_READ,
			32, 0, NULL}},
    /* Characteristic Declaration */
	[DIS_IDX_SERIAL_NUM_CHAR] =
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
		1, 1, (uint8_t *)&char_prop_read}},
    /* Characteristic Value */
	[DIS_IDX_SERIAL_NUM_CHAR_VAL] =
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&device_info_serial_num_uuid, ESP_GATT_PERM_READ,
			32, 0, NULL}},
	/* Characteristic Declaration */
	[DIS_IDX_HW_VERSION_CHAR] =
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
		1, 1, (uint8_t *)&char_prop_read}},
    /* Characteristic Value */
	[DIS_IDX_HW_VERSION_CHAR_VAL] =
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&device_info_hw_version_uuid, ESP_GATT_PERM_READ,
			32, 0, NULL}},
	/* Characteristic Declaration */
	[DIS_IDX_SW_VERSION_CHAR] =
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
		1, 1, (uint8_t *)&char_prop_read}},
    /* Characteristic Value */
	[DIS_IDX_SW_VERSION_CHAR_VAL] =
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&device_info_sw_version_uuid, ESP_GATT_PERM_READ,
			32, 0, NULL}},
};
#endif

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

//#define GATTS_SERVICE_UUID_TEST_B   0x00EE
//#define GATTS_CHAR_UUID_TEST_B      0xEE01
//#define GATTS_DESCR_UUID_TEST_B     0x2222
//#define GATTS_NUM_HANDLE_TEST_B     4

//#define DEVICE_NAME_PREFIX            "BRNC"
//#define TEST_MANUFACTURER_DATA_LEN  17
//char BT_Dev_name[32] = {0};

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;
//static esp_gatt_char_prop_t b_property = 0;


static uint8_t BLE_Enabled = 0;
static uint32_t BLE_Enable_tick = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[/*32*/16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    //0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    //.include_txpower = true,
    //.min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    //.max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0, //sizeof(adv_service_uuid128),
    .p_service_uuid = NULL, //adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    //.include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    //.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

bool bleOpen = false;
esp_err_t ble_Open(void);
esp_err_t ble_Close(void);
static esp_bd_addr_t remoteAddr;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    //uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    //esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = 0xFFFF,
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
//static prepare_type_env_t b_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

//#define MAX_CONCURRENT_CLIENT	16
//struct notify_record
//{
//	uint8_t last_gatts_if;
//	uint16_t last_conn_id;
//	uint16_t last_attr_handle;
//};
//struct notify_record notifyRecord[MAX_CONCURRENT_CLIENT];

uint8_t last_gatts_if = ESP_GATT_IF_NONE;
uint16_t last_conn_id = 0;
uint16_t last_attr_handle = 0;

//void ClearNotifyRecord()
//{
//	int i;
//	for (i = 0; i < MAX_CONCURRENT_CLIENT; i ++)
//	{
//		notifyRecord[i].last_gatts_if = 0xFF;
//		notifyRecord[i].last_conn_id = 0xFFFF;
//		notifyRecord[i].last_attr_handle = 0xFFFF;
//	}
//}
//
//int AddNotifyRecord(uint8_t gattsIf, uint16_t connId, uint16_t attrHandle)
//{
//	int i;
//	for (i = 0; i < MAX_CONCURRENT_CLIENT; i ++)
//	{
//		if (notifyRecord[i].last_conn_id == connId)
//			return 2;
//	}
//	for (i = 0; i < MAX_CONCURRENT_CLIENT; i ++)
//	{
//		if (notifyRecord[i].last_conn_id == 0xFFFF)
//		{
//			notifyRecord[i].last_gatts_if = gattsIf;
//			notifyRecord[i].last_conn_id = connId;
//			notifyRecord[i].last_attr_handle = attrHandle;
//			return 1;
//		}
//	}
//	return 0;
//}
//
//int DelNotifyRecord(uint16_t connId)
//{
//	int i;
//	int count = 0;
//	for (i = 0; i < MAX_CONCURRENT_CLIENT; i ++)
//	{
//		if (notifyRecord[i].last_conn_id == connId)
//		{
//			notifyRecord[i].last_gatts_if = 0xFF;
//			notifyRecord[i].last_conn_id = 0xFFFF;
//			notifyRecord[i].last_attr_handle = 0xFFFF;
//			count ++;
//		}
//	}
//	return count;
//}
//
//void ProcessNotifyRecord(void)
//{
//	int i;
//	// uint8_t notifyData[15];
//	// for (i = 0; i < sizeof(notifyData); ++i)
//		// notifyData[i] = i%0xff;
//	uint8_t notifyData[3] = {0xFF, 0xFF, 0xFF};
//	
//	for (i = 0; i < MAX_CONCURRENT_CLIENT; i ++)
//	{
//		if (notifyRecord[i].last_conn_id != 0xFFFF)
//		{
//			notifyData[1] = (notifyRecord[i].last_conn_id&0xFF00)>>8;
//			notifyData[2] = (notifyRecord[i].last_conn_id&0x00FF);
//            ////////////////////////////////////////////////////////////////////////////////////////////
//			ESP_LOGI(GATTS_TAG, "ProcessNotifyRecord, CONN_ID %d, ATTR_HANDLE %d, value :", notifyRecord[i].last_conn_id, notifyRecord[i].last_attr_handle);
//			esp_log_buffer_hex(GATTS_TAG, notifyData, sizeof(notifyData));
//			////////////////////////////////////////////////////////////////////////////////////////////
//			esp_ble_gatts_send_indicate(notifyRecord[i].last_gatts_if, notifyRecord[i].last_conn_id, notifyRecord[i].last_attr_handle,
//									sizeof(notifyData), notifyData, false);
//		}
//		vTaskDelay(1000 / portTICK_PERIOD_MS);
//	}
//}
//
//int RelplyACKNotifyRecord(uint16_t connId)
//{
//	int i;
//	uint8_t ackData[3] = {0xDD, 0x00, 0x00};
//	for (i = 0; i < MAX_CONCURRENT_CLIENT; i ++)
//	{
//		if (notifyRecord[i].last_conn_id == connId)
//		{
//			ackData[1] = (notifyRecord[i].last_conn_id&0xFF00)>>8;
//			ackData[2] = (notifyRecord[i].last_conn_id&0x00FF);
//			////////////////////////////////////////////////////////////////////////////////////////////
//			ESP_LOGI(GATTS_TAG, "RelplyACKNotifyRecord, CONN_ID %d, ATTR_HANDLE %d, value :", notifyRecord[i].last_conn_id, notifyRecord[i].last_attr_handle);
//			esp_log_buffer_hex(GATTS_TAG, ackData, sizeof(ackData));
//			////////////////////////////////////////////////////////////////////////////////////////////
//			esp_ble_gatts_send_indicate(notifyRecord[i].last_gatts_if, notifyRecord[i].last_conn_id, notifyRecord[i].last_attr_handle,
//									sizeof(ackData), ackData, false);
//			return 1;
//		}
//	}
//	return 0;
//}


//BLE provision process
static uint8_t BLE_ProvAckMask = 0;
static bool BLE_ProvAck_bWiFiSend = false;
static bool BLE_ProvAck_bMQTTSend = false;
static uint8_t BLE_ProvAck_WiFiCnt = 0;
static uint8_t BLE_ProvAck_MQTTCnt = 0;

static void BLE_ProvAckMaskInit(void)
{
	BLE_ProvAckMask = WAIT_MACADDR_ACK_BIT;
	BLE_ProvAck_bWiFiSend = false;
	BLE_ProvAck_bMQTTSend = false;
	BLE_ProvAck_WiFiCnt = 0;
	BLE_ProvAck_MQTTCnt = 0;
}

bool BLE_ProvAckMaskIsSet(uint8_t bitmask)
{
	if (BLE_ProvAckMask &  bitmask) {
		return true;
	} else {
		return false;
	}
}

static void BLE_ProvAckMaskSet(uint8_t bitmask)
{
	if (bitmask & WAIT_WIFI_ACK_BIT) {
		BLE_ProvAck_WiFiCnt = 0;
	}
	if (bitmask & WAIT_MQTT_ACK_BIT) {
		BLE_ProvAck_MQTTCnt = 0;
	}
	BLE_ProvAckMask |= bitmask;
}

static void BLE_ProvAckMaskClear(uint8_t bitmask)
{
	BLE_ProvAckMask &= ~bitmask;
}

#define BLE_PROVACK_CHECK_PERIOD		3 //in seconds
#define BLE_PROVACK_SEND_WIFI_FAIL		15 //in seconds (multiple of check period)
#define BLE_PROVACK_SEND_MQTT_FAIL		15 //in seconds (multiple of check period)
static void BLE_ProvAckCheck(void)
{
	if (BLE_ProvAckMask)
	{
		static uint32_t last_check_time = 0;
		uint32_t now_time = appCommon_GetTime_sec();
		if (appCommon_TimeDiff(now_time, last_check_time) >= BLE_PROVACK_CHECK_PERIOD)
		{
			if (BLE_ProvAckMask & WAIT_MACADDR_ACK_BIT)
			{
				BRNC_BT_Send_data = BT2APP_MAC_ADDR;
			}
			else if (BLE_ProvAckMask & WAIT_WIFI_ACK_BIT)
			{
				if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
					BRNC_BT_Send_data = BT2APP_WIFI_CONNECT_OK;
				} else {
					BLE_ProvAck_WiFiCnt++;
					if (BLE_ProvAck_WiFiCnt >= (BLE_PROVACK_SEND_WIFI_FAIL/BLE_PROVACK_CHECK_PERIOD)) {
						BLE_ProvAck_WiFiCnt = 0;
						BRNC_BT_Send_data = BT2APP_WIFI_CONNECT_FAIL;
					}
				}
			}
			else if (BLE_ProvAckMask & WAIT_MQTT_ACK_BIT)
			{
				if (mqtt_IsConnected()) {
					BRNC_BT_Send_data = BT2APP_MQTT_CONNECT_OK;
				} else {
					BLE_ProvAck_MQTTCnt++;
					if (BLE_ProvAck_MQTTCnt >= (BLE_PROVACK_SEND_MQTT_FAIL/BLE_PROVACK_CHECK_PERIOD)) {
						BLE_ProvAck_MQTTCnt = 0;
						BRNC_BT_Send_data = BT2APP_MQTT_CONNECT_FAIL;
					}
				}
			}
			last_check_time = now_time;
		}
	}
}

static int ble_cmd_handle(uint8_t* buffer, uint32_t len)
{
	int ret = 0;
	const char* debug_info_string = "debug_info=";
	//check if starting with "http", do OTA firmware update test
	if (strncmp((char *)buffer, "http", strlen("http")) == 0)
	{
		ESP_LOGI(GATTS_TAG, "do FW_UP");
		char* url = malloc(len+1);
		if (!url) {
			return -1;
		}
		memcpy(url, buffer, len);
		url[len] = '\0';

		if (strstr((char *)url, "cert")) {
			//CERT
			ota_task_run(0, NULL, 0, NULL, 0, url, NULL, NULL);
		} else if (strstr((char *)url, "BRNC")) {
			//STM32 FW
			ota_task_run(0, url, 0, NULL, 0, NULL, NULL, NULL);
		} else if (strstr((char *)url, "Broan")) {
			//ESP32 FW
			ota_task_run(0, NULL, 0, url, 0, NULL, NULL, NULL);
		}
		free(url);
	}
	else if (strncmp((char *)buffer, debug_info_string, strlen(debug_info_string)) == 0)
	{
		uint32_t buffer_offset = strlen(debug_info_string);
		uint32_t value = 0;
		char* tmp_string = calloc(1, 512);
		
		if (len > buffer_offset) {
			char value_string[8] = {0};
			strncpy(value_string, (char *)buffer+buffer_offset, sizeof(value_string)-1);
			value = atoi(value_string);
		}
		if (tmp_string)
		{
			//not to poll MAC address
			if (BLE_ProvAckMask & WAIT_MACADDR_ACK_BIT) {
				BLE_ProvAckMaskClear(WAIT_MACADDR_ACK_BIT);
			}

			if (value == 1) {
				sprintf(tmp_string+strlen(tmp_string), "SSID=%.*s  ", sizeof(WIFI_SSID[0]), WIFI_SSID[0]);
				sprintf(tmp_string+strlen(tmp_string), "PWD=%.*s\n", sizeof(WIFI_PASS[0]), WIFI_PASS[0]);
			} else if (value == 8) {
				if(wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
					sprintf(tmp_string+strlen(tmp_string), "WiFi (%.*s) connected", sizeof(WIFI_SSID[0]), WIFI_SSID[0]);
				} else {
					sprintf(tmp_string+strlen(tmp_string), "Try to connect WiFi (%.*s)", sizeof(WIFI_SSID[0]), WIFI_SSID[0]);
					wifi_StationJoin(WIFI_SSID[0], WIFI_PASS[0]);
				}
			} else if (value == 3) {
				if (firmwareVersion == 0x11){
					sprintf(tmp_string+strlen(tmp_string), "Temperature=%0.2f,Humidity=%0.2f,TVOC=%0.2f,CO2=%0.2f,PM2.5=%0.2f", BLE_send_Temperature, BLE_send_Humidity, BLE_send_TVOC, BLE_send_CO2, BLE_send_PM2p5);
				}else if (firmwareVersion == 0x21){
					sprintf(tmp_string+strlen(tmp_string), "Temperature=%0.2f,Humidity=%0.2f,TVOC=%0.2f,eCO2=%0.2f", BLE_send_Temperature, BLE_send_Humidity, BLE_send_TVOC, BLE_send_eCO2);
				}
			} else if (value == 79) {
				storage_EraseCertData();
				sprintf(tmp_string+strlen(tmp_string), "Erased CertData");
			} else { // value == 0
				sprintf(tmp_string+strlen(tmp_string), "MQTT=%.*s  ", sizeof(MQTT_BROKER_URL), MQTT_BROKER_URL);
				sprintf(tmp_string+strlen(tmp_string), "MQTT_Test=%.*s\n", sizeof(MQTT_BROKER_URL_TestMode), MQTT_BROKER_URL_TestMode);
			}
			esp_ble_gatts_send_indicate(last_gatts_if, last_conn_id, last_attr_handle,
											strlen(tmp_string)+1, (uint8_t *)tmp_string, false);
			free(tmp_string);
		}
	}
#if 0
	//check if starting with "log"
	else if (strncmp((char *)buffer, "log", strlen("log")) == 0)
	{
		buffer += 3;
		len -= 3;
		memset(UpHttpPath, 0, sizeof(UpHttpPath));
		if(len > sizeof(UpHttpPath) - 1) {
			len = sizeof(UpHttpPath) - 1;
		}
		memcpy(UpHttpPath, buffer, len);
		ESP_LOGI(GATTS_TAG, "UpHttpPath = %s", UpHttpPath);
	}
#endif
	else
	{
#if 0//#ifdef BLE_DATA_DEBUG
		//debug
		printf("BRNC_BT_rev_data[%d]: ", len);
		for (uint8_t k = 0; k < len; k++){
			printf("%02X ", buffer[k]);
		}
		printf("\r\n");
#endif

		if (len > sizeof(BRNC_BT_rev_data)) {
			ESP_LOGE(GATTS_TAG, "BRNC_BT_rev_data length too log");
			return -1;
		} else {
			memset(BRNC_BT_rev_data, 0, sizeof(BRNC_BT_rev_data));
			memcpy(BRNC_BT_rev_data, buffer, len);
			BRNC_BT_rev_len = len;
			BRNC_BT_Get_data = 1;
		}
	}
	return ret;
}


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            ble_Open();
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT");
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            ble_Open();
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT");
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            ble_Open();
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            ble_Open();
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_ADV_START_COMPLETE_EVT");
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *_gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            _gatt_rsp->attr_value.len = param->write.len;
            _gatt_rsp->attr_value.handle = param->write.handle;
            _gatt_rsp->attr_value.offset = param->write.offset;
            _gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(_gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, _gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(_gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){

    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        //esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    //memset(BRNC_BT_rev_data, 0, sizeof(BRNC_BT_rev_data));
	
	switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

		char deviceName[32] = {0};
		//ESP_LOGI(BLEWS_TAG, "BD ADDR: "ESP_BD_ADDR_STR"\n", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
		const uint8_t* macAddr = esp_bt_dev_get_address();
		ESP_LOGI(GATTS_TAG, "BD ADDR: "ESP_BD_ADDR_STR"\n", ESP_BD_ADDR_HEX(macAddr));
		if (firmwareVersion == 0x11) {
			sprintf(deviceName, "BROAN_RS-");
		} else if (firmwareVersion == 0x21) {
			sprintf(deviceName, "BROAN_WC-");
		} else { //0x31
			sprintf(deviceName, "BROAN_SP-");
		}
		sprintf(deviceName+strlen(deviceName), "%01x%02x%02x", macAddr[3]&0xf, macAddr[4], macAddr[5]);
		ESP_LOGI(GATTS_TAG, "BT_Dev_name: %s \r\n", deviceName);
		//
		
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(deviceName);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT_A, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->read.handle)
		{
			rsp.attr_value.handle = param->read.handle;
			rsp.attr_value.len = 2;
			rsp.attr_value.value[0] = 0;
			if (gl_profile_tab[PROFILE_A_APP_ID].property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
				rsp.attr_value.value[0] |= 0x01;
			if (gl_profile_tab[PROFILE_A_APP_ID].property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
				rsp.attr_value.value[0] |= 0x02;
			rsp.attr_value.value[1] = 0;
			esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
		}
		else
		{
			rsp.attr_value.handle = param->read.handle;
			rsp.attr_value.len = 4;
			rsp.attr_value.value[0] = 0xde;
			rsp.attr_value.value[1] = 0xed;
			rsp.attr_value.value[2] = 0xbe;
			rsp.attr_value.value[3] = 0xef;
			esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
		}
		

        break;
    }
    case ESP_GATTS_WRITE_EVT: {
		BLE_Enable_tick = 0;
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT_A, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT_A, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[PROFILE_A_APP_ID].char_handle == param->write.handle)
			{
				ble_cmd_handle(param->write.value, param->write.len);
			}
			// ******************************** //
			// receive data in here //
			
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");
						gl_profile_tab[PROFILE_A_APP_ID].property |= ESP_GATT_CHAR_PROP_BIT_NOTIFY;
                        //uint8_t notify_data[15];
                        //for (int i = 0; i < sizeof(notify_data); ++i)
                        //{
                        //    notify_data[i] = i%0x07;
                        //}
						
                        //the size of notify_data[] need less than MTU size
						////////////////////////////////////////////////////////////////////////////////////////////
						//AddNotifyRecord(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle);
						last_gatts_if = gatts_if;
						last_conn_id = param->write.conn_id;
						last_attr_handle = gl_profile_tab[PROFILE_A_APP_ID].char_handle;
						////////////////////////////////////////////////////////////////////////////////////////////
						//ESP_LOGI(GATTS_TAG, "GATT_SEND_INDICATE_A, CONN_ID %d, ATTR_HANDLE %d, value :", last_conn_id, last_attr_handle);
						//esp_log_buffer_hex(GATTS_TAG, notify_data, sizeof(notify_data));
						////////////////////////////////////////////////////////////////////////////////////////////
                        // esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                // sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
						gl_profile_tab[PROFILE_A_APP_ID].property |= ESP_GATT_CHAR_PROP_BIT_INDICATE;
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
					gl_profile_tab[PROFILE_A_APP_ID].property = 0;
					//DelNotifyRecord(param->write.conn_id);
                }else{
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                }

            }
			else
			{
				//if (param->write.len == 3 && param->write.value[0] == 0xFF)
				//	RelplyACKNotifyRecord(param->write.conn_id);
			}
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
		BLE_Enable_tick = 0;
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT, conn_id %d, trans_id %d", param->exec_write.conn_id, param->exec_write.trans_id);
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT, value len %d, value :", a_prepare_write_env.prepare_len);
		esp_log_buffer_hex(GATTS_TAG, a_prepare_write_env.prepare_buf, a_prepare_write_env.prepare_len);
		if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && a_prepare_write_env.prepare_buf)
		{
			ble_cmd_handle(a_prepare_write_env.prepare_buf, a_prepare_write_env.prepare_len);
		}

        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = /*ESP_GATT_CHAR_PROP_BIT_READ | */ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
#ifdef BLE_DEVICEINFO_SERVICE
		{
			esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_device_info_db, device_info_svc.gatts_if, NUMBER_OF_DIS_IDX, 0);
			if (create_attr_ret) {
				ESP_LOGE(GATTS_TAG, "create attr table failed, error code = %x", create_attr_ret);
			}
		}
#endif
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
		memcpy(remoteAddr, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
		
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
		BLE_SetState(BLE_STATE_CONNECTED);
		BLE_ProvAckMaskInit();
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
		
		//xxxx////////////////////////////////////////////////////////////////////////////////////
		//esp_ble_gap_start_advertising(&adv_params);
        //xxxx////////////////////////////////////////////////////////////////////////////////////
		break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, conn_id %d, disconnect reason 0x%x", param->disconnect.conn_id, param->disconnect.reason);
		last_gatts_if = ESP_GATT_IF_NONE;
		last_conn_id = 0;
		last_attr_handle = 0;
		gl_profile_tab[PROFILE_A_APP_ID].conn_id = 0xFFFF;
		gl_profile_tab[PROFILE_A_APP_ID].property = 0;
		BLE_SetState(BLE_STATE_DISCONNECTED);

		//DelNotifyRecord(param->disconnect.conn_id);
		esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, (client receive confirmed) status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

#ifdef BLE_DEVICEINFO_SERVICE
static void gatts_info_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
	ESP_LOGW(GATTS_TAG, "gatts_info_event_handler: %d", event);
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        device_info_svc.service_handle = param->add_attr_tab.handles[DIS_IDX_SVC];
		device_info_svc.model_num_handle = param->add_attr_tab.handles[DIS_IDX_MODEL_NUM_CHAR_VAL];
		device_info_svc.serial_num_handle = param->add_attr_tab.handles[DIS_IDX_SERIAL_NUM_CHAR_VAL];
		device_info_svc.hw_version_handle = param->add_attr_tab.handles[DIS_IDX_HW_VERSION_CHAR_VAL];
		device_info_svc.sw_version_handle = param->add_attr_tab.handles[DIS_IDX_SW_VERSION_CHAR_VAL];
        ESP_LOGI(GATTS_TAG, "Dev Info service handle = %d num = %d", device_info_svc.service_handle, param->add_attr_tab.num_handle);
        esp_ble_gatts_start_service(device_info_svc.service_handle);
        break;
	case ESP_GATTS_START_EVT:
        break;
	case ESP_GATTS_READ_EVT: {
		esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)calloc(1, sizeof(esp_gatt_rsp_t));
		if (param->read.handle == device_info_svc.model_num_handle) {
			const char* pString = appCommon_GetModelString();
			gatt_rsp->attr_value.handle = param->read.handle;
			gatt_rsp->attr_value.len = strlen(pString);
			strncpy((char*)gatt_rsp->attr_value.value, pString, sizeof(gatt_rsp->attr_value.value)-1);
			esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, gatt_rsp);
		} else if (param->read.handle == device_info_svc.serial_num_handle) {
			char tmp_string[128] = {0};
			char serialNumber[64] = {0};
			size_t serialNumber_len = 0;
			const char* pDeviceIDString = appCommon_GetDeviceID();
			const char* pSNString = "<null>";
			if (storage_ReadSN(serialNumber, &serialNumber_len, sizeof(serialNumber)) == ESP_OK) {
				pSNString = serialNumber;
			}
			sprintf(tmp_string, "ID=%s MAC=%02X%02X%02X%02X%02X%02X SN=%s", pDeviceIDString, myMacAddr[0], myMacAddr[1], myMacAddr[2], myMacAddr[3], myMacAddr[4], myMacAddr[5], pSNString);
			gatt_rsp->attr_value.handle = param->read.handle;
			gatt_rsp->attr_value.len = strlen(tmp_string);
			strncpy((char*)gatt_rsp->attr_value.value, tmp_string, sizeof(gatt_rsp->attr_value.value)-1);
			esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, gatt_rsp);
		} else if (param->read.handle == device_info_svc.hw_version_handle) {
			char tmp_string[64] = {0};
			sprintf(tmp_string, "v%d", appCommon_GetHwVersion());
			gatt_rsp->attr_value.handle = param->read.handle;
			gatt_rsp->attr_value.len = strlen(tmp_string);
			strncpy((char*)gatt_rsp->attr_value.value, tmp_string, sizeof(gatt_rsp->attr_value.value)-1);
			esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, gatt_rsp);
		} else if (param->read.handle == device_info_svc.sw_version_handle) {
			char tmp_string[64] = {0};
			sprintf(tmp_string, "%s", appCommon_GetVersionString());
			if (appCommon_GetModel() != 0x31) {
				sprintf(tmp_string+strlen(tmp_string), " %d.%d.%d", STM_FW_VER_Major, STM_FW_VER_Minor, STM_FW_VER_Sub);
			}
			sprintf(tmp_string+strlen(tmp_string), " (%s, %s)",
				appCommon_GetTestMode()?"Test":"Release",
				appCommon_IsFlashEncrypted()?"Secure":"Legacy");
			gatt_rsp->attr_value.handle = param->read.handle;
			gatt_rsp->attr_value.len = strlen(tmp_string);
			strncpy((char*)gatt_rsp->attr_value.value, tmp_string, sizeof(gatt_rsp->attr_value.value)-1);
			esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, gatt_rsp);
		}
		free(gatt_rsp);
		break;
	}

    default:
        break;
    }
}
#endif

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
#ifdef BLE_DEVICEINFO_SERVICE
			if (param->reg.app_id == device_info_svc_uuid) {
				device_info_svc.gatts_if = gatts_if;
			} else {
				gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
			}
#else
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
#endif
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
#ifdef BLE_DEVICEINFO_SERVICE
	if (device_info_svc.gatts_if && gatts_if == device_info_svc.gatts_if) {
		gatts_info_event_handler(event, gatts_if, param);
	}
#endif
}

esp_err_t ble_Open(void)
{
	esp_err_t err;
	uint8_t retry = 3;
	do {
		err = esp_ble_gap_start_advertising(&adv_params);
	}while(retry-- && err != ESP_OK);
	
	if(err == ESP_OK)
		bleOpen = true;
	ESP_LOGI(GATTS_TAG, "%s BLE Open", __func__);
	return err;
}

esp_err_t ble_Close(void)
{
	esp_err_t err;
	uint8_t retry = 3;
	esp_ble_gap_disconnect(remoteAddr);
	vTaskDelay(100/ portTICK_RATE_MS);
	do {
		err = esp_ble_gap_stop_advertising();
	}while(retry-- && err != ESP_OK);
	
	if(err == ESP_OK)
		bleOpen = false;
	ESP_LOGI(GATTS_TAG, "%s BLE Close", __func__);
	return err;
}

esp_err_t BLE_Stop(void){
	esp_err_t ret;
	ret = esp_bluedroid_disable();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s failed: %s\n", __func__, esp_err_to_name(ret));
		return ret;
	}
	ret = esp_bluedroid_deinit();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s failed: %s\n", __func__, esp_err_to_name(ret));
		return ret;
	}
	ret = esp_bt_controller_disable();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s failed: %s\n", __func__, esp_err_to_name(ret));
		return ret;
	}
	ret = esp_bt_controller_deinit();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s failed: %s\n", __func__, esp_err_to_name(ret));
		return ret;
	}
	BLE_Enabled = 0;
	BLE_SetState(BLE_STATE_DISABLED);
	ESP_LOGI("BLE", "BLE stopped");
	return ret;
}

esp_err_t BLE_Start(void){
	esp_err_t ret;
	
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
	
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
	
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return ret;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }
    //ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    //if (ret){
    //    ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
    //    return ret;
    //}
#ifdef BLE_DEVICEINFO_SERVICE
    ret = esp_ble_gatts_app_register(device_info_svc_uuid);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }
#endif
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
	BLE_Enabled = 1;
	BLE_Enable_tick = 0;
	BLE_SetState(BLE_STATE_INIT);
	ESP_LOGI("BLE", "BLE started, now run...");
	
	
	return ret;
	
}


static BLE_state_t BLE_state = BLE_STATE_DISABLED;
static void (*BLE_state_callback)(BLE_state_t state, BLE_state_t state_prev) = NULL;

static void BLE_SetState(BLE_state_t state)
{
	if (BLE_state != state) {
		BLE_state_t state_prev = BLE_state;
		BLE_state = state;
		if (BLE_state_callback) {
			BLE_state_callback(BLE_state, state_prev);
		}
	}
}

BLE_state_t BLE_GetState(void)
{
	return BLE_state;
}

void BLE_SetStateCallback(void (*callback)(BLE_state_t state, BLE_state_t state_prev))
{
	BLE_state_callback = callback;
}

bool BLE_IsEnabled(void)
{
	return (BLE_Enabled)?true:false;
}

bool BLE_IsConnectedAndEnabledNotify(void)
{
	if (BLE_Enabled
		&& (gl_profile_tab[PROFILE_A_APP_ID].conn_id != 0xFFFF)
		&& (gl_profile_tab[PROFILE_A_APP_ID].property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
		return true;
	} else {
		return false;
	}
}

static bool BLE_bDebugMode = false;
void BLE_SetDebugMode(bool bDebugMode)
{
	if (bDebugMode) {
		BLE_bDebugMode = true;
	} else {
		BLE_bDebugMode = false;
	}
}

bool BLE_IsDebugMode(void)
{
	return BLE_bDebugMode;
}

void BLE_ResetEnableTimeout(void)
{
	BLE_Enable_tick = 0;
}

esp_err_t BLE_Init(bool bStart)
{
	esp_err_t ret;
	ret = ESP_OK;
	//ClearNotifyRecord();
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	if (bStart) {
		ret = BLE_Start();
	}
	return  ret;
}
	
void BLE_Task(void *pvParameters)
{
	uint8_t sendData[64];
	uint8_t index = 0;
	uint8_t BT_data_Len = 0;
	uint8_t SendRes = 0;
	uint8_t Get_error = 0;
	esp_err_t res;
	
	ESP_LOGI("BLE", "[BLE] Ble Task Start");
	 vTaskDelay(4000 / portTICK_PERIOD_MS);
	// sendData[index] = BRNC_BT_START; index++;
	// sendData[index] = BRNC_BT_VER; index++;
	// sendData[index] = 2; index++;	//length index = 2;
	// sendData[index] = BRNC_BT_SEND_MAC; index++;	//CMD
	// memcpy(&sendData[index], myMacAddr, sizeof(myMacAddr)); index+=sizeof(myMacAddr);
	// BT_data_Len = index -1;
	// sendData[BRNC_BT_LEN_INDEX] = BT_data_Len;	
	// sendData[index] = BRNC_BT_CheckSum(&sendData[1], index); index++;
	// sendData[index] = BRNC_BT_END;

			// esp_ble_gatts_send_indicate(last_gatts_if, last_conn_id, last_attr_handle,
									// index, sendData, false);
							
	// ProcessNotifyRecord(sendData, BT_data_Len);
    // const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "pki");
    // if (!partition) 
	// {
        // printf("No user data partition");
		// Get_error = 1;
    // }

	while (1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		BLE_Enable_tick++;
		if (BLE_IsConnectedAndEnabledNotify()) {
			BLE_ProvAckCheck();
		}
		
		if (BLE_Enabled == 1) {
			//BT mode timeout
			uint32_t BLE_timeout_cnt = 45*10;
			if (appCommon_GetTestMode() || BLE_IsDebugMode()) { //timeout=120s in test mode
				BLE_timeout_cnt = 120*10;
			}
			else if (BLE_GetState() == BLE_STATE_CONNECTED) {
				BLE_timeout_cnt = 120*10;
			}
			if(BLE_Enable_tick > BLE_timeout_cnt) {
				Call_BT_disable = 1;
			}
		}
		
		if (Call_BT_enable){
			Call_BT_enable = 0;
			if (!BLE_IsEnabled()) {
				printf("extern Call Enable BlueTooth!\r\n");
				BLE_Start();
			} else {
				BLE_ResetEnableTimeout();
			}
		}
		
		if (Call_BT_disable){
			Call_BT_disable = 0;
			if (BLE_IsEnabled()) {
				BLE_Stop();
				printf("extern Call Disable BlueTooth!\r\n");
			}
		}
		
		while (Suspend_BleTask) {
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		//ProcessNotifyRecord();
			Get_error = 0;
			index = 0;
			memset(sendData, 0, sizeof(sendData));
			sendData[index] = BRNC_BT_START; index++;
			sendData[index] = BRNC_BT_VER; index++;
			sendData[index] = 2; index++;	//length index = 2;		
			
		if (BRNC_BT_Get_data){
			BRNC_BT_Get_data = 0;
			
#ifdef BLE_DATA_DEBUG
			printf("BRNC_BT_rev_data[%d]: ", BRNC_BT_rev_len);
			for (uint8_t k = 0; k < BRNC_BT_rev_len; k++){
				printf("%02X ", BRNC_BT_rev_data[k]);
			}
			printf("\r\n");
#endif
			if (BRNC_BT_rev_data[0] != BRNC_BT_START){
				printf("BRNC_BT_START byte not correct\r\n");
				Get_error = 1;
			}
			if (BRNC_BT_rev_data[1] != BRNC_BT_VER){
				printf("BRNC_BT_VER bit not correct\r\n");
				Get_error = 1;
			}
			if (BRNC_BT_rev_len < 6){
				printf("BRNC_BT_rev_len too small %d\r\n", BRNC_BT_rev_len);
				Get_error = 1;
			}
			if (BRNC_BT_rev_len < BRNC_BT_rev_data[2] + 4){
				printf("BRNC_BT_rev_len incomplete (%d < %d + 4) \r\n", BRNC_BT_rev_len, BRNC_BT_rev_data[2]);
				Get_error = 1;
			}
			
			uint8_t checksum = BRNC_BT_CheckSum(&BRNC_BT_rev_data[1], BRNC_BT_rev_data[2]+1);
			if (checksum != BRNC_BT_rev_data[BRNC_BT_rev_data[2]+2]){
				printf("CheckSum not correct (%02x != %02x)\r\n", checksum, BRNC_BT_rev_data[BRNC_BT_rev_data[2]+2]);
				Get_error = 1;
			}
			if (Get_error == 0){
				switch (BRNC_BT_rev_data[3]){
					case BRNC_BT_REV_MAC_ACK:
					{
						printf("Got Send MAC Add ACK\r\n");
						BLE_ProvAckMaskClear(WAIT_MACADDR_ACK_BIT);
						break;
					}
					case BRNC_BT_REV_SSID:
					{
						printf("Got Send SSID CMD\r\n");
						if (BRNC_BT_rev_len > 6)
						{
							uint8_t* pNewSSID = &BRNC_BT_rev_data[4];
							uint8_t newSSID_len = BRNC_BT_rev_len - 6;
							if (newSSID_len > 0)
							{
								if (newSSID_len > sizeof(WIFI_SSID[0])) {
									newSSID_len = sizeof(WIFI_SSID[0]);
								}
								memset(WIFI_SSID[0], 0, sizeof(WIFI_SSID[0]));
								memcpy(WIFI_SSID[0], pNewSSID, newSSID_len);
								sendData[index] = BRNC_BT_RES_SSID_ACK; index++;	//CMD
								SendRes = 1;
							}
						}
						break;
					}
					case BRNC_BT_REV_PW:
					{
						printf("Got Send PW CMD\r\n");
						if (BRNC_BT_rev_len > 6)
						{
							uint8_t* pNewPassword = &BRNC_BT_rev_data[4];
							uint8_t newPassword_len = BRNC_BT_rev_len - 6;
							if (newPassword_len > 0)
							{
								if (newPassword_len > sizeof(WIFI_PASS[0])) {
									newPassword_len = sizeof(WIFI_PASS[0]);
								}
								memset(WIFI_PASS[0], 0, sizeof(WIFI_PASS[0]));
								memcpy(WIFI_PASS[0], pNewPassword, newPassword_len);

								//store user data
								UD_info_t UD_wifi[2];
								UD_wifi[0].offset = PKI_WIFI_SSID_ADDR;
								UD_wifi[0].size = sizeof(WIFI_SSID[0]);
								UD_wifi[0].data = (uint8_t *)WIFI_SSID[0];
								UD_wifi[1].offset = PKI_WIFI_PW_ADDR;
								UD_wifi[1].size = sizeof(WIFI_PASS[0]);
								UD_wifi[1].data = (uint8_t *)WIFI_PASS[0];
								StoreUserData_Multi(UD_wifi, 2);

								wifi_isSet = 1;
								res = wifi_StationJoin(WIFI_SSID[0], WIFI_PASS[0]);
								if (res == ESP_OK) {
									printf("Setting WiFi configuration SSID %.*s...\r\n", sizeof(WIFI_SSID[0]), WIFI_SSID[0]);
									printf("Setting WiFi configuration PW %.*s...\r\n", sizeof(WIFI_PASS[0]), WIFI_PASS[0]);
									printf("WiFi reconnect...\r\n");
								} else {
									printf("Wifi join failed, res = %d\n", res);
								}
								sendData[index] = BRNC_BT_RES_PW_ACK; index++;	//CMD
								BLE_ProvAckMaskSet(WAIT_WIFI_ACK_BIT);
								SendRes = 1;
							}
						}
						break;
					}
					case BRNC_BT_WIFI_CONN_STATUS_ACK:
					{
						printf("Got BRNC_BT_WIFI_CONN_STATUS_ACK\r\n");
						if (BLE_ProvAckMaskIsSet(WAIT_WIFI_ACK_BIT) && BLE_ProvAck_bWiFiSend) {
							BLE_ProvAckMaskClear(WAIT_WIFI_ACK_BIT);
							BLE_ProvAck_bWiFiSend = false;

							if (wifi_GetConnectionState() == WIFI_STATE_CONNECTED) {
								BLE_ProvAckMaskSet(WAIT_MQTT_ACK_BIT);
							}
						}
						break;
					}
					case BRNC_BT_MQTT_CONN_STATUS_ACK:
					{
						printf("Got BRNC_BT_MQTT_CONN_STATUS_ACK and disable BT \r\n");
						if (BLE_ProvAckMaskIsSet(WAIT_MQTT_ACK_BIT) && BLE_ProvAck_bMQTTSend) {
							BLE_ProvAckMaskClear(WAIT_MQTT_ACK_BIT);
							BLE_ProvAck_bMQTTSend = false;

							//Call_BT_disable = 1;
							BLE_Stop();
						}
						break;
					}
					case BRNC_BT_REV_DEVICE_CONFIG:
					{
						if (BRNC_BT_rev_len > 6)
						{
							uint8_t value = BRNC_BT_rev_data[4];
							if (value > 0)
							{
								if (value != SwitchConfig) {
									SwitchConfig = value;
									storage_WriteDeviceConfig(SwitchConfig);
								}
								printf("Got SwitchConfig = %d \r\n", SwitchConfig);
								switchCommand |= (1UL << SW_CMD_SW_COFIG);
								switchCommand_Repeat = SW_CMD_REPEAT_CNT;
								Call_BT_disable = 1;
							}
							sendData[index] = BRNC_BT_DEVICE_CONFIG_ACK; index++;	//CMD
							SendRes = 1;
						}
						break;
					}
					case BRNC_BT_MQTT_SERVER_URL:
					{
						if (BRNC_BT_rev_len > 6)
						{
							char brokerAddress[UD_MQTT_BROKER_FIELD_LEN] = {0};
							if (BRNC_BT_rev_len - 6 < UD_MQTT_BROKER_FIELD_LEN) {
								memcpy(brokerAddress, &BRNC_BT_rev_data[4], BRNC_BT_rev_len - 6);
							} else {
								memcpy(brokerAddress, &BRNC_BT_rev_data[4], UD_MQTT_BROKER_FIELD_LEN-1);
							}
							//write MQTT broker and disable test mode
							UD_info_t UD_settings[2];
							uint8_t uTestMode = 0;
							appCommon_SetTestMode(uTestMode);
							UD_settings[0].offset = PKI_MQTT_BROKER_ADDR;
							UD_settings[0].size = sizeof(brokerAddress);
							UD_settings[0].data = (uint8_t *)brokerAddress;
							UD_settings[1].offset = PKI_TESTMODE_ADDR;
							UD_settings[1].size = sizeof(uTestMode);
							UD_settings[1].data = &uTestMode;
							StoreUserData_Multi(UD_settings, 2);
							if (!strncmp(brokerAddress, "mqtt", 4)) {
								sprintf(MQTT_BROKER_URL, "%.*s", sizeof(brokerAddress), brokerAddress); 
							} else {
								sprintf(MQTT_BROKER_URL, "mqtt://%.*s", sizeof(brokerAddress), brokerAddress);
							}
							mqtt_app_stop();
							mqtt_app_start();
							sendData[index] = BRNC_BT_MQTT_SERVER_URL_ACK; index++;	//CMD
							SendRes = 1;
						}
						break;
					}
//					case BRNC_BT_SET_BT_NAME:
//					{
//						if ((BRNC_BT_rev_len - 6) <= 16){
//							memset(BT_Dev_name, 0, sizeof(BT_Dev_name));
//							memcpy(BT_Dev_name, &BRNC_BT_rev_data[4], BRNC_BT_rev_len - 6);
//							StoreUserData(PKI_BT_NAME_ADDR, (uint8_t*)BT_Dev_name, sizeof(BT_Dev_name));
//							sendData[index] = BRNC_BT_SET_BT_NAME_ACK; index++;	//CMD
//							SendRes = 1;
//							esp_restart();
//						}else{
//							printf("error: BT_Dev_name too long = %s \r\n", BT_Dev_name);
//						}
//						break;
//					}
					case BRNC_BT_SET_CO2_FRC:
					{
						if (BRNC_BT_rev_len > 6)
						{
							char CO2_FRC_string[32] = {0};
							uint8_t len = BRNC_BT_rev_len - 6;
							if (len >= sizeof(CO2_FRC_string)) {
								len = sizeof(CO2_FRC_string)-1;
							}
							memcpy(CO2_FRC_string, &BRNC_BT_rev_data[4], len);
							FRCSetvalue = atoi(CO2_FRC_string);
							switchAppCmd |= (1UL << SW_APPCMD_CO2_FRC_SET);
						}
						break;
					}
					case BRNC_BT_SET_MQTT_BROKER:
					{
						if (BRNC_BT_rev_len > 6)
						{
							char brokerAddress[UD_MQTT_BROKER_FIELD_LEN] = {0};
							if (BRNC_BT_rev_len - 6 < UD_MQTT_BROKER_FIELD_LEN) {
								memcpy(brokerAddress, &BRNC_BT_rev_data[4], BRNC_BT_rev_len - 6);
							} else {
								memcpy(brokerAddress, &BRNC_BT_rev_data[4], UD_MQTT_BROKER_FIELD_LEN-1);
							}
							//write MQTT broker and disable test mode
							UD_info_t UD_settings[2];
							uint8_t uTestMode = 0;
							appCommon_SetTestMode(uTestMode);
							UD_settings[0].offset = PKI_MQTT_BROKER_ADDR;
							UD_settings[0].size = sizeof(brokerAddress);
							UD_settings[0].data = (uint8_t *)brokerAddress;
							UD_settings[1].offset = PKI_TESTMODE_ADDR;
							UD_settings[1].size = sizeof(uTestMode);
							UD_settings[1].data = &uTestMode;
							StoreUserData_Multi(UD_settings, 2);
							if (!strncmp(brokerAddress, "mqtt", 4)) {
								sprintf(MQTT_BROKER_URL, "%.*s", sizeof(brokerAddress), brokerAddress); 
							} else {
								sprintf(MQTT_BROKER_URL, "mqtt://%.*s", sizeof(brokerAddress), brokerAddress);
							}
							mqtt_app_stop();
							mqtt_app_start();
						}
						break;
					}
					case BRNC_BT_REQ_ESP_VER:
					{
						sendData[index] = BRNC_BT_REPLY_ESP_VER; index++;	//CMD
						sendData[index] = ESP_FW_inByte_Major; index++;
						sendData[index] = ESP_FW_inByte_Minor; index++;
						sendData[index] = ESP_FW_inByte_Sub; index++;
						SendRes = 1;
						break;
					}
					case BRNC_BT_REQ_STM_VER:
					{
						if (!STM_FW_VER_bValid) {
							switchAppCmd |= (1UL << SW_APPCMD_REQ_STM_VER);
							vTaskDelay(1000/portTICK_PERIOD_MS);
						}
						sendData[index] = BRNC_BT_REPLY_STM_VER; index++;	//CMD
						sendData[index] = STM_FW_VER_Major; index++;
						sendData[index] = STM_FW_VER_Minor; index++;
						sendData[index] = STM_FW_VER_Sub; index++;
						SendRes = 1;
						break;
					}
					case BRNC_BT_REQ_DEVICEID:
					{
						size_t deviceID_len = strlen(appCommon_GetDeviceID());
						sendData[index] = BRNC_BT_REPLY_DEVICEID; index++;	//CMD
						memcpy(&sendData[index], appCommon_GetDeviceID(), deviceID_len); index += deviceID_len;
						SendRes = 1;
						break;
					}
					case BRNC_BT_SET_TESTMODE:
					{
						if (BRNC_BT_rev_len > 6)
						{
							uint8_t value = BRNC_BT_rev_data[4];
							if (value != appCommon_GetTestMode()) {
								appCommon_SetTestMode(value);
								StoreUserData(PKI_TESTMODE_ADDR, &value, 1);

								//test mode is changed, re-connect correct MQTT broker
								mqtt_app_stop();
								mqtt_app_start();
							}
						}
						break;
					}
					case BRNC_BT_REQ_TESTMODE:
					{
						sendData[index] = BRNC_BT_REPLY_TESTMODE; index++;	//CMD
						sendData[index] = appCommon_GetTestMode(); index++;
						SendRes = 1;
						break;
					}
					case BRNC_BT_SET_TESTMODE_MQTT_BROKER:
					{
						if (BRNC_BT_rev_len > 6)
						{
							char brokerAddress[UD_MQTT_BROKER_TESTMODE_FIELD_LEN] = {0};
							if (BRNC_BT_rev_len - 6 < UD_MQTT_BROKER_TESTMODE_FIELD_LEN) {
								memcpy(brokerAddress, &BRNC_BT_rev_data[4], BRNC_BT_rev_len - 6);
							} else {
								memcpy(brokerAddress, &BRNC_BT_rev_data[4], UD_MQTT_BROKER_TESTMODE_FIELD_LEN-1);
							}
							StoreUserData(PKI_MQTT_BROKER_TESTMODE_ADDR, (uint8_t *)brokerAddress, sizeof(brokerAddress));
							if (!strncmp(brokerAddress, "mqtt", 4)) {
								sprintf(MQTT_BROKER_URL_TestMode, "%.*s", sizeof(brokerAddress), brokerAddress); 
							} else {
								sprintf(MQTT_BROKER_URL_TestMode, "mqtt://%.*s", sizeof(brokerAddress), brokerAddress);
							}
							if (appCommon_GetTestMode()) {
								mqtt_app_stop();
								mqtt_app_start();
							}
						}
						break;
					}
					case BRNC_BT_FACTORY_RESET:
					{
						system_event_run(SYS_EVENT_FACTORY_RESET);
						break;
					}
				};
				
			}


			
		}
		if ( Get_error == 0){
			if (BRNC_BT_Send_data && (SendRes==0)){
				switch (BRNC_BT_Send_data){
				case BT2APP_WIFI_CONNECT_OK:
					if (BLE_ProvAckMaskIsSet(WAIT_WIFI_ACK_BIT)) {
						BLE_ProvAck_bWiFiSend = true;
					}
					sendData[index] = BRNC_BT_SEND_WIFI_CONN_STATUS; index++;	//CMD
					sendData[index] = BRNC_BT_SUCCESS; index++;
					SendRes = 1;
					break;
				case BT2APP_WIFI_CONNECT_FAIL:
					if (BLE_ProvAckMaskIsSet(WAIT_WIFI_ACK_BIT)) {
						BLE_ProvAck_bWiFiSend = true;
					}
					sendData[index] = BRNC_BT_SEND_WIFI_CONN_STATUS; index++;	//CMD
					sendData[index] = BRNC_BT_FAIL; index++;
					SendRes = 1;
					break;
				case BT2APP_MQTT_CONNECT_OK:
					if (BLE_ProvAckMaskIsSet(WAIT_MQTT_ACK_BIT)) {
						BLE_ProvAck_bMQTTSend = true;
					}
					sendData[index] = BRNC_BT_SEND_MQTT_CONN_STATUS; index++;	//CMD
					sendData[index] = BRNC_BT_SUCCESS; index++;
					SendRes = 1;
					break;
				case BT2APP_MQTT_CONNECT_FAIL:
					if (BLE_ProvAckMaskIsSet(WAIT_MQTT_ACK_BIT)) {
						BLE_ProvAck_bMQTTSend = true;
					}
					sendData[index] = BRNC_BT_SEND_MQTT_CONN_STATUS; index++;	//CMD
					sendData[index] = BRNC_BT_FAIL; index++;
					SendRes = 1;
					break;
				case BT2APP_MAC_ADDR:
					sendData[index] = BRNC_BT_SEND_MAC; index++;	//CMD
					SendRes = 1;
					memcpy(&sendData[index], myMacAddr, sizeof(myMacAddr)); index+=sizeof(myMacAddr);
					break;
				case BT2APP_SSID_ACK:
				case BT2APP_PW_ACK:
				default:
					break;
				};
				BRNC_BT_Send_data = 0;
			}
			
			
			if (SendRes){
				SendRes = 0;
				BT_data_Len = index -2;
				sendData[BRNC_BT_LEN_INDEX] = BT_data_Len;	
				sendData[index] = BRNC_BT_CheckSum(&sendData[1], BT_data_Len + 1); index++;
				sendData[index] = BRNC_BT_END; index++;

				// esp_ble_gatts_send_indicate(notifyRecord[i].last_gatts_if, notifyRecord[i].last_conn_id, notifyRecord[i].last_attr_handle,
								// BT_data_Len, sendData, false);
				// ProcessNotifyRecord(sendData, BT_data_Len);
				
#ifdef BLE_DATA_DEBUG
				printf("BRNC_BT_send_data[%d]: ", index);
				for (uint8_t k = 0; k < index; k++){
					printf("%02X ", sendData[k]);
				}
				printf("\r\n");
#endif
				esp_err_t err = esp_ble_gatts_send_indicate(last_gatts_if, last_conn_id, last_attr_handle,
										index, sendData, false);
				if(gl_profile_tab[PROFILE_A_APP_ID].property & (ESP_GATT_CHAR_PROP_BIT_NOTIFY|ESP_GATT_CHAR_PROP_BIT_INDICATE)) {
					ESP_LOGI("BLE", "send notification:%d err:0x%x", sendData[3], err);
				} else {
					ESP_LOGE("BLE", "send notification:%d err:0x%x", sendData[3], err);
				}			
			}
		}
	}
}

static uint8_t BRNC_BT_CheckSum(uint8_t *value, uint8_t len){
	uint8_t i = 0, retCheckSum = 0;
	uint32_t Calchecksum = 0;
	
	for (i = 0; i < len; i++){
		Calchecksum += value[i];

	}
	Calchecksum = 0xFF - Calchecksum;
	Calchecksum += 1;
	
	retCheckSum = Calchecksum&0xFF;
	
	//printf("BT Calchecksum = %X , retCheckSum = %X \r\n", Calchecksum, retCheckSum);
	return retCheckSum;
}




/*
void app_main()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

	xTaskCreate(&BLE_Task, "BLE_Task", 50000, NULL, 5, NULL);
    //return;
}
*/


//esp_bt_controller_disable