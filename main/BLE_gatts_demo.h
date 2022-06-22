#ifndef __BLE__
#define __BLE__

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "esp_https_ota.h"

#include "CommonUse.h"

#define BRNC_BT_LEN_INDEX 2

#define BRNC_BT_START	0x01
#define BRNC_BT_VER		0x01
#define BRNC_BT_END		0x0F
#define BRNC_BT_SEND_MAC	0x01
#define BRNC_BT_REV_MAC_ACK	0xA1
#define BRNC_BT_REV_SSID	0xA2
#define BRNC_BT_RES_SSID_ACK	0x02
#define BRNC_BT_REV_PW		0xA3
#define BRNC_BT_RES_PW_ACK	0x03
#define BRNC_BT_SEND_WIFI_CONN_STATUS	0x04
#define BRNC_BT_WIFI_CONN_STATUS_ACK	0xA4
#define BRNC_BT_SEND_MQTT_CONN_STATUS	0x05
#define BRNC_BT_MQTT_CONN_STATUS_ACK	0xA5
#define BRNC_BT_REV_DEVICE_CONFIG		0xA6
#define BRNC_BT_DEVICE_CONFIG_ACK		0x06
#define BRNC_BT_MQTT_SERVER_URL			0xA7
#define BRNC_BT_MQTT_SERVER_URL_ACK		0x07

#define BRNC_BT_SUCCESS					0x01
#define BRNC_BT_FAIL					0x0F

/* Internal command */
//#define BRNC_BT_SET_BT_NAME				0xB1
//#define BRNC_BT_SET_BT_NAME_ACK			0x11
#define BRNC_BT_SET_CO2_FRC				0xB2
#define BRNC_BT_SET_MQTT_BROKER			0xB3
#define BRNC_BT_REQ_ESP_VER				0xB4
#define BRNC_BT_REPLY_ESP_VER			0x14
#define BRNC_BT_REQ_STM_VER				0xB5
#define BRNC_BT_REPLY_STM_VER			0x15
#define BRNC_BT_REQ_DEVICEID			0xB6
#define BRNC_BT_REPLY_DEVICEID			0x16
#define BRNC_BT_SET_TESTMODE			0xB7
#define BRNC_BT_REQ_TESTMODE			0xB8
#define BRNC_BT_REPLY_TESTMODE			0x18
#define BRNC_BT_SET_TESTMODE_MQTT_BROKER	0xB9
#define BRNC_BT_FACTORY_RESET			0xBA


typedef enum {
    BT2APP_WIFI_CONNECT_OK = 1,
    BT2APP_WIFI_CONNECT_FAIL,
    BT2APP_MQTT_CONNECT_OK,
    BT2APP_MQTT_CONNECT_FAIL,
	BT2APP_MAC_ADDR,
	BT2APP_SSID_ACK,
	BT2APP_PW_ACK,

}BT_to_App_Msg;

typedef enum {
	BLE_STATE_DISABLED = 0,
	BLE_STATE_INIT,
	BLE_STATE_CONNECTED,
	BLE_STATE_DISCONNECTED,
	NUM_OF_BLE_STATE,
} BLE_state_t;

uint8_t myMacAddr[6];
volatile uint8_t	BRNC_BT_Get_data;
volatile uint8_t	BRNC_BT_Send_data;
uint8_t BRNC_BT_rev_data[100];

volatile uint8_t BRNC_BT_rev_len;

//extern char BT_Dev_name[32];

esp_err_t do_firmware_upgrade(char *url_);
esp_err_t BLE_Init(bool bStart);
esp_err_t BLE_Start(void);
esp_err_t BLE_Stop(void);
BLE_state_t BLE_GetState(void);
void BLE_SetStateCallback(void (*callback)(BLE_state_t state, BLE_state_t state_prev));
bool BLE_IsEnabled(void);
bool BLE_IsConnectedAndEnabledNotify(void);
void BLE_SetDebugMode(bool bDebugMode);
bool BLE_IsDebugMode(void);
void BLE_ResetEnableTimeout(void);


//BLE provision process
enum {
	WAIT_MACADDR_ACK_BIT = 0x01,
	SEND_WIFI_RESULT_BIT = 0x02,
	WAIT_WIFI_ACK_BIT = 0x04,
	WAIT_MQTT_ACK_BIT = 0x08,
};
bool BLE_ProvAckMaskIsSet(uint8_t bitmask);

#endif
