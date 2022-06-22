#ifndef _USERDATA_H
#define _USERDATA_H

#define UD_HEADER_VERSION_UNKNOWN	0x0
#define UD_HEADER_VERSION_V3		0x3
#define UD_HEADER_VERSION_V4		0x4
#define UD_HEADER_VERSION_V5		0x5
#define UD_HEADER_VERSION_CURR		UD_HEADER_VERSION_V5


//user data format
#define UD_FILE_MARKER		"USER_DATA:"
#define UD_FILE_MARKER_LEN	10
#define UD_TOTAL_SIZE		sizeof(user_data_t)

#define UD_COMMON_DATA_FIELD_LEN	(16) //must not change
#define UD_DEV_ID_FIELD_LEN			(UD_COMMON_DATA_FIELD_LEN)
#define UD_WIFI_SSID_FIELD_LEN		(UD_COMMON_DATA_FIELD_LEN*2)
#define UD_WIFI_PW_FIELD_LEN		(UD_COMMON_DATA_FIELD_LEN*4)
#define UD_MQTT_BROKER_FIELD_LEN	(UD_COMMON_DATA_FIELD_LEN*4)
#define UD_BT_DEV_NAME_FIELD_LEN	(UD_COMMON_DATA_FIELD_LEN)
//#define UD_TESTMODE_FIELD_LEN		1
#define UD_MQTT_BROKER_TESTMODE_FIELD_LEN	(UD_COMMON_DATA_FIELD_LEN*4)


//user data address
#define PKI_DEV_ID_ADDR			0x10
#define PKI_WIFI_SSID_ADDR		0x20
#define PKI_WIFI_PW_ADDR		0x40
#define PKI_MQTT_BROKER_ADDR	0x80
#define PKI_BT_NAME_ADDR		0xC0
#define PKI_TESTMODE_ADDR		0xD0
#define PKI_MQTT_BROKER_TESTMODE_ADDR	0xE0

#pragma pack(1)
typedef struct {
	uint8_t marker[10];
	uint16_t version;
	uint8_t HW_version;
	uint8_t reserved[3];
} user_data_header;

typedef struct {
	user_data_header hdr;
	char device_id[UD_COMMON_DATA_FIELD_LEN];
	char wifi_ssid[UD_COMMON_DATA_FIELD_LEN*2];
	char wifi_password[UD_COMMON_DATA_FIELD_LEN*2];
	char mqtt_broker_url[UD_COMMON_DATA_FIELD_LEN*2];
	char bt_dev_name[UD_COMMON_DATA_FIELD_LEN];
} user_data_t_v3;

//version 4
typedef struct {
	user_data_header hdr;
	char device_id[UD_COMMON_DATA_FIELD_LEN];
	char wifi_ssid[UD_COMMON_DATA_FIELD_LEN*2];
	char wifi_password[UD_COMMON_DATA_FIELD_LEN*4];
	char mqtt_broker_url[UD_COMMON_DATA_FIELD_LEN*4];
	char bt_dev_name[UD_COMMON_DATA_FIELD_LEN];
} user_data_t_v4;

//version 5
typedef struct {
	user_data_header hdr;
	char device_id[UD_DEV_ID_FIELD_LEN];
	char wifi_ssid[UD_WIFI_SSID_FIELD_LEN];
	char wifi_password[UD_WIFI_PW_FIELD_LEN];
	char mqtt_broker_url[UD_MQTT_BROKER_FIELD_LEN];
	char bt_dev_name[UD_BT_DEV_NAME_FIELD_LEN];
	uint8_t test_mode;
	uint8_t reserved[UD_COMMON_DATA_FIELD_LEN-1];
	char mqtt_broker_url_testMode[UD_MQTT_BROKER_FIELD_LEN];
} user_data_t_v5, user_data_t;
#pragma pack()/* reverting back to non-packing of structures*/

_Static_assert( (uint32_t)((user_data_t *)0)->device_id == PKI_DEV_ID_ADDR, "PKI_DEV_ID_ADDR mismatch");
_Static_assert( (uint32_t)((user_data_t *)0)->wifi_ssid == PKI_WIFI_SSID_ADDR, "PKI_WIFI_SSID_ADDR mismatch");
_Static_assert( (uint32_t)((user_data_t *)0)->wifi_password == PKI_WIFI_PW_ADDR, "PKI_WIFI_PW_ADDR mismatch");
_Static_assert( (uint32_t)((user_data_t *)0)->mqtt_broker_url == PKI_MQTT_BROKER_ADDR, "PKI_MQTT_BROKER_ADDR mismatch");
_Static_assert( (uint32_t)((user_data_t *)0)->bt_dev_name == PKI_BT_NAME_ADDR, "PKI_BT_NAME_ADDR mismatch");
_Static_assert( (uint32_t)(&((user_data_t *)0)->test_mode) == PKI_TESTMODE_ADDR, "PKI_TESTMODE_ADDR mismatch");
_Static_assert( (uint32_t)((user_data_t *)0)->mqtt_broker_url_testMode == PKI_MQTT_BROKER_TESTMODE_ADDR, "PKI_MQTT_BROKER_TESTMODE_ADDR mismatch");


#endif /* _USERDATA_H */
