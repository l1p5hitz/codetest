#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "CommonUse.h"
#include "app_main.h"

#include "app_storage.h"


#define STORAGE_TAG		"STORAGE"

#define NVS_STORAGE_NAMESPACE	"BRNC_storage"
//maximum key length = 15 characters
#define NVS_STORAGE_KEY_IAQLEDINTENSITY		"LEDIntensity" //u8
#define NVS_STORAGE_KEY_DEVICECONFIG		"DeviceConfig" //u8
#define NVS_STORAGE_KEY_STM32_OTA_FLAG		"STM32OTAflag" //u8
#define NVS_STORAGE_KEY_OTA_STM_DEPLOY_ID	"OTA_STMDeployId" //string
#define NVS_STORAGE_KEY_OTA_ESP_DEPLOY_ID	"OTA_ESPDeployId" //string
#define NVS_STORAGE_KEY_OTA_CERT_DEPLOY_ID	"OTACERTDeployId" //string
#define NVS_STORAGE_KEY_CERTDATA			"CertData" //binary
#define NVS_STORAGE_KEY_STM32_FW_SIGN		"STM32FWSign" //binary
#define NVS_STORAGE_KEY_SN					"SN" //string
#define NVS_STORAGE_KEY_DEVICEID_BAK		"DeviceId_bak" //binary
//#define NVS_STORAGE_KEY_WIFISSID_BAK		"WiFiSSID_bak" //
//#define NVS_STORAGE_KEY_WIFIPWD_BAK			"WiFiPwd_bak" //
#define NVS_STORAGE_KEY_HWVER_BAK			"HwVer_bak" //u8
static nvs_handle_t nvs_storage_handle;


void storage_Init(void)
{
	esp_err_t err;
	err = nvs_open(NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_storage_handle);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS open");
		return;
	}
}

void storage_Deinit(void)
{
	if (nvs_storage_handle) {
		nvs_close(nvs_storage_handle);
		nvs_storage_handle = 0;
	}
}

esp_err_t storage_EraseKey(const char* key)
{
	return nvs_erase_key(nvs_storage_handle, key);
}

void storage_EraseCertData(void)
{
	storage_EraseKey(NVS_STORAGE_KEY_CERTDATA);
	storage_EraseKey(NVS_STORAGE_KEY_OTA_CERT_DEPLOY_ID);
}

static esp_err_t storage_CommonWrite_u8(const char* key, uint8_t value)
{
	esp_err_t err;
	if (key == NULL) {
		return ESP_FAIL;
	}

	err = nvs_set_u8(nvs_storage_handle, key, value);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS set \"%s\"", key);
		return err;
	}
	err = nvs_commit(nvs_storage_handle);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS commit");
	}
	return err;
}

static esp_err_t storage_CommonRead_u8(const char* key, uint8_t* pValue)
{
	esp_err_t err;
	if (key == NULL || pValue == NULL) {
		return ESP_FAIL;
	}

	err = nvs_get_u8(nvs_storage_handle, key, pValue);
	if (err == ESP_OK) {
	} else if (err == ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGW(STORAGE_TAG, "NVS_NOT_FOUND \"%s\"", key);
	} else {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS get \"%s\"", key);
	}
	return err;
}

static esp_err_t storage_CommonWrite_string(const char* key, const char* string)
{
	esp_err_t err;
	if (key == NULL) {
		return ESP_FAIL;
	}

	err = nvs_set_str(nvs_storage_handle, key, string);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS set \"%s\"", key);
		return err;
	}
	err = nvs_commit(nvs_storage_handle);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS commit");
	}
	return err;
}

static esp_err_t storage_CommonRead_string(const char* key, char* string, size_t* pLength, size_t max_length)
{
	esp_err_t err;
	size_t required_size = 0;
	if (key == NULL || pLength == NULL) {
		return ESP_FAIL;
	}

	err = nvs_get_str(nvs_storage_handle, key, NULL, &required_size);
	if (err != ESP_OK) {
		if (err == ESP_ERR_NVS_NOT_FOUND) {
			//ESP_LOGW(STORAGE_TAG, "NVS_NOT_FOUND \"%s\"", key);
		} else {
			ESP_LOGE(STORAGE_TAG, "Fail to NVS get \"%s\"", key);
		}
		return err;
	}
	if(string == NULL) {
		if(err == ESP_OK) {
			*pLength = required_size;
		}
		return err;
	}

	if (required_size == 0) {
		ESP_LOGE(STORAGE_TAG, "\"%s\" zero size", key);
		err = ESP_FAIL;
		return err;
	} else if (required_size > max_length) {
		ESP_LOGE(STORAGE_TAG, "\"%s\" size too large", key);
		err = ESP_FAIL;
		return err;
	}
	*pLength = required_size;
	err = nvs_get_str(nvs_storage_handle, key, string, pLength);
	return err;
}

static esp_err_t storage_CommonWrite_binary(const char* key, const void* pValue, size_t length)
{
	esp_err_t err;
	if (key == NULL) {
		return ESP_FAIL;
	}

	err = nvs_set_blob(nvs_storage_handle, key, pValue, length);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS set \"%s\"", key);
		return err;
	}
	err = nvs_commit(nvs_storage_handle);
	if (err != ESP_OK) {
		ESP_LOGE(STORAGE_TAG, "Fail to NVS commit");
	}
	return err;
}

static esp_err_t storage_CommonRead_binary(const char* key, void* pValue, size_t* pLength, size_t max_length)
{
	esp_err_t err;
	size_t required_size = 0;
	if (key == NULL || pLength == NULL) {
		return ESP_FAIL;
	}

	err = nvs_get_blob(nvs_storage_handle, key, NULL, &required_size);
	if (err != ESP_OK) {
		if (err == ESP_ERR_NVS_NOT_FOUND) {
			//ESP_LOGW(STORAGE_TAG, "NVS_NOT_FOUND \"%s\"", key);
		} else {
			ESP_LOGE(STORAGE_TAG, "Fail to NVS get \"%s\"", key);
		}
		return err;
	}
	if(pValue == NULL) {
		if(err == ESP_OK) {
			*pLength = required_size;
		}
		return err;
	}

	if (required_size == 0) {
		ESP_LOGE(STORAGE_TAG, "\"%s\" zero size", key);
		err = ESP_FAIL;
		return err;
	} else if (required_size > max_length) {
		ESP_LOGE(STORAGE_TAG, "\"%s\" size too large", key);
		err = ESP_FAIL;
		return err;
	}
	*pLength = required_size;
	err = nvs_get_blob(nvs_storage_handle, key, pValue, pLength);
	return err;
}


esp_err_t storage_WriteIAQLEDIntensity(uint8_t value)
{
	return storage_CommonWrite_u8(NVS_STORAGE_KEY_IAQLEDINTENSITY, value);
}

esp_err_t storage_ReadIAQLEDIntensity(uint8_t* pValue)
{
	return storage_CommonRead_u8(NVS_STORAGE_KEY_IAQLEDINTENSITY, pValue);
}

esp_err_t storage_WriteDeviceConfig(uint8_t value)
{
	return storage_CommonWrite_u8(NVS_STORAGE_KEY_DEVICECONFIG, value);
}

esp_err_t storage_ReadDeviceConfig(uint8_t* pValue)
{
	return storage_CommonRead_u8(NVS_STORAGE_KEY_DEVICECONFIG, pValue);
}

esp_err_t storage_WriteSTM32OTAFlag(uint8_t flag)
{
	return storage_CommonWrite_u8(NVS_STORAGE_KEY_STM32_OTA_FLAG, flag);
}

esp_err_t storage_ReadSTM32OTAFlag(uint8_t* pFlag)
{
	return storage_CommonRead_u8(NVS_STORAGE_KEY_STM32_OTA_FLAG, pFlag);
}

esp_err_t storage_WriteSTM32OtaDeploymentId(const char* string)
{
	return storage_CommonWrite_string(NVS_STORAGE_KEY_OTA_STM_DEPLOY_ID, string);
}

esp_err_t storage_ReadSTM32OtaDeploymentId(char* string, size_t* pLength, size_t max_length)
{
	return storage_CommonRead_string(NVS_STORAGE_KEY_OTA_STM_DEPLOY_ID, string, pLength, max_length);
}

esp_err_t storage_WriteESP32OtaDeploymentId(const char* string)
{
	return storage_CommonWrite_string(NVS_STORAGE_KEY_OTA_ESP_DEPLOY_ID, string);
}

esp_err_t storage_ReadESP32OtaDeploymentId(char* string, size_t* pLength, size_t max_length)
{
	return storage_CommonRead_string(NVS_STORAGE_KEY_OTA_ESP_DEPLOY_ID, string, pLength, max_length);
}

esp_err_t storage_WriteCertOtaDeploymentId(const char* string)
{
	return storage_CommonWrite_string(NVS_STORAGE_KEY_OTA_CERT_DEPLOY_ID, string);
}

esp_err_t storage_ReadCertOtaDeploymentId(char* string, size_t* pLength, size_t max_length)
{
	return storage_CommonRead_string(NVS_STORAGE_KEY_OTA_CERT_DEPLOY_ID, string, pLength, max_length);
}

#ifdef BROAN_SECURE_RELEASE
esp_err_t storage_WriteCertData(const void* pValue, size_t length)
{
	return storage_CommonWrite_binary(NVS_STORAGE_KEY_CERTDATA, pValue, length);
}

esp_err_t storage_ReadCertData(void* pValue, size_t* pLength, size_t max_length)
{
	return storage_CommonRead_binary(NVS_STORAGE_KEY_CERTDATA, pValue, pLength, max_length);
}
#endif

esp_err_t storage_WriteSTM32Sign(const void* pValue, size_t length)
{
	return storage_CommonWrite_binary(NVS_STORAGE_KEY_STM32_FW_SIGN, pValue, length);
}

esp_err_t storage_ReadSTM32Sign(void* pValue, size_t* pLength, size_t max_length)
{
	return storage_CommonRead_binary(NVS_STORAGE_KEY_STM32_FW_SIGN, pValue, pLength, max_length);
}

esp_err_t storage_WriteSN(const char* string)
{
	return storage_CommonWrite_string(NVS_STORAGE_KEY_SN, string);
}

esp_err_t storage_ReadSN(char* string, size_t* pLength, size_t max_length)
{
	return storage_CommonRead_string(NVS_STORAGE_KEY_SN, string, pLength, max_length);
}

bool storage_BackupDeviceID(char* device_id)
{

	esp_err_t err;
	char data[UD_DEV_ID_FIELD_LEN] = {0};
	size_t len = 0;
	if (device_id == NULL)
	{
		return false;
	}
	if ((device_id[0] == 0x0) || (device_id[0] == 0xff) || (device_id[1] == 0x0) || (device_id[1] == 0xff))
	{
		return false;
	}

	err = storage_CommonRead_binary(NVS_STORAGE_KEY_DEVICEID_BAK, data, &len, UD_DEV_ID_FIELD_LEN);
	if (err == ESP_OK) {
		if (memcmp(device_id, data, UD_DEV_ID_FIELD_LEN) == 0) {
			//the backup is good
			return true;
		}
	}
	err = storage_CommonWrite_binary(NVS_STORAGE_KEY_DEVICEID_BAK, device_id, UD_DEV_ID_FIELD_LEN);
	if (err == ESP_OK) {
		return true;
	}
	return false; 
}

bool storage_RecoverDeviceID(char* device_id)
{
	esp_err_t err;
	size_t len = 0;
	if (device_id == NULL)
	{
		return false;
	}
	err = storage_CommonRead_binary(NVS_STORAGE_KEY_DEVICEID_BAK, device_id, &len, UD_DEV_ID_FIELD_LEN);
	if (err == ESP_OK) {
		return true;
	}
	return false;
}

bool storage_BackupHwVersion(uint8_t version)
{
	esp_err_t err;
	uint8_t value = 0;
	err = storage_CommonRead_u8(NVS_STORAGE_KEY_HWVER_BAK, &value);
	if (err == ESP_OK) {
		if (version == value) {
			//the backup is good
			return true;
		}
	}
	storage_CommonWrite_u8(NVS_STORAGE_KEY_HWVER_BAK, version);
	if (err == ESP_OK) {
		return true;
	}
	return false; 
}

bool storage_RecoverHwVersion(uint8_t* pVersion)
{
	esp_err_t err;
	if (pVersion == NULL)
	{
		return false;
	}
	err = storage_CommonRead_u8(NVS_STORAGE_KEY_HWVER_BAK, pVersion);
	if (err == ESP_OK) {
		return true;
	}
	return false;
}

