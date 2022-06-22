#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#if CONFIG_SECURE_FLASH_ENC_ENABLED
#include "esp_flash_encrypt.h"
#endif
#include "nvs_flash.h"

#define COMMONUSE_INIT
#include "CommonUse.h"

#define UDATA_TAG	"UDATA"

int StoreUserData_Multi(UD_info_t* multi_data, uint8_t count)
{
	int ret = 0;
	uint8_t *PartitionData = NULL;
	bool bWrite = false;
	if (multi_data == NULL || count == 0)
	{
		ESP_LOGE(UDATA_TAG, "NULL multi_data");
		return -3;
	}

	const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "pki");
	if (partition == NULL) 
	{
        ESP_LOGE(UDATA_TAG, "No UD partition");
        return -1;
    }

	PartitionData = malloc(0x1000);
	if(PartitionData == NULL)
	{
		ESP_LOGE(UDATA_TAG, "Fail to malloc");
		return -4;
	}
	memset(PartitionData, 0, 0x1000);

	esp_err_t err;
	err = esp_partition_read(partition, 0, PartitionData, partition->size);
	if (err != ESP_OK) {
		ESP_LOGE(UDATA_TAG, "Fail to read UD partition");
		ret = -5;
	} else {
		ESP_LOGI(UDATA_TAG, "Read UD partition:");
		ESP_LOG_BUFFER_HEXDUMP(UDATA_TAG, PartitionData, UD_TOTAL_SIZE, ESP_LOG_INFO);

		for (uint8_t i = 0; i < count; i++)
		{
			if ((multi_data[i].offset + multi_data[i].size) > 0x1000)
			{
				ESP_LOGE(UDATA_TAG, "Write data length > 0x1000");
				ret = -2;
			}
			else if(multi_data[i].data == NULL)
			{
				ESP_LOGE(UDATA_TAG, "NULL data");
				ret = -3;
			}
			else
			{
				memcpy(&PartitionData[multi_data[i].offset], multi_data[i].data, multi_data[i].size);
				bWrite = true;
			}
		}

		if (bWrite) {
			ret = 0;
			esp_partition_erase_range(partition, 0, partition->size);
			err = esp_partition_write(partition, 0, PartitionData, partition->size);
			if (err == ESP_OK) {
				ESP_LOGI(UDATA_TAG, "Write data with esp_partition_write:");
				ESP_LOG_BUFFER_HEXDUMP(UDATA_TAG, PartitionData, UD_TOTAL_SIZE, ESP_LOG_INFO);

				//read back user data
				uint8_t *readData = malloc(UD_TOTAL_SIZE);
				if (readData) {
					err = esp_partition_read(partition, 0, readData, UD_TOTAL_SIZE);
					if (err == ESP_OK) {
						if (memcmp(PartitionData, readData, UD_TOTAL_SIZE) == 0) {
							ESP_LOGI(UDATA_TAG, "Verify written data OK");
						} else {
							ESP_LOGE(UDATA_TAG, "Read back data mismatch:");
							ESP_LOG_BUFFER_HEXDUMP(UDATA_TAG, readData, UD_TOTAL_SIZE, ESP_LOG_ERROR);
							ret = -7;
						}
					} else {
						ESP_LOGE(UDATA_TAG, "Fail to read UD partition");
						ret = -8;
					}
					free(readData);
				} else {
					ESP_LOGE(UDATA_TAG, "Fail to malloc");
					//not consider as error
				}
			} else {
				ESP_LOGE(UDATA_TAG, "Fail to write UD partition");
				ret = -6;
			}
		}
	}

	if (PartitionData) {
		free(PartitionData);
	}
	return ret;
}

int StoreUserData(size_t offset, uint8_t *data, size_t size)
{
	UD_info_t single_data;
	single_data.offset = offset;
	single_data.size = size;
	single_data.data = data;
	return StoreUserData_Multi(&single_data, 1);
}

void DumpUserData(void)
{
	esp_err_t err;
	const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "pki");
	if (partition) {
		uint8_t *readData = malloc(UD_TOTAL_SIZE);
		if (readData) {
			err = esp_partition_read(partition, 0, readData, UD_TOTAL_SIZE);
			if (err == ESP_OK) {
				ESP_LOGI(UDATA_TAG, "Read UD partition:");
				ESP_LOG_BUFFER_HEXDUMP(UDATA_TAG, readData, UD_TOTAL_SIZE, ESP_LOG_INFO);
			}
			free(readData);
		}
	}
}

const char* appCommon_GetVersionString(void)
{
	return ESP_FW_string;
}

extern unsigned char firmwareVersion;
uint8_t appCommon_GetModel(void)
{
	return firmwareVersion;
}

const char* appCommon_GetModelString(void)
{
	if (firmwareVersion == 0x21) {
		return "BIAQWC100P";//"BRNC021";
	} else if (firmwareVersion == 0x31) {
		return "BIAQSP100P";//"BRNC031";
	} else { //0x11
		return "BIAQRS100";//"BRNC011";
	}
}

extern char HW_ID[];
const char* appCommon_GetDeviceID(void)
{
	return HW_ID;
}

static uint8_t uHwVersion = 0;
void appCommon_SetHwVersion(uint8_t version)
{
	uHwVersion = version;
}

uint8_t appCommon_GetHwVersion(void)
{
	return uHwVersion;
}


static uint8_t uTestMode = 0;
void appCommon_SetTestMode(uint8_t value)
{
	uTestMode = value;
}

uint8_t appCommon_GetTestMode(void)
{
	return uTestMode;
}


static TempSensorRawData_t TempSensorRawData = {0};
void appCommon_SetTempSensorRawData(TempSensorRawData_t *pData)
{
	if(pData) {
		memcpy(&TempSensorRawData, pData, sizeof(TempSensorRawData_t));
	}
}

void appCommon_GetTempSensorRawData(TempSensorRawData_t *pData)
{
	if(pData) {
		memcpy(pData, &TempSensorRawData, sizeof(TempSensorRawData_t));
	}
}


#include <time.h>
uint32_t appCommon_TimeDiff(uint32_t time1, uint32_t time0)
{
	uint32_t diff;
	if(time1 >= time0)
		diff = time1 - time0;
	else
		diff = ((uint32_t)-1) - time0 + time1;
	return diff;
}

uint32_t appCommon_GetTime_ms(void)
{
	struct timespec curr_time;
	uint32_t ms;
	clock_gettime(CLOCK_MONOTONIC, &curr_time);
	ms = curr_time.tv_sec * 1000 + curr_time.tv_nsec / 1000000;
	return ms;
}

uint32_t appCommon_GetTime_sec(void)
{
	struct timespec curr_time;
	clock_gettime(CLOCK_MONOTONIC, &curr_time);
	return curr_time.tv_sec;
}

void appCommon_TaskDelay_ms(uint32_t ms)
{
	//minimum value should be 10ms
	if((ms/portTICK_RATE_MS) > 0)
	{
		vTaskDelay(ms/portTICK_RATE_MS);
	}
}

static bool bFlashEncryption = false;
void appCommon_CheckFlashEncrypted(void)
{
#if CONFIG_SECURE_FLASH_ENC_ENABLED
	bFlashEncryption = esp_flash_encryption_enabled();
#endif
}

bool appCommon_IsFlashEncrypted(void)
{
	return bFlashEncryption;
}
