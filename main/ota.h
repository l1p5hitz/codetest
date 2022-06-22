#ifndef _OTA_H
#define _OTA_H

#include "esp_log.h"


typedef struct {
	uint32_t STM32_version;
	char* STM32_url;
	uint32_t ESP32_version;
	char* ESP32_url;
	uint32_t CERT_version;
	char* CERT_url;
	char* appKey;
	char* deploymentId;
} ota_info_t;


bool ota_STM32_IsFlagSet(void);
void ota_STM32_boot_check(void);

int ota_task_run(uint32_t STM32_version, char* STM32_url, uint32_t ESP32_version, char* ESP32_url, uint32_t CERT_version, char* CERT_url, char* appKey, char* deploymentId);
void ota_free(ota_info_t* pOtaInfo);
void ota_info_print(void);

#endif /* _OTA_H */
