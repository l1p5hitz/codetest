#ifndef _APP_STORAGE_H
#define _APP_STORAGE_H


void storage_Init(void);
void storage_Deinit(void);

esp_err_t storage_EraseKey(const char* key);
void storage_EraseCertData(void);

//IAQ LED intensity
esp_err_t storage_WriteIAQLEDIntensity(uint8_t value);
esp_err_t storage_ReadIAQLEDIntensity(uint8_t* pValue);

//DeviceConfig
esp_err_t storage_WriteDeviceConfig(uint8_t value);
esp_err_t storage_ReadDeviceConfig(uint8_t* value);

//OTA
esp_err_t storage_WriteSTM32OTAFlag(uint8_t flag);
esp_err_t storage_ReadSTM32OTAFlag(uint8_t* pFlag);
esp_err_t storage_WriteSTM32OtaDeploymentId(const char* string);
esp_err_t storage_ReadSTM32OtaDeploymentId(char* string, size_t* pLength, size_t max_length);
esp_err_t storage_WriteESP32OtaDeploymentId(const char* string);
esp_err_t storage_ReadESP32OtaDeploymentId(char* string, size_t* pLength, size_t max_length);
esp_err_t storage_WriteCertOtaDeploymentId(const char* string);
esp_err_t storage_ReadCertOtaDeploymentId(char* string, size_t* pLength, size_t max_length);
#ifdef BROAN_SECURE_RELEASE
esp_err_t storage_WriteCertData(const void* pValue, size_t length);
esp_err_t storage_ReadCertData(void* pValue, size_t* pLength, size_t max_length);
#endif

esp_err_t storage_WriteSTM32Sign(const void* pValue, size_t length);
esp_err_t storage_ReadSTM32Sign(void* pValue, size_t* pLength, size_t max_length);

esp_err_t storage_WriteSN(const char* string);
esp_err_t storage_ReadSN(char* string, size_t* pLength, size_t max_length);

//data backup
bool storage_BackupDeviceID(char* device_id);
bool storage_RecoverDeviceID(char* device_id);
bool storage_BackupHwVersion(uint8_t version);
bool storage_RecoverHwVersion(uint8_t* pVersion);

#endif /* _APP_STORAGE_H */
