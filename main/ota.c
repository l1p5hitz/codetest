#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp32/rom/crc.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include <errno.h>
//#include "driver/gpio.h"

#include "CommonUse.h"
#include "app_storage.h"
#include "CertData.h"

#ifdef BROAN_SECURE_RELEASE
#include "../include_bootloader/bootloader_flash.h"
#include "../include_bootloader/bootloader_sha.h"
#include "esp_image_format.h"
#include "esp_secure_boot.h"
#endif

#include "ota_internal.h"
#include "ota.h"


extern bool Debug_bAllowUpdateSTMBootLoader;
extern void StartSTM_SetFWParameter(uint32_t offset, uint32_t size);
extern int StartSTM_FWup(void);


#define OTA_TAG "OTA_TAG"

#define MAX_HTTP_RECV_BUFFER 1024

typedef struct {
	uint32_t downloaded_size;
	bool downloaded_bHeader;
	uint32_t ota_offset;
	uint32_t ota_size;
} ota_STM32_context_t;
static ota_STM32_context_t ota_STM32_ctx = {0};

#define ota_STM32_getMajorVer(ver)	((ver >> 16) & 0xff)
#define ota_STM32_getMinorVer(ver)	((ver >> 8) & 0xff)
#define ota_STM32_getSubVer(ver)	(ver & 0xff)

static bool isOtaHeaderNameCorrect(EmberAfOtaHeader* pOtaHeader, bool bGeneralHeaderString)
{
	char* target_headerString = "STM32";
	if (!pOtaHeader) {
		return false;
	}
#ifdef USE_STM32_CT_BOOTLOADER
	if (STM_CT_BOOTLOADER_VER_bValid) {
		target_headerString = "STMv2";
	}
#endif
	if (bGeneralHeaderString) {
		//check "STM" prefix only
		target_headerString = "STM";
	}
	if (strncmp(pOtaHeader->headerString, target_headerString, strlen(target_headerString)) == 0) {
		return true;
	} else {
#ifdef ALLOW_UPDATE_BOOTLOADER
		if(!STM_CT_BOOTLOADER_VER_bValid
			&& !strncmp(pOtaHeader->headerString, "STMv2", strlen("STMv2"))
			&& (appCommon_GetTestMode() || Debug_bAllowUpdateSTMBootLoader)
			&& (ota_STM32_getMajorVer(pOtaHeader->firmwareVersion) >= 1))
		{
			STM_bPrepareUpdateBootloader = true;
			return true;
		}
#endif
	}
	return false;
}

static esp_err_t ota_STM32_verify(uint32_t downloaded_size, uint32_t *ota_offset, uint32_t *ota_size, bool bGeneralHeaderString)
{
	EmberAfOtaHeader* pOtaHeader = NULL;
	EmberAfTagData* pOtaTagData = NULL;
	uint8_t* pOtaData = NULL;
	esp_err_t err;
	uint32_t crc32_magicNumber;
	uint32_t crc32_value;
	uint32_t crc32_result;

	const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "stm_fw_0");
	if (!partition) {
		return ESP_FAIL;
	}
	/* Map the partition to data memory */
	const uint8_t* p = NULL;
	spi_flash_mmap_handle_t map_handle;
	err = esp_partition_mmap(partition, 0, partition->size, SPI_FLASH_MMAP_DATA, (const void**)&p, &map_handle);
	if (err != ESP_OK) {
		return err;
	}

	pOtaHeader = (EmberAfOtaHeader*)p;
	if (pOtaHeader->MagicNumber != EMBER_AF_OTA_FILE_MAGIC_NUMBER) {
		err = ESP_ERR_NOT_SUPPORTED;
		spi_flash_munmap(map_handle);
		return err;
	}
	if (downloaded_size > 0) {
		if ((pOtaHeader->imageSize) > downloaded_size) {
			err = ESP_ERR_INVALID_SIZE;
			spi_flash_munmap(map_handle);
			return err;
		}
	}

	ESP_LOGI(OTA_TAG, "STM32 OTA image ver = %d.%d.%d",
		ota_STM32_getMajorVer(pOtaHeader->firmwareVersion),
		ota_STM32_getMinorVer(pOtaHeader->firmwareVersion),
		ota_STM32_getSubVer(pOtaHeader->firmwareVersion));

	//check ota name
	if (!isOtaHeaderNameCorrect(pOtaHeader, false)) {
		err = ESP_FAIL;
		spi_flash_munmap(map_handle);
		return err;
	}
	//get ota data length
	pOtaTagData = (EmberAfTagData*)(p + pOtaHeader->headerLength);
	//if (pOtaTagData->id != EMBER_AF_OTA_TAG_UPGRADE_IMAGE) {
	//	err = ESP_ERR_NOT_FOUND;
	//	spi_flash_munmap(map_handle);
	//	return err;
	//}
	pOtaData = (uint8_t*)pOtaTagData + sizeof(*pOtaTagData);
	crc32_magicNumber = *((uint32_t*)(pOtaData + pOtaTagData->length - 8));
	crc32_value = *((uint32_t*)(pOtaData + pOtaTagData->length - 4));
	if(crc32_magicNumber != OTA_CRC32_MAGIC_NUMBER) {
		err = ESP_ERR_INVALID_SIZE;
		spi_flash_munmap(map_handle);
		return err;
	}

	//CRC32 verify
	crc32_result = crc32_le(0, pOtaData, pOtaTagData->length - 8);
	if (crc32_result != crc32_value) {
		err = ESP_ERR_INVALID_CRC;
		spi_flash_munmap(map_handle);
		return err;
	}

	if (ota_offset) {
		*ota_offset = (uint32_t)pOtaData - (uint32_t)pOtaHeader;
	}
	if (ota_size) {
		*ota_size = pOtaTagData->length - 8;
	}
	err = ESP_OK;

	return err;
}

//check if STM32 OTA is failed to complete
bool ota_STM32_IsFlagSet(void)
{
	uint8_t STM32_OTA_flag = WRITE_STM_FLAG_DOWN;
	if (storage_ReadSTM32OTAFlag(&STM32_OTA_flag) != ESP_OK)
	{
		return false;
	}

	return (STM32_OTA_flag == WRITING_STM_FLAG_UP)?true:false;
}

void ota_STM32_boot_check(void)
{
	esp_err_t err;
	uint8_t STM32_OTA_flag = WRITE_STM_FLAG_DOWN;
	if (storage_ReadSTM32OTAFlag(&STM32_OTA_flag) == ESP_OK)
	{
		if (STM32_OTA_flag != WRITE_STM_FLAG_DOWN)
		{
			bool bOTASuccess = false;
			uint32_t ota_offset = 0;
			uint32_t ota_size = 0;
			ESP_LOGW(OTA_TAG, "STM32 OTA flag is set");
#ifdef ALLOW_UPDATE_BOOTLOADER
			Debug_bAllowUpdateSTMBootLoader = true;
#endif
			//verify and do OTA again
			err = ota_STM32_verify(0, &ota_offset, &ota_size, false);
			if (err == ESP_OK) {
				StartSTM_SetFWParameter(ota_offset, ota_size);
				if (StartSTM_FWup() < 0) {
					ESP_LOGE(OTA_TAG, "Fail to FW update STM32");
				} else {
					bOTASuccess = true;
				}
			} else if (err == ESP_ERR_NOT_SUPPORTED) {
#if 1
				ESP_LOGE(OTA_TAG, "No STM32 OTA header");
				STM32_OTA_flag = WRITE_STM_FLAG_DOWN;
				storage_WriteSTM32OTAFlag(STM32_OTA_flag);
#else
				//If no OTA header, copy whole stm_fw_0 partition
				const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "stm_fw_0");
				if (partition) {
					ESP_LOGW(OTA_TAG, "No STM32 OTA header, copy whole stm_fw_bin partition");
					StartSTM_SetFWParameter(0, partition->size);
					if (StartSTM_FWup() < 0) {
						ESP_LOGE(OTA_TAG, "Fail to FW update STM32");
					} else {
						bOTASuccess = true;
					}
				} else {
					ESP_LOGE(OTA_TAG, "No stm_fw_bin partition");
					STM32_OTA_flag = WRITE_STM_FLAG_DOWN;
					storage_WriteSTM32OTAFlag(STM32_OTA_flag);
				}
#endif
			} else {
				ESP_LOGE(OTA_TAG, "Fail to verify STM32 ota image: 0x%x %s", err, esp_err_to_name(err));
				STM32_OTA_flag = WRITE_STM_FLAG_DOWN;
				storage_WriteSTM32OTAFlag(STM32_OTA_flag);
			}

			if (bOTASuccess) {
				ESP_LOGI(OTA_TAG, "restart now!");
				esp_restart();
			}
#ifdef ALLOW_UPDATE_BOOTLOADER
			Debug_bAllowUpdateSTMBootLoader = false;
#endif
		}
	}
}

static esp_err_t ota_STM32_http_download(ota_info_t* pInfo)
{
	esp_http_client_handle_t client;
	esp_http_client_config_t http_config = {0};
	esp_err_t err = ESP_OK;

	ota_STM32_ctx.downloaded_bHeader = false;
	if ((pInfo == NULL) || (pInfo->STM32_url == NULL)) {
		return ESP_FAIL;
	}
	http_config.url = pInfo->STM32_url;
	//http_config.event_handler = http_ota_event_handler;
	client = esp_http_client_init(&http_config);
	if (client == NULL) {
		return ESP_FAIL;
	}
	char *buffer = malloc(MAX_HTTP_RECV_BUFFER + sizeof(EmberAfOtaHeader));
	char *tmp_data = malloc(MAX_HTTP_RECV_BUFFER + 16 + sizeof(EmberAfOtaHeader));
	if ((buffer == NULL) || (tmp_data == NULL)) {
		esp_http_client_close(client);
		esp_http_client_cleanup(client);
		return ESP_ERR_NO_MEM;
	}

	esp_http_client_set_method(client, HTTP_METHOD_GET);
	if (pInfo->appKey) {
		esp_http_client_set_header(client, "appKey", pInfo->appKey);
	}
	err = esp_http_client_open(client, 0);
	if (err != ESP_OK) {
		ESP_LOGE(OTA_TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
	} else {
		//prepare STM OTA partition
		const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "stm_fw_0");
		if (!partition) {
			ESP_LOGE(OTA_TAG, "No stm_fw_bin partition");
			err = ESP_FAIL;
		} else {
			err = esp_partition_erase_range(partition, 0, partition->size);

			//get OTA data from HTTP server
			bool bWriteImage = false;
			bool bWriteImageFinished = false;
			int ota_image_offset = 0;
			int total_read_len = 0;
			int total_write_len = 0;
			int read_len;
			int tmp_data_offset = 0;
			int content_length =  esp_http_client_fetch_headers(client);
			int status_code = esp_http_client_get_status_code(client);
			uint8_t *pDummyHeader = malloc(sizeof(EmberAfOtaHeader)); //56 bytes
			int dummyHeader_partial_len = 0;
			ESP_LOGI(OTA_TAG, "HTTP status_code = %d content_length = %d", status_code, content_length);
			//4xx Client errors, e.g. 404 Not Found
			//5xx Server errors, e.g. 500 Internal Server Error
			if ((status_code >= 400 && status_code <= 599) || status_code == -1) {
				ESP_LOGE(OTA_TAG, "HTTP status error");
				err = ESP_FAIL;
			} else if (pDummyHeader == NULL){
				ESP_LOGE(OTA_TAG, "fail to malloc ota header");
				err = ESP_FAIL;
			} else {
				//if (content_length > 0) { //not "Transfer-Encoding: chunked"
				//	if (content_length > partition->size) {
				//		err = ESP_ERR_INVALID_SIZE;
				//		ESP_LOGE(OTA_TAG, "ota file size too large");
				//	}
				//}
				while (err == ESP_OK) {
					read_len = esp_http_client_read(client, buffer, MAX_HTTP_RECV_BUFFER);
					if (read_len == 0) {
						if (esp_http_client_is_complete_data_received(client)) {
							//download complete
							break;
						} else if ((errno == ENOTCONN || errno == ECONNRESET || errno == ECONNABORTED)) {
							ESP_LOGE(OTA_TAG, "HTTP connection closed");
							err = ESP_FAIL;
							break;
						} else {
							//no data received, try it later
							vTaskDelay(100/portTICK_PERIOD_MS);
						}
					} else if (read_len < 0) {
						err = ESP_FAIL;
						break;
					}

					//Step 1) If bWriteImage = true, go to step 2
					//Step 2) If bCheckHeader = true, go to step 3
					//Step 3) If bWriteImage = true, start to write OTA image and go to step 4
					//Step 4) If bWriteImageFinished = true, OTA image writing is finished
					bool bCheckHeader = false;
					int check_offset = 0;
					int check_len = read_len;
					while (!bWriteImage && (err == ESP_OK) && ((total_read_len + read_len) > ota_image_offset)) {
						if (ota_image_offset == 0) {
							check_offset = 0;
							check_len = read_len;
							bCheckHeader = true;
						} else if ((total_read_len + read_len) > ota_image_offset) {
							if (ota_image_offset >= total_read_len) {
								check_offset = ota_image_offset - total_read_len;
								check_len = read_len - check_offset;
							} else {
								//should not have this case
								check_offset = 0;
								check_len = read_len;
							}
							bCheckHeader = true;
						}

						if (bCheckHeader) {
							ESP_LOGI(OTA_TAG, "check_offset:%d check_len:%d", check_offset, check_len);
							//override check_offset & check_len if dummyHeader_partial_len > 0
							if (dummyHeader_partial_len) {
								memcpy(tmp_data, pDummyHeader, dummyHeader_partial_len);
								memcpy(tmp_data + dummyHeader_partial_len, buffer, read_len);
								memcpy(buffer, tmp_data, dummyHeader_partial_len + read_len);
								check_offset = 0;
								check_len = dummyHeader_partial_len + read_len;
								ESP_LOGI(OTA_TAG, "add partial header, check_offset:%d check_len:%d", check_offset, check_len);
								dummyHeader_partial_len = 0;
							}

							if (check_len < sizeof(EmberAfOtaHeader)) {
								//OTA header incomplete, wait next buffer
								memcpy(pDummyHeader, &buffer[check_offset], check_len);
								dummyHeader_partial_len = check_len;
								bCheckHeader = false;
								break;
							}
						}

						if (bCheckHeader) {
							uint8_t *start_byte = (uint8_t *)&buffer[check_offset];
							if (*((uint32_t *)start_byte) == EMBER_AF_OTA_FILE_MAGIC_NUMBER) {
								//ota_STM32_ctx.downloaded_bHeader = true;

								EmberAfOtaHeader* pOtaHeader = NULL;
								pOtaHeader = (EmberAfOtaHeader *)start_byte;
								ESP_LOGI(OTA_TAG, "STM32 OTA image ver = %d.%d.%d (name = %.*s)",
									ota_STM32_getMajorVer(pOtaHeader->firmwareVersion),
									ota_STM32_getMinorVer(pOtaHeader->firmwareVersion),
									ota_STM32_getSubVer(pOtaHeader->firmwareVersion),
									sizeof(pOtaHeader->headerString), pOtaHeader->headerString);

								ESP_LOGI(OTA_TAG, "decoded image size = %d", pOtaHeader->imageSize);
								ota_image_offset += pOtaHeader->imageSize;
								
								//check ota name
								if (isOtaHeaderNameCorrect(pOtaHeader, false)) {
									ESP_LOGI(OTA_TAG, "OTA name correct");
									ota_STM32_ctx.downloaded_bHeader = true;
									bWriteImage = true;
								} else {
									ESP_LOGE(OTA_TAG, "OTA name incorrect");
								}
							} else {
								ESP_LOGE(OTA_TAG, "no ota header");
								err = ESP_FAIL;
								break;
							}
						}
					}
					if (err != ESP_OK) {
						break;
					}

					if (bWriteImage) {
						if (bWriteImageFinished) {
							//ignore the remaining data
						} else {
							char *write_src;
							int write_size;
							int _offset;
							int _len;
							if (bCheckHeader) {
								//the first data of OTA image
								_offset = check_offset;
								_len = check_len;
							} else if ((total_read_len + read_len) >= ota_image_offset) {
								//the last data of OTA image
								_offset = 0;
								_len = ota_image_offset - total_read_len;
								bWriteImageFinished = true;
							} else {
								_offset = 0;
								_len = read_len;
							}
							ESP_LOGI(OTA_TAG, "_offset:%d _len:%d", _offset, _len);

							//addr & size need be 16-byte alignment for encrypted OTA partition
							memcpy(tmp_data + tmp_data_offset, buffer + _offset, _len);
							write_src = tmp_data;
							write_size = ((tmp_data_offset + _len)/16)*16;
							tmp_data_offset = (tmp_data_offset + _len)%16; //next tmp_data_offset

							//write data to STM OTA partition
							if (write_size) {
								err = esp_partition_write(partition, total_write_len, write_src, write_size);
								if (err != ESP_OK) {
									ESP_LOGI(OTA_TAG, "fail to write partition");
									break;
								}

								if (tmp_data_offset) {
									memset(tmp_data, 0, 16);
									memcpy(tmp_data, write_src + write_size, tmp_data_offset);
								}
								total_write_len += write_size;
							}

							if (bWriteImageFinished) {
								if (tmp_data_offset) {
									ESP_LOGI(OTA_TAG, "tmp_data_offset:%d", tmp_data_offset);
									//write the rest of tmp_data
									//addr & size need be 16-byte alignment for encrypted OTA partition
									write_src = tmp_data;
									write_size = ((tmp_data_offset + 15)/16)*16;
									err = esp_partition_write(partition, total_write_len, write_src, write_size);
									if (err != ESP_OK) {
										ESP_LOGI(OTA_TAG, "fail to write partition");
										break;
									}
									total_write_len += tmp_data_offset;
									tmp_data_offset = 0;
								}
							}
						}
					}
					total_read_len += read_len;
				}

				if (err == ESP_OK) {
					if (!bWriteImage) {
						ESP_LOGE(OTA_TAG, "nothing to write");
						err = ESP_FAIL;
					} else if(!bWriteImageFinished) {
						ESP_LOGE(OTA_TAG, "write incomplete");
						err = ESP_FAIL;
					}
				}
				if (err == ESP_OK) {
					ESP_LOGI(OTA_TAG, "downloaded size: %d", total_write_len);
					ota_STM32_ctx.downloaded_size = total_write_len;
				}
			}
			if (pDummyHeader) {
				free(pDummyHeader);
				pDummyHeader = NULL;
			}
		}
	}

	if (client) {
		esp_http_client_close(client);
		esp_http_client_cleanup(client);
	}
	if (buffer) {
		free(buffer);
	}
	if (tmp_data) {
		free(tmp_data);
	}
	return err; 
}

#ifdef BROAN_SECURE_RELEASE
static esp_err_t ota_check_signature(uint32_t ota_offset, uint32_t ota_size, void* signature_buffer, const char* partition_label)
{
#if (defined CONFIG_SECURE_SIGNED_ON_UPDATE) && (defined CONFIG_SECURE_SIGNED_APPS_ECDSA_SCHEME)
#else
	#error
#endif
	esp_err_t err;
	#define HASH_LEN ESP_IMAGE_HASH_LEN 
	uint8_t image_digest[HASH_LEN] = { [ 0 ... 31] = 0xEE };
	uint8_t verified_digest[HASH_LEN] = { [ 0 ... 31 ] = 0x01 };
	bootloader_sha256_handle_t sha_handle = NULL;

	if (!partition_label) {
		return ESP_FAIL;
	}
	const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, partition_label);
	if (!partition) {
		return ESP_FAIL;
	}
	if (partition->encrypted) {
		/* Map the partition to data memory */
		const uint8_t* p = NULL;
		spi_flash_mmap_handle_t map_handle;
		err = esp_partition_mmap(partition, 0, partition->size, SPI_FLASH_MMAP_DATA, (const void**)&p, &map_handle);
		if (err != ESP_OK) {
			return err;
		}

		//at least larger than signature block
		if(ota_size <= sizeof(esp_secure_boot_sig_block_t)) {
			spi_flash_munmap(map_handle);
			return ESP_FAIL;
		}

		sha_handle = bootloader_sha256_start();
		if (sha_handle == NULL) {
			spi_flash_munmap(map_handle);
			return ESP_ERR_NO_MEM;
		}
		ESP_LOGI(OTA_TAG, "verify signature...");
		bootloader_sha256_data(sha_handle, p + ota_offset, ota_size - sizeof(esp_secure_boot_sig_block_t)); //data_len need to be multiple of 4
		bootloader_sha256_finish(sha_handle, image_digest);
		// Log the hash for debugging
		//ESP_LOGI(OTA_TAG, "image_digest:");
		//esp_log_buffer_hex(OTA_TAG, image_digest, HASH_LEN);

		const void *sig_block;
		sig_block = p + ota_offset + ota_size - sizeof(esp_secure_boot_sig_block_t);
		err = esp_secure_boot_verify_ecdsa_signature_block(sig_block, image_digest, verified_digest);
		if (err == ESP_OK) {
			ESP_LOGI(OTA_TAG, "signature OK");
			if (signature_buffer) {
				memcpy(signature_buffer, ((const esp_secure_boot_sig_block_t *)sig_block)->signature, sizeof(((const esp_secure_boot_sig_block_t *)sig_block)->signature));
			}
		} else {
			ESP_LOGE(OTA_TAG, "signature FAIL");
		}
		spi_flash_munmap(map_handle);
	} else {
		//at least larger than signature block
		if(ota_size <= sizeof(esp_secure_boot_sig_block_t)) {
			return ESP_FAIL;
		}

		uint32_t remaining_len = ota_size - sizeof(esp_secure_boot_sig_block_t);
		uint32_t read_offset = ota_offset;
		#define MAX_READ_BUFFER_SIZE	(1024)
		uint8_t *read_buffer = malloc(MAX_READ_BUFFER_SIZE);
		if(!read_buffer) {
			return ESP_FAIL;
		}

		sha_handle = bootloader_sha256_start();
		if (sha_handle == NULL) {
			return ESP_ERR_NO_MEM;
		}
		ESP_LOGI(OTA_TAG, "verify signature...");
		while (remaining_len > 0) {
			uint32_t read_len = (remaining_len > 1024)?1024:remaining_len;
			err = esp_partition_read(partition, read_offset, read_buffer, read_len);
			if (err != ESP_OK) {
				break;
			}
			bootloader_sha256_data(sha_handle, read_buffer, read_len); //data_len need to be multiple of 4
			read_offset += read_len;
			remaining_len -= read_len;
		}
		bootloader_sha256_finish(sha_handle, image_digest);
		// Log the hash for debugging
		//ESP_LOGI(OTA_TAG, "image_digest:");
		//esp_log_buffer_hex(OTA_TAG, image_digest, HASH_LEN);

		if(err == ESP_OK) {
			const void *sig_block;
			err = esp_partition_read(partition, ota_offset + ota_size - sizeof(esp_secure_boot_sig_block_t), read_buffer, sizeof(esp_secure_boot_sig_block_t));
			if (err == ESP_OK) {
				sig_block = read_buffer;
				err = esp_secure_boot_verify_ecdsa_signature_block(sig_block, image_digest, verified_digest);
				if (err == ESP_OK) {
					ESP_LOGI(OTA_TAG, "signature OK");
					if (signature_buffer) {
						memcpy(signature_buffer, ((const esp_secure_boot_sig_block_t *)sig_block)->signature, sizeof(((const esp_secure_boot_sig_block_t *)sig_block)->signature));
					}
				} else {
					ESP_LOGE(OTA_TAG, "signature FAIL");
				}
			}
		}
		if(read_buffer) {
			free(read_buffer);
		}
	}
	return err;
}

static esp_err_t ota_STM32_check_signature(uint32_t ota_offset, uint32_t ota_size, void* signature_buffer)
{
	return ota_check_signature(ota_offset, ota_size, signature_buffer, "stm_fw_0");
}

static esp_err_t ota_CERT_check_signature(uint32_t ota_offset, uint32_t ota_size, void* signature_buffer)
{
	return ota_check_signature(ota_offset, ota_size, signature_buffer, "cert");
}
#endif

static esp_err_t ota_STM32_proc(ota_info_t* pInfo)
{
	uint8_t retry_time = 3;
	esp_err_t err;
	ESP_LOGI(OTA_TAG, "now do STM OTA......");

#ifdef BROAN_SECURE_RELEASE
	//stop signature verification before starting OTA
	if (!STM_FW_SIGN_bValid) {
		STM_FW_SIGN_bValid = true;
	}
#endif

	for (uint8_t i = 0; i < retry_time; i++) {
		err = ota_STM32_http_download(pInfo);
		if (err == ESP_OK) {
			break;
		}
	}
	if (err != ESP_OK) {
		return err;
	}

	STM_bPrepareUpdateBootloader = false;
	if (ota_STM32_ctx.downloaded_bHeader) {
		//verify header and CRC32 checksum
		err = ota_STM32_verify(ota_STM32_ctx.downloaded_size, &ota_STM32_ctx.ota_offset, &ota_STM32_ctx.ota_size, false);
		if (err != ESP_OK) {
			return err;
		}
		StartSTM_SetFWParameter(ota_STM32_ctx.ota_offset, ota_STM32_ctx.ota_size);
	} else {
#if 1
		//fail if no OTA header
		ESP_LOGE(OTA_TAG, "No STM32 OTA header");
		return ESP_FAIL;
#else
		//backward compatible with no OTA header FW
		ota_STM32_ctx.ota_offset = 0;
		ota_STM32_ctx.ota_size = ota_STM32_ctx.downloaded_size;
		StartSTM_SetFWParameter(ota_STM32_ctx.ota_offset, ota_STM32_ctx.ota_size);
#endif
	}
#ifdef BROAN_SECURE_RELEASE
	//check STM32 ECDSA signature
	err = ota_STM32_check_signature(ota_STM32_ctx.ota_offset, ota_STM32_ctx.ota_size, STM_FW_SIGN_ARR);
	if (err != ESP_OK) {
		ESP_LOGE(OTA_TAG, "Invalid STM32 sign");
		return err;
	}
	STM_FW_SIGN_bOTAValid = true;
#endif
	for (uint8_t i = 0; i < retry_time; i++) {
		if (StartSTM_FWup() < 0) {
			err = ESP_FAIL;
		} else {
			err = ESP_OK;
			break;
		}
	}
#ifdef BROAN_SECURE_RELEASE
	if (err == ESP_OK) {
		bool bWriteSign = true;
		uint8_t readSignBuffer[64] = {0};
		size_t read_sign_len = 0;
		if(storage_ReadSTM32Sign(readSignBuffer, &read_sign_len, sizeof(readSignBuffer)) == ESP_OK) {
			if(memcmp(readSignBuffer, STM_FW_SIGN_ARR, sizeof(STM_FW_SIGN_ARR)) == 0) {
				ESP_LOGI(OTA_TAG, "same STM32 signature");
				bWriteSign = false;
			}
		}
		if (bWriteSign) {
			storage_WriteSTM32Sign(STM_FW_SIGN_ARR, sizeof(STM_FW_SIGN_ARR));
		}
	}
	STM_FW_SIGN_bOTAValid = false;
#endif

	return err;
}

static esp_err_t ota_ESP32_http_update(ota_info_t* pInfo)
{
	esp_http_client_handle_t client;
	esp_http_client_config_t http_config = {0};
	esp_err_t err = ESP_OK;
	if ((pInfo == NULL) || (pInfo->ESP32_url == NULL)) {
		return ESP_FAIL;
	}

	char *buffer = malloc(MAX_HTTP_RECV_BUFFER);
	if (buffer == NULL) {
		return ESP_ERR_NO_MEM;
	}

	http_config.url = pInfo->ESP32_url;
	client = esp_http_client_init(&http_config);
	if (client == NULL) {
		err = ESP_FAIL;
	} else {
		esp_http_client_set_method(client, HTTP_METHOD_GET);
		if (pInfo->appKey) {
			esp_http_client_set_header(client, "appKey", pInfo->appKey);
		}
		err = esp_http_client_open(client, 0);
		if (err != ESP_OK) {
			ESP_LOGE(OTA_TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
		} else {
			//prepare OTA partition
			const esp_partition_t *running = esp_ota_get_running_partition();
			const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
			/* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
			esp_ota_handle_t update_handle = 0 ;
			if (update_partition == NULL) {
				ESP_LOGE(OTA_TAG, "open partition failed");
				err = ESP_FAIL;
			} else {
				//get OTA data from HTTP server
				uint32_t ota_progress_count = 0;
				int total_write_len = 0;
				int read_len;
				int content_length =  esp_http_client_fetch_headers(client);
				int status_code = esp_http_client_get_status_code(client);
				ESP_LOGI(OTA_TAG, "HTTP status_code = %d content_length = %d", status_code, content_length);
				//4xx Client errors, e.g. 404 Not Found
				//5xx Server errors, e.g. 500 Internal Server Error
				if ((status_code >= 400 && status_code <= 599) || status_code == -1) {
					err = ESP_FAIL;
					ESP_LOGE(OTA_TAG, "HTTP status error");
				} else {
					bool image_header_was_checked = false;
					while (err == ESP_OK) {
						read_len = esp_http_client_read(client, buffer, MAX_HTTP_RECV_BUFFER);
						if (read_len == 0) {
							if (esp_http_client_is_complete_data_received(client)) {
								//download complete
								break;
							} else if ((errno == ENOTCONN || errno == ECONNRESET || errno == ECONNABORTED)) {
								ESP_LOGE(OTA_TAG, "HTTP connection closed");
								err = ESP_FAIL;
								break;
							} else {
								//no data received, try it later
								vTaskDelay(100/portTICK_PERIOD_MS);
							}
						} else if (read_len < 0) {
							err = ESP_FAIL;
							break;
						}

						if (image_header_was_checked == false) {
							esp_app_desc_t new_app_info;
							if (read_len > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
								// check current version with downloading
								memcpy(&new_app_info, &buffer[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
								ESP_LOGI(OTA_TAG, "New firmware version: %s", new_app_info.version);
#ifdef USE_STM32_CT_BOOTLOADER
								//new version must be 2.X.X or above
								bool bValidVersion = false;
								char * pch;
								pch = strtok(new_app_info.version, " .");
								if (pch) {
									uint32_t ver_major = atoi(pch);
									if (ver_major >= 2) {
										bValidVersion = true;
									}
								}
								if (!bValidVersion) {
									ESP_LOGE(OTA_TAG, "new version is below 2.X.X");
									err = ESP_FAIL;
									break;
								}
#endif
								esp_app_desc_t running_app_info;
								if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
									ESP_LOGI(OTA_TAG, "Running firmware version: %s", running_app_info.version);
								}

								//const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
								//esp_app_desc_t invalid_app_info;
								//if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
								//	ESP_LOGI(OTA_TAG, "Last invalid firmware version: %s", invalid_app_info.version);
								//}
								//// check current version with last invalid partition
								//if (last_invalid_app != NULL) {
								//	if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
								//		err = ESP_FAIL;
								//		goto exit;
								//	}
								//}

								if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
									ESP_LOGW(OTA_TAG, "Current running version (%s) is the same as new version", new_app_info.version);
								}
							#ifdef BROAN_LEGACY_RELEASE
								//version need to be 0.XX.XX
								if (strncmp(new_app_info.version, "0.", strlen("0.")) != 0) {
									ESP_LOGE(OTA_TAG, "New version (%s) is not legacy release version (0.XX.XX)", new_app_info.version);
									err = ESP_FAIL;
									break;
								}
							#endif
								image_header_was_checked = true;

								err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
								if (err != ESP_OK) {
									ESP_LOGE(OTA_TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
									break;
								}
								ESP_LOGI(OTA_TAG, "esp_ota_begin succeeded");
							} else {
								ESP_LOGE(OTA_TAG, "received package is not fit len");
								err = ESP_FAIL;
								break;
							}
						}
						err = esp_ota_write(update_handle, (const void *)buffer, read_len);
						if (err != ESP_OK) {
							break;
						}
						total_write_len += read_len;

						//print OTA progress
						if (content_length > 0) {
							if ((total_write_len*100)/content_length >= (ota_progress_count+5)) {
								ota_progress_count = (total_write_len*100)/content_length;
								ESP_LOGI(OTA_TAG, "percentage %d%% (%d)", ota_progress_count, total_write_len);
							}
						} else {
							if (total_write_len >= (ota_progress_count+200*1024)) {
								ota_progress_count = total_write_len;
								ESP_LOGI(OTA_TAG, "write len %d", total_write_len);
							}
						}
					}

					if (err == ESP_OK) {
						err = esp_ota_end(update_handle);
						if (err != ESP_OK) {
							if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
								ESP_LOGE(OTA_TAG, "Image validation failed, image is corrupted");
							}
							ESP_LOGE(OTA_TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
						}
					} else {
						if (image_header_was_checked) {
							//because only invoke esp_ota_begin(...) when image_header_was_checked = true
							esp_ota_end(update_handle);
						}
					}

					if (err == ESP_OK) {
						err = esp_ota_set_boot_partition(update_partition);
						if (err != ESP_OK) {
							ESP_LOGE(OTA_TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
						}
					}
				}
			}
		}
	}

	if (client) {
		esp_http_client_close(client);
		esp_http_client_cleanup(client);
	}
	if (buffer) {
		free(buffer);
	}
	return err;
}

static esp_err_t ota_ESP32_proc(ota_info_t* pInfo)
{
	uint8_t retry_time = 3;
	esp_err_t err;
	ESP_LOGI(OTA_TAG, "now do ESP OTA......");

	//gpio_set_level(STM_RST_PIN, STM_RST_RESET);
	for (uint8_t i = 0; i < retry_time; i++) {
		err = ota_ESP32_http_update(pInfo);
		if (err == ESP_OK) {
			break;
		}
	}
	if (err == ESP_OK) {
		//...
	} else {
		ESP_LOGE(OTA_TAG, "ota update failed: 0x%x %s", err, esp_err_to_name(err));
	}
	return err;
}

typedef struct {
	//uint32_t ota_offset;
	uint32_t ota_size;
} ota_CERT_context_t;
static ota_CERT_context_t ota_CERT_ctx = {0};

static esp_err_t ota_CERT_http_download(ota_info_t* pInfo)
{
	esp_http_client_handle_t client;
	esp_http_client_config_t http_config = {0};
	esp_err_t err = ESP_OK;

	ota_CERT_ctx.ota_size = 0;
	if ((pInfo == NULL) || (pInfo->CERT_url == NULL)) {
		return ESP_FAIL;
	}
	http_config.url = pInfo->CERT_url;
	//http_config.event_handler = http_ota_event_handler;
	client = esp_http_client_init(&http_config);
	if (client == NULL) {
		return ESP_FAIL;
	}
	char *buffer = malloc(MAX_HTTP_RECV_BUFFER);
	char *tmp_data = malloc(MAX_HTTP_RECV_BUFFER + 16);
	if ((buffer == NULL) || (tmp_data == NULL)) {
		esp_http_client_close(client);
		esp_http_client_cleanup(client);
		return ESP_ERR_NO_MEM;
	}

	esp_http_client_set_method(client, HTTP_METHOD_GET);
	if (pInfo->appKey) {
		esp_http_client_set_header(client, "appKey", pInfo->appKey);
	}
	err = esp_http_client_open(client, 0);
	if (err != ESP_OK) {
		ESP_LOGE(OTA_TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
	} else {
		//prepare CERT OTA partition
		const esp_partition_t *partition = NULL;
		partition = certData_GetPartition();
		if (!partition) {
			ESP_LOGE(OTA_TAG, "No CERT partition");
			err = ESP_FAIL;
		} else {
			err = esp_partition_erase_range(partition, 0, partition->size);

			//get OTA data from HTTP server
			int total_write_len = 0;
			char *write_src;
			int write_size;
			int read_len;
			int tmp_data_offset = 0;
			int content_length =  esp_http_client_fetch_headers(client);
			int status_code = esp_http_client_get_status_code(client);
			ESP_LOGI(OTA_TAG, "HTTP status_code = %d content_length = %d", status_code, content_length);
			//4xx Client errors, e.g. 404 Not Found
			//5xx Server errors, e.g. 500 Internal Server Error
			if ((status_code >= 400 && status_code <= 599) || status_code == -1) {
				ESP_LOGE(OTA_TAG, "HTTP status error");
				err = ESP_FAIL;
			} else {
				if (content_length > 0) { //not "Transfer-Encoding: chunked"
					if (content_length > partition->size) {
						err = ESP_ERR_INVALID_SIZE;
						ESP_LOGE(OTA_TAG, "ota file size too large");
					}
				}
				while (err == ESP_OK) {
					read_len = esp_http_client_read(client, buffer, MAX_HTTP_RECV_BUFFER);
					if (read_len == 0) {
						if (esp_http_client_is_complete_data_received(client)) {
							//download complete
							if (tmp_data_offset) {
								//write the rest of tmp_data
								//addr & size need be 16-byte alignment for encrypted OTA partition
								write_src = tmp_data;
								write_size = ((tmp_data_offset + 15)/16)*16;
								err = esp_partition_write(partition, total_write_len, write_src, write_size);
								if (err != ESP_OK) {
									ESP_LOGI(OTA_TAG, "fail to write partition");
									break;
								}
								total_write_len += tmp_data_offset;
								tmp_data_offset = 0;
							}
							break;
						} else if ((errno == ENOTCONN || errno == ECONNRESET || errno == ECONNABORTED)) {
							ESP_LOGE(OTA_TAG, "HTTP connection closed");
							err = ESP_FAIL;
							break;
						} else {
							//no data received, try it later
							vTaskDelay(100/portTICK_PERIOD_MS);
						}
					} else if (read_len < 0) {
						err = ESP_FAIL;
						break;
					}

					//addr & size need be 16-byte alignment for encrypted OTA partition (optional)
					memcpy(tmp_data + tmp_data_offset, buffer, read_len);
					write_src = tmp_data;
					write_size = ((tmp_data_offset + read_len)/16)*16;
					tmp_data_offset = (tmp_data_offset + read_len)%16; //next tmp_data_offset

					//write data to CERT OTA partition
					if (write_size) {
						err = esp_partition_write(partition, total_write_len, write_src, write_size);
						if (err != ESP_OK) {
							ESP_LOGI(OTA_TAG, "fail to write partition");
							break;
						}

						if (tmp_data_offset) {
							memset(tmp_data, 0, 16);
							memcpy(tmp_data, write_src + write_size, tmp_data_offset);
						}
						total_write_len += write_size;
					}
				}
				if (err == ESP_OK) {
					ESP_LOGI(OTA_TAG, "downloaded size: %d", total_write_len);
					ota_CERT_ctx.ota_size = total_write_len;
				}
			}
		}
	}

	if (client) {
		esp_http_client_close(client);
		esp_http_client_cleanup(client);
	}
	if (buffer) {
		free(buffer);
	}
	if (tmp_data) {
		free(tmp_data);
	}
	return err; 
}

static esp_err_t ota_CERT_proc(ota_info_t* pInfo)
{
	uint8_t retry_time = 3;
	esp_err_t err;
	bool bVerifyOK = false;
	ESP_LOGI(OTA_TAG, "now do CERT OTA......");
	for (uint8_t i = 0; i < retry_time; i++) {
		err = ota_CERT_http_download(pInfo);
		if (err == ESP_OK) {
			break;
		}
	}
	if (err != ESP_OK) {
		return err;
	}

#ifdef BROAN_SECURE_RELEASE
	err = ota_CERT_check_signature(0, ota_CERT_ctx.ota_size, NULL);
	if (err != ESP_OK) {
		ESP_LOGE(OTA_TAG, "Invalid CERT sign");
		return err;
	}

	if (certData_VerifyPartition(ota_CERT_ctx.ota_size, NULL)) {
		uint8_t *buffer = malloc(ota_CERT_ctx.ota_size);
		if(buffer) {
			const esp_partition_t *partition = certData_GetPartition();
			if (partition) {
				err = esp_partition_read(partition, 0, buffer, ota_CERT_ctx.ota_size);
				if (err == ESP_OK) {
					err = storage_WriteCertData(buffer, ota_CERT_ctx.ota_size);
					if (err == ESP_OK) {
						bVerifyOK = true;
					}
				}
			}
			free(buffer);
		}
	}
#else
	if (certData_VerifyPartition(ota_CERT_ctx.ota_size, NULL)) {
		bVerifyOK = true;
	}
#endif

	if (bVerifyOK) {
		err = ESP_OK;
	} else {
		err = ESP_FAIL;
	}
	return err;
}

static TaskHandle_t otaTask_handle = NULL;
static void ota_task(void *arg)
{
	ota_info_t* pInfo = (ota_info_t *)arg;
	esp_err_t err;
	bool bRestart = false;

	if (pInfo->STM32_url) {
		ESP_LOGI(OTA_TAG, "STM32 OTA ver = 0x%06x url = %s", pInfo->STM32_version, pInfo->STM32_url);
	}
	if (pInfo->ESP32_url) {
		ESP_LOGI(OTA_TAG, "ESP32 OTA ver = 0x%06x url = %s", pInfo->ESP32_version, pInfo->ESP32_url);
	}

	ESP_LOGI(OTA_TAG, "suspend some task and delay 1s");
	//Suspend_BleTask = 1;
	Suspend_ButtonTask = 1;
	Suspend_SensorTask = 1;
	Suspend_Uart = 1;
	Suspend_MsgTask = 1;
	vTaskDelay(1000/portTICK_PERIOD_MS);

	
	//STM32 OTA
	if (pInfo->STM32_url) {
		err = ota_STM32_proc(pInfo);
		if (err == ESP_OK) {
			ESP_LOGI(OTA_TAG, "STM OTA OK!");
			if (pInfo->deploymentId) {
				storage_WriteSTM32OtaDeploymentId(pInfo->deploymentId);
			} else {
				size_t len = 0;
				if (storage_ReadSTM32OtaDeploymentId(NULL, &len, 0) == ESP_OK) {
					if(len > 1) {
						storage_WriteSTM32OtaDeploymentId("");
					}
				}
			}
			bRestart = true;
		} else {
			ESP_LOGE(OTA_TAG, "STM OTA fail!, err = %d", err);
		}
		
	}
	//ESP32 OTA
	if (pInfo->ESP32_url) {
		err = ota_ESP32_proc(pInfo);
		if (err == ESP_OK) {
			ESP_LOGI(OTA_TAG, "ESP OTA OK!");
			if (pInfo->deploymentId) {
				storage_WriteESP32OtaDeploymentId(pInfo->deploymentId);
			} else {
				size_t len = 0;
				if (storage_ReadESP32OtaDeploymentId(NULL, &len, 0) == ESP_OK) {
					if(len > 1) {
						storage_WriteESP32OtaDeploymentId("");
					}
				}
			}
			bRestart = true;
		} else {
			ESP_LOGE(OTA_TAG, "ESP OTA fail!, err = %d", err);
		}
	}
	//CERT OTA
	if (pInfo->CERT_url) {
		err = ota_CERT_proc(pInfo);
		if (err == ESP_OK) {
			ESP_LOGI(OTA_TAG, "CERT OTA OK!");
			if (pInfo->deploymentId) {
				storage_WriteCertOtaDeploymentId(pInfo->deploymentId);
			} else {
				size_t len = 0;
				if (storage_ReadCertOtaDeploymentId(NULL, &len, 0) == ESP_OK) {
					if(len > 1) {
						storage_WriteCertOtaDeploymentId("");
					}
				}
			}
			bRestart = true;
		} else {
			ESP_LOGE(OTA_TAG, "CERT OTA fail!, err = %d", err);
		}
	}
	ESP_LOGI(OTA_TAG, "OTA task finished");

	if (bRestart) {
		ESP_LOGI(OTA_TAG, "restart now!");
		vTaskDelay(200/portTICK_PERIOD_MS);
		esp_restart();
	} else {
		vTaskDelay(2000/portTICK_PERIOD_MS);
	}
	//Suspend_BleTask = 0;
	Suspend_ButtonTask = 0;
	Suspend_SensorTask = 0;
	Suspend_Uart = 0;
	Suspend_MsgTask = 0;

	if (pInfo) {
		ota_free(pInfo);
	}
	otaTask_handle = NULL;
	vTaskDelete(NULL);
}


ota_info_t otaInfo = {0}; 
int ota_task_run(uint32_t STM32_version, char* STM32_url, uint32_t ESP32_version, char* ESP32_url, uint32_t CERT_version, char* CERT_url, char* appKey, char* deploymentId)
{
	if (otaTask_handle) {
		ESP_LOGE(OTA_TAG, "OTA task is running");
		return -1;
	}

	ota_free(&otaInfo);
	if (STM32_url) {
		otaInfo.STM32_url = malloc(strlen(STM32_url)+1);
		if (otaInfo.STM32_url) {
			strcpy(otaInfo.STM32_url, STM32_url);
		}
		otaInfo.STM32_version = STM32_version;
	}
	if (ESP32_url) {
		otaInfo.ESP32_url = malloc(strlen(ESP32_url)+1);
		if (otaInfo.ESP32_url) {
			strcpy(otaInfo.ESP32_url, ESP32_url);
		}
		otaInfo.ESP32_version = ESP32_version;
	}
	if (CERT_url) {
		otaInfo.CERT_url = malloc(strlen(CERT_url)+1);
		if (otaInfo.CERT_url) {
			strcpy(otaInfo.CERT_url, CERT_url);
		}
		otaInfo.CERT_version = CERT_version;
	}
	if (appKey) {
		otaInfo.appKey = malloc(strlen(appKey)+1);
		if (otaInfo.appKey) {
			strcpy(otaInfo.appKey, appKey);
		}
	}
	if (deploymentId) {
		otaInfo.deploymentId = malloc(strlen(deploymentId)+1);
		if (otaInfo.deploymentId) {
			strcpy(otaInfo.deploymentId, deploymentId);
		}
	}

	xTaskCreate(ota_task, "ota_task", 8*1024, &otaInfo, 20, &otaTask_handle);
	return 0;
}

void ota_free(ota_info_t* pOtaInfo)
{
	if (pOtaInfo) {
		if (pOtaInfo->STM32_url) {
			free(pOtaInfo->STM32_url);
			pOtaInfo->STM32_url = NULL;
		}
		if (pOtaInfo->ESP32_url) {
			free(pOtaInfo->ESP32_url);
			pOtaInfo->ESP32_url = NULL;
		}
		if (pOtaInfo->CERT_url) {
			free(pOtaInfo->CERT_url);
			pOtaInfo->CERT_url = NULL;
		}
		if (pOtaInfo->appKey) {
			free(pOtaInfo->appKey);
			pOtaInfo->appKey = NULL;
		}
		if (pOtaInfo->deploymentId) {
			free(pOtaInfo->deploymentId);
			pOtaInfo->deploymentId = NULL;
		}
	}
}

void ota_info_print(void)
{
	//print ESP32 OTA partition info
	const esp_partition_t *factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
	const esp_partition_t *ota_0 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
	const esp_partition_t *ota_1 = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_app_desc_t app_desc;
	if(running) {
		printf("running partition\n");
		printf("type: 0x%x subtype: 0x%x\n", running->type, running->subtype);
		printf("addr: 0x%x size: 0x%x\n", running->address, running->size);
		printf("label: %s\n", running->label);
		if(esp_ota_get_partition_description(running, &app_desc)==ESP_OK) {
			printf("version: %s %s %s\n\n", app_desc.version, app_desc.date, app_desc.time);
		}
	}
	if(factory && esp_ota_get_partition_description(factory, &app_desc)==ESP_OK) {
		printf("[%s] %s %s %s\n", "factory", app_desc.version, app_desc.date, app_desc.time);
	}
	if(ota_0 && esp_ota_get_partition_description(ota_0, &app_desc)==ESP_OK) {
		printf("[%s] %s %s %s\n", "ota_0", app_desc.version, app_desc.date, app_desc.time);
	}
	if(ota_1 && esp_ota_get_partition_description(ota_1, &app_desc)==ESP_OK) {
		printf("[%s] %s %s %s\n", "ota_1", app_desc.version, app_desc.date, app_desc.time);
	}
}

