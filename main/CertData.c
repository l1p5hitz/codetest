#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "esp_partition.h"
#include "http_parser.h"

#include "CommonUse.h"
#include "app_storage.h"

#include "CertData.h"


#define CERTDATA_TAG	"CERTDATA"

typedef struct {
	uint8_t *pData;
	size_t len;
	bool bValid;
	cert_info_t info;
} certData_ctx_t;
static certData_ctx_t certData_ctx = {0};


bool certData_CompareHostname(const char* url, const char* hostname)
{
	struct http_parser_url puri;
	if (url == NULL || hostname == NULL) {
		return false;
	}
	http_parser_url_init(&puri);
	int parser_status = http_parser_parse_url(url, strlen(url), 0, &puri);
	if (parser_status != 0) {
		return false;
	}
	if (puri.field_data[UF_HOST].len == 0) {
		return false;
	}
	if (strncmp(hostname, url + puri.field_data[UF_HOST].off, puri.field_data[UF_HOST].len) == 0) {
		return true;
	}
	return false;
}

void certData_DebugPrint(cert_info_t *pInfo, bool bDumpHex)
{
	if(pInfo == NULL) {
		return;
	}

	ESP_LOGI(CERTDATA_TAG, "version=%d.%d.%d", pInfo->version_major, pInfo->version_minor, pInfo->version_sub);
	ESP_LOGI(CERTDATA_TAG, "hostname=%s",  (pInfo->hostname)?pInfo->hostname:"<null>");
	ESP_LOGI(CERTDATA_TAG, "username=%s",  (pInfo->username)?pInfo->username:"<null>");
	ESP_LOGI(CERTDATA_TAG, "password=%s",  (pInfo->password)?pInfo->password:"<null>");
	ESP_LOGI(CERTDATA_TAG, "ca_cert_len=%d",  pInfo->ca_cert_len);
	if (bDumpHex && pInfo->ca_cert_len) {
		ESP_LOG_BUFFER_HEXDUMP(CERTDATA_TAG, pInfo->ca_cert, pInfo->ca_cert_len, ESP_LOG_INFO);
	}
	ESP_LOGI(CERTDATA_TAG, "client_cert_len=%d",  pInfo->client_cert_len);
	if (bDumpHex && pInfo->client_cert_len) {
		ESP_LOG_BUFFER_HEXDUMP(CERTDATA_TAG, pInfo->client_cert, pInfo->client_cert_len, ESP_LOG_INFO);
	}
	ESP_LOGI(CERTDATA_TAG, "client_key_len=%d",  pInfo->client_key_len);
	if (bDumpHex && pInfo->client_key_len) {
		ESP_LOG_BUFFER_HEXDUMP(CERTDATA_TAG, pInfo->client_key, pInfo->client_key_len, ESP_LOG_INFO);
	}
}

const esp_partition_t * certData_GetPartition(void)
{
#ifdef BROAN_SECURE_RELEASE
	return esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "cert");
#else
	return esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "stm_fw_1");
#endif
}

bool certData_VerifyPartition(size_t size, size_t *pCertDataLen)
{
	esp_err_t err = ESP_OK;
	const esp_partition_t *partition = NULL;
	cert_header_t certHeader = {0};
	partition = certData_GetPartition();
	if (!partition) {
		return false;
	}
	if (size > 0) {
		if ((size < sizeof(cert_header_t)) || (size > partition->size)) {
			return false;
		}
	}
	err = esp_partition_read(partition, 0, &certHeader, sizeof(certHeader));
	if (err != ESP_OK) {
		return false;
	}
	if (memcmp(certHeader.marker, CERT_FILE_MARKER, CERT_FILE_MARKER_LEN)) {
		ESP_LOGE(CERTDATA_TAG, "No cert data header");
		return false;
	}
	if (certHeader.header_version != CERT_HEADER_VERSION_CURR) {
		ESP_LOGE(CERTDATA_TAG, "Unknown cert data version");
		return false;
	}

	//check header checksum
	uint8_t CRC_result;
	CRC_result = ~ esp_crc8_be((uint8_t)~0x00, (uint8_t *)&certHeader, sizeof(certHeader)-1);
	if(CRC_result != certHeader.header_checksum) {
		ESP_LOGE(CERTDATA_TAG, "Incorrect header checksum");
		return false;
	}
	if (size > 0) {
		//check payload_len
		if ((certHeader.payload_len + sizeof(certHeader)) > size) {
			ESP_LOGE(CERTDATA_TAG, "Incorrect payload length");
			return false;
		}
	}

	uint32_t remaining_len = certHeader.payload_len;
	uint32_t read_offset = sizeof(cert_header_t);
	#define MAX_READ_BUFFER_SIZE	(1024)
	uint8_t *read_buffer = malloc(MAX_READ_BUFFER_SIZE);
	if(!read_buffer) {
		return false;
	}
	//check payload checksum
	CRC_result = ~0x00;
	while (remaining_len > 0) {
		uint32_t read_len = (remaining_len > 1024)?1024:remaining_len;
		err = esp_partition_read(partition, read_offset, read_buffer, read_len);
		if (err != ESP_OK) {
			break;
		}
		CRC_result = esp_crc8_be(CRC_result, read_buffer, read_len);
		read_offset += read_len;
		remaining_len -= read_len;
	}
	CRC_result = ~CRC_result;
	if(err == ESP_OK) {
		if (CRC_result == certHeader.payload_checksum) {
			//payload checksum OK
			if (pCertDataLen) {
				*pCertDataLen = sizeof(cert_header_t) + certHeader.payload_len;
			}
		} else {
			ESP_LOGE(CERTDATA_TAG, "Incorrect payload checksum");
			err = ESP_FAIL;
		}
	}

	if(read_buffer) {
		free(read_buffer);
	}
	return (err == ESP_OK)?true:false;
}

bool certData_Decode(uint8_t *data, size_t size, cert_info_t *pInfo)
{
	cert_header_t *pHeader = NULL;
	if (data == NULL || pInfo == NULL) {
		return false;
	}
	pHeader = (cert_header_t *)data;
	if (memcmp(pHeader->marker, CERT_FILE_MARKER, CERT_FILE_MARKER_LEN)) {
		ESP_LOGE(CERTDATA_TAG, "No cert data header");
		return false;
	}
	if (pHeader->header_version != CERT_HEADER_VERSION_CURR) {
		ESP_LOGE(CERTDATA_TAG, "Unknown cert data version");
		return false;
	}

	//check header checksum
	uint8_t CRC_result;
	CRC_result = ~ esp_crc8_be((uint8_t)~0x00, (uint8_t *)pHeader, sizeof(cert_header_t)-1);
	if(CRC_result != pHeader->header_checksum) {
		ESP_LOGE(CERTDATA_TAG, "Incorrect header checksum");
		return false;
	}
	//check payload_len
	if ((pHeader->payload_len + sizeof(cert_header_t)) > size) {
		ESP_LOGE(CERTDATA_TAG, "Incorrect payload length");
		return false;
	}

	uint32_t offset = 0;
	offset = sizeof(cert_header_t);
	//check payload checksum
	CRC_result = ~ esp_crc8_be((uint8_t)~0x00, data+offset, pHeader->payload_len);
	if (CRC_result != pHeader->payload_checksum) {
		ESP_LOGE(CERTDATA_TAG, "Incorrect payload checksum");
		return false;
	}

	memset(pInfo, 0, sizeof(cert_info_t));
	pInfo->version_major = pHeader->version_major;
	pInfo->version_minor = pHeader->version_minor;
	pInfo->version_sub = pHeader->version_sub;
	if (pHeader->hostname_len) {
		pInfo->hostname = (char *)data+offset;
		offset += pHeader->hostname_len;
	}
	if (pHeader->username_len) {
		pInfo->username = (char *)data+offset;
		offset += pHeader->username_len;
	}
	if (pHeader->password_len) {
		pInfo->password = (char *)data+offset;
		offset += pHeader->password_len;
	}
	if (pHeader->ca_cert_len) {
		pInfo->ca_cert_len = pHeader->ca_cert_len;
		pInfo->ca_cert = (char *)data+offset;
		offset += pHeader->ca_cert_len;
	}
	if (pHeader->client_cert_len) {
		pInfo->client_cert_len = pHeader->client_cert_len;
		pInfo->client_cert = (char *)data+offset;
		offset += pHeader->client_cert_len;
	}
	if (pHeader->client_key_len) {
		pInfo->client_key_len = pHeader->client_key_len;
		pInfo->client_key = (char *)data+offset;
		offset += pHeader->client_key_len;
	}
	return true;
}


cert_info_t* certData_GetInfo(void)
{
	if(certData_ctx.bValid) {
		return &certData_ctx.info;
	} else {
		return NULL;
	}
}

void certData_Init(void)
{
	esp_err_t err;
	size_t len = 0;
	if (certData_ctx.pData) {
		return;
	}
#ifdef BROAN_SECURE_RELEASE
	err = storage_ReadCertData(NULL, &len, 0);
	if (err != ESP_OK) {
		return;
	}
	certData_ctx.pData = calloc(1, len);
	if (certData_ctx.pData == NULL) {
		return;
	}

	certData_ctx.len = len;
	err = storage_ReadCertData(certData_ctx.pData, &certData_ctx.len, certData_ctx.len);
	if (err == ESP_OK) {
		if (certData_Decode(certData_ctx.pData, certData_ctx.len, &certData_ctx.info)) {
			certData_ctx.bValid = true;
			ESP_LOGI(CERTDATA_TAG, "certData version: %d.%d.%d", certData_ctx.info.version_major, certData_ctx.info.version_minor, certData_ctx.info.version_sub);
		}
	}
#else
	const esp_partition_t *partition = certData_GetPartition();
	if (!partition) {
		return;
	}
	if (!certData_VerifyPartition(0, &len)) {
		return;
	}
	certData_ctx.pData = calloc(1, len);
	if (certData_ctx.pData == NULL) {
		return;
	}

	certData_ctx.len = len;
	err = esp_partition_read(partition, 0, certData_ctx.pData, certData_ctx.len);
	if (err == ESP_OK) {
		if (certData_Decode(certData_ctx.pData, certData_ctx.len, &certData_ctx.info)) {
			certData_ctx.bValid = true;
			ESP_LOGI(CERTDATA_TAG, "certData (partition) version: %d.%d.%d", certData_ctx.info.version_major, certData_ctx.info.version_minor, certData_ctx.info.version_sub);
		}
	}
#endif
}

void certData_DeInit(void)
{
	if (certData_ctx.pData) {
		free(certData_ctx.pData);
		certData_ctx.pData = NULL;
	}
	certData_ctx.len = 0;
	certData_ctx.bValid = false;
	memset(&certData_ctx.info, 0, sizeof(certData_ctx.info));
}




#if 0
#include "esp_partition.h"
static void store_cert_partition(uint8_t *data, size_t size)
{
	esp_err_t err;
	const esp_partition_t *partition = esp_partition_find_first(64, ESP_PARTITION_SUBTYPE_ANY, "cert");
	if (partition == NULL) 
	{
		ESP_LOGE(CERTDATA_TAG, "No cert partition");
		return;
	}
	if (size > partition->size) {
		ESP_LOGE(CERTDATA_TAG, "Size too large");
		return;
	}
	ESP_LOGI(CERTDATA_TAG, "write size: %d", size);
	esp_partition_erase_range(partition, 0, partition->size);
	err = esp_partition_write(partition, 0, data, size);
	if (err == ESP_OK) {
		ESP_LOGI(CERTDATA_TAG, "Write OK");
	} else {
		ESP_LOGE(CERTDATA_TAG, "Fail to write");
	}
}

extern const char broan_iotdev_ca_cert[];
extern const char broan_iotdev_client_cert[];
extern const char broan_iotdev_client_key[];
void generate_cert_iotdev(void)
{
	cert_header_t *pHeader = NULL;
	uint8_t *buffer = NULL;
	uint32_t offset_start;
	uint32_t offset_end;
	char* hostname = "iotdev.broan-nutone.com";
	buffer = calloc(1, 5120);
	if (!buffer) {
		return;
	}
	pHeader = (cert_header_t *)buffer;
	memcpy(pHeader->marker, CERT_FILE_MARKER, CERT_FILE_MARKER_LEN);
	pHeader->header_version = CERT_HEADER_VERSION_CURR;
	pHeader->version_major = 0;
	pHeader->version_minor = 0;
	pHeader->version_sub = 1;

	pHeader->hostname_len = strlen(hostname)+1;
	pHeader->username_len = 0;
	pHeader->password_len = 0;
	pHeader->ca_cert_len = strlen(broan_iotdev_ca_cert)+1;
	pHeader->client_cert_len = strlen(broan_iotdev_client_cert)+1;
	pHeader->client_key_len = strlen(broan_iotdev_client_key)+1;
	pHeader->payload_len = pHeader->hostname_len + pHeader->username_len + pHeader->password_len\
							+ pHeader->ca_cert_len + pHeader->client_cert_len + pHeader->client_key_len;

	//copy payload data
	offset_start = sizeof(cert_header_t);
	offset_end = sizeof(cert_header_t);
	if (pHeader->hostname_len) {
		memcpy(buffer+offset_end, hostname, pHeader->hostname_len);
		offset_end += pHeader->hostname_len;
	}
	if (pHeader->username_len) {
		//memcpy(buffer+offset_end, XXX, pHeader->username_len);
		//offset_end += pHeader->username_len;
	}
	if (pHeader->password_len) {
		//memcpy(buffer+offset_end, XXX, pHeader->password_len);
		//offset_end += pHeader->password_len;
	}
	if (pHeader->ca_cert_len) {
		memcpy(buffer+offset_end, broan_iotdev_ca_cert, pHeader->ca_cert_len);
		offset_end += pHeader->ca_cert_len;
	}
	if (pHeader->client_cert_len) {
		memcpy(buffer+offset_end, broan_iotdev_client_cert, pHeader->client_cert_len);
		offset_end += pHeader->client_cert_len;
	}
	if (pHeader->client_key_len) {
		memcpy(buffer+offset_end, broan_iotdev_client_key, pHeader->client_key_len);
		offset_end += pHeader->client_key_len;
	}
	//calculate checksum (CRC-8)
	pHeader->payload_checksum = ~ esp_crc8_be((uint8_t)~0x00, buffer+offset_start, offset_end-offset_start);
	pHeader->header_checksum = ~ esp_crc8_be((uint8_t)~0x00, (uint8_t *)pHeader, sizeof(cert_header_t)-1);

	ESP_LOGI(CERTDATA_TAG, "Cert size=%d", offset_end);
	ESP_LOGI(CERTDATA_TAG, "Cert header:");
	ESP_LOG_BUFFER_HEXDUMP(CERTDATA_TAG, buffer, sizeof(cert_header_t), ESP_LOG_INFO);

#if 1
	cert_info_t certInfo = {0};
	bool bSuccess = certData_Decode(buffer, offset_end, &certInfo);
	if (bSuccess) {
		certData_DebugPrint(&certInfo, true);
	}
#endif

	store_cert_partition(buffer, offset_end);

	free(buffer);
}

extern const char broan_iotqa_ca_cert[];
extern const char broan_iotqa_client_cert[];
extern const char broan_iotqa_client_key[];
void generate_cert_iotqa(void)
{
	cert_header_t *pHeader = NULL;
	uint8_t *buffer = NULL;
	uint32_t offset_start;
	uint32_t offset_end;
	char* hostname = "iotqa.broan-nutone.com";
	buffer = calloc(1, 5120);
	if (!buffer) {
		return;
	}
	pHeader = (cert_header_t *)buffer;
	memcpy(pHeader->marker, CERT_FILE_MARKER, CERT_FILE_MARKER_LEN);
	pHeader->header_version = CERT_HEADER_VERSION_CURR;
	pHeader->version_major = 0;
	pHeader->version_minor = 0;
	pHeader->version_sub = 1;

	pHeader->hostname_len = strlen(hostname)+1;
	pHeader->username_len = 0;
	pHeader->password_len = 0;
	pHeader->ca_cert_len = strlen(broan_iotqa_ca_cert)+1;
	pHeader->client_cert_len = strlen(broan_iotqa_client_cert)+1;
	pHeader->client_key_len = strlen(broan_iotqa_client_key)+1;
	pHeader->payload_len = pHeader->hostname_len + pHeader->username_len + pHeader->password_len\
							+ pHeader->ca_cert_len + pHeader->client_cert_len + pHeader->client_key_len;

	//copy payload data
	offset_start = sizeof(cert_header_t);
	offset_end = sizeof(cert_header_t);
	if (pHeader->hostname_len) {
		memcpy(buffer+offset_end, hostname, pHeader->hostname_len);
		offset_end += pHeader->hostname_len;
	}
	if (pHeader->username_len) {
		//memcpy(buffer+offset_end, XXX, pHeader->username_len);
		//offset_end += pHeader->username_len;
	}
	if (pHeader->password_len) {
		//memcpy(buffer+offset_end, XXX, pHeader->password_len);
		//offset_end += pHeader->password_len;
	}
	if (pHeader->ca_cert_len) {
		memcpy(buffer+offset_end, broan_iotqa_ca_cert, pHeader->ca_cert_len);
		offset_end += pHeader->ca_cert_len;
	}
	if (pHeader->client_cert_len) {
		memcpy(buffer+offset_end, broan_iotqa_client_cert, pHeader->client_cert_len);
		offset_end += pHeader->client_cert_len;
	}
	if (pHeader->client_key_len) {
		memcpy(buffer+offset_end, broan_iotqa_client_key, pHeader->client_key_len);
		offset_end += pHeader->client_key_len;
	}
	//calculate checksum (CRC-8)
	pHeader->payload_checksum = ~ esp_crc8_be((uint8_t)~0x00, buffer+offset_start, offset_end-offset_start);
	pHeader->header_checksum = ~ esp_crc8_be((uint8_t)~0x00, (uint8_t *)pHeader, sizeof(cert_header_t)-1);

	ESP_LOGI(CERTDATA_TAG, "Cert size=%d", offset_end);
	ESP_LOGI(CERTDATA_TAG, "Cert header:");
	ESP_LOG_BUFFER_HEXDUMP(CERTDATA_TAG, buffer, sizeof(cert_header_t), ESP_LOG_INFO);

#if 1
	cert_info_t certInfo = {0};
	bool bSuccess = certData_Decode(buffer, offset_end, &certInfo);
	if (bSuccess) {
		certData_DebugPrint(&certInfo, true);
	}
#endif

	store_cert_partition(buffer, offset_end);

	free(buffer);
}

void generate_cert_cmd(void)
{
	//generate_cert_iotqa();
	generate_cert_iotdev();
}
#endif

