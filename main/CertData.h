#ifndef _CERTDATA_H
#define _CERTDATA_H

#include "esp_partition.h"

#define CERT_HEADER_VERSION_V1		0x1
#define CERT_HEADER_VERSION_CURR	CERT_HEADER_VERSION_V1


//cert data format
#define CERT_FILE_MARKER		"CERT_DATA:"
#define CERT_FILE_MARKER_LEN	10

#pragma pack(1)
typedef struct {
	uint8_t marker[10];
	uint8_t header_version;
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_sub;
	uint8_t reserved[2];

	uint8_t hostname_len;
	uint8_t username_len;
	uint8_t password_len;
	uint8_t reserved2;
	uint16_t ca_cert_len;
	uint16_t client_cert_len;
	uint16_t client_key_len;
	uint32_t payload_len;
	uint8_t payload_checksum; //CRC-8
	uint8_t header_checksum; //CRC-8 //must place at the end of cert_header_t
} cert_header_t_v1, cert_header_t;
#pragma pack()/* reverting back to non-packing of structures*/

typedef struct {
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_sub;
	char *hostname;
	char *username;
	char *password;
	char *ca_cert;
	uint16_t ca_cert_len;
	char *client_cert;
	uint16_t client_cert_len;
	char *client_key;
	uint16_t client_key_len;
} cert_info_t;

_Static_assert( sizeof(cert_header_t) == 32, "cert_header_t size mismatch");


bool certData_CompareHostname(const char* url, const char* hostname);
void certData_DebugPrint(cert_info_t *pInfo, bool bDumpHex);

const esp_partition_t * certData_GetPartition(void);
bool certData_VerifyPartition(size_t size, size_t *pCertDataLen);

bool certData_Decode(uint8_t *data, size_t size, cert_info_t *pInfo);
cert_info_t* certData_GetInfo(void);
void certData_Init(void);
void certData_DeInit(void);


#endif /* _CERTDATA_H */
