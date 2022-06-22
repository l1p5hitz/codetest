#ifndef __OTA_INTERNAL_H__
#define __OTA_INTERNAL_H__

/**
 * Silicon Labs EmberZNet OTA structure
 */
#define EMBER_AF_OTA_FILE_MAGIC_NUMBER        0x0BEEF11EL

#define EMBER_AF_OTA_HEADER_VERSION 0x0100

#define EMBER_AF_OTA_TAG_UPGRADE_IMAGE               0x0000
#define EMBER_AF_OTA_TAG_ECDSA_SIGNATURE             0x0001
#define EMBER_AF_OTA_TAG_ECDSA_SIGNING_CERTIFICATE   0x0002
#define EMBER_AF_OTA_TAG_RESERVED_START              0x0003
#define EMBER_AF_OTA_TAG_RESERVED_END                0xEFFF
#define EMBER_AF_OTA_TAG_MANUFACTURER_SPECIFIC_START 0xFFFF

#define SECURITY_CREDENTIAL_VERSION_FIELD_PRESENT_MASK 0x0001
#define DEVICE_SPECIFIC_FILE_PRESENT_MASK              0x0002
#define HARDWARE_VERSIONS_PRESENT_MASK                 0x0004

#define headerHasSecurityCredentials(header) \
  ((header)->fieldControl & SECURITY_CREDENTIAL_VERSION_FIELD_PRESENT_MASK)
#define headerHasUpgradeFileDest(header)  \
  ((header)->fieldControl & DEVICE_SPECIFIC_FILE_PRESENT_MASK)
#define headerHasHardwareVersions(header) \
  ((header)->fieldControl & HARDWARE_VERSIONS_PRESENT_MASK)

#define EMBER_AF_OTA_MAX_HEADER_STRING_LENGTH 32
#pragma pack(1)
typedef struct {
  uint32_t MagicNumber;
  uint16_t headerVersion;
  uint16_t headerLength;
  uint16_t fieldControl;
  uint16_t manufacturerId;
  uint16_t imageTypeId;           // a.k.a. Device ID
  uint32_t firmwareVersion;
  uint16_t zigbeeStackVersion;
  char headerString[EMBER_AF_OTA_MAX_HEADER_STRING_LENGTH];  
  uint32_t imageSize;
  //optional field is not included
} EmberAfOtaHeader;
_Static_assert(sizeof(EmberAfOtaHeader) == 56, "EmberAfOtaHeader size mismatch");

typedef struct {
  uint16_t id;
  uint32_t length;
} EmberAfTagData;
#pragma pack()/* reverting back to non-packing of structures*/
_Static_assert(sizeof(EmberAfTagData) == 6, "EmberAfTagData size mismatch");

//CRC32 magic number & verify mechanism are defined by computime ourselves
#define OTA_CRC32_MAGIC_NUMBER	0xA55A5AA5

#endif /*  __AYLA_OTA_INTERNAL_H__ */

