#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "uart_os.h"

#define ACK                             0x79
#define NACK                            0x1F
#define BUSY                            0x76
#define Get                             0x00    //Gets the version and the allowed commands supported by the current version of the bootloader
#define GetVersion                      0x01    //Gets the bootloader version
#define GetID                           0x02    //Gets the chip ID
#define ReadMemory                      0x11    //Reads up to 256 bytes of memory, starting from an address specified by the application
#define Go                              0x21    //Jumps to user application code located in the internal Flash memory
#define WriteMemory                     0x31    //Writes up to 256 bytes to the memory, starting from an address specified by the application
#define No_StretchWriteMemory           0x32    //Writes up to 256 bytes to the memory, starting from an address specified by the application and returns busy state while operation is ongoing
#define Erase                           0x44    //Erases from one to all Flash memory pages or sectors using two-byte addressing mode
#define No_StretchErase                 0x45    //Erases from one to all Flash memory pages or sectors using two-byte addressing mode and returns busy state while operation is ongoing
#define WriteProtect                    0x63    //Enables write protection for some sectors
#define No_StretchWriteProtect          0x64    //Enables write protection for some sectors and returns busy state while operation is ongoing
#define WriteUnprotect                  0x73    //Disables write protection for all Flash memory sectors
#define No_StretchWriteUnprotect        0x74    //Disables write protection for all Flash memory sectors and returns busy state while operation is ongoing
#define ReadoutProtect                  0x82    //Enables read protection
#define No_StretchReadoutProtect        0x83    //Enables read protection and returns busy state while operation is ongoing
#define ReadoutUnprotect                0x92    //Disables read protection
#define No_StretchReadoutUnprotect      0x93    //Disables read protection and returns busy state while operation is ongoing

//Error Code define
#define ACK_ERR                             -1
#define NACK_ERR                            -2
#define Get_ERR                             -3    
#define GetVersion_ERR                      -4    
#define GetID_ERR                           -5    
#define ReadMemory_ERR                      -6    
#define Go_ERR                              -7    
#define WriteMemory_ERR                     -8    
#define No_StretchWriteMemory_ERR           -9    
#define Erase_ERR                           -10    
#define No_StretchErase_ERR                 -11   
#define WriteProtect_ERR                    -12    
#define No_StretchWriteProtect_ERR          -13    
#define WriteUnprotect_ERR                  -14    
#define No_StretchWriteUnprotect_ERR        -15    
#define ReadoutProtect_ERR                  -16    
#define No_StretchReadoutProtect_ERR        -17    
#define ReadoutUnprotect_ERR                -18    
#define No_StretchReadoutUnprotect_ERR      -19

#define OTA_TAG "OTA_TAG"

#define STM_BOOT0_PIN 4
#define STM_RST_PIN 5

//CT bootloader option bytes
#define CT_BOOTLOADER_SPEC_CHAR_1		0xAB
#define CT_BOOTLOADER_SPEC_CHAR_2		0xDD

//CT application address
#define STM_FW_GO_ADDR						0x08008000

#define STM_FW_SIZE_ADDR					0x0801FFB0 //use 4 bytes
#define STM_FW_SIGN_DATA_ADDR				0x0801FFC0


bool BootLoader_GoCmd(void);

// void EnterBootLoaderMode(void);
// void LeaveBootLoaderMode(void);
int BootLoader_Test_Flow(void);
void StartSTM_SetFWParameter(uint32_t offset, uint32_t size); //temp. function
int StartSTM_FWup(void);

void STM_Boot2OrgBootLoader(void);

