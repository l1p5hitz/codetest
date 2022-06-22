#ifndef __ESP32_STM_COMM_H
#define __ESP32_STM_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>	//add for printf()
#include "CommonUse.h"
#include "stdint.h"
#include "stdbool.h"
//#include "stm32g0xx_hal.h"
//#include "stm32g0xx.h"
//#include "sensirion_common.h"
  
  
#define ESP_STM_UART_BUF_SIZE 256

#define START_BYTE 0x7C
  
#define SCD30_RES_DATA_LEN 1// 1+1+1+1+3*4  //Start Byte+Data Len+Cmd Type+Cmd ID + 3 set 4byte data + crc
#define UART_DATA_LEN_POS 1
  



  
typedef enum
{
  SENSOR_REQ = 1,
  LED_CONTROL,
  POWER_CONTROL,
  SPEC_TYPE_REQ,
  BUTTON_STATUS_CONTROL,
  
}ESP32toSTM_CmdType;

typedef enum
{
  SCD30_REQ = 1,    //CO2, Temp, Humidity
  SPS30_REQ,        //PM2.5
  SI7021_REQ,       //Temp, Humidity
  ZMOD4410_REQ,     //TVOC
  
  LED_1 = 1,
  LED_2,                        //
  IAQ_COLOR,                    //
  IAQ_INDEX,                            
  LED_ANIMATION,
  BT_PAIRING,                   //6
  BT_PAIR_OK,                   //7
  WIFI_CONNECT_OK,              //8
  WIFI_CONNECTED_NOT_ACTIVE,    //9
  WIFI_CONNECTED_ACTIVE,        //10
  WIFI_DISCONNECTED,            //11
  MANUAL_ON,                    //12
  NO_WIFI_BT,                   //13  
  IAQ_LED_INTENSITY,             //14
  
  FAN_CLOUD_STATE = 1,
  TRIAC_1,
  TRIAC_2,
  
  
  VER_REQ_ESP_TO_STM = 1,
  VER_RES_ESP_TO_STM,
  MODEL_REQ_ESP_TO_STM,
  MODEL_RES_ESP_TO_STM,
  SWITCH_CONFIG,
  ENG_CO2_FRC_TURN,
  BLE_ENABLE_STATUS,
  TVOC_CLN_REQ,
  IAQ_PWM_TUNE,
  YELLO_IAQ_TUNE_RED,
  YELLO_IAQ_TUNE_GREEN,
  SIGN_REQ_ESP_TO_STM,
  HW_VER_FROM_ESP,
  CO2_ONOFF,
  ONOFF_BL1_FUNC,       //ON/OFF CT 2nd bootloader function
  HEALTH_CHECK = 17,
  
  BTN_1_CONT_STATUS = 1,
  BTN_2_CONT_STATUS,
  FAN_APP_STATUS,
  
  
}ESP32toSTM_CmdID;

typedef enum
{
  SENSOR_RES = 1,
  KEY_EVENT,
  POWER_CONTROL_RES,
  SPEC_TYPE_RES,

  
}STMtoESP32_CmdType;

typedef enum
{
  SCD30_RES = 1,    //CO2, Temp, Humidity
  SPS30_RES,        //PM2.5
  SI7021_RES,       //Temp, Humidity
  ZMOD4410_RES,     //TVOC
  SI7021_RAW_RES,
  STM_OTHER_STATUS,
  
  KEY_DEVICE_ONOFF = 1,
  KEY_CIAQ_OVERRIDE,
  KEY_BLE_PAIR,
  BTN_1_STATUS_RES,
  BTN_2_STATUS_RES,
  KEY_SYS_RESET,
  
  FAN_REAL_STATUS_RES = 1,
  TRIAC_1_STATUS_RES,
  TRIAC_2_STATUS_RES,
  
  VER_REQ_STM_TO_ESP = 1,
  VER_RES_STM_TO_ESP,
  MODEL_REQ_STM_TO_ESP,
  MODEL_RES_STM_TO_ESP,
  M_OVERRIDE,
  ESP_WIFI_STATE,
  SIGN_REQ_STM_TO_ESP,
  STM32_JUST_START,

}STMtoESP32_SensorType_CmdID;



typedef struct
{
  uint8_t StartByte;
  uint8_t DataLen;
  uint8_t CmdType;
  uint8_t CmdID;
  uint8_t Data;
  uint8_t _CRC;
  
}UartRevStruct_t;

typedef struct
{
  uint8_t StartByte;
  uint8_t DataLen;
  uint8_t CmdType;
  uint8_t CmdID;
  uint32_t Data[4];
  uint8_t _CRC;
  
}UartSendStruct_t;

typedef struct
{
  bool SCD30_data_requested;
  bool SPS30_data_requested;
  bool Si7021_data_requested;
  bool ZMOD4410_data_requested;
  
  bool LED1_control_requested;
  bool LED1_isON;       //1 = on, 0 = off;
  bool LED2_control_requested;
  bool LED2_isON;       //1 = on, 0 = off;
  bool IAQcolor_control_requested;
  uint8_t IAQ_color_value;
  bool IAQindex_control_requested;
  uint8_t IAQ_index_value;
  bool LEDAnimation_control_requested;
  uint8_t LEDAnimation;
  bool No_WiFi_BT;
  bool BT_Pairing;
  bool BT_Pair_OK;
  bool Wifi_Connect_OK;
  bool Wifi_Connected_Not_Active;
  bool Wifi_Connected_Active;
  bool Wifi_Disconnected;
  bool Manual_on;
  bool IAQ_Intensity_requested;
  uint8_t IAQ_Intensity_value;
  
  bool Triac1_control_requested;
  uint8_t Triac1_value;
  bool Triac2_control_requested;
  uint8_t Triac2_value;
  bool FanCloudState_control_requested;
  uint8_t FanCloudState_Command;
  
  bool Version_requested;
  bool Ver_response_from_ESP;
  uint8_t ESP_MainVer;
  uint8_t ESP_MinorVer;
  uint8_t ESP_SubVer;
  bool Model_requested;
  bool Model_response_from_ESP;
  uint8_t ESP_ModelNoMsb;
  uint8_t ESP_ModelNoLsb;
  bool Switch_config_change_req;
  uint8_t Switch_config;
  bool EngCo2FrcTurn_change_req;
  uint16_t Eng_Co2FrcValue;
  bool BleEnableStatus_change_req;
  uint8_t BleEnableStatus_Value;
  bool Send_Sign_to_ESP_req;
  
  bool Btn1_status_cont_req;
  bool Btn1_isON;
  bool Btn2_status_cont_req;
  bool Btn2_isON;
  bool Fan_App_Status_Req;
  uint8_t Fan_App_Status_Value;
  
}RX_REQ_decode_t;

typedef enum
{
  STM_START_ADD_0X0 = 0,
  STM_START_ADD_0X8000,
  
}START_ADDR_define_T;
  

//static __IO ITStatus Uart1Ready = RESET;
//static uint8_t aRxBuffer[ESP_STM_UART_BUF_SIZE];
  
//void STM_ESP_Comm(void);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
