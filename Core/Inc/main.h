/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "CommonUse.h"
  
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"
#include "scd30.h"
#include "sps30.h"
#include "Si7021_driver.h"
#include "hicom_i2c.h"

#include "ESP32_STM_Comm.h"
#include "Si7021_cal.h"

#include "zmod4410.h"
  

  

  
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//nyc{
#include <stdio.h>	//add for printf()
#include <string.h>
#include "stdbool.h"
//#include "stm32g081b_eval.h"
//}nyc
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void Clear_IWDG_Counter(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */


/* -----------------------------------------------------------------------
TIM1 Configuration: generate 4 PWM signals with 4 different duty cycles.

    In this example TIM1 input clock (TIM1CLK) is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
      TIM1CLK = PCLK1
      PCLK1 = HCLK
      => TIM1CLK = HCLK = SystemCoreClock

    To get TIM1 counter clock at SystemCoreClock, the prescaler is set to 0

    To get TIM1 output clock at 24 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM1 counter clock / TIM1 output clock) - 1
           = 2295

    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
    TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
    TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32g0xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

/* Initialize TIMx peripheral as follows:
   + Prescaler = (SystemCoreClock / 56000000) - 1
   + Period = (2296 - 1)
   + ClockDivision = 0
   + Counter direction = Up
*/
#define  PERIOD_VALUE       (uint32_t)(4096 - 1)  /* Period Value  */
#define PWM_R 0 //(PERIOD_VALUE/PERIOD_VALUE)
#define PWM_G 0 //(PERIOD_VALUE/PERIOD_VALUE)
#define PWM_B 0 //(PERIOD_VALUE/PERIOD_VALUE)
/* USER CODE END Private defines */

//#define ESP_STM_UART_BUF_SIZE 26

//typedef enum
//{
  
//#ifdef SENSOR_SCD30
//  STATE_GETSCD30,
//#endif
//#ifdef SENSOR_SPS30
//  STATE_GETSPS30,
//#endif
//#ifdef SENSOR_ZMOD4410
//  STATE_GETTVOC,
//#endif
//#ifdef SENSOR_SI7021
//  STATE_GETSI7021,
//#endif
//  NUM_STATE
//}StateType;

typedef enum
{
  IAQ_GREEN = 0,
  IAQ_YELLOW,
  IAQ_RED,
  IAQ_BLUE,
  IAQ_ORANGE,
  IAQ_PURPLE,
  IAQ_NONE = 255,
}IAQ_Color_define_;

#define BL_LED1 1
#define BL_LED2 2
#define IAQ_LEDS 3

typedef enum
{
  IAQ_C_HUMIDITY = 0,
  IAQ_C_PM2P5,
  IAQ_C_TVOC,
  IAQ_C_CO2,
  IAQ_C_IAQ_R,
}Offline_IAQ_Components_;


typedef enum
{
  LED_OFF = 0,
  LED_EFFECT_BLINK,
  LED_EFFECT_FADE,
  LED_ON_SOLID,
  IAQ_BLUE_BLINK_3X,
  IAQ_BLINK,
  IAQ_FADE,
}LED_Effect_;


//Manual Override Feature Code//

#define DEVICECONFIG_TABLE_CNT		7

typedef enum 
{
	DEVICECONFIG_OUTPUT_IGNORE = 0,
	DEVICECONFIG_OUTPUT_OFF,
	DEVICECONFIG_OUTPUT_ON,
	DEVICECONFIG_OUTPUT_DISABLE,
	NUM_OF_DEVICECONFIG_OUTPUT
}DEVICECONFIG_OUTPUT_ENUM_T;

typedef struct {
	uint8_t Output1;
	uint8_t Output2;
} DeviceConfigOutput_t;

typedef struct {
	const DeviceConfigOutput_t *pOutput;
	size_t size;
	bool bFanAlgoCmd;
} DeviceConfigOutputTable_t;

static const DeviceConfigOutput_t DeviceConfigOutput_1[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_DISABLE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_DISABLE},
};

static const DeviceConfigOutput_t DeviceConfigOutput_2[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_IGNORE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_IGNORE},
};

static const DeviceConfigOutput_t DeviceConfigOutput_3[3] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_OFF},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_OFF},
        {DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_ON},
};

static const DeviceConfigOutput_t DeviceConfigOutput_4[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_DISABLE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_DISABLE},
};

static const DeviceConfigOutput_t DeviceConfigOutput_5[2] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_IGNORE},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_IGNORE},
};

static const DeviceConfigOutput_t DeviceConfigOutput_6[3] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_OFF},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_OFF},
        {DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_ON},
};

static const DeviceConfigOutput_t DeviceConfigOutput_7[4] =
{
	{DEVICECONFIG_OUTPUT_OFF,	DEVICECONFIG_OUTPUT_OFF},
	{DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_ON},
        {DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_ON},
        {DEVICECONFIG_OUTPUT_ON,	DEVICECONFIG_OUTPUT_ON},
};

static DeviceConfigOutputTable_t DeviceConfigTable[DEVICECONFIG_TABLE_CNT+1] =
{
	{NULL, 0, false},
	{DeviceConfigOutput_1, sizeof(DeviceConfigOutput_1)/sizeof(DeviceConfigOutput_1[0]), true},
	{DeviceConfigOutput_2, sizeof(DeviceConfigOutput_2)/sizeof(DeviceConfigOutput_2[0]), true},
	{DeviceConfigOutput_3, sizeof(DeviceConfigOutput_3)/sizeof(DeviceConfigOutput_3[0]), true},
	{DeviceConfigOutput_4, sizeof(DeviceConfigOutput_4)/sizeof(DeviceConfigOutput_4[0]), true},
	{DeviceConfigOutput_5, sizeof(DeviceConfigOutput_5)/sizeof(DeviceConfigOutput_5[0]), true},
	{DeviceConfigOutput_6, sizeof(DeviceConfigOutput_6)/sizeof(DeviceConfigOutput_6[0]), true},
        {DeviceConfigOutput_7, sizeof(DeviceConfigOutput_7)/sizeof(DeviceConfigOutput_7[0]), true},
};

bool device_isValidDeviceConfig(uint8_t _DeviceConfig);
uint8_t device_calcFanRealState(uint8_t _DeviceConfig, uint8_t _Bttn1RealState, uint8_t _Bttn2RealState);
uint8_t device_calcMOverride(uint8_t _DeviceConfig, uint8_t _FanAlgoState, uint8_t _FanRealState);
bool device_isAlgoActive(void);
uint8_t device_getOutput1Config(void);
uint8_t device_getOutput2Config(void);
void device_updateResult(void);

//Manual Override Feature Code//

//typedef struct
//{
//  StateType State;
//  void (*func)(void);
//  
//}StateMachineType;

#ifdef SENSOR_SCD30
  void GetSCD30_data(void);
#endif
#ifdef SENSOR_SPS30
  void GetSPS30_data(void);
#endif
#ifdef SENSOR_ZMOD4410
  void GetTVOC_data(void);
#endif
#ifdef SENSOR_SI7021
  void GPIO_GetSi7021(void);
#endif

  void STM_ESP_Comm(void);



//static StateMachineType StateMachine[] = 
//{
//  
//#ifdef SENSOR_SCD30
//  { STATE_GETSCD30, GetSCD30_data },
//#endif
//#ifdef SENSOR_SPS30
//  { STATE_GETSPS30, GetSPS30_data },
//#endif
//#ifdef SENSOR_ZMOD4410
//  { STATE_GETTVOC, GetTVOC_data },
//#endif
//#ifdef SENSOR_SI7021
//  { STATE_GETSI7021, GPIO_GetSi7021 },
//#endif
//
//
//
//
//  
//};

typedef struct
{
  uint8_t SCD30;        //CO2, Temp, Humidity Sensor
  uint8_t SPS30;        //PM2.5 Sensor
  uint8_t ZMOD4410;     //TVOC sensor
  uint8_t Si7021;       //Temp, Humidity Sensor
  uint8_t NTC1;
  uint8_t NTC3;
}ModuleInitState_;

//static StateType CurrStatus = STATE_GETSCD30;

static __IO ITStatus Uart1Ready = RESET;

static ModuleInitState_ ModuleInitState;


void USART1_IRQHandler(void);
static uint8_t aRxBuffer[ESP_STM_UART_BUF_SIZE];
static uint8_t * msg = aRxBuffer;
static uint8_t UART_RX_Cnt = 0;
static uint8_t UART_RX_Len;



typedef struct
{
  uint32_t co2;
  uint32_t temperature;
  uint32_t humidity;
}SCD30outRawData_;

typedef struct
{
  uint32_t co2;
  uint32_t IAQ;
  uint32_t TVOC;
  uint32_t eCO2;
  uint32_t r_mox;
}ZMOD4410outRawData_;

typedef struct
{
  uint32_t temperature;
  uint32_t humidity;
  uint32_t RawTemp;
  uint32_t RawHumidity;
  uint32_t RawNTC1;
  uint32_t RawNTC2;
  uint32_t RawNTC3;
}Si7021outRawData_;

typedef struct
{
  uint32_t temperature;
  uint32_t humidity;
}NTC_Out_RawData_;

typedef struct
{
  SCD30outRawData_ SCD30outRawData;
  uint32_t PM2p5_32bit;
  ZMOD4410outRawData_ ZMOD4410outRawData;
  Si7021outRawData_ Si7021outRawData;
}SensorsOutRawData_;

typedef struct
{
  bool LED1_IsChange;
  bool LED1;
  bool LED2_IsChange;
  bool LED2;
  bool IAQcolor_IsChange;
  uint8_t IAQvalue;
  uint8_t B4IAQvalue;
  bool IAQindex_IsChange;
  uint8_t IAQindex;
  bool LED_Animation_IsChange;
  uint8_t LED_Animation;
  bool No_Wifi_BT_IsChange;
  bool BT_Pairing_IsChange;
  uint8_t BT_Pairing_Value;
  bool BT_Pair_OK_IsChange;
  bool Wifi_Connect_OK_IsChange;
  bool Wifi_Connected_Not_Active_IsChange;
  bool Wifi_Connected_Active_IsChange;
  bool Wifi_Disconnected_IsChange;
  bool Manual_on_IsChange;
  bool Manual_on_OnOff;
  bool Btn1_status_IsChange;
  GPIO_PinState Btn1_OnOff;
  GPIO_PinState Btn1_OnOff_B4;
  bool Btn2_status_IsChange;
  GPIO_PinState Btn2_OnOff;
  GPIO_PinState Btn2_OnOff_B4;
  bool Fan_App_Status_IsChange;
  uint8_t Fan_App_Status_Value;
  uint8_t Fan_App_Status_Value_B4;
  bool Triac1_IsChange;
  uint8_t Triac1_value;
  bool Triac2_IsChange;
  uint8_t Triac2_value;
  bool Relay_IsChange;
  bool Relay_OnOff;
  bool SysReset_IsChange;
  bool IAQ_Intensity_IsChange;
  uint8_t IAQ_Intensity_value;
  bool Fan_AlgoApp_State_IsChange;
  uint8_t Fan_AlgoApp_State_value;
  uint8_t B4_Fan_AlgoApp_State_value;
  bool FanRealState_IsChange;
  uint8_t FanRealState_Value;
  bool SwtichConfig_IsChange;
  bool BLE_Enable_status_;
  uint8_t Get_BLE_Enable_status_Value;
  uint8_t Get_BLE_Enable_status_Value_B4;
  bool Btn1_RealState_isChange;
  bool Btn2_RealState_isChange;
  bool Send_Sign_to_ESP;

}LED_Control_data_;

static uint8_t EngMP_mode = 0;

static uint8_t UART_Transfering = 0;

static uint32_t Dual_Press_Fall_T = 0;


void Store_UserData(uint32_t Addr, uint8_t Data);
uint8_t Get_UserData(uint32_t Addr);
static uint32_t GetPage(uint32_t Addr);
void GetSTM_SignData(uint8_t *Data);
void GetSTM_FlashUserData(uint8_t *Data);
void SetSTM_FlashUserData(uint8_t FlashStoreType, uint8_t *Data);
uint8_t Get_Flash_IAQintensityData(void);
uint8_t Get_Flash_SWconfigData(void);
uint32_t Erase_FlashUserData(uint32_t StartSector);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
