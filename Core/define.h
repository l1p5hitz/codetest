#ifndef __DEFINE_H
#define __DEFINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>	//add for printf()

//#define BROAN_BRNC011
//#define BROAN_BRNC021

#define BRNC_ES2
  
#define BRNC_ES4
  
#define FLASH_RDP_ENABLE
  
#ifdef BROAN_BRNC011
  #ifndef BRNC_ES4
      #define BRNC_ES3
  #endif
#endif

  #define BRNC_ES5_HWVER 2    // WC P2.1, 10 LEDs, 

  #define BRNC_ES4_HWVER 1    //P2.0 Hardware
  
  #define BRNC_ES3_HWVER 0   //P1.5, not suppport

  
//#define IS_TH_TEST_FW
  
  
#define STM_VER_MAIN 1
#define STM_VER_MINOR 0
#ifdef IS_TH_TEST_FW
  #define STM_VER_SUB 113       //235,236,237 -- just for testing with state
#else
  #define STM_VER_SUB 46       //1 to 10 for offical release, 238 for removed Humidity Delta, just Tony method
#endif

#define FW_DATE "b 09/07/2021"
#ifdef BROAN_BRNC011
  #define FW_VER "BROAN_BRNC011"
  #define STM_MODEL_NO 0x11
#endif
  
#ifdef BROAN_BRNC021
  #define FW_VER "BROAN_BRNC021"
  #define STM_MODEL_NO 0x21
#endif

#define Yello_IAQ_Red_value 2520 //3350// - ((STM_VER_SUB - 200) * 100)

//System Store Address
//#define FLASH_USER_END_ADDR     (FLASH_BASE + FLASH_SIZE - 1)   /* End @ of user Flash area */
//#define FLASH_USER_START_ADDR   (FLASH_USER_END_ADDR - 4048)   /* Start @ of user Flash area -- 2 k bytes*/
#define FLASH_USER_START_ADDR   0x801F800 //(FLASH_BASE + (3 * FLASH_PAGE_SIZE))   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (FLASH_BASE + FLASH_SIZE - 1) //(FLASH_USER_START_ADDR + 2048)   /* End @ of user Flash area */
  
#define STM_SIGN_DATA_ADDR      0x0801FFC0
#define STM_FLASH_USER_DATA_ADDR      0x0801F000
  
//#define APP_VECT_TB_OFFSET      0x0000U


/* Private defines -----------------------------------------------------------*/
#define DEBUG_RX_Pin GPIO_PIN_9
#define DEBUG_RX_GPIO_Port GPIOB
//#define TP_1_Pin GPIO_PIN_14
//#define TP_1_GPIO_Port GPIOC
#define TP_2_Pin GPIO_PIN_15
#define TP_2_GPIO_Port GPIOC
#define BUTTON_1_Pin GPIO_PIN_5
#define BUTTON_1_GPIO_Port GPIOB
#ifdef BRNC_ES4
  #define BUTTON_2_Pin GPIO_PIN_6
  #define BUTTON_2_GPIO_Port GPIOC
#else
  #define BUTTON_2_Pin GPIO_PIN_2
  #define BUTTON_2_GPIO_Port GPIOB
#endif

#define L_RELAY_OFF_2_Pin GPIO_PIN_2
#define L_RELAY_OFF_2_GPIO_Port GPIOA
#define L_RELAY_ON_2_Pin GPIO_PIN_3
#define L_RELAY_ON_2_GPIO_Port GPIOA
#define L_RELAY_ON_1_Pin GPIO_PIN_4
#define L_RELAY_ON_1_GPIO_Port GPIOA
#define L_RELAY_OFF_1_Pin GPIO_PIN_14
#define L_RELAY_OFF_1_GPIO_Port GPIOC

//#define TRIAC_1_Pin GPIO_PIN_2
//#define TRIAC_1_GPIO_Port GPIOA
//#define TRIAC_2_Pin GPIO_PIN_3
//#define TRIAC_2_GPIO_Port GPIOA
//#define RELAY_Pin GPIO_PIN_4
//#define RELAY_GPIO_Port GPIOA
//#ifdef UICONT_BRNC011_BUTTON
  #define BL_LED_1_Pin GPIO_PIN_5
  #define BL_LED_1_GPIO_Port GPIOA
  #define BL_LED_2_Pin GPIO_PIN_6
  #define BL_LED_2_GPIO_Port GPIOA
//#elif defined(UICONT_BRNC021_BUTTON)
//  #define BL_LED_2_Pin GPIO_PIN_5
//  #define BL_LED_2_GPIO_Port GPIOA
//  #define BL_LED_1_Pin GPIO_PIN_6
//  #define BL_LED_1_GPIO_Port GPIOA
//#endif
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOB
//#define HW_ID_Pin GPIO_PIN_2
//#define HW_ID_GPIO_Port GPIOB
#define ZMOD_RES_Pin GPIO_PIN_8
#define ZMOD_RES_GPIO_Port GPIOA
//#ifndef BRNC_ES4
//  #define ZMOD_INT_Pin GPIO_PIN_6
//  #define ZMOD_INT_GPIO_Port GPIOC
////  #define ZMOD_INT_EXTI_IRQn EXTI4_15_IRQn
//#endif

#define I2C2_SCL_Pin GPIO_PIN_11
#define I2C2_SCL_GPIO_Port GPIOA
#define I2C2_SDA_Pin GPIO_PIN_12
#define I2C2_SDA_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SCD_RDY_Pin GPIO_PIN_15
#define SCD_RDY_GPIO_Port GPIOA
#define SCD_RDY_EXTI_IRQn EXTI4_15_IRQn
#define SPS_PWR_Pin GPIO_PIN_3
#define SPS_PWR_GPIO_Port GPIOB
#define ZERO_DET_Pin GPIO_PIN_4
#define ZERO_DET_GPIO_Port GPIOB
#define ZERO_DET_EXTI_IRQn EXTI4_15_IRQn
//#define WiFi_EN_Pin GPIO_PIN_5
//#define WiFi_EN_GPIO_Port GPIOB

#ifdef BRNC_ES2
  #define STM_UART_TX_Pin GPIO_PIN_9
  #define STM_UART_TX_GPIO_Port GPIOA
  #define STM_UART_RX_Pin GPIO_PIN_10
  #define STM_UART_RX_GPIO_Port GPIOA
  #define I2C1_SDA_Pin GPIO_PIN_7
  #define I2C1_SDA_GPIO_Port GPIOB
  #define I2C1_SCL_Pin GPIO_PIN_6
  #define I2C1_SCL_GPIO_Port GPIOB
#else           //ES1
  #define STM_UART_TX_Pin GPIO_PIN_6
  #define STM_UART_TX_GPIO_Port GPIOB
  #define STM_UART_RX_Pin GPIO_PIN_7
  #define STM_UART_RX_GPIO_Port GPIOB
  #define I2C1_SDA_Pin GPIO_PIN_10
  #define I2C1_SDA_GPIO_Port GPIOA
  #define I2C1_SCL_Pin GPIO_PIN_9
  #define I2C1_SCL_GPIO_Port GPIOA
#endif


#define DEBUG_TX_Pin GPIO_PIN_8
#define DEBUG_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* Compute the prescaler value to have TIM1 counter clock equal to 56000000 Hz */
#define PRESCALER_VALUE     (uint32_t)((SystemCoreClock / 56000000) - 1)


typedef enum
{
  STORE_START_BYTE = 0,
  STORE_LED_1_STATUS,
  STORE_LED_2_STATUS,
  STORE_IAQ_COLOR,
  STORE_IAQ_INDEX,
  STORE_LED_ANIMATION,
  STORE_TRIAC_1_STATUS,
  STORE_TRIAC_2_STATUS,
  STORE_Relay_STATUS,
  STORE_SW_CONFIG,
  STORE_BTN_1_STATUS,
  STORE_BTN_2_STATUS,
  STORE_MOVERRIDE,
  STORE_END_BYTE
}UserDataInFlash_t;
  


#ifdef BROAN_BRNC011
  #define SENSOR_SCD30
  #define SENSOR_SPS30
  #define SENSOR_SI7021
  #define SENSOR_ZMOD4410
  #define UICONT_BRNC011_BUTTON
  #define BRNC011_RELAY_CONTROL
  #define SENSOR_NTC1
  #define SENSOR_NTC2
  #define LED_RELAY_OPER
#elif defined(BROAN_BRNC021)
  #define SENSOR_SI7021
  #define SENSOR_ZMOD4410
  #define UICONT_BRNC021_BUTTON

  #define SENSOR_NTC1
  #define SENSOR_NTC2
  #define LED_RELAY_OPER
//outdate function//
  //#define BRNC021_TRIAC_CONTROL
  //#define TRIAC_TIMEING_TURNING
//outdate function//
#endif


//for Key buttone event
#define BTN_ID_1		0x0
#define BTN_ID_2		0x1
#define BTN_ID_BOTH		0x2	//both 1 and 2
#define BTN_EVENT_SHORT_RELEASED	0
#define BTN_EVENT_LONG_PRESSED		1
#define BTN_EVENT_LONG_RELEASED		2
  
#define RX_BUF_REDUCE  15

#define SensorTaskDelay 1000
#define UartTaskDelay   100
#define GPIOTaskDelay   100
#define KeyTaskDelay    10

#define LongPressDelayCnt   500    //   if task delay = 10, long press cnt is 500
#define ShortPressDelayCnt   1        //

#define SensorI2C_Timeout 100   //ms
#define I2C_retryCnt 3
#define I2C_MakeRstCnt 3
#define I2C_RECOVER_NUM_CLOCKS 10

typedef enum
{
  FLASH_SET_IAQ_INTENSITY_FLAG_POS = 0,
  FLASH_SET_SW_CONFIG_FLAG_POS,
  
  
}Flash_StoreType_flag_t;
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
