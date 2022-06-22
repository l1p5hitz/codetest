/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "STM32g0xx_hal_iwdg.h"
#include "stm32g0xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_SCD_EXIT
//#define USE_ZMOD_EXIT
/* USER CODE END PD */
#define IWDG_WINDOW IWDG_WINDOW_DISABLE
#define IWDG_RELOAD 0xFFF//(10000 / 32)
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
IWDG_HandleTypeDef hiwdg;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;


osThreadId sensorTaskHandle;
osThreadId UartTaskHandle;
osThreadId GPIOTaskHandle;
osThreadId NTCTaskHandle;
osThreadId KeyTaskHandle;
osThreadId EngMP_Mode_Task_Handle;
/* USER CODE BEGIN PV */

uint8_t InitSCD30(void);
uint8_t InitSPS30(void);
//uint8_t InitZMOD4410(void) ;
uint8_t InitSi7021(void);


zmod4xxx_dev_t zmod4410_dev;
uint8_t zmod4410_EN = 0;

uint8_t Button1_SingleClick = 0;
uint8_t Button2_SingleClick = 0;
uint8_t isButton1_LongPress = 0;
uint8_t isButton2_LongPress = 0;
uint8_t isDualPress = 0;
uint8_t isLongDualPress_Hold = 0;
uint8_t isButton1_LongPress_Hold = 0;

uint8_t SCD30_EN = 0;
uint8_t SCD_RDY = 0;
LED_Control_data_ LED_Control_data;
uint8_t BL_LED1_status = 0;
uint8_t BL_LED2_status = 0;
uint8_t IAQ_LED_currColor = 0;
uint8_t FAN1_status = 0;
uint8_t Relay_status = 0;
uint8_t RGB_LED_status = 0;
uint8_t Switch_config_setting = SW_CONFIG_DEFAULT;
uint8_t Switch_config_setting_B4 = SW_CONFIG_DEFAULT;
uint8_t Manual_on_status = 0;
uint8_t BLE_Enable_status = 0;

uint8_t  Triac_Status1 =0;
uint8_t  Triac_Status2 =0;
uint16_t Relay_Delay_Time=0;

uint8_t Triac_Time1=200;
uint8_t Triac_Time2=200;
uint8_t ESP_Cmd_Send_En = 0;
uint8_t ESP_Cmd_Rev_Proc = 0;
uint8_t GPIO_Cont = 0;
uint16_t SCD30_FRC_Value = 0;
//bool Si7021_TwoSecTick = false;
//bool SCD30_TwoSecTick = false;
//bool SPS30_TwoSecTick = false;
//bool ZMOD4410_TwoSecTick = false;
uint8_t ThreeSecTick = 0;
static uint8_t SW_Config7_RangeHoodSpeed = 0;
SensorsOutRawData_ SensorsOutRawData;


uint32_t but1PCnt = 0;
uint32_t but2PCnt = 0;
uint32_t DualBtnCnt = 0;
uint8_t Btn1Status_curr = 1;
uint8_t Btn1Status_B4 = 1;
uint8_t Btn2Status_curr = 1;
uint8_t Btn2Status_B4 = 1;
uint8_t Btn1_detectFall_Cnt = 0;
uint8_t Btn2_detectFall_Cnt = 0;
uint8_t DualBtn_Detected = 0;

bool WifiOKchagne_FormESP = false;
bool WifiDischagne_FormESP = false;

uint8_t IWDG_ResetClearCounter_Flag1=0;
uint8_t IWDG_ResetClearCounter_Flag2=0;

uint32_t BLE_TimeOutCnt = 0;
uint8_t HW_VER = 0;
uint8_t OnOff_BL1_func;

uint8_t UART1_reinit_Cnt;

uint8_t just_StartUP_tick = 0;
bool just_StartUP_canSend = true;

uint32_t APP_Vect_offset = 0;

GPIO_InitTypeDef GPIO_InitStruct_button2 = {0};

bool OffAllLEDs_Output1 = false;
bool OffAllLEDs_Output2 = false;
bool Receive_Cloud_IAQ_color = false;

#ifdef DEBUG_SENSOR_
bool PrintIAQrate_OneSecTick = false;
#endif

#ifdef TRIAC_TIMEING_TURNING
static uint8_t Triac_time_Value1 = 0;
static uint8_t Triac_time_Value2 = 0;
#endif

//uint8_t I2CDirectReturnErrCnt;
//unsigned long long I2CDirectReturnErrCntNo = 0;
//unsigned long long TriggerI2CErrCntNo = 0;

static uint8_t WifiBT_Connect_Status = WIFI_DISCONNECTED;
static uint8_t WifiBT_Connect_Status_B4 = WIFI_DISCONNECTED;

static uint32_t LED_Run_Freq = LED_RUN_FREQ_0p5HZ;
static uint32_t LED_Indicate_Cnt = 0;
static uint8_t BL_LED1_PWM_value = 0;
static uint8_t BL_LED2_PWM_value = 0;
static uint8_t IAQ_LED_PWM_value = 0;
static bool BL_LED1_effect_isFade = false;
static bool BL_LED2_effect_isFade = false;
static bool IAQ_LED_effect_isFade = false;
static bool BL_LED1_effect_isBlink = false;
static bool BL_LED2_effect_isBlink = false;
static bool IAQ_LED_effect_isBlink = false;
static bool BL_LED1_PwmAdd = true;
static bool BL_LED2_PwmAdd = true;


static uint8_t CO2_Mark = 0;
static uint8_t TVOC_Mark = 0;
static uint8_t PM2p5_Mark = 0;
static uint8_t RH_Mark = 0;
static uint8_t IAQ_Rate = 0;
static uint8_t B4_IAQ_Rate = 0;

static bool bAlgoActive = false;
static uint8_t output1_config = DEVICECONFIG_OUTPUT_IGNORE;
static uint8_t output2_config = DEVICECONFIG_OUTPUT_IGNORE;

static unsigned long long I2C1_rstCnt_TH = 0;
static unsigned long long I2C1_rstCnt_CO2 = 0;
static unsigned long long I2C1_rstCnt_PM25 = 0;
static unsigned long long I2C2_rstCnt_TVOC = 0;

static unsigned long long I2C1_OkCnt_TH = 0;
static unsigned long long I2C1_OkCnt_CO2 = 0;
static unsigned long long I2C1_OkCnt_PM25 = 0;
static unsigned long long I2C2_OkCnt_TVOC = 0;

void SW_Config_7_BtnLED_show(uint8_t speed);
uint8_t GetMAX(uint8_t CO2_M, uint8_t TVOC_M, uint8_t PM2p5_M, uint8_t RH_M);
void StartUartTask(void const * argument);
void StartGPIOTask(void const * argument);
void StartNTCTask(void const * argument);
void STM_Data_Init(void);
void JumpToBootloader(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);

//extern void MX_ADC1_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART3_UART_Init(void);
void StartSensorTask(void const * argument);
void StartKeyTask(void const * argument);
static void MX_IWDG_Init(void);
void Clear_IWDG_Counter(void);
void  traic_control(uint8_t Traic_Select, uint8_t Voltage_Level,uint8_t Triac_Set);
void Latch_Relay_Control(uint8_t whichRelay, uint8_t OnOff);
void Traic_Relay_Control(GPIO_PinState GPIO_PinSet);
void SW_Config_7_RangeHood_Control(uint8_t whichButton);
void IAQ_Color_Control(uint8_t ColorValue, uint8_t brightness, bool isFade);
void LEDs_Effect(uint8_t whichLED, uint8_t whatEffect);
void RangeHood_Speed_control(uint8_t whichButton);
void WC_Single_Speed_control(uint8_t speed);
void Button_event(uint8_t buttonID, uint8_t event);

void Start_EngMP_mode_Task(void const * argument);

void Reinit_I2C1(bool Trigger9Pulse);
void Reinit_I2C2(void);

uint8_t Pre_EngMPmode(void);
/* USER CODE BEGIN PFP */
//nyc{ add for printf
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
//}nyc
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  APP_Vect_offset = APP_VECT_TB_OFFSET;

//I2CDirectReturnErrCnt = 0;
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;
  /* USER CODE BEGIN Init */

    
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
HW_VER = BRNC_ES4_HWVER;
  MX_ADC1_Init();
  MX_USART1_UART_Init();
#ifdef DEBUG_ENABLE_UART3_
  MX_USART3_UART_Init();
#endif
  
  
  
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
    DEBUG_SYS_INFO("RDPLevel = %X\r\n", OptionsBytesStruct.RDPLevel);
    DEBUG_SYS_INFO("OptionsBytesStruct.USERConfig = %X\r\n", OptionsBytesStruct.USERConfig);
    


#ifdef FLASH_RDP_ENABLE

//
//
    if (OptionsBytesStruct.RDPLevel == OB_RDP_LEVEL_0){
      //MX_IWDG_Init();//Open watchdog
      
          HAL_FLASH_Unlock();
          HAL_FLASH_OB_Unlock();
          OptionsBytesStruct.OptionType= OPTIONBYTE_RDP;
          OptionsBytesStruct.RDPLevel = OB_RDP_LEVEL_1;
          if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)   
           {
             DEBUG_SYS_INFO("Fail to Read-Out Protection Settings! \r\n");
           }else{
             DEBUG_SYS_INFO("Read-Out Protection Settings Set to BB return OK! \r\n");
           }
//          HAL_FLASH_OB_Launch();          
          DEBUG_SYS_INFO("HAL_FLASH_OB_Launch OK! \r\n");
          HAL_FLASH_OB_Lock();
          DEBUG_SYS_INFO("HAL_FLASH_OB_Lock OK! \r\n");
          HAL_FLASH_Lock();
//          HAL_FLASH_OB_Launch();
          HAL_PWR_EnterSTANDBYMode();
          DEBUG_SYS_INFO("HAL_FLASH_Lock OK! \r\n");
          HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
          DEBUG_SYS_INFO("RDPLevel = %X\r\n", OptionsBytesStruct.RDPLevel);

    }
     
#endif
     

 
//111010011110110000000000000 74F6000 all clicked
//010010011110110000000000000 24F6000 
//110010011110110000000000000 64F6000
//
//bit 0 nboot0
//bit 1 nboot 1
//bit 2 nboot sel
    if (APP_Vect_offset != 0U){
      if (OptionsBytesStruct.USERConfig != 0x74F6000){
            HAL_FLASH_Unlock();
            HAL_FLASH_OB_Unlock();
            OptionsBytesStruct.USERConfig = 0x74F6000;
            if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
             {
               DEBUG_SYS_INFO("Fail to USERConfig Settings! \r\n");
             }else{
               DEBUG_SYS_INFO("Read-Out USERConfig Set to 0x74F6000 return OK! \r\n");
             }
            HAL_FLASH_OB_Lock();
            DEBUG_SYS_INFO("HAL_FLASH_OB_Lock OK! \r\n");
            HAL_FLASH_Lock();
//            HAL_FLASH_OB_Launch();
            HAL_PWR_EnterSTANDBYMode();
            DEBUG_SYS_INFO("HAL_FLASH_Lock OK! \r\n");
            HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
            DEBUG_SYS_INFO("OptionsBytesStruct.USERConfig = %X\r\n", OptionsBytesStruct.USERConfig);

            
      }
    }else{
      if (OptionsBytesStruct.USERConfig != 0x24F6000){
            HAL_FLASH_Unlock();
            HAL_FLASH_OB_Unlock();
            OptionsBytesStruct.USERConfig = 0x24F6000;
            if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
             {
               DEBUG_SYS_INFO("Fail to USERConfig Settings! \r\n");
             }else{
               DEBUG_SYS_INFO("Read-Out USERConfig Set to 0x24F6000 return OK! \r\n");
             }
            HAL_FLASH_OB_Lock();
            DEBUG_SYS_INFO("HAL_FLASH_OB_Lock OK! \r\n");
            HAL_FLASH_Lock();
//            HAL_FLASH_OB_Launch();            
            HAL_PWR_EnterSTANDBYMode();
            DEBUG_SYS_INFO("HAL_FLASH_Lock OK! \r\n");
            HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
            DEBUG_SYS_INFO("OptionsBytesStruct.USERConfig = %X\r\n", OptionsBytesStruct.USERConfig);
      }
    }
  
  
  
  
  
    NTC_getTempValue(0);
    
    if (GetNTCTemp3() > 10 && GetNTCTemp3() < 60){
      HW_VER = BRNC_ES4_HWVER;
      DEBUG_SYS_INFO("_______________________HW_VER = %d \r\n", HW_VER);
    }else{
      HW_VER = BRNC_ES3_HWVER;
      DEBUG_SYS_INFO("_______________________ERR HW_VER = %d , GetNTCTemp3() = %f\r\n", HW_VER, GetNTCTemp3());
    }
  
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_DMA_Init();
  
  MX_TIM3_Init();
//  MX_ADC1_Init();
//  MX_TIM15_Init();
  



  STM_Data_Init();
  /* USER CODE BEGIN 2 */



//  printf("JumpToBootloader, wait 1s...\r\n");
//  HAL_Delay(1000);
//  JumpToBootloader();
 
/*## Start PWM signals generation #######################################*/
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  /* Start channel 4 */
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
  
  
  

    DEBUG_SYS_INFO("System Starting...\r\n");
  DEBUG_SYS_INFO("Firmware version: %s %d.%d.%d %s\r\n", FW_VER, STM_VER_MAIN,STM_VER_MINOR,STM_VER_SUB,FW_DATE);
  

  
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

 
  if (HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_RESET){
    if (APP_Vect_offset != 0U){
      __enable_irq();
    }
      
    HAL_UART_Receive_IT(&huart1, (uint8_t *)msg, 1);
    EngMP_mode = Pre_EngMPmode();
    if (APP_Vect_offset != 0U){
      __disable_irq();
    }
    
    Btn1Status_curr = 0;
    
    Btn1Status_B4 = Btn1Status_curr;
  }
  
  
//  if (Btn1Status_curr == GPIO_PIN_RESET){
//    EngMP_mode = 1;
  if (EngMP_mode == 1){

    

    
    osThreadDef(EngMP_mode_Task, Start_EngMP_mode_Task, osPriorityNormal, 0, 512);
    EngMP_Mode_Task_Handle = osThreadCreate(osThread(EngMP_mode_Task), NULL);
    
    osThreadDef(UartTask, StartUartTask, osPriorityNormal, 0, 512);
    UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);
    
    osThreadDef(keyTask, StartKeyTask, osPriorityNormal, 0, 300);
    KeyTaskHandle = osThreadCreate(osThread(keyTask), NULL);
    
  }else{
    
    
    MX_IWDG_Init();//Open watchdog
    WifiBT_Connect_Status = WIFI_DISCONNECTED;
    WifiBT_Connect_Status_B4 = WIFI_DISCONNECTED;
    LED_Control_data.IAQvalue = IAQ_GREEN;
    LED_Control_data.B4IAQvalue = LED_Control_data.IAQvalue;
    LED_Run_Freq = LED_RUN_FREQ_2p0HZ;
    LEDs_Effect(BL_LED1, LED_OFF);
    LEDs_Effect(BL_LED2, LED_OFF);
    LEDs_Effect(IAQ_LEDS, LED_EFFECT_BLINK);
    

    
    osThreadDef(sensorTask, StartSensorTask, osPriorityLow, 0, 800);
    sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

    osThreadDef(UartTask, StartUartTask, osPriorityNormal, 0, 800);
    UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

    osThreadDef(GPIOTask, StartGPIOTask, osPriorityBelowNormal, 0, 712);
    GPIOTaskHandle = osThreadCreate(osThread(GPIOTask), NULL);
    
    osThreadDef(keyTask, StartKeyTask, osPriorityAboveNormal, 0, 300);
    KeyTaskHandle = osThreadCreate(osThread(keyTask), NULL);
    
//    osThreadDef(NTCTask, StartNTCTask, osPriorityNormal, 0, 300);
//    NTCTaskHandle = osThreadCreate(osThread(NTCTask), NULL);
    
  }
//  IAQ_Color_Control(LED_Control_data.IAQvalue, 100);
//  IAQ_LED_effect_isFade = true;
  /* Create the thread(s) */
  /* definition and creation of sensorTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = IWDG_WINDOW;
  hiwdg.Init.Reload = IWDG_RELOAD;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  DEBUG_SYS_INFO("HAL_IWDG_Init OK\r\n");
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
void Clear_IWDG_Counter(void)
{
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      /* Refresh Error */
      Error_Handler();
    }
}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10606DA4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10606DA4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PRESCALER_VALUE;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PERIOD_VALUE;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = PWM_R;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PWM_G;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PWM_B;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim3, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
#ifdef LED_RELAY_OPER
  HAL_TIM_MspPostInit(&htim3);
#endif

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN; //UART_PARITY_NONE;//UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L_RELAY_OFF_1_Pin|TP_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L_RELAY_ON_1_Pin|BL_LED_1_Pin|BL_LED_2_Pin|L_RELAY_OFF_2_Pin|L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     // before ES4: Green, ES4, BLUE
            TIM3->CCR4 = 0;     //before ES4: Blue, ES4, Green

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ZMOD_RES_GPIO_Port, ZMOD_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
#ifdef BROAN_BRNC011
  HAL_GPIO_WritePin(GPIOB, SPS_PWR_Pin, GPIO_PIN_SET);

  #ifdef IS_TH_TEST_FW
    #ifndef SENSOR_SPS30
      HAL_GPIO_WritePin(GPIOB, SPS_PWR_Pin, GPIO_PIN_RESET);
    #endif
  #endif
#endif

  /*Configure GPIO pins : L_RELAY_OFF_1_PIn TP_2_Pin */
  GPIO_InitStruct.Pin = L_RELAY_OFF_1_Pin|TP_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, L_RELAY_OFF_1_Pin|TP_2_Pin, GPIO_PIN_SET);
  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; //GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);
  
//#ifdef BRNC_ES4
//  #define BUTTON_2_Pin GPIO_PIN_6
//  #define BUTTON_2_GPIO_Port GPIOC
//#else
//  #define BUTTON_2_Pin GPIO_PIN_2
//  #define BUTTON_2_GPIO_Port GPIOB
//#endif
  
  if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
    GPIO_InitStruct_button2.Pin = GPIO_PIN_6;
    GPIO_InitStruct_button2.Mode = GPIO_MODE_IT_RISING_FALLING; //GPIO_MODE_IT_FALLING;
    GPIO_InitStruct_button2.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_button2);
  }else if (HW_VER == BRNC_ES3_HWVER){
    GPIO_InitStruct_button2.Pin = GPIO_PIN_2;
    GPIO_InitStruct_button2.Mode = GPIO_MODE_IT_RISING_FALLING; //GPIO_MODE_IT_FALLING;
    GPIO_InitStruct_button2.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_button2);
  }
  
#ifdef LED_RELAY_OPER
  /*Configure GPIO pins : L_RELAY_ON_1_Pin BL_LED_1_Pin BL_LED_2_Pin ZMOD_RES_Pin */
  GPIO_InitStruct.Pin = L_RELAY_ON_1_Pin|BL_LED_1_Pin|BL_LED_2_Pin|L_RELAY_OFF_2_Pin|L_RELAY_ON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif 
#ifdef SENSOR_ZMOD4410
  /*Configure GPIO pins : L_RELAY_ON_1_Pin BL_LED_1_Pin BL_LED_2_Pin ZMOD_RES_Pin */
  GPIO_InitStruct.Pin = ZMOD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  if (HW_VER == 0){
//    /*Configure GPIO pin : ZMOD_INT_Pin - PC6*/
//    GPIO_InitStruct.Pin = GPIO_PIN_6;  
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; //GPIO_MODE_IT_FALLING;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//  }
#endif
  /*Configure GPIO pin : SCD_RDY_Pin */
  GPIO_InitStruct.Pin = SCD_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SCD_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPS_PWR_Pin WiFi_EN_Pin */
  GPIO_InitStruct.Pin = SPS_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ZERO_DET_Pin */
  GPIO_InitStruct.Pin = ZERO_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ZERO_DET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */
//nyc{ add for printf()
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
//}nyc
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
 static int Algo_or_AppState_set = -1;
void StartSensorTask(void const * argument)
{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */

    
#ifdef SENSOR_SCD30
    ModuleInitState.SCD30 = InitSCD30();
#endif
#ifdef SENSOR_SPS30
    ModuleInitState.SPS30 = InitSPS30();
#endif
#ifdef SENSOR_ZMOD4410
    ModuleInitState.ZMOD4410 = InitZMOD4410();
#endif
#ifdef SENSOR_SI7021
    ModuleInitState.Si7021 = InitSi7021();
#endif
    

    
    while (1) {
      vTaskDelay(SensorTaskDelay);
      IWDG_ResetClearCounter_Flag1=1;//Clear_IWDG_Counter();//Clear IWDG Counter
      
#ifdef SENSOR_SCD30
//      if (SCD30_TwoSecTick){
//        printf("_______________  B4 NTC SCD30_TwoSecTick = %d \r\n", SCD30_TwoSecTick);
//        SCD30_TwoSecTick = false;
        GetSCD30_data();
//        printf("_______________  B4 NTC SCD30_TwoSecTick = %d \r\n", SCD30_TwoSecTick);
//      }
#endif
#ifdef SENSOR_SPS30
//      if (SPS30_TwoSecTick){
//        printf("_______________  B4 NTC SPS30_TwoSecTick = %d \r\n", SPS30_TwoSecTick);
//        SPS30_TwoSecTick = false;
        GetSPS30_data();
//        printf("_______________  B4 NTC SPS30_TwoSecTick = %d \r\n", SPS30_TwoSecTick);
//      }
#endif

//       if (Si7021_TwoSecTick){
         
//         printf("_______________  B4 NTC Si7021_TwoSecTick = %d \r\n", Si7021_TwoSecTick);
//         Si7021_TwoSecTick = false;
  #if defined (SENSOR_NTC1) | defined (SENSOR_NTC2)
         NTC_getTempValue(1);
  #endif
         
  #ifdef SENSOR_SI7021
         GPIO_GetSi7021();
  #endif
//         printf("_______________ After Get Si Si7021_TwoSecTick = %d \r\n", Si7021_TwoSecTick);
//       }
#ifdef SENSOR_ZMOD4410
//      if (ZMOD4410_TwoSecTick){      
//        printf("_______________  B4 NTC ZMOD4410_TwoSecTick = %d \r\n", ZMOD4410_TwoSecTick);
//        ZMOD4410_TwoSecTick = false;
        GetTVOC_data();
//        printf("_______________  B4 NTC ZMOD4410_TwoSecTick = %d \r\n", ZMOD4410_TwoSecTick);
//      }
#endif
      //DEBUG_DRIVER("Sensor Task remain Stack %d\r\n", uxTaskGetStackHighWaterMark(sensorTaskHandle));
    }


  /* USER CODE END 5 */ 
}

void StartUartTask(void const * argument){
  Uart1Ready = RESET;
//  vTaskDelay(1000);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)msg, 1);
  while (1) {
    vTaskDelay(UartTaskDelay);
    
    STM_ESP_Comm();
    IWDG_ResetClearCounter_Flag2=1;// Clear_IWDG_Counter();//Clear IWDG Counter
  }
}

void StartNTCTask(void const * argument){
  while (1) {
    vTaskDelay(10000);
    
    
  }
}


void StartGPIOTask(void const * argument){
  
  
  while (1) {
  vTaskDelay(GPIOTaskDelay);
  
//  BL_LED1_PWM_value = 10;
//  BL_LED2_PWM_value = 10;


  if(IWDG_ResetClearCounter_Flag1&IWDG_ResetClearCounter_Flag2)
  {
     Clear_IWDG_Counter();//Clear IWDG Counter
     IWDG_ResetClearCounter_Flag1=0;
     IWDG_ResetClearCounter_Flag2=0;
  }
  if (Switch_config_setting < 1 || Switch_config_setting > 7){
    Switch_config_setting = SW_CONFIG_DEFAULT;
    Switch_config_setting_B4 = SW_CONFIG_DEFAULT;
  }
//  for (uint16_t i = 0; i <=PERIOD_VALUE; i++){
//    TIM3->CCR2 = i;     //Red
//    TIM3->CCR3 = i;     //Green
//    TIM3->CCR4 = i;     //Blue
//    if (i == PERIOD_VALUE){
//      i = i;
//    }
//    vTaskDelay(1);
//  }
//  
//  for (uint16_t i = PERIOD_VALUE; i >=1; i--){
//    TIM3->CCR2 = i;     //Red
//    TIM3->CCR3 = i;     //Green
//    TIM3->CCR4 = i;     //Blue
//    vTaskDelay(1);
//  }
  //Button detection
  if (ESP_Cmd_Rev_Proc == 1){
    ESP_Cmd_Rev_Proc = 0;
    if (LED_Control_data.LED1_IsChange){
      if(LED_Control_data.LED1){
          BL_LED1_PWM_value = PWM_MAX;
          DEBUG_DRIVER("Blue LED1 on!\r\n");
          BL_LED1_status = 1;
      }else{
          BL_LED1_PWM_value = 0;
          DEBUG_DRIVER("Blue LED1 off!\r\n");
          BL_LED1_status = 0;
      }
      LED_Control_data.LED1_IsChange = false;
    }
    if (LED_Control_data.LED2_IsChange){
      if(LED_Control_data.LED2){
          BL_LED2_PWM_value = 1;
          DEBUG_DRIVER("Blue LED2 on!_\r\n");
          BL_LED2_status = 1;
      }else{
          BL_LED2_PWM_value = 0;
          DEBUG_DRIVER("Blue LED2 off_!\r\n");
          BL_LED2_status = 0;
      }
      LED_Control_data.LED2_IsChange = false;
    }
    if (LED_Control_data.IAQcolor_IsChange){
      Receive_Cloud_IAQ_color = true;
      LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
      LED_Control_data.IAQcolor_IsChange = false;
    }
    if (LED_Control_data.IAQindex_IsChange){
      
      LED_Control_data.IAQindex_IsChange = false;
    }
    if (LED_Control_data.LED_Animation_IsChange){
      
      LED_Control_data.LED_Animation_IsChange = false;
    }
    if (LED_Control_data.Btn1_status_IsChange){
      Button1_SingleClick = 1;
      LED_Control_data.Btn1_status_IsChange = false;
      DEBUG_DRIVER("________ Btn1_status_IsChange\r\n");
    }
    if (LED_Control_data.Btn2_status_IsChange){
      Button2_SingleClick = 1;
      LED_Control_data.Btn2_status_IsChange = false;
      DEBUG_DRIVER("________ Btn2_status_IsChange\r\n");
    }
    if (LED_Control_data.IAQ_Intensity_IsChange){
      if (LED_Control_data.IAQ_Intensity_value < 1)
        LED_Control_data.IAQ_Intensity_value = 1;
      else if(LED_Control_data.IAQ_Intensity_value > 10)
        LED_Control_data.IAQ_Intensity_value = 10;
      LED_Control_data.IAQ_Intensity_IsChange = false;
      DEBUG_DRIVER("get LED_Control_data.IAQ_Intensity_value = %d \r\n", LED_Control_data.IAQ_Intensity_value);
      if (Get_Flash_IAQintensityData() !=  LED_Control_data.IAQ_Intensity_value){
        SetSTM_FlashUserData(FLASH_SET_IAQ_INTENSITY_FLAG_POS, &LED_Control_data.IAQ_Intensity_value);
      }
      DEBUG_DRIVER("Get IAQ instensity = %d \r\n", LED_Control_data.IAQ_Intensity_value);
      LEDs_Effect(IAQ_LEDS, LED_EFFECT_FADE);
      
    }
  
    if (LED_Control_data.Relay_IsChange){
      if (LED_Control_data.Relay_OnOff){
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
        DEBUG_DRIVER("Relay1 on!  1\r\n");
        Relay_status = 1;
      }else{
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
        DEBUG_DRIVER("Relay1 off! 1\r\n");
        Relay_status = 0;
      }
      LED_Control_data.Relay_IsChange = false;
    }
    
//    if (LED_Control_data.BT_Pairing_IsChange){
//         WifiBT_Connect_Status = BT_PAIRING;
//         Receive_Cloud_IAQ_color = false;
//        LED_Run_Freq = LED_RUN_FREQ_2p0HZ;
//        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
//        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
//        LEDs_Effect(IAQ_LEDS, LED_OFF);
//
//      
//      LED_Control_data.BT_Pairing_IsChange = false;
//      
////      Button1_SingleClick = 1;
////      Button2_SingleClick = 1;
////      LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
////      LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
////      LED_Control_data.Fan_AlgoApp_State_value = 0;
////      LED_Control_data.FanRealState_Value = 0;
////      device_updateResult();
//       printf("______________   BT_Pairing_IsChange\r\n");
//       vTaskDelay(2000);
//    }
    
    if (LED_Control_data.BT_Pair_OK_IsChange){
//      WifiBT_Connect_Status = BT_PAIR_OK;
//      WifiBT_Connect_Status_B4 = BT_PAIR_OK;
//      LED_Run_Freq = LED_RUN_FREQ_0p5HZ;
//      LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
//      LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
//      //LED_Control_data.IAQvalue = IAQ_BLUE;
//      LEDs_Effect(IAQ_LEDS, LED_OFF);
//      LED_Control_data.BT_Pair_OK_IsChange = false;
//       printf("______________   BT_Pair_OK_IsChange\r\n");
    }
    
    if (LED_Control_data.Wifi_Connect_OK_IsChange){
      WifiOKchagne_FormESP = true;
      WifiBT_Connect_Status = WIFI_CONNECT_OK;

      //LEDs_Effect(BL_LED1, LED_OFF);
      //LEDs_Effect(BL_LED2, LED_OFF);
      //LEDs_Effect(IAQ_LEDS, IAQ_BLUE_BLINK_3X);
      
      LED_Control_data.Wifi_Connect_OK_IsChange = false;
      Receive_Cloud_IAQ_color = false;
#ifndef IS_TH_TEST_FW
      if (LED_Control_data.IAQvalue == IAQ_NONE){
        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
      }
#endif
      if (LED_Control_data.Get_BLE_Enable_status_Value == 0){
        if (WifiBT_Connect_Status_B4 == BT_PAIRING){
          LEDs_Effect(BL_LED1, LED_OFF);
          LEDs_Effect(BL_LED2, LED_OFF);
        } 
        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);        
//        LED_Control_data.BT_Pairing_Value = 0;
        
      }else if (LED_Control_data.Get_BLE_Enable_status_Value == 1){
//        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
//        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
//        LEDs_Effect(IAQ_LEDS, LED_OFF);
        
      }
      WifiBT_Connect_Status_B4 = WIFI_CONNECT_OK;
//      printf("______________   Wifi_Connect_OK_IsChange, Get_BLE_Enable_status_Value = %d call device_updateResult\r\n", LED_Control_data.Get_BLE_Enable_status_Value);
      device_updateResult();
      
    }
    
    if (LED_Control_data.Wifi_Connected_Not_Active_IsChange){
//      WifiBT_Connect_Status = WIFI_CONNECTED_NOT_ACTIVE;
//      Receive_Cloud_IAQ_color = false;
////      LEDs_Effect(BL_LED1, LED_ON_SOLID);
////      LEDs_Effect(BL_LED2, LED_ON_SOLID);
//      if (LED_Control_data.IAQvalue == IAQ_NONE){
//        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
//      }
//      LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
//      
//      LED_Control_data.Wifi_Connected_Not_Active_IsChange = false;
//      printf("______________   Wifi_Connected_Not_Active_IsChange\r\n");
    }
    

    
    if (LED_Control_data.Wifi_Connected_Active_IsChange){
//      WifiBT_Connect_Status = WIFI_CONNECTED_ACTIVE;
//      LED_Run_Freq = LED_RUN_FREQ_0p5HZ;
////      LEDs_Effect(BL_LED1, LED_EFFECT_FADE);
////      LEDs_Effect(BL_LED2, LED_EFFECT_FADE);
////      LEDs_Effect(IAQ_LEDS, LED_EFFECT_FADE);
//      if (LED_Control_data.IAQvalue == IAQ_NONE){
//        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
//      }
//      if (LED_Control_data.Fan_AlgoApp_State_value){
//        LEDs_Effect(IAQ_LEDS, LED_EFFECT_FADE);
//      }else{
//        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
//      }
//      LED_Control_data.Wifi_Connected_Active_IsChange = false;
//      printf("______________   Wifi_Connected_Active_IsChange\r\n");
    }
    
    if (LED_Control_data.Wifi_Disconnected_IsChange){
      WifiDischagne_FormESP = true;
      WifiBT_Connect_Status = WIFI_DISCONNECTED;

      Receive_Cloud_IAQ_color = false;
      
//      LEDs_Effect(BL_LED1, LED_ON_SOLID);
//      LEDs_Effect(BL_LED2, LED_ON_SOLID);
      //LED_Control_data.IAQvalue = IAQ_BLUE;
#ifndef IS_TH_TEST_FW
      if (LED_Control_data.IAQvalue == IAQ_NONE){
        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
      }
#endif
      if (LED_Control_data.Get_BLE_Enable_status_Value == 0){
        if (WifiBT_Connect_Status_B4 == BT_PAIRING){
          LEDs_Effect(BL_LED1, LED_OFF);
          LEDs_Effect(BL_LED2, LED_OFF);
        } 
        LED_Run_Freq = LED_RUN_FREQ_2p0HZ;
        LEDs_Effect(IAQ_LEDS, LED_EFFECT_BLINK);        
//        LED_Control_data.BT_Pairing_Value = 0;
      }else if (LED_Control_data.Get_BLE_Enable_status_Value == 1){
//        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
//        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
//        LEDs_Effect(IAQ_LEDS, LED_OFF);

      }
      WifiBT_Connect_Status_B4 = WIFI_DISCONNECTED;
      LED_Control_data.Wifi_Disconnected_IsChange = false;
//      printf("______________   Wifi_Disconnected_IsChange, Get_BLE_Enable_status_Value = %d , call device_updateResult()\r\n", LED_Control_data.Get_BLE_Enable_status_Value);
      
      device_updateResult();
    }
    
    if (LED_Control_data.BLE_Enable_status_){
      
      if (LED_Control_data.Get_BLE_Enable_status_Value == 0){
        if (LED_Control_data.Get_BLE_Enable_status_Value_B4 != LED_Control_data.Get_BLE_Enable_status_Value){
            BLE_Enable_status = 0;

            if (WifiBT_Connect_Status == WIFI_DISCONNECTED){
              LEDs_Effect(IAQ_LEDS, LED_EFFECT_BLINK);
            }else if(WifiBT_Connect_Status == WIFI_CONNECT_OK){
              LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
            }
            
            if (LED_Control_data.Btn1_OnOff == GPIO_PIN_RESET){
              LEDs_Effect(BL_LED1, LED_OFF);
            }else{
              LEDs_Effect(BL_LED1, LED_ON_SOLID);
            }
            if (LED_Control_data.Btn2_OnOff == GPIO_PIN_RESET){
              LEDs_Effect(BL_LED2, LED_OFF);
            }else{
              LEDs_Effect(BL_LED2, LED_ON_SOLID);
            }
          LED_Control_data.Get_BLE_Enable_status_Value_B4 = LED_Control_data.Get_BLE_Enable_status_Value;
          LED_Control_data.BT_Pairing_Value = 0;
        }
      }else if (LED_Control_data.Get_BLE_Enable_status_Value == 1){
        
//        BLE_Enable_status = 1;
//        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
//        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
//        LEDs_Effect(IAQ_LEDS, LED_OFF);
//        LED_Control_data.Get_BLE_Enable_status_Value_B4 = LED_Control_data.Get_BLE_Enable_status_Value;
        
        LED_Control_data.Get_BLE_Enable_status_Value_B4 = LED_Control_data.Get_BLE_Enable_status_Value;
        WifiBT_Connect_Status = BT_PAIRING;
        WifiBT_Connect_Status_B4 = BT_PAIRING;
        BLE_Enable_status = 1;
        OffAllLEDs_Output1 = 1;            //if get in BT pairing mode, off all outputs
        OffAllLEDs_Output2 = 1;
        LED_Control_data.B4IAQvalue = LED_Control_data.IAQvalue;
        
//      Button1_SingleClick = 1;
//      Button2_SingleClick = 1;
        LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Fan_AlgoApp_State_value = 0;
        LED_Control_data.Fan_App_Status_Value = 0;
        LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
      //device_updateResult();
      
        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
        LED_Control_data.BT_Pairing_Value = 1;
        
      }

      LED_Control_data.BLE_Enable_status_ = false;
    }
    

    
    
//    if (LED_Control_data.Manual_on_IsChange){
//      
//      LED_Control_data.Manual_on_IsChange = false;
//      if (LED_Control_data.IAQvalue == IAQ_NONE){
//        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
//      }
//      printf("______________   Manual_on_IsChange\r\n");
//    }
    
    if (LED_Control_data.No_Wifi_BT_IsChange){
      
      LED_Control_data.No_Wifi_BT_IsChange = false;
#ifndef IS_TH_TEST_FW
      if (LED_Control_data.IAQvalue == IAQ_NONE){
        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
      }
#endif
//      printf("______________   No_Wifi_BT_IsChange\r\n");
    }
    
    if (LED_Control_data.SwtichConfig_IsChange){
      
        OffAllLEDs_Output1 = 1; 
        OffAllLEDs_Output2 = 1;

        GPIO_Cont = 0;
        LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
            LED_Control_data.Btn1_OnOff_B4 = LED_Control_data.Btn1_OnOff;
            LED_Control_data.Btn2_OnOff_B4 = LED_Control_data.Btn2_OnOff;
        LED_Control_data.Fan_AlgoApp_State_value = 0;
        LED_Control_data.Fan_App_Status_Value = 0;
        LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
        
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
          HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
        }else{
          Latch_Relay_Control(1, 0);
          Latch_Relay_Control(2, 0);
        }
        LEDs_Effect(BL_LED1, LED_OFF);
        LEDs_Effect(BL_LED2, LED_OFF);
        
//      printf("______________   SwtichConfig_IsChange, call device_updateResult()\r\n");
      
      device_updateResult();
      SetSTM_FlashUserData(FLASH_SET_SW_CONFIG_FLAG_POS, &Switch_config_setting);
      
      LED_Control_data.SwtichConfig_IsChange = false;
    }
    
    if (LED_Control_data.Fan_App_Status_IsChange || LED_Control_data.Fan_AlgoApp_State_IsChange){
  
      
      if (LED_Control_data.Fan_AlgoApp_State_IsChange){
        if (LED_Control_data.Manual_on_OnOff == 0){
          Algo_or_AppState_set = LED_Control_data.Fan_AlgoApp_State_value;
        }
          device_updateResult();
        
//          printf("______________   Fan_AlgoApp_State_value, value = %d\r\n", LED_Control_data.Fan_AlgoApp_State_value);
          
      }else if(LED_Control_data.Fan_App_Status_IsChange){
        Algo_or_AppState_set = LED_Control_data.Fan_App_Status_Value;
//        printf("______________   Fan_App_Status_Value, value = %d\r\n", LED_Control_data.Fan_App_Status_Value);
        
      }
        
      
      
#ifndef IS_TH_TEST_FW
      if (LED_Control_data.IAQvalue == IAQ_NONE){
        LED_Control_data.IAQvalue = LED_Control_data.B4IAQvalue;
      }
#endif
      
      if (Algo_or_AppState_set >= 0){
          switch(Switch_config_setting)
          {
          case 1:
          case 2:
          case 4:
          case 5:
            Button1_SingleClick = 1;
            if (Algo_or_AppState_set == 0){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
            }else if(Algo_or_AppState_set == 1){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_SET;
            }
            break;
          case 3:
          case 6:
            Button1_SingleClick = 1;
  #ifdef UICONT_BRNC021_BUTTON
            Button2_SingleClick = 1;
  #endif
            if (Algo_or_AppState_set == 0){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
              LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
            }else if(Algo_or_AppState_set == 1){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_SET;
              LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
            }else if(Algo_or_AppState_set == 2){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
              LED_Control_data.Btn2_OnOff = GPIO_PIN_SET;
            }
            break;
          case 7:
  //          if (LED_Control_data.Fan_App_Status_IsChange){
  //            WC_Single_Speed_control(Algo_or_AppState_set);
  //          }else if (LED_Control_data.Fan_AlgoApp_State_value){
              WC_Single_Speed_control(Algo_or_AppState_set);
  //          }
            
  //          LED_Control_data.FanRealState_Value = SW_Config7_RangeHoodSpeed;
            DEBUG_DRIVER("Algo_or_AppState_set >=0 and device config = 7, call device_updateResult()\r\n");
            device_updateResult();
            break;
            
          };
      }
      Algo_or_AppState_set = -1;
      LED_Control_data.Fan_App_Status_IsChange = false;
      LED_Control_data.Fan_AlgoApp_State_IsChange = false;
      //device_updateResult();

    }
    
#ifdef BROAN_BRNC021
//      if (LED_Control_data.Triac1_IsChange){
//        Button1_SingleClick = 1;
//        LED_Control_data.Triac1_IsChange = false;
//      }
//      if (LED_Control_data.Triac2_IsChange){
//        Button2_SingleClick = 1;
//        LED_Control_data.Triac2_IsChange = false;
//      }
#endif
  }
  
  
#ifdef UICONT_BRNC021_BUTTON
  if (Button1_SingleClick == 1 || Button2_SingleClick == 1){// || ((HAL_GetTick() - BLE_TimeOutCnt) > 120000)){
#else
    if (Button1_SingleClick == 1){// || ((HAL_GetTick() - BLE_TimeOutCnt) > 120000)){
#endif
    if (BLE_Enable_status == 0){
//        if (GPIO_Cont == 1){
//          ESP_Cmd_Send_En = 1;
//          LED_Control_data.Manual_on_IsChange = 1;
//          LED_Control_data.Manual_on_OnOff = true;
//        }else{
//          ESP_Cmd_Send_En = 1;
//          LED_Control_data.Manual_on_IsChange = 1;
//          LED_Control_data.Manual_on_OnOff = false;
//        }
      BLE_TimeOutCnt = 0;
          //vTaskDelay(500);
    }else{
      if (LED_Control_data.Get_BLE_Enable_status_Value == 1){
          DEBUG_DRIVER("BLE_Enable_status = 1\r\n");
          GPIO_Cont = 0;
          Button1_SingleClick = 0;
          Button2_SingleClick = 0;
          ESP_Cmd_Send_En = 1;
          LED_Control_data.BT_Pairing_IsChange = 1;
          LED_Control_data.BT_Pairing_Value = 0;
          vTaskDelay(ESP_CMD_SEND_DELAY_T);
          WifiBT_Connect_Status = WIFI_DISCONNECTED;
          BLE_Enable_status = 0;
          B4_IAQ_Rate = 0;
          LEDs_Effect(BL_LED1, LED_OFF);
          LEDs_Effect(BL_LED2, LED_OFF);
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
          vTaskDelay(500);
      }
    }
      
  }
#ifdef UICONT_BRNC021_BUTTON
  
//    if (isLongDualPress == 1 && BLE_Enable_status == 0){
  
//  if (isLongDualPress == 1) {
//    isLongDualPress = 0;
//  }

//    if (((HAL_GetTick() - Check_CallBLE_LongPress_Fall_T) > LONG_PRESS_TIME) && Check_CallBLE_LongPress_Fall_T != 0 && LED_Control_data.BT_Pairing_Value == 0){
  if (isLongDualPress_Hold == 1){
        DEBUG_DRIVER("\n\r EXTI Long Hold press\r\n");
        isLongDualPress_Hold = 0;
        GPIO_Cont = 0;
      if (LED_Control_data.BT_Pairing_Value == 0){
        ESP_Cmd_Send_En = 1;
        LED_Control_data.BT_Pairing_IsChange = 1;
        LED_Control_data.BT_Pairing_Value = 1;
        WifiBT_Connect_Status = BT_PAIRING;
        WifiBT_Connect_Status_B4 = BT_PAIRING;
        vTaskDelay(ESP_CMD_SEND_DELAY_T);
        BLE_Enable_status = 1;
        OffAllLEDs_Output1 = 1;            //if get in BT pairing mode, off all outputs
        OffAllLEDs_Output2 = 1;
        LED_Control_data.B4IAQvalue = LED_Control_data.IAQvalue;
        
//      Button1_SingleClick = 1;
//      Button2_SingleClick = 1;
        LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Fan_AlgoApp_State_value = 0;
        LED_Control_data.Fan_App_Status_Value = 0;
        LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
      //device_updateResult();
      
        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
        
        vTaskDelay(2000);
      }
    }
#endif

      

#ifdef UICONT_BRNC011_BUTTON
    if (isButton1_LongPress == 1){
      isButton1_LongPress = 0;
      GPIO_Cont = 0;
//        Button1_SingleClick = 1;

//         vTaskDelay(2000);
        

    }

    if (isButton1_LongPress_Hold == 1){
      
        isButton1_LongPress_Hold = 0;
//        Check_CallBLE_LongPress_Fall_T = 0;
        GPIO_Cont = 0;
      if (LED_Control_data.BT_Pairing_Value == 0){
       
        WifiBT_Connect_Status = BT_PAIRING;
        WifiBT_Connect_Status_B4 = BT_PAIRING;
        LED_Run_Freq = LED_RUN_FREQ_2p0HZ;

        
        ESP_Cmd_Send_En = 1;
        LED_Control_data.BT_Pairing_IsChange = 1;
        LED_Control_data.BT_Pairing_Value = 1;
        vTaskDelay(ESP_CMD_SEND_DELAY_T);
        BLE_Enable_status = 1;
        LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
        LED_Control_data.Fan_AlgoApp_State_value = 0;
        LED_Control_data.Fan_App_Status_Value = 0;
        LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
         OffAllLEDs_Output1 = 1;            //if get in BT pairing mode, off all outputs
        LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
//        LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
        vTaskDelay(2000);
      }
    }
#endif
        
#ifdef UICONT_BRNC021_BUTTON
    if (isButton1_LongPress == 1 && BLE_Enable_status == 0){
      isButton1_LongPress = 0;
        if (Switch_config_setting == 7){
          if (GPIO_Cont == 1){
            GPIO_Cont = 0;
            WC_Single_Speed_control(3);
          }
        }
    }
#endif

  
//    if (isButton1_ExtendedPress == 1 && BLE_Enable_status == 0){
//      isButton1_ExtendedPress = 0;
//        #ifdef UICONT_BRNC011_BUTTON
//          if (GPIO_Cont == 1){
//            GPIO_Cont = 0;
//            printf("GPIO_Cont 0  - 4 \r\n");
//            ESP_Cmd_Send_En = 1;
//            LED_Control_data.SysReset_IsChange = 1;
//          }
//        #elif defined(UICONT_BRNC021_BUTTON)
//      
//        #endif
//    }  
    
    if (BLE_Enable_status == 0 && (Button1_SingleClick == 1 || Button2_SingleClick == 1)){
      if (Switch_config_setting == 3 || Switch_config_setting == 6){
          DEBUG_DRIVER("______Detected devicConfig = %d, pressed Button  now wait 100ms\r\n", Switch_config_setting);
          if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
            if (device_getOutput1Config() != DEVICECONFIG_OUTPUT_DISABLE ){
              HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
            }
            if (device_getOutput2Config() != DEVICECONFIG_OUTPUT_DISABLE ){
              HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
            }
          }else{
            if (device_getOutput1Config() != DEVICECONFIG_OUTPUT_DISABLE ){
              Latch_Relay_Control(1, 0);
            }
            if (device_getOutput2Config() != DEVICECONFIG_OUTPUT_DISABLE ){
              Latch_Relay_Control(2, 0);
            }
          }
          
          vTaskDelay(100);
      }
    }
    
    
  
    if ((Button1_SingleClick == 1 && BLE_Enable_status == 0) || (OffAllLEDs_Output1))
    {
      
      
      
      Button1_SingleClick = 0;
      DEBUG_DRIVER("Button 1 click,  get Switch_config_setting = %d, WifiBT_Connect_Status = %d, OffAllLEDs_Output1 = %d \r\n", Switch_config_setting, WifiBT_Connect_Status, OffAllLEDs_Output1);
      
      if (OffAllLEDs_Output1){
        OffAllLEDs_Output1 = false;
        LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
        GPIO_Cont = 0;
      }
      
      
        #ifdef UICONT_BRNC011_BUTTON
          if (GPIO_Cont == 1){
//            ESP_Cmd_Send_En = 1;
//            LED_Control_data.Manual_on_IsChange = 1;
            if (BL_LED1_status == 0){
              if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  Latch_Relay_Control(1, GPIO_PIN_SET);
              }else{
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
              }
                  
//              BL_LED1_PWM_value = PWM_MAX;
              LED_Run_Freq = LED_RUN_FREQ_0p5HZ;
              LEDs_Effect(BL_LED1, LED_EFFECT_FADE);
              LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
//              printf("Relay on! 22\r\n");
              BL_LED1_status = 1;
            }else{
              if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  Latch_Relay_Control(1, GPIO_PIN_RESET);
              }else{
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
              }
//              BL_LED1_PWM_value = 0;
              LEDs_Effect(BL_LED1, LED_OFF);
//              printf("Relay off! 22\r\n");
              BL_LED1_status = 0;
            }
            GPIO_Cont = 0;
            ESP_Cmd_Send_En = 1;
            LED_Control_data.Btn1_RealState_isChange = 1;
            
            if (LED_Control_data.Btn1_OnOff){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
            }else{
              LED_Control_data.Btn1_OnOff = GPIO_PIN_SET;
            }
            LED_Control_data.Btn1_OnOff_B4 = LED_Control_data.Btn1_OnOff;
            vTaskDelay(ESP_CMD_SEND_DELAY_T);
          }else{
            if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                Latch_Relay_Control(1, LED_Control_data.Btn1_OnOff);
            }else{
                HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, LED_Control_data.Btn1_OnOff);
            }
            if (LED_Control_data.Btn1_OnOff){
//              BL_LED1_PWM_value = PWM_MAX;
              LEDs_Effect(BL_LED1, LED_ON_SOLID);
              LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
            }else {
              if (LED_Control_data.BT_Pairing_Value == 1){
                LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
                LEDs_Effect(IAQ_LEDS, LED_OFF);
              }else if (LED_Control_data.BT_Pairing_Value == 0){
                if (WifiBT_Connect_Status != BT_PAIRING){
                  LEDs_Effect(BL_LED1, LED_OFF);
                  LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
                }
              }
            }
            BL_LED1_status = LED_Control_data.Btn1_OnOff;
          }
          
        #elif defined(UICONT_BRNC021_BUTTON)
      if (device_getOutput1Config() != DEVICECONFIG_OUTPUT_DISABLE){
        
//        if (Switch_config_setting == 3 || Switch_config_setting == 6){
//          DEBUG_DRIVER("______Detected devicConfig = %d, pressed Button 1, now wait 100ms\r\n", Switch_config_setting);
//          if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
//            HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
//          }else{
//            Latch_Relay_Control(1, 0);
//            Latch_Relay_Control(2, 0);
//          }
//          
//          vTaskDelay(100);
//        }
        
          if (GPIO_Cont == 1){
            GPIO_Cont = 0;
//            ESP_Cmd_Send_En = 1;
//            LED_Control_data.Btn1_status_IsChange = 1;
            if (LED_Control_data.Btn1_OnOff){
              LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
              if (Switch_config_setting == 7){
                RangeHood_Speed_control(1);
//              }else if (Switch_config_setting == 3 || Switch_config_setting == 6){ //0.1.55 Switch config 3 and 6 can turn off 
//                WC_Single_Speed_control(2);
              }else{   
                //traic_control(1,0,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(1, 0);
                }
                
              }
            }else{      //LED_Control_data.Btn1_OnOff = true
              LED_Control_data.Btn1_OnOff = GPIO_PIN_SET;
              switch(Switch_config_setting)
              {
              case 1:
              case 2:
              case 4:
              case 5:
                //traic_control(1,4,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
                }else{
                 Latch_Relay_Control(1,1);// Latch_Relay_Control(1, 2);
                }
                break;
              case 3:
              case 6:
//                ESP_Cmd_Send_En = 1;
//                LED_Control_data.Btn2_status_IsChange = 1;

//                vTaskDelay(ESP_CMD_SEND_DELAY_T);
                LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
                BL_LED2_status = 0;
                LEDs_Effect(BL_LED2, LED_OFF);
                
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(1, 1);
                  Latch_Relay_Control(2, 0);
                }
                
                
//                Latch_Relay_Control(1, 1);
//                Latch_Relay_Control(2, 0);
                
//                WC_Single_Speed_control(1);
                break;
              case 7:
                //SW_Config_7_RangeHood_Control(1);
                RangeHood_Speed_control(1);
                break;
              };
              
              
                
            }

          }else{
            if (LED_Control_data.Btn1_OnOff){
              switch(Switch_config_setting)
              {
              case 1:
              case 2:
              case 4:
              case 5:
//                traic_control(1,4,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
                }else{
                  Latch_Relay_Control(1, 1);
                }
                break;
              case 3:
              case 6:
//                ESP_Cmd_Send_En = 1;
//                LED_Control_data.Btn2_status_IsChange = 1;
//                vTaskDelay(ESP_CMD_SEND_DELAY_T);
                
                LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
                BL_LED2_status = 0;
                LEDs_Effect(BL_LED2, LED_OFF);
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(1, 1);
                  Latch_Relay_Control(2, 0);
                }
                
//                WC_Single_Speed_control(1);
                break;
              case 7:
                //SW_Config_7_RangeHood_Control(1);
                RangeHood_Speed_control(1);
                
                break;
              };
            }else{              //LED_Control_data.Btn1_OnOff = false
              if (Switch_config_setting == 7){
                //SW_Config_7_RangeHood_Control(1);
                if (BLE_Enable_status == 0){
                  RangeHood_Speed_control(1);
                }
//              }else if (Switch_config_setting == 3 || Switch_config_setting == 6){
//                WC_Single_Speed_control(2);
              }else{  
//                traic_control(1,0,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(1, 0);
                }
              }
            }
            
          }
            LED_Control_data.Btn1_OnOff_B4 = LED_Control_data.Btn1_OnOff;
            LED_Control_data.Btn2_OnOff_B4 = LED_Control_data.Btn2_OnOff;
          BL_LED1_status = LED_Control_data.Btn1_OnOff;
          DEBUG_DRIVER("Button1_SingleClick , call device_updateResult()\r\n");
          device_updateResult();
      }
          if (LED_Control_data.BT_Pairing_Value == 1){
            LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
            LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
            LEDs_Effect(IAQ_LEDS, LED_OFF);
          }else if (BL_LED1_status == 0 && Switch_config_setting != 7){
            if (WifiBT_Connect_Status != BT_PAIRING){
              LEDs_Effect(BL_LED1, LED_OFF);
              LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
            }
          }else{
            //BL_LED1_PWM_value = PWM_MAX;
            if (Switch_config_setting != 7){
              LEDs_Effect(BL_LED1, LED_ON_SOLID);
              LEDs_Effect(IAQ_LEDS, LED_EFFECT_FADE);
            }
          }
        #endif

    }

  
#ifdef UICONT_BRNC021_BUTTON

//  
//  if (isExtendedDualPress == 1 && BLE_Enable_status == 0){
//    isExtendedDualPress = 0;
//    if (GPIO_Cont == 1){
//      GPIO_Cont = 0;
//      ESP_Cmd_Send_En = 1;
//      LED_Control_data.SysReset_IsChange = 1;
//    }
//  }
  

    if ((Button2_SingleClick == 1 && BLE_Enable_status == 0) || (OffAllLEDs_Output2))// && Switch_config_setting != 1 && Switch_config_setting != 4)
    {
//      ESP_Cmd_Send_En = 1;
      Button2_SingleClick = 0;      
      DEBUG_DRIVER("Button 2 click,  get Switch_config_setting = %d, GPIO_Cont = %d, OffAllLEDs_Output2 = %d \r\n", Switch_config_setting, GPIO_Cont, OffAllLEDs_Output2);
      
      if (OffAllLEDs_Output2){
        OffAllLEDs_Output2 = false;
        LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
        GPIO_Cont = 0;
      }
      if (device_getOutput2Config() != DEVICECONFIG_OUTPUT_DISABLE ){
//        if (Switch_config_setting == 3 || Switch_config_setting == 6){
//          DEBUG_DRIVER("______Detected devicConfig = %d, pressed Button 2, now wait 100ms\r\n", Switch_config_setting);
//          if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
//            HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
//          }else{
//            Latch_Relay_Control(1, 0);
//            Latch_Relay_Control(2, 0);
//          }
//          vTaskDelay(100);
//        }
          if (GPIO_Cont == 1){
            GPIO_Cont = 0;
//            ESP_Cmd_Send_En = 1;
//            LED_Control_data.Btn2_status_IsChange = 1;
            
            if (BL_LED2_status == 0){
              //BL_LED2_PWM_value = PWM_MAX;
              //DEBUG_DRIVER("Blue LED2 on!\r\n");
              BL_LED2_status = 1;
            }else{
              //BL_LED2_PWM_value = 0;
              //DEBUG_DRIVER("Blue LED2 off!\r\n");
              BL_LED2_status = 0;
            }
            
            if (LED_Control_data.Btn2_OnOff){
              LED_Control_data.Btn2_OnOff = GPIO_PIN_RESET;
              if (Switch_config_setting == 7){
                //SW_Config_7_RangeHood_Control(2);
                RangeHood_Speed_control(2);
//              }else if (Switch_config_setting == 3 || Switch_config_setting == 6){
//                WC_Single_Speed_control(1);
              }else{
//                traic_control(2,0,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
                }else{
                    Latch_Relay_Control(2, 0);
                }
              }
            }else{              //LED_Control_data.Btn2_OnOff = true
              LED_Control_data.Btn2_OnOff = GPIO_PIN_SET;
//              if (Switch_config_setting == 1 || Switch_config_setting == 2 || Switch_config_setting == 4 || Switch_config_setting == 5)
//                traic_control(2,3,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
//              else if (Switch_config_setting == 3 || Switch_config_setting == 6)
//                traic_control(2,1,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
              switch(Switch_config_setting)
              {
              case 1:
              case 4:
               // break;
              case 2:
              case 5:
//                traic_control(2,4,1);//1_120V
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
                }else{
                  Latch_Relay_Control(2, 1);
                }
                break;
              case 3:
              case 6:
//                ESP_Cmd_Send_En = 1;
//                LED_Control_data.Btn1_status_IsChange = 1;
                
                LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
                BL_LED1_status = 0;
                LEDs_Effect(BL_LED1, LED_OFF);
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(2, 1);
                  Latch_Relay_Control(1, 0);
                }
                
//                WC_Single_Speed_control(2);
                break;
              case 7:
                //SW_Config_7_RangeHood_Control(2);
                RangeHood_Speed_control(2);
                break;
              default :break;
              };
            }            
          }else{
            if (LED_Control_data.Btn2_OnOff){
//                if (Switch_config_setting == 1 || Switch_config_setting == 2 || Switch_config_setting == 4 || Switch_config_setting == 5)
//                  traic_control(2,3,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
//                else if (Switch_config_setting == 3 || Switch_config_setting == 6)
//                  traic_control(2,1,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
              switch(Switch_config_setting)
              {
              case 1:
              case 4:
                break;
              case 2:
              case 5:
//                traic_control(2,4,1);//ÂúµçÑ¹120V
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
                }else{
                  Latch_Relay_Control(2, 1);
                }
                break;
              case 3:
              case 6:
//                ESP_Cmd_Send_En = 1;
//                LED_Control_data.Btn1_status_IsChange = 1;
                LED_Control_data.Btn1_OnOff = GPIO_PIN_RESET;
                BL_LED1_status = 0;
                LEDs_Effect(BL_LED1, LED_OFF);
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(2, 1);
                  Latch_Relay_Control(1, 0);
                }
                
//                WC_Single_Speed_control(2);
                break;
              case 7:
                //SW_Config_7_RangeHood_Control(2);
                RangeHood_Speed_control(2);
                break;
              default :break;
              };
            }else{      //LED_Control_data.Btn2_OnOff == false
              if (Switch_config_setting == 7){
                //SW_Config_7_RangeHood_Control(2);
                if (BLE_Enable_status == 0){
                  RangeHood_Speed_control(2);
                }
//              }else if (Switch_config_setting == 3 || Switch_config_setting == 6){
//                WC_Single_Speed_control(1);
              }else{
//                traic_control(2,0,1);         //1 = triac 1, 2 = triac; 0 = off, 1 = 30v, 2 = 60v, 3 = 120v; must use 1
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
                }else{
                  Latch_Relay_Control(2, 0);
                }
              }
            }
            
          }
            LED_Control_data.Btn1_OnOff_B4 = LED_Control_data.Btn1_OnOff;
            LED_Control_data.Btn2_OnOff_B4 = LED_Control_data.Btn2_OnOff;
          BL_LED2_status = LED_Control_data.Btn2_OnOff;
          DEBUG_DRIVER("Button2_SingleClick , call device_updateResult()\r\n");
          device_updateResult();
      }

            if (LED_Control_data.BT_Pairing_Value == 1){
              LEDs_Effect(BL_LED1, LED_EFFECT_BLINK);
              LEDs_Effect(BL_LED2, LED_EFFECT_BLINK);
              LEDs_Effect(IAQ_LEDS, LED_OFF);
            }else if (BL_LED2_status == 0 && Switch_config_setting != 7){
              if (WifiBT_Connect_Status != BT_PAIRING){
                LEDs_Effect(BL_LED2, LED_OFF);
                LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
                DEBUG_DRIVER("Blue LED2 off!\r\n");
              }
            }else{
              if (Switch_config_setting != 7 && Switch_config_setting != 1 && Switch_config_setting != 4){
                LEDs_Effect(BL_LED2, LED_ON_SOLID);
                LEDs_Effect(IAQ_LEDS, LED_EFFECT_BLINK);
                DEBUG_DRIVER("Blue LED2 on!\r\n");
              }
            }

    }
  
  if (isButton2_LongPress == 1 && BLE_Enable_status == 0){
    isButton2_LongPress = 0;
    
    if (GPIO_Cont == 1){
      GPIO_Cont = 0;
        if (Switch_config_setting == 7){
          WC_Single_Speed_control(0);
        }

      
    }
  }
  
#endif
  
  if (WifiBT_Connect_Status == WIFI_CONNECT_OK && Receive_Cloud_IAQ_color == true ){
    IAQ_Rate = 0;
    B4_IAQ_Rate = 0;
  }else if (LED_Control_data.Manual_on_OnOff == true || WifiBT_Connect_Status == WIFI_CONNECTED_NOT_ACTIVE|| WifiBT_Connect_Status == WIFI_CONNECT_OK || WifiBT_Connect_Status == WIFI_DISCONNECTED || Receive_Cloud_IAQ_color == false){
#ifdef BROAN_BRNC011
    IAQ_Rate = CO2_Mark + TVOC_Mark + PM2p5_Mark + RH_Mark;
#else
    IAQ_Rate = CO2_Mark + TVOC_Mark + RH_Mark;
#endif
    
    IAQ_Rate = GetMAX(CO2_Mark , TVOC_Mark , PM2p5_Mark , RH_Mark);
    
    
    if (IAQ_Rate != B4_IAQ_Rate){
      B4_IAQ_Rate = IAQ_Rate;
      
      if (IAQ_Rate >= 80){
        LED_Control_data.IAQvalue = IAQ_RED;
      }else if (IAQ_Rate >= 40){
        LED_Control_data.IAQvalue = IAQ_YELLOW;
      }else{
        LED_Control_data.IAQvalue = IAQ_GREEN;
      }
      

        
      LED_Control_data.B4IAQvalue = LED_Control_data.IAQvalue;
        
      LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
    }
  }
#ifdef DEBUG_SENSOR_
  if (PrintIAQrate_OneSecTick){
    PrintIAQrate_OneSecTick = false;
    DEBUG_SENSOR("WifiBT_Connect_Status = %d, IAQ color = %d, IAQ_Rate: = %d, CO2_Mark: = %d, TVOC_Mark = %d, PM2p5_Mark = %d, RH_Mark = %d, Receive_Cloud_IAQ_color = %d \r\n", WifiBT_Connect_Status, LED_Control_data.IAQvalue, IAQ_Rate, CO2_Mark, TVOC_Mark, PM2p5_Mark, RH_Mark, Receive_Cloud_IAQ_color);
  }
#endif
  }
}

uint8_t GetMAX(uint8_t CO2_M, uint8_t TVOC_M, uint8_t PM2p5_M, uint8_t RH_M){
  uint8_t ret = 0;
  
  if (CO2_M >= ret){
    ret = CO2_M;
  }
  
  if (TVOC_M >= ret){
    ret = TVOC_M;
  }
  
  if (PM2p5_M >= ret){
    ret = PM2p5_M;
  }
  
  if (RH_M >= ret){
    ret = RH_M;
  }
  
  return ret;
}

void Start_EngMP_mode_Task(void const * argument){
  uint8_t i = 0;
  int all_sensor_OK = 0;
  uint8_t BT1_LED_status = 0;
  uint8_t BT2_LED_status = 0;
  uint8_t IAQ_color_Rolling_Cnt = 0;
  //uint8_t wait_wifi_cnt = 20;
  int ret = 0;
  
  DEBUG_MP_MODE("Engineer MP mode start....\r\n");
  Uart1Ready = RESET;
  
  NTC_getTempValue(0);
  
    LED_Control_data.IAQvalue = IAQ_BLUE;
    LED_Run_Freq = LED_RUN_FREQ_2p0HZ;
    LEDs_Effect(BL_LED1, LED_ON_SOLID);
    LEDs_Effect(BL_LED2, LED_ON_SOLID);
    LEDs_Effect(IAQ_LEDS, LED_OFF);
    vTaskDelay(1000);
    LEDs_Effect(BL_LED1, LED_OFF);
    LEDs_Effect(BL_LED2, LED_OFF);
    LED_Control_data.IAQvalue = IAQ_GREEN;
    LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
    vTaskDelay(1000);
    LED_Control_data.IAQvalue = IAQ_YELLOW;
    LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
    vTaskDelay(1000);
    LED_Control_data.IAQvalue = IAQ_RED;
    LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
    vTaskDelay(1000);
    
    LED_Control_data.IAQvalue = IAQ_BLUE;
    LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
    vTaskDelay(1000);
#ifdef SENSOR_SCD30
    ModuleInitState.SCD30 = InitSCD30();
#endif
#ifdef SENSOR_SPS30
    ModuleInitState.SPS30 = InitSPS30();
#endif
#ifdef SENSOR_ZMOD4410
    ModuleInitState.ZMOD4410 = InitZMOD4410();
#endif
#ifdef SENSOR_SI7021
    ModuleInitState.Si7021 = InitSi7021();
    
    if (GetNTCTemp3() < 10 || GetNTCTemp3() > 60){
      ModuleInitState.Si7021 = HAL_ERROR;
    }
    if (GetNTCTemp1() < -40 || GetNTCTemp1() > 125){
      ModuleInitState.Si7021 = HAL_ERROR;
    }

#endif
    
//    if (LED_Control_data.Wifi_Connect_OK_IsChange == false){
//      for (wait_wifi_cnt = 0; wait_wifi_cnt < 20; wait_wifi_cnt++){
//        if (LED_Control_data.Wifi_Connect_OK_IsChange == true ||LED_Control_data.Wifi_Connected_Active_IsChange == true ){
//          break;
//        }
//        vTaskDelay(1000);
//      }
//    }
    
  while (1) {
    vTaskDelay(100);
    
    if (Button1_SingleClick == 1){
      Button1_SingleClick = 0;
      DEBUG_MP_MODE("BTN 1 single click\r\n");
      
//      if ((Yello_IAQ_Red_value - 100) < 0){
//        Yello_IAQ_Red_value = 3350;
//      }else{
//        Yello_IAQ_Red_value -= 100;
//      }
//      DEBUG_MP_MODE("Red_value =  %d \r\n", Yello_IAQ_Red_value);
//        LED_Control_data.IAQvalue = IAQ_YELLOW;
//        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
      
      if (BT1_LED_status == 0){
        LEDs_Effect(BL_LED1, LED_ON_SOLID);
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
      #ifdef BROAN_BRNC011
            Latch_Relay_Control(1, 1);
      #else
            HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
      #endif
        }else{
      #ifdef BROAN_BRNC011
            HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
      #else
            Latch_Relay_Control(1, 1);
      #endif
        }
        BT1_LED_status = 1;
      }else{
        LEDs_Effect(BL_LED1, LED_OFF);
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
      #ifdef BROAN_BRNC011
            Latch_Relay_Control(1, 0);
      #else
            HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      #endif
        }else{
      #ifdef BROAN_BRNC011
            HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      #else
            Latch_Relay_Control(1, 0);
      #endif
        }
        BT1_LED_status = 0;
      }

    }
#ifdef UICONT_BRNC021_BUTTON
    if (Button2_SingleClick == 1){
      Button2_SingleClick = 0;
      DEBUG_MP_MODE("BTN 2 single click\r\n");
      if (BT2_LED_status == 0){
        LEDs_Effect(BL_LED2, LED_ON_SOLID);
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
    #ifdef BROAN_BRNC011
          Latch_Relay_Control(2, 1);
    #else
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
    #endif
      }else{
    #ifdef BROAN_BRNC011
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
    #else
          Latch_Relay_Control(2, 1);
    #endif
      }
        BT2_LED_status = 1;
      }else{
        LEDs_Effect(BL_LED2, LED_OFF);
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
        #ifdef BROAN_BRNC011
              Latch_Relay_Control(2, 0);
        #else
              HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
        #endif
        }else{
        #ifdef BROAN_BRNC011
              HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
        #else
              Latch_Relay_Control(2, 0);
        #endif
        }
        BT2_LED_status = 0;
      }

    }
#endif
    
    if (isButton1_LongPress == 1){
      isButton1_LongPress = 0;
      
    /* This function starts the cleaning procedure. It's
    * recommended to be executed after product assembly. This
    * helps to clean the metal oxide surface from assembly.
    * IMPORTANT NOTE: The cleaning procedure can be run only once
    * during the modules lifetime and takes 10 minutes. */
    if (ModuleInitState.ZMOD4410 == HAL_OK && LED_Control_data.IAQvalue != IAQ_PURPLE){
      LED_Control_data.IAQvalue = IAQ_PURPLE;
      LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
      vTaskDelay(100);
      ret = zmod4xxx_cleaning_run(&zmod4410_dev);
      
      //vTaskDelay(2000);
      if (ret) {
          DEBUG_SENSOR("Error %d during cleaning procedure, exiting program!\r\n", ret);
          LED_Control_data.IAQvalue = IAQ_RED;
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
          vTaskDelay(100);
      }else{
        LED_Control_data.IAQvalue = IAQ_GREEN;
        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        vTaskDelay(100);
      }
    }
      
    }
    
    if (all_sensor_OK != 1){
      if (ModuleInitState.Si7021 != HAL_OK){
       all_sensor_OK = -1;
        for (i = 0; i < 1; i++){
          LED_Control_data.IAQvalue = IAQ_RED;
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
          vTaskDelay(500);
          LEDs_Effect(IAQ_LEDS, LED_OFF);
          vTaskDelay(500);
        }
        LED_Control_data.IAQvalue = IAQ_BLUE;
        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        vTaskDelay(500);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
      }else if (ModuleInitState.ZMOD4410 != HAL_OK){
        all_sensor_OK = -2;
        for (i = 0; i < 2; i++){
          LED_Control_data.IAQvalue = IAQ_RED;
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
          vTaskDelay(500);
          LEDs_Effect(IAQ_LEDS, LED_OFF);
          vTaskDelay(500);
        }
        LED_Control_data.IAQvalue = IAQ_BLUE;
        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        vTaskDelay(500);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
#ifdef SENSOR_SPS30
      }else if (ModuleInitState.SPS30 != HAL_OK){
        all_sensor_OK = -3;
        for (i = 0; i < 3; i++){
          LED_Control_data.IAQvalue = IAQ_RED;
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
          vTaskDelay(500);
          LEDs_Effect(IAQ_LEDS, LED_OFF);
          vTaskDelay(500);
        }
        LED_Control_data.IAQvalue = IAQ_BLUE;
        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        vTaskDelay(500);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
#endif
#ifdef SENSOR_SCD30
      }else if (ModuleInitState.SCD30 != HAL_OK){
        all_sensor_OK = -4;
        for (i = 0; i < 4; i++){
          LED_Control_data.IAQvalue = IAQ_RED;
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
          vTaskDelay(500);
          LEDs_Effect(IAQ_LEDS, LED_OFF);
          vTaskDelay(500);
        }
        LED_Control_data.IAQvalue = IAQ_BLUE;
        LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        vTaskDelay(500);
        LEDs_Effect(IAQ_LEDS, LED_OFF);
#endif
        
      }else{
        //LED_Control_data.IAQvalue = IAQ_GREEN;
        //LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        all_sensor_OK = 1;
      }
      DEBUG_MP_MODE("all_sensor_OK = %d\r\n", all_sensor_OK);
      vTaskDelay(900);
    }
    else if (all_sensor_OK == 1){
      
        if (IAQ_color_Rolling_Cnt > 10){
          DEBUG_MP_MODE("all_sensor_OK = %d\r\n", all_sensor_OK);
          IAQ_color_Rolling_Cnt = 0;
          if (LED_Control_data.IAQvalue >= IAQ_BLUE){
            LED_Control_data.IAQvalue = IAQ_GREEN;
          }else{
            LED_Control_data.IAQvalue++;
          }        
          LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
        }else{
          IAQ_color_Rolling_Cnt++;
        }
    }
    
  }
}


uint8_t Pre_EngMPmode(void){
  uint8_t res = 0;
  uint16_t iLoop = 0;
  uint8_t OutPeroid_PressCnt = 0;
  uint8_t InPeroid_PressCnt = 0;
  
  uint16_t Period_1 = 200;
  uint16_t Period_2 = 400;
  
  
  IAQ_LED_effect_isFade = false;
  IAQ_LED_effect_isBlink = false;
  IAQ_Color_Control(IAQ_GREEN, PWM_MAX*LED_Control_data.IAQ_Intensity_value/10, false);
      
  for (iLoop = 0; iLoop < 500; iLoop++){
    if (iLoop % 100 == 0){
      DEBUG_MP_MODE("Pre_EngMPmode cnt %d s\r\n", iLoop/100);
    }
    
    if (iLoop <= Period_1){
      if (HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_SET){
        OutPeroid_PressCnt++;
      }
      if (OutPeroid_PressCnt >= 10){
        
        break;
      }
    }else if(iLoop > Period_1 && iLoop <= Period_2){
      IAQ_Color_Control(IAQ_RED, PWM_MAX*LED_Control_data.IAQ_Intensity_value/10, false);
      if (HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_SET){
        InPeroid_PressCnt++;
      }
      if (InPeroid_PressCnt >= 10){
        DEBUG_MP_MODE("Pre_EngMPmode return 1 \r\n");
        return 1;
      }
    }else if (iLoop > Period_2){
      IAQ_Color_Control(IAQ_GREEN, PWM_MAX*LED_Control_data.IAQ_Intensity_value/10, false);
      break;
    }
    
    HAL_Delay(10);
  }
  
  DEBUG_MP_MODE("Pre_EngMPmode return 0 \r\n");
  
  return res;
}

void LEDs_Effect(uint8_t whichLED, uint8_t whatEffect){
  //whatEffect = LED_OFF;
  
//  if (OffAllLEDs_Output == 1){
//    whatEffect = LED_OFF;
//  }

  
  if (whatEffect != LED_OFF && EngMP_mode == 0){

      switch (WifiBT_Connect_Status){
      case BT_PAIRING:
        LED_Run_Freq = LED_RUN_FREQ_2p0HZ;
        if (whichLED == IAQ_LEDS){
          whatEffect = LED_OFF;
        }else{
          whatEffect = LED_EFFECT_BLINK;
        }
        break;
      case WIFI_CONNECT_OK:
        if (whichLED == IAQ_LEDS){
          #ifdef BROAN_BRNC021
            if (Switch_config_setting != 7){
              if (device_isAlgoActive() && (LED_Control_data.Btn1_OnOff == GPIO_PIN_SET || LED_Control_data.Btn2_OnOff == GPIO_PIN_SET)){
                
                LED_Run_Freq = LED_RUN_FREQ_0p5HZ;
                whatEffect = LED_EFFECT_FADE;
              }else{
                whatEffect = LED_ON_SOLID;
              }
            }else{
              if (LED_Control_data.Fan_AlgoApp_State_value == SW_Config7_RangeHoodSpeed && SW_Config7_RangeHoodSpeed != 0){
                whatEffect = LED_EFFECT_FADE;
              }else{
                whatEffect = LED_ON_SOLID;
              }
                
            }
          #elif defined(BROAN_BRNC011)
            whatEffect = LED_ON_SOLID;
          #endif
        }else{
          whatEffect = LED_ON_SOLID;
        }
        break;
      case WIFI_DISCONNECTED:        
        LED_Run_Freq = LED_RUN_FREQ_2p0HZ;
        if (whichLED == IAQ_LEDS){
            whatEffect = LED_EFFECT_BLINK;
        }else{
          whatEffect = LED_ON_SOLID;
        }
        break; 
      }
      
      
        
      if (LED_Control_data.Get_BLE_Enable_status_Value == 1){
        if (WifiBT_Connect_Status == WIFI_DISCONNECTED || WifiBT_Connect_Status == WIFI_CONNECT_OK){
          if (whichLED == BL_LED1){
            whatEffect = LED_OFF;
          }
          if (whichLED == BL_LED2){
            whatEffect = LED_OFF;
          }
        }
      }

      
//    }
  }
  

  
  if (whichLED == BL_LED1){
    if (whatEffect == LED_EFFECT_BLINK){
      BL_LED1_effect_isFade = false;
      BL_LED1_effect_isBlink = true;
    }else if (whatEffect == LED_EFFECT_FADE){
      BL_LED1_effect_isFade = true;
      BL_LED1_effect_isBlink = false;
      BL_LED1_PWM_value = BL_LED2_PWM_value;
      BL_LED1_PwmAdd = BL_LED2_PwmAdd;
    }else if (whatEffect == LED_OFF){
      BL_LED1_effect_isFade = false;
      BL_LED1_effect_isBlink = false;
      BL_LED1_PWM_value = 0;
    }else if (whatEffect == LED_ON_SOLID){
      BL_LED1_effect_isFade = false;
      BL_LED1_effect_isBlink = false;
      BL_LED1_PWM_value = PWM_MAX;
    }
  }
  if (whichLED == BL_LED2){ 
    if (whatEffect == LED_EFFECT_BLINK){
      BL_LED2_effect_isFade = false;
      BL_LED2_effect_isBlink = true;
    }else if (whatEffect == LED_EFFECT_FADE){
      BL_LED2_effect_isFade = true;
      BL_LED2_effect_isBlink = false;
      BL_LED2_PWM_value = BL_LED1_PWM_value;
      BL_LED2_PwmAdd = BL_LED1_PwmAdd;
    }else if (whatEffect == LED_OFF){
      BL_LED2_effect_isFade = false;
      BL_LED2_effect_isBlink = false;
      BL_LED2_PWM_value = 0;
    }else if (whatEffect == LED_ON_SOLID){
      BL_LED2_effect_isFade = false;
      BL_LED2_effect_isBlink = false;
      BL_LED2_PWM_value = PWM_MAX;
    }
  }
  
  if (whichLED == IAQ_LEDS){
    if (whatEffect == LED_EFFECT_FADE){
      IAQ_LED_effect_isFade = true;
      IAQ_LED_effect_isBlink = false;
//      IAQ_Color_Control(LED_Control_data.IAQvalue, 100);
    }else if (whatEffect == LED_EFFECT_BLINK){
      IAQ_LED_effect_isFade = false;
      IAQ_LED_effect_isBlink = true;
//      IAQ_Color_Control(LED_Control_data.IAQvalue, 100);
    }else if(whatEffect == IAQ_BLUE_BLINK_3X){
      IAQ_LED_effect_isFade = false;
      IAQ_LED_effect_isBlink = false;
      IAQ_Color_Control(IAQ_BLUE, 100, false);
      vTaskDelay(500);
      IAQ_Color_Control(IAQ_NONE, 0, false);
      vTaskDelay(500);
      IAQ_Color_Control(IAQ_BLUE, 100, false);
      vTaskDelay(500);
      IAQ_Color_Control(IAQ_NONE, 0, false);
      vTaskDelay(500);
      IAQ_Color_Control(IAQ_BLUE, 100, false);
      vTaskDelay(500);
      IAQ_Color_Control(IAQ_NONE, 0, false);
      vTaskDelay(500);  
    }else if (whatEffect == LED_OFF){
      IAQ_LED_effect_isFade = false;
      IAQ_LED_effect_isBlink = false;
      IAQ_LED_PWM_value = 0;
      IAQ_Color_Control(IAQ_NONE, 0, false);
    }else if (whatEffect == LED_ON_SOLID){
      IAQ_LED_effect_isFade = false;
      IAQ_LED_effect_isBlink = false;
      IAQ_Color_Control(LED_Control_data.IAQvalue, PWM_MAX*LED_Control_data.IAQ_Intensity_value/10, false);
    }
    
  }
  
  
}

void RangeHood_Speed_control(uint8_t whichButton){
  uint8_t i = 0;
  Clear_IWDG_Counter();
  if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
      vTaskDelay(300);
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      vTaskDelay(500);
  }else{
  
      HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
      vTaskDelay(300);
      HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      vTaskDelay(500);
  }
      
      if (whichButton == 1){
        if (SW_Config7_RangeHoodSpeed >= 3)
          SW_Config7_RangeHoodSpeed = 0;
        else
          SW_Config7_RangeHoodSpeed++;        
      }else if (whichButton == 2){
        if (SW_Config7_RangeHoodSpeed <= 0)
          SW_Config7_RangeHoodSpeed = 3;
        else
          SW_Config7_RangeHoodSpeed--;   
      }
      LED_Control_data.Fan_App_Status_Value = SW_Config7_RangeHoodSpeed;
      LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
      SW_Config_7_BtnLED_show(SW_Config7_RangeHoodSpeed);
      DEBUG_DRIVER("SW_Config7_RangeHoodSpeed = %d \r\n", SW_Config7_RangeHoodSpeed);
      for (i = 0; i < SW_Config7_RangeHoodSpeed; i++){
        Clear_IWDG_Counter();
        
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
          vTaskDelay(500);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
          vTaskDelay(1000);
        }else{
          HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
          vTaskDelay(500);
          HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
          vTaskDelay(1000);
        }
        DEBUG_DRIVER("loop time = %d \r\n", i);
      }
}

void WC_Single_Speed_control(uint8_t speed){
  Clear_IWDG_Counter();
  uint8_t i;
  if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
        vTaskDelay(100);
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
        vTaskDelay(300);
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
        vTaskDelay(500);
  }else{
    
        HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
        vTaskDelay(100);
        HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
        vTaskDelay(300);
        HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
        vTaskDelay(500);
  }
      
      if (speed > 3)
        speed = 3;
      
      SW_Config7_RangeHoodSpeed = speed;
       LED_Control_data.Fan_App_Status_Value  = SW_Config7_RangeHoodSpeed;
      SW_Config_7_BtnLED_show(SW_Config7_RangeHoodSpeed);
      
      for (i = 0; i < speed; i++){
        Clear_IWDG_Counter();
        if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
          vTaskDelay(500);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
          vTaskDelay(1000);
        }else{
          HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
          vTaskDelay(500);
          HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
          vTaskDelay(1000);
        }
      }


  
}

void SW_Config_7_BtnLED_show(uint8_t speed){
  
  switch (speed){
  case 0:
    LEDs_Effect(BL_LED1, LED_OFF);
    LEDs_Effect(BL_LED2, LED_OFF);
    break;
  case 1:
    LEDs_Effect(BL_LED1, LED_ON_SOLID);
    LEDs_Effect(BL_LED2, LED_OFF);
    break;
  case 2:
    LEDs_Effect(BL_LED1, LED_OFF);
    LEDs_Effect(BL_LED2, LED_ON_SOLID);
    break;
  case 3:
    LEDs_Effect(BL_LED1, LED_ON_SOLID);
    LEDs_Effect(BL_LED2, LED_ON_SOLID);
    break;
  };
  
  LEDs_Effect(IAQ_LEDS, LED_ON_SOLID);
}

void Latch_Relay_Control(uint8_t whichRelay, uint8_t OnOff){
  if (whichRelay == 1){
    if (OnOff == 1){
      HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
    }else{
      HAL_GPIO_WritePin(L_RELAY_ON_1_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_SET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(L_RELAY_OFF_1_GPIO_Port, L_RELAY_OFF_1_Pin, GPIO_PIN_RESET);
    }
  }else if (whichRelay == 2){
    if (OnOff == 1){
      HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
    }else{
      HAL_GPIO_WritePin(L_RELAY_ON_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_SET);
      vTaskDelay(100);
      HAL_GPIO_WritePin(L_RELAY_OFF_2_GPIO_Port, L_RELAY_OFF_2_Pin, GPIO_PIN_RESET);
    }
  }
}

#ifdef BRNC021_TRIAC_CONTROL
/**
  * @brief  Function control Triac
  * @param  argument: Traic_Select  : 1  triac1  
                                      2  triac2 
                      Voltage_Level : 0  OFF
                                      1  
                                      2   
                                      3 
                      Triac_Set     : 1  
                                    :2  TRIAC
                      
  * @retval None
  */

void  traic_control(uint8_t Traic_Select, uint8_t Voltage_Level,uint8_t Triac_Set)
{
  if(Triac_Set==1)
  {
    
    switch (Traic_Select)
    {
    case 1:
      switch (Voltage_Level)
      {
      case 0:
        
//        DEBUG_DRIVER("Blue LED1 off!\r\n");
        Relay_Delay_Time = 0;
        Triac_Status1= 0;
        if(Triac_Status2==0)
        {
          Traic_Relay_Control(GPIO_PIN_RESET);
//          HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
           Relay_status = 0;
        }
        Triac_Time1=200;
        HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_RESET);
        break;
      case 1:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status1=1;
        Triac_Time1=71;//30V
        break;
      case 2:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status1=1;
        Triac_Time1=60;//60V
        break;
      case 3:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status1=1;
        Triac_Time1=47;//90V
        break;
      case 4:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status1=1;
#ifdef TRIAC_TIMEING_TURNING
        if (Triac_time_Value1 > 255)
          Triac_time_Value1 = 0;
        else
          Triac_time_Value1++;
        
        Triac_Time1 = Triac_time_Value1;
        DEBUG_DRIVER("****************************Triac 1 Time Value = %d ***********************\r\n", Triac_Time1);
#else
        Triac_Time1=1;
#endif
        break;
      default : break;
      } 
     
      break;
    case 2:
      switch (Voltage_Level)
      {
      case 0:
        
//        DEBUG_DRIVER("Blue LED1 off!\r\n");
        Relay_Delay_Time = 0;
        Triac_Status2= 0;
        if(Triac_Status1==0)
        {
          Traic_Relay_Control(GPIO_PIN_RESET);
//          HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
           Relay_status = 0;
        }
        Triac_Time2=200;
        HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
        break;
      case 1:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status2=1;
        Triac_Time2=71;//30V
        break;
      case 2:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status2=1;
        Triac_Time2=60;//60V
        break;
      case 3:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status2=1;
        Triac_Time2=47;//90V
        break;
      case 4:
        Traic_Relay_Control(GPIO_PIN_SET);
//        HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
//        DEBUG_DRIVER("Blue LED1 on!\r\n");
        Relay_status = 1;
        Triac_Status2=1;
#ifdef TRIAC_TIMEING_TURNING
        if (Triac_time_Value2 > 255)
          Triac_time_Value2 = 0;
        else
          Triac_time_Value2++;
        
        Triac_Time2 = Triac_time_Value2;
        DEBUG_DRIVER("****************************Triac 2 Time Value = %d ***********************\r\n" , Triac_Time2);
#else
        Triac_Time2=1;
#endif
        break;
      default : break;
      }
      break;
    default :break;   
    }
  }
  else if(Triac_Set==2)
  {  
    if(Traic_Time_Count<=1)
     {
       HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
     }
     Traic_Time_Count++;
     
         if(Triac_Status1==1)
         {
             if((Traic_Time_Count>=Triac_Time1)&&(Traic_Time_Count<(Triac_Time1+170)))
             {
//                Traic_Time_Count=100;//Triac_Time1+1;
                HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_SET);   
             }
             else
             {
                HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_RESET);
             }    
         }
         if(Triac_Status2==1)
         {
             if((Traic_Time_Count>=Triac_Time2)&&(Traic_Time_Count<(Triac_Time2+170)))
             {
//                Traic_Time_Count=100;//Triac_Time2+1;
                HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET);
             }
             else
             {
                HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET);
             }
         }
  }

}


void Traic_Relay_Control(GPIO_PinState GPIO_PinSet){
  if (Switch_config_setting != 7)
    HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PinSet);
}

void SW_Config_7_RangeHood_Control(uint8_t whichButton){
                Clear_IWDG_Counter();
                HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET); 
                HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_RESET);  
                vTaskDelay(50);
                HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
                vTaskDelay(150);
                HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_SET); 
                vTaskDelay(300);
                HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, L_RELAY_OFF_2_PIn, GPIO_PIN_RESET); 
                vTaskDelay(500);
                if (SW_Config7_RangeHoodSpeed >= 3 || whichButton == 1)
                  SW_Config7_RangeHoodSpeed = 0;
                else
                  SW_Config7_RangeHoodSpeed++;
                for (uint8_t iloop = 0; iloop < SW_Config7_RangeHoodSpeed; iloop++){
                  Clear_IWDG_Counter();
                  HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_SET); 
                  vTaskDelay(500);
                  HAL_GPIO_WritePin(RELAY_GPIO_Port, L_RELAY_ON_1_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, L_RELAY_ON_2_Pin, GPIO_PIN_RESET); 
                  vTaskDelay(1000);
                }
                
}

 #endif
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
//  static bool WifiIsConnect_IAQ_Blink_Blue = false;
  static uint8_t  Count_T=0;
  static uint32_t Fade_status = 0;
  static uint32_t WifiDisMode_Fade_status = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  static uint32_t Count_0p5s = 0;
  static uint32_t Count_1s = 0;
  static uint8_t  Count_PWM_BL_LED1=0;
  static uint8_t  Count_PWM_BL_LED2=0;
  static uint32_t Count_3s = 0;
  static uint32_t PWM_Count_T = 0;      //10ms

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    Count_T++;
    Count_0p5s++;
    Count_1s++;
    Count_3s++;
    Count_PWM_BL_LED1++;
    Count_PWM_BL_LED2++;
    
    if (Count_PWM_BL_LED1 >= PWM_MAX && BL_LED1_PWM_value != 0){
      Count_PWM_BL_LED1 = 0;
      HAL_GPIO_WritePin(BL_LED_1_GPIO_Port, BL_LED_1_Pin, GPIO_PIN_SET);
    }else if (BL_LED1_PWM_value == 0 ||  Count_PWM_BL_LED1 >= BL_LED1_PWM_value){
      HAL_GPIO_WritePin(BL_LED_1_GPIO_Port, BL_LED_1_Pin, GPIO_PIN_RESET);
    }
    if (Count_PWM_BL_LED2 >= PWM_MAX && BL_LED2_PWM_value != 0){
      Count_PWM_BL_LED2 = 0;
      HAL_GPIO_WritePin(BL_LED_2_GPIO_Port, BL_LED_2_Pin, GPIO_PIN_SET);
    }else if (BL_LED2_PWM_value == 0 ||  Count_PWM_BL_LED2 >= BL_LED2_PWM_value){
      HAL_GPIO_WritePin(BL_LED_2_GPIO_Port, BL_LED_2_Pin, GPIO_PIN_RESET);
    }
    
 
      if (LED_Indicate_Cnt < LED_Run_Freq){
        if (BL_LED1_effect_isBlink || BL_LED2_effect_isBlink || IAQ_LED_effect_isBlink){
          LED_Indicate_Cnt += 1;
        }else{
          LED_Indicate_Cnt += 100;
        }
      }else{
        LED_Indicate_Cnt = 0;
        if (BL_LED1_effect_isFade){
          if (BL_LED1_PwmAdd){
            if (BL_LED1_PWM_value < PWM_MAX){
              BL_LED1_PWM_value++;
            }else{
              BL_LED1_PwmAdd = !(BL_LED1_PwmAdd);
            }
          }else{
            if (BL_LED1_PWM_value > 0){
              BL_LED1_PWM_value--;
            }else{
              BL_LED1_PwmAdd = !(BL_LED1_PwmAdd);
            }
          }
        }else if(BL_LED1_effect_isBlink){
          if (BL_LED1_PWM_value == PWM_MAX)
            BL_LED1_PWM_value = 0;
          else
            BL_LED1_PWM_value = PWM_MAX;
        }
        
        if (BL_LED2_effect_isFade){
          if (BL_LED2_PwmAdd){
            if (BL_LED2_PWM_value < PWM_MAX){
              BL_LED2_PWM_value++;
            }else{
              BL_LED2_PwmAdd = !(BL_LED2_PwmAdd);
            }
          }else{
            if (BL_LED2_PWM_value > 0){
              BL_LED2_PWM_value--;
            }else{
              BL_LED2_PwmAdd = !(BL_LED2_PwmAdd);
            }
          }
        }else if(BL_LED2_effect_isBlink){
          if (WifiBT_Connect_Status == BT_PAIRING){
            if (BL_LED1_PWM_value >= PWM_MAX)
              BL_LED2_PWM_value = 0;
            else if (BL_LED1_PWM_value == 0)
              BL_LED2_PWM_value = PWM_MAX;
          }else{
            if (BL_LED2_PWM_value >= PWM_MAX)
              BL_LED2_PWM_value = 0;
            else
              BL_LED2_PWM_value = PWM_MAX;
          }
        }
        /*
        if (IAQ_LED_effect_isFade){
          
          if (IAQ_LED_PwmAdd){
            if (IAQ_LED_PWM_value < PWM_MAX){
              if (PWM_Count_T >= (10-(LED_Control_data.IAQ_Intensity_value-1))){
                PWM_Count_T = 0;
                IAQ_LED_PWM_value++;
                //IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value*LED_Control_data.IAQ_Intensity_value/10, true);
                IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value, true);//LED_Control_data.IAQ_Intensity_value/10, true);
              }
            }else{
              IAQ_LED_PwmAdd = !(IAQ_LED_PwmAdd);
            }
          }else{
            if (IAQ_LED_PWM_value > 0){
              if (PWM_Count_T >= (10-(LED_Control_data.IAQ_Intensity_value-1))){
                PWM_Count_T = 0;
                IAQ_LED_PWM_value--;
                //IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value*LED_Control_data.IAQ_Intensity_value/10, true);
                IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value, true);//LED_Control_data.IAQ_Intensity_value/10, true);
              }
            }else{
              IAQ_LED_PwmAdd = !(IAQ_LED_PwmAdd);
            }
          }
          
        }
        else if(IAQ_LED_effect_isBlink){
            if (IAQ_LED_PWM_value >= (PWM_MAX)){
              IAQ_LED_PWM_value = 0;
              IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value*LED_Control_data.IAQ_Intensity_value/10, false);
              
            }else{
              
              IAQ_LED_PWM_value = (PWM_MAX);
              
              if (WifiBT_Connect_Status == WIFI_DISCONNECTED){
                if (WifiIsConnect_IAQ_Blink_Blue){
                  WifiIsConnect_IAQ_Blink_Blue = false;
                  IAQ_Color_Control(IAQ_BLUE, IAQ_LED_PWM_value*LED_Control_data.IAQ_Intensity_value/10, false);
                }else{
                  WifiIsConnect_IAQ_Blink_Blue = true;
                  IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value*LED_Control_data.IAQ_Intensity_value/10, false);
                }
              }else{
                IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value*LED_Control_data.IAQ_Intensity_value/10, false);
              }
            }

        }
       */
      }
    
    if (Count_0p5s >= 5000){
      Count_0p5s = 0;
      UART1_reinit_Cnt++;
    }
    
    if (Count_1s >= 10000){
      Count_1s = 0;
      if (just_StartUP_canSend == true){
        just_StartUP_tick++;
        if (just_StartUP_tick >= 10){
          ESP_Cmd_Send_En = 1;
//          vTaskDelay(50);
        }
        //printf("just_StartUP_tick = %d\r\n", just_StartUP_tick);
      }
#ifdef DEBUG_SENSOR_
      PrintIAQrate_OneSecTick = true;
#endif
    }
    
    if (Count_3s >= 50000){
      Count_3s = 0;
//      SCD30_TwoSecTick = true;
//      Si7021_TwoSecTick = true;
//      SPS30_TwoSecTick = true;
//      ZMOD4410_TwoSecTick = true;

    }
    
    if(Count_T>=10)     //10 = 1ms
    {
      Count_T=0;
      PWM_Count_T++;
      Fade_status++;
      WifiDisMode_Fade_status++;
      HAL_IncTick();      
    }
    
    if(IAQ_LED_effect_isBlink == true && WifiBT_Connect_Status == WIFI_DISCONNECTED){
      
      switch (WifiDisMode_Fade_status){
        case	OFF9_IAQ_FADE_UP_1	:
          IAQ_LED_PWM_value = 1;
          break;
        case	OFF9_IAQ_FADE_UP_2	:
          IAQ_LED_PWM_value = 2;
          break;
        case	OFF9_IAQ_FADE_UP_3	:
          IAQ_LED_PWM_value = 3;
          break;
        case	OFF9_IAQ_FADE_UP_4	:
          IAQ_LED_PWM_value = 4;
          break;
        case	OFF9_IAQ_FADE_UP_5	:
          IAQ_LED_PWM_value = 5;
          break;
        case	OFF9_IAQ_FADE_UP_6	:
          IAQ_LED_PWM_value = 6;
          break;
        case	OFF9_IAQ_FADE_UP_7	:
          IAQ_LED_PWM_value = 7;
          break;
        case	OFF9_IAQ_FADE_UP_8	:
          IAQ_LED_PWM_value = 8;
          break;
        case	OFF9_IAQ_FADE_UP_9	:
          IAQ_LED_PWM_value = 9;
          break;
        case	OFF9_IAQ_FADE_UP_10	:
          IAQ_LED_PWM_value = 10;
          break;
        case	OFF9_IAQ_FADE_STABLE	:
          IAQ_LED_PWM_value = 10;
          break;
        case	OFF9_IAQ_FADE_DN_10	:
          IAQ_LED_PWM_value = 10;
          break;
        case	OFF9_IAQ_FADE_DN_9	:
          IAQ_LED_PWM_value = 9;
          break;
        case	OFF9_IAQ_FADE_DN_8	:
          IAQ_LED_PWM_value = 8;
          break;
        case	OFF9_IAQ_FADE_DN_7	:
          IAQ_LED_PWM_value = 7;
          break;
        case	OFF9_IAQ_FADE_DN_6	:
          IAQ_LED_PWM_value = 6;
          break;
        case	OFF9_IAQ_FADE_DN_5	:
          IAQ_LED_PWM_value = 5;
          break;
        case	OFF9_IAQ_FADE_DN_4	:
          IAQ_LED_PWM_value = 4;
          break;
        case	OFF9_IAQ_FADE_DN_3	:
          IAQ_LED_PWM_value = 3;
          break;
        case	OFF9_IAQ_FADE_DN_2	:
          IAQ_LED_PWM_value = 2;
          break;
        case	OFF9_IAQ_FADE_DN_1	:
          IAQ_LED_PWM_value = 1;
          break;
        case	OFF9_BLUE_FADE_UP_1	:
          IAQ_LED_PWM_value = 1;
          break;
        case	OFF9_BLUE_FADE_UP_2	:
          IAQ_LED_PWM_value = 2;
          break;
        case	OFF9_BLUE_FADE_UP_3	:
          IAQ_LED_PWM_value = 3;
          break;
        case	OFF9_BLUE_FADE_UP_4	:
          IAQ_LED_PWM_value = 4;
          break;
        case	OFF9_BLUE_FADE_UP_5	:
          IAQ_LED_PWM_value = 5;
          break;
        case	OFF9_BLUE_FADE_UP_6	:
          IAQ_LED_PWM_value = 6;
          break;
        case	OFF9_BLUE_FADE_UP_7	:
          IAQ_LED_PWM_value = 7;
          break;
        case	OFF9_BLUE_FADE_UP_8	:
          IAQ_LED_PWM_value = 8;
          break;
        case	OFF9_BLUE_FADE_UP_9	:
          IAQ_LED_PWM_value = 9;
          break;
        case	OFF9_BLUE_FADE_UP_10	:
          IAQ_LED_PWM_value = 10;
          break;
        case	OFF9_BLUE_FADE_STABLE	:
          IAQ_LED_PWM_value = 10;
          break;
        case	OFF9_BLUE_FADE_DN_10	:
          IAQ_LED_PWM_value = 10;
          break;
        case	OFF9_BLUE_FADE_DN_9	:
          IAQ_LED_PWM_value = 9;
          break;
        case	OFF9_BLUE_FADE_DN_8	:
          IAQ_LED_PWM_value = 8;
          break;
        case	OFF9_BLUE_FADE_DN_7	:
          IAQ_LED_PWM_value = 7;
          break;
        case	OFF9_BLUE_FADE_DN_6	:
          IAQ_LED_PWM_value = 6;
          break;
        case	OFF9_BLUE_FADE_DN_5	:
          IAQ_LED_PWM_value = 5;
          break;
        case	OFF9_BLUE_FADE_DN_4	:
          IAQ_LED_PWM_value = 4;
          break;
        case	OFF9_BLUE_FADE_DN_3	:
          IAQ_LED_PWM_value = 3;
          break;
        case	OFF9_BLUE_FADE_DN_2	:
          IAQ_LED_PWM_value = 2;
          break;
        case	OFF9_BLUE_FADE_DN_1	:
          IAQ_LED_PWM_value = 1;
          break;

      };
      

      
      
      if (WifiDisMode_Fade_status >= OFF9_IAQ_FADE_UP_1 && WifiDisMode_Fade_status <= OFF9_IAQ_FADE_DN_1){
        IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value, true);
      }else if (WifiDisMode_Fade_status >= OFF9_BLUE_FADE_UP_1 && WifiDisMode_Fade_status <= OFF9_BLUE_FADE_DN_1){
        IAQ_Color_Control(IAQ_BLUE, IAQ_LED_PWM_value, true);
      }
      
      if (WifiDisMode_Fade_status >= OFF9_BLUE_FADE_DN_1){
        WifiDisMode_Fade_status = 0;
      }
      
    }
    
    if (IAQ_LED_effect_isFade){
   
      
        switch (Fade_status){
          case	ON9_FADE_UP_1	:
            IAQ_LED_PWM_value = 1;// LED_Control_data.IAQ_Intensity_value * 1;
            break;
          case	ON9_FADE_UP_2	:
            IAQ_LED_PWM_value = 2;//LED_Control_data.IAQ_Intensity_value * 2;
            break;
          case	ON9_FADE_UP_3	:
            IAQ_LED_PWM_value = 3;//LED_Control_data.IAQ_Intensity_value * 3;
            break;
          case	ON9_FADE_UP_4	:
            IAQ_LED_PWM_value = 4;//LED_Control_data.IAQ_Intensity_value * 4;
            break;
          case	ON9_FADE_UP_5	:
            IAQ_LED_PWM_value = 5;//LED_Control_data.IAQ_Intensity_value * 5;
            break;
          case	ON9_FADE_UP_6	:
            IAQ_LED_PWM_value = 6;//LED_Control_data.IAQ_Intensity_value * 6;
            break;
          case	ON9_FADE_UP_7	:
            IAQ_LED_PWM_value = 7;//LED_Control_data.IAQ_Intensity_value * 7;
            break;
          case	ON9_FADE_UP_8	:
            IAQ_LED_PWM_value = 8;//LED_Control_data.IAQ_Intensity_value * 8;
            break;
          case	ON9_FADE_UP_9	:
            IAQ_LED_PWM_value = 9;//LED_Control_data.IAQ_Intensity_value * 9;
            break;
          case	ON9_FADE_UP_10	:
            IAQ_LED_PWM_value = 10;//LED_Control_data.IAQ_Intensity_value * 10;
            break;
          case	ON9_FADE_STABLE :
            IAQ_LED_PWM_value = 10;//LED_Control_data.IAQ_Intensity_value * 10;
            break;
          case	ON9_FADE_DN_1	:
            IAQ_LED_PWM_value = 10;//LED_Control_data.IAQ_Intensity_value * 10;
            break;
          case	ON9_FADE_DN_2	:
            IAQ_LED_PWM_value = 9;//LED_Control_data.IAQ_Intensity_value * 9;
            break;
          case	ON9_FADE_DN_3	:
            IAQ_LED_PWM_value = 8;//LED_Control_data.IAQ_Intensity_value * 8;
            break;
          case	ON9_FADE_DN_4	:
            IAQ_LED_PWM_value = 7;//LED_Control_data.IAQ_Intensity_value * 7;
            break;
          case	ON9_FADE_DN_5	:
            IAQ_LED_PWM_value = 6;//LED_Control_data.IAQ_Intensity_value * 6;
            break;
          case	ON9_FADE_DN_6	:
            IAQ_LED_PWM_value = 5;//LED_Control_data.IAQ_Intensity_value * 5;
            break;
          case	ON9_FADE_DN_7	:
            IAQ_LED_PWM_value = 4;//LED_Control_data.IAQ_Intensity_value * 4;
            break;
          case	ON9_FADE_DN_8	:
            IAQ_LED_PWM_value = 3;//LED_Control_data.IAQ_Intensity_value * 3;
            break;
          case	ON9_FADE_DN_9	:
            IAQ_LED_PWM_value = 2;//LED_Control_data.IAQ_Intensity_value * 2;
            break;
          case	ON9_FADE_DN_10	:
            IAQ_LED_PWM_value = 1;//LED_Control_data.IAQ_Intensity_value * 1;
            break;

          
        };
        
        if (Fade_status >= ON9_FADE_DN_10){
          Fade_status = 0; 
        }
        
        IAQ_Color_Control(LED_Control_data.IAQvalue, IAQ_LED_PWM_value, true);//*LED_Control_data.IAQ_Intensity_value/10, true);
    }
    

#ifdef BRNC021_TRIAC_CONTROL
    if(Switch_config_setting != 7)
//      traic_control(0,0,2);   

#endif
  }

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


void IAQ_Color_Control(uint8_t ColorValue, uint8_t brightness, bool isFade){
  
  //LED_Control_data.IAQvalue = ColorValue;
  uint32_t RealControlData;
  //uint32_t Yellow_RealControlData;
  if (brightness > PWM_MAX){
    brightness = PWM_MAX;
  }
#ifdef BROAN_BRNC021
  if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
    if (isFade == false){
      if (brightness == 0){
        RealControlData = 0;
      }else{
        brightness = brightness/10; //back to 1 to 10 level
        RealControlData =((brightness-1)*27+11) *16; // 10 level change to 255 lv:  (brightness-1) * 27 + 10  then -->255 lv change to 4096 lv
            
        if (RealControlData > 4095){
          RealControlData = 4016;   // is MAX LED brightness value = 250
        }
      }

      
    }else{
      //LED_Control_data.IAQ_Intensity_value;
//      if (brightness >= LED_Control_data.IAQ_Intensity_value * 10){              //check brighness > MAX instensity or not
//          brightness = LED_Control_data.IAQ_Intensity_value * 10;
//      }
      
        RealControlData = (18 + (43 * (LED_Control_data.IAQ_Intensity_value - 1))) * brightness;
            //  lv      each setup      @ lv 10        
            //  1	18	        180
            //  2	61	        610
            //  3	104	        1040
            //  4	147	        1470
            //  5	190	        1900
            //  6	233	        2330
            //  7	276	        2760
            //  8	319	        3190
            //  9	362	        3620
            //  10	405	        4050
  

      
    }
    
//	10 lv	LED brghtness value	PWM control value {= (LED brghtness value + 1) * 16}
//	1	10                      176
//	2	37                      608
//	3	64                      1040
//	4	91                      1472
//	5	118                      1904
//	6	145                      2336
//	7	172                      2768
//	8	199                      3200
//	9	226                      3632
//	10	250                      4016
    



    
      switch (ColorValue){
      case IAQ_GREEN:   //green
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = RealControlData; //2900 * brightness / PWM_MAX;     //Green
        break;
      case IAQ_BLUE:   //
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = RealControlData; //1620 * brightness / PWM_MAX;     //blue
            TIM3->CCR4 = 0;     //Green
        break;
      case IAQ_YELLOW:   //yellow
            TIM3->CCR2 = RealControlData; //2900 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = RealControlData * 3 / 11; //790 * brightness / PWM_MAX;     //Green
        break;
      case IAQ_ORANGE:   //orange
            TIM3->CCR2 = RealControlData; //4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = RealControlData / 4; //1040 * brightness / PWM_MAX;     //Green
        break;
      case IAQ_RED:   //Red
            TIM3->CCR2 = RealControlData; //2900 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = 0;     //Green
        break;
      case IAQ_PURPLE:
            TIM3->CCR2 = RealControlData; //4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = RealControlData * 3 / 11; //3350 * brightness / PWM_MAX;     //Blue
            TIM3->CCR4 = 0;     //Green
        break;
      case IAQ_NONE:
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = 0;     //Green
        break;
      };
  }else{
      switch (ColorValue){
      case IAQ_GREEN:   //green
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 4095 * brightness / PWM_MAX;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_BLUE:   //blue
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = 4095 * brightness / PWM_MAX;     //blue
        break;
      case IAQ_YELLOW:   //yellow
            TIM3->CCR2 = 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = Yello_IAQ_Red_value * brightness / PWM_MAX;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_ORANGE:   //orange
            TIM3->CCR2 = 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 1040 * brightness / PWM_MAX;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_RED:   //Red
            TIM3->CCR2 = 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_PURPLE:
            TIM3->CCR2 = 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = Yello_IAQ_Red_value * brightness / PWM_MAX;     //Green
        break;
      case IAQ_NONE:
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      };
  }

#endif

#ifdef BROAN_BRNC011
  if (isFade == false){
    RealControlData = 4095 * brightness / PWM_MAX;
  }else{
    RealControlData = (18 + (43 * (LED_Control_data.IAQ_Intensity_value - 1))) * brightness;
  }
  
      switch (ColorValue){
      case IAQ_GREEN:   //green
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = RealControlData;// 4095 * brightness / PWM_MAX;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_BLUE:   //blue
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = RealControlData;// 4095 * brightness / PWM_MAX;     //blue
        break;
      case IAQ_YELLOW:   //yellow
            TIM3->CCR2 = RealControlData;// 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = RealControlData * 7 / 17;      //8 /13; //Yello_IAQ_Red_value * brightness / PWM_MAX;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_ORANGE:   //orange
            TIM3->CCR2 = RealControlData;// 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = RealControlData / 4;// 1040 * brightness / PWM_MAX;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_RED:   //Red
            TIM3->CCR2 = RealControlData;// 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      case IAQ_PURPLE:
            TIM3->CCR2 = RealControlData;// 4095 * brightness / PWM_MAX;     //Red
            TIM3->CCR3 = 0;     //Blue
            TIM3->CCR4 = RealControlData * 8 / 13;// Yello_IAQ_Red_value * brightness / PWM_MAX;     //Green
        break;
      case IAQ_NONE:
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = 0;     //Blue
        break;
      };
#endif
      IAQ_LED_currColor = ColorValue;
}




/**
  * @brief  This function is executed in case of error occurrence.d
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


//Status Machine Function
//#define USE_ZMOD_EXIT
#ifdef SENSOR_ZMOD4410
void GetTVOC_data(void){
  uint8_t ret, i;
  
  float TVOC_MarkF = 0;
  float PrintTVOC_32bit, Print_IAQ_32bit;
  i = 0;
  if (ModuleInitState.ZMOD4410 == HAL_OK){
//nyc          ret = cont_run(&zmod4410_dev, &SensorsOutRawData.ZMOD4410outRawData.TVOC);
//          ret = zmod4410_task();
#ifdef USE_ZMOD_EXIT
	if(zmod4410_EN==1&&zmod4410_RDY==1){
        //if(zmod4410_EN==1&&HAL_GPIO_ReadPin(ZMOD_INT_GPIO_Port,ZMOD_INT_Pin) ==0){
          DEBUG_SENSOR("zmod4410_RDY = 1, ZMOD4410 get reading..............................");
#else
	if(zmod4410_EN==1){
#endif
//          zmod4410_RDY=0;
          
//          HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); //nyc test
          Clear_IWDG_Counter();//Clear IWDG Counter
          ret = cont_run(&zmod4410_dev, &SensorsOutRawData.ZMOD4410outRawData.TVOC, &SensorsOutRawData.ZMOD4410outRawData.r_mox, &SensorsOutRawData.ZMOD4410outRawData.IAQ, &SensorsOutRawData.ZMOD4410outRawData.eCO2);
          DEBUG_SENSOR("ZMOD4410 ret = %d, cnt = %d\r\n", ret, i);
          if(ret) {
//            
//            DEBUG_SENSOR("zmod4410 cont_run Error %d\n", ret);
//
//            //zmod4410_deinit();
//            return;
//            
            for (i = 1; i < I2C_retryCnt; i++){
              ret = cont_run(&zmod4410_dev, &SensorsOutRawData.ZMOD4410outRawData.TVOC, &SensorsOutRawData.ZMOD4410outRawData.r_mox, &SensorsOutRawData.ZMOD4410outRawData.IAQ, &SensorsOutRawData.ZMOD4410outRawData.eCO2);
              DEBUG_SENSOR("ZMOD4410 ret = %d, cnt = %d\r\n", ret, i);
              if (ret == 0) {
                break;
              }
            }
            if (i >= I2C_retryCnt){
              DEBUG_SENSOR("ZMOD4410 error reading measurement, I2C2 Reinit...\r\n");
              I2C2_rstCnt_TVOC++;
              Reinit_I2C2();

              
            }
            
            
            
          }
          
       
          if (i < I2C_retryCnt){
            I2C2_rstCnt_TVOC = 0;
            I2C2_OkCnt_TVOC++;
            memcpy(&PrintTVOC_32bit, &SensorsOutRawData.ZMOD4410outRawData.TVOC, 4);
            memcpy(&Print_IAQ_32bit, &SensorsOutRawData.ZMOD4410outRawData.IAQ, 4);
            
            if( PrintTVOC_32bit>10 ){ // 81-100
                    TVOC_MarkF=((80.0f-40.0f)/(10.0f-1.0f))*(PrintTVOC_32bit-10.0f)+80.0f;
            }else if( PrintTVOC_32bit>1 ){ // 41-80
                    TVOC_MarkF=((80.0f-40.0f)/(10.0f-1.0f))*(PrintTVOC_32bit-10.0f)+80.0f;
            }else{ // 0-40
                    TVOC_MarkF=((40.0f-0.0f)/(1.0f-0.0f))*(PrintTVOC_32bit-1.0f)+40.0f;
            }
            if (TVOC_MarkF>100){
                    TVOC_MarkF=100;	   
            }
            if (TVOC_MarkF<0){
                    TVOC_MarkF=0;	   
            }
            TVOC_Mark = (uint8_t)TVOC_MarkF;
            
            DEBUG_SENSOR("PrintTVOC_32bit = %f\r\n", PrintTVOC_32bit);
          }
        }
   
  }
        
//  CurrStatus = STATE_GETSI7021;
}


#endif




#ifdef SENSOR_SCD30
uint8_t InitSCD30(void){
    uint8_t ret = 0;
    uint16_t interval_in_seconds = 2;
    
    while (scd30_probe() != STATUS_OK) {
        DEBUG_DRIVER("SCD30 sensor probing failed\r\n");
        sensirion_sleep_usec(1000000u);
        ret++;
        if (ret == 5){
          return HAL_ERROR;
        }
    }
    
    scd30_set_measurement_interval(interval_in_seconds);
    sensirion_sleep_usec(20000u);
    scd30_enable_automatic_self_calibration(1);
    //scd30_set_forced_recalibration(700);
    
    scd30_start_periodic_measurement(0);
    sensirion_sleep_usec(interval_in_seconds * 1000000u);
    
    DEBUG_DRIVER("SCD30 sensor probing successful\r\n");

    //enable external interrupt
    /*Configure GPIO pin : SCD_RDY_Pin */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = SCD_RDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SCD_RDY_GPIO_Port, &GPIO_InitStruct);
    if(HAL_GPIO_ReadPin(SCD_RDY_GPIO_Port, SCD_RDY_Pin))
       SCD_RDY=1;
    return HAL_OK;
}

void GetSCD30_data(void){
  float co2_ppm, temperature, relative_humidity;//, tempData;
  float CO2_MarkF = 0;
  uint8_t i;
  uint8_t ret;
  i = 0;
#ifdef USE_SCD_EXIT
  if (ModuleInitState.SCD30 == HAL_OK && UART_Transfering == 0 && SCD_RDY == 1){
#else
    if (ModuleInitState.SCD30 == HAL_OK){// && SCD30_TwoSecTick == true){// && UART_Transfering == 0){
#endif
      
//      SCD30_TwoSecTick = false;
    /* Measure co2, temperature and relative humidity and store into
         * variables.
         */
//        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); //nyc test
        SCD_RDY = 0;
        if (SCD30_FRC_Value > 0){
          scd30_set_forced_recalibration(SCD30_FRC_Value);
          SCD30_FRC_Value = 0;
          
        }
        
        
        ret = scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
//        I2CDirectReturnErrCnt++;
        
//        DEBUG_SENSOR("SCD30 ret = %d, cnt = %d, I2CDirectReturnErrCnt = %d\r\n", ret, i, I2CDirectReturnErrCnt);
        if (ret != STATUS_OK) {
          for (i = 1; i < I2C_retryCnt; i++){
            ret = scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
            DEBUG_SENSOR("SCD30 ret = %d, cnt = %d\r\n", ret, i);
            if (ret == STATUS_OK) {
              break;
            }
          }
          if (i >= I2C_retryCnt){
            DEBUG_SENSOR("SCD30 error reading measurement, I2C1 Reinit...\r\n");
            I2C1_rstCnt_CO2++;
            Reinit_I2C1(true);
//            InitSCD30();
//            HAL_Delay(10);
            
          }

        } 
        if (i < I2C_retryCnt){
          I2C1_rstCnt_CO2 = 0;
          I2C1_OkCnt_CO2++;
          
          //memcpy(&tempData, &co2_ppm, 4);
          

          
          
          //DEBUG_DRIVER("tempData : %X\r\n", tempData);
//          if (tempData < -1){

          if (co2_ppm > 0 && co2_ppm != isnan(NAN)){
            memcpy(&SensorsOutRawData.SCD30outRawData.co2, &co2_ppm, 4);
          
          
          
//            memcpy(&SensorsOutRawData.SCD30outRawData.temperature, &temperature, 4);
//            memcpy(&SensorsOutRawData.SCD30outRawData.humidity, &relative_humidity, 4);
                DEBUG_SENSOR("SCD30:---co2 : %0.2f ppm,\r\n"
//                             "temperature: %0.2f degreeCelsius, "
//                             "humidity: %0.2f %%RH\r\n",
                             ,co2_ppm);
              
              if (co2_ppm > 2000){
                CO2_MarkF = ((80.0f-60.0f)/(2000.0f-1000.0f))*(co2_ppm-2000.0f)+80.0f;
              }else if (co2_ppm > 1000){
                CO2_MarkF = ((80.0f-40.0f)/(2000.0f-1000.0f))*(co2_ppm-2000.0f)+80.0f;
              }else{
                CO2_MarkF = ((40.0f-0.0f)/(1000.0f-0.0f))*(co2_ppm-1000.0f)+40.0f;
              }
              
              if (CO2_MarkF > 100)
                CO2_MarkF = 100;
              if (CO2_MarkF < 0)
                CO2_MarkF = 0;
              CO2_Mark = (uint8_t)CO2_MarkF;          
          
          }
//          }
        }
        
//        if (I2CDirectReturnErrCnt >= 20){
//          TriggerI2CErrCntNo++;
//          DEBUG_DRIVER("I2C erro Trigger Time = %llu\r\n", TriggerI2CErrCntNo);
//        }
        
        //vTaskDelay(SPS30_MEASUREMENT_DURATION_USEC/portTICK_PERIOD_MS);
        //sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC);
  
  }
  
//  CurrStatus = STATE_GETSPS30;
}

void SCD30_deinit(void){
  DEBUG_SENSOR("SCD30_deinit\n");
  HAL_GPIO_DeInit(SCD_RDY_GPIO_Port, SCD_RDY_Pin);

  /*Disable SCD_RDY_Pin External Interrupt */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SCD_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(SCD_RDY_GPIO_Port, &GPIO_InitStruct);
  SCD30_EN=0;
  SCD_RDY=0;
  ModuleInitState.SCD30 = 1;
}

#endif

#ifdef SENSOR_SPS30
uint8_t InitSPS30(void){
  uint8_t ret = 0;
  uint32_t getFanCleanInterval;
    while (sps30_probe() != 0) {
        DEBUG_SENSOR("SPS sensor probing failed\r\n");
        sensirion_sleep_usec(1000000); /* wait 1s */
        ret++;
        if (ret == 3){
          return HAL_ERROR;
        }
    }
    

//    ret = sps30_set_fan_auto_cleaning_interval(0);
//    DEBUG_DRIVER("ret = %d\r\n",ret);
//    sps30_reset();
//    HAL_Delay(1000);

    ret = sps30_get_fan_auto_cleaning_interval(&getFanCleanInterval);
    DEBUG_SENSOR("ret = %d, getFanCleanInterval = %d\r\n",ret , getFanCleanInterval);

    
    ret = sps30_start_measurement();
    if (ret != HAL_OK)
        DEBUG_SENSOR("error starting measurement\r\n");
    DEBUG_SENSOR("SPS sensor probing successful\r\n");
    
    ret = sps30_start_manual_fan_cleaning();
    DEBUG_SENSOR("ret = %d\r\n",ret);
    return ret;
}


void GetSPS30_data(void){
  uint8_t ret, i;
  float PM2p5_MarkF = 0;
  struct sps30_measurement m;
  float pm25Range1;
  
  pm25Range1 = 19.0f;
  i = 0;
  
  if (ModuleInitState.SPS30 == HAL_OK){
        ret = sps30_read_measurement(&m);
        DEBUG_SENSOR("SPS30 ret = %d, cnt = %d\r\n", ret, i);
        if (ret != 0) {
          for (i = 1; i < I2C_retryCnt; i++){
            ret = sps30_read_measurement(&m);
            DEBUG_SENSOR("SPS30 ret = %d, cnt = %d\r\n", ret, i);
            if (ret == 0) {
              break;
            }
          }
          if (i >= I2C_retryCnt){
            DEBUG_SENSOR("SPS30 error reading measurement, I2C1 Reinit...\r\n");
            Clear_IWDG_Counter();//Clear IWDG Counter
            HAL_GPIO_WritePin(GPIOB, SPS_PWR_Pin, GPIO_PIN_RESET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(GPIOB, SPS_PWR_Pin, GPIO_PIN_SET);
            I2C1_rstCnt_PM25++;
            Reinit_I2C1(false);
            HAL_Delay(100);
            sps30_probe();
            HAL_Delay(100);
            sps30_start_measurement();
          }
        } 
        if (i < I2C_retryCnt) {
          I2C1_rstCnt_PM25 = 0;
          I2C1_OkCnt_PM25++;
            DEBUG_SENSOR("SPS30 "
//                   "pm1.0: %0.2f | "
                   "pm2.5: %0.2f | "
//                   "pm4.0: %0.2f | "
//                   "pm10.0: %0.2f | "
//                   "nc0.5: %0.2f | "
//                   "nc1.0: %0.2f | "
//                   "nc2.5: %0.2f | "
//                   "nc4.5: %0.2f | "
//                   "nc10.0: %0.2f | "
                   "typical particle size: %0.2f\r\n",
//                   m.mc_1p0, 
                   m.mc_2p5, 
//                   m.mc_4p0, 
//                   m.mc_10p0, 
//                   m.nc_0p5, 
//                   m.nc_1p0,
//                   m.nc_2p5, 
//                   m.nc_4p0, 
//                   m.nc_10p0, 
                   m.typical_particle_size);
            if (m.mc_2p5 != isnan(NAN)){
                  m.mc_2p5 = m.mc_2p5 * 1.1f;
                  memcpy(&SensorsOutRawData.PM2p5_32bit, &m.mc_2p5, 4);
                  

                  if( m.mc_2p5>250){ // 61-80
                          PM2p5_MarkF=((80.0f-40.0f)/(250.0f-pm25Range1))*(m.mc_2p5-250.0f)+80.0f;
                  }else if( m.mc_2p5>pm25Range1){ // 41-80
                          PM2p5_MarkF=((80.0f-40.0f)/(250.0f-pm25Range1))*(m.mc_2p5-250.0f)+80.0f;
                  }else{ // 0-40
                          PM2p5_MarkF=((40.0f-0.0f)/(pm25Range1-0.0f))*(m.mc_2p5-pm25Range1)+40.0f;
                  }
                  if (PM2p5_MarkF>100){
                          PM2p5_MarkF=100;	   
                  }
                  if (PM2p5_MarkF<0){
                          PM2p5_MarkF=0;	   
                  }
                  
                  PM2p5_Mark = (uint8_t)PM2p5_MarkF;
            }

        }
        //vTaskDelay(SPS30_MEASUREMENT_DURATION_USEC/portTICK_PERIOD_MS);
        //sensirion_sleep_usec(SPS30_MEASUREMENT_DURATION_USEC);
  }
  
//  CurrStatus = STATE_GETTVOC;
}
#endif


#ifdef SENSOR_SI7021
uint8_t InitSi7021(void){
  float Si7021_Temperature = -1;
  float Si7021_Humidity = -1;
  uint8_t cnt = 0, ret;
  ret = r_both_Si7021(&Si7021_Humidity,&Si7021_Temperature);
  while (ret){
    ret = r_both_Si7021(&Si7021_Humidity,&Si7021_Temperature);
    if (ret == 0){
      break;
    }
    cnt++;
    if (cnt >= 3){
      DEBUG_SENSOR("Si7021 sensor Init fail, ret = %d......................................................\r\n", ret);
      return ret;
    }
  }
  DEBUG_SENSOR("Si7021 sensor Init OK\r\n");
  return ret;
}

void GPIO_GetSi7021(void){
  float Si7021_Temperature = -1;
  float Si7021_Temperature_Raw = -1;
  float Si7021_Humidity_Raw = -1;
  float Si7021_Humidity = -1;
  float NTC_temp_1 = -1;
  float NTC_temp_2 = -1;
  float NTC_temp_3 = -1;
  float LuleDelta_data = -1;
  float RH_MarkF = 0;
  int8_t ret;
  uint8_t i;
  i = 0;

//  if (ModuleInitState.Si7021 == HAL_OK){
    
    ret = r_both_Si7021(&Si7021_Humidity,&Si7021_Temperature);
    DEBUG_SENSOR("Si7021 ret = %d, cnt = %d\r\n", ret, i);
    if (ret != 0) {
      for (i = 1; i < I2C_retryCnt; i++){
        ret = r_both_Si7021(&Si7021_Humidity,&Si7021_Temperature);
        DEBUG_SENSOR("Si7021 ret = %d, cnt = %d\r\n", ret, i);
        if (ret == 0) {
          break;
        }
      }
       if (i >= I2C_retryCnt){
         DEBUG_SENSOR("T/H sensor error reading measurement, I2C1 Reinit...\r\n");
         I2C1_rstCnt_TH++;
            Reinit_I2C1(true);
            
            
            rst_Si7021();
            return;
       }
    }
    I2C1_rstCnt_TH = 0;
    I2C1_OkCnt_TH++;
    Si7021_Temperature_Raw = Si7021_Temperature;
    Si7021_Humidity_Raw = Si7021_Humidity;
    NTC_temp_1 = GetNTCTemp1();
    NTC_temp_2 = GetNTCTemp2();
    if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
      NTC_temp_3 = GetNTCTemp3();
    }

    
#ifdef BROAN_BRNC021
    memcpy(&SensorsOutRawData.Si7021outRawData.RawHumidity, &Si7021_Humidity,4);
    memcpy(&SensorsOutRawData.Si7021outRawData.RawTemp, &Si7021_Temperature,4);
    memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC1, &NTC_temp_1,4);
//    memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC2, &NTC_temp_2,4);
  
    LuleDelta_data = GetOutDelta_data();
    memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC2, &LuleDelta_data,4);
    
    memcpy(&SensorsOutRawData.SCD30outRawData.temperature, &Si7021_Temperature, 4);
    memcpy(&SensorsOutRawData.SCD30outRawData.humidity, &Si7021_Humidity, 4);
    
    if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
      Si7021_Temperature = CalTemperature(Si7021_Temperature, NTC_temp_1, NTC_temp_3, LED_Control_data.IAQvalue, LED_Control_data.Btn1_OnOff, LED_Control_data.Btn2_OnOff, LED_Control_data.IAQ_Intensity_value);
      Si7021_Humidity = CalHumidity(Si7021_Humidity, Si7021_Temperature_Raw, Si7021_Temperature, NTC_temp_3, LED_Control_data.IAQ_Intensity_value, LED_Control_data.Btn1_OnOff, LED_Control_data.Btn2_OnOff);
      
      
      memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC3, &NTC_temp_3,4);
    }
#endif
#ifdef BROAN_BRNC011
    memcpy(&SensorsOutRawData.Si7021outRawData.RawHumidity, &Si7021_Humidity,4);
    memcpy(&SensorsOutRawData.Si7021outRawData.RawTemp, &Si7021_Temperature,4);
    memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC1, &NTC_temp_1,4);
    memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC2, &NTC_temp_2,4);


    if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
      Si7021_Temperature = CalTemperature(Si7021_Temperature, NTC_temp_1, NTC_temp_3, LED_Control_data.IAQvalue, LED_Control_data.Btn1_OnOff, LED_Control_data.Btn2_OnOff, LED_Control_data.IAQ_Intensity_value);
      Si7021_Humidity = CalHumidity(Si7021_Humidity, Si7021_Temperature_Raw, Si7021_Temperature, NTC_temp_3, LED_Control_data.IAQ_Intensity_value, LED_Control_data.Btn1_OnOff, LED_Control_data.Btn2_OnOff);
      

      
      memcpy(&SensorsOutRawData.Si7021outRawData.RawNTC3, &NTC_temp_3,4);
    }
#endif
    
//  if (Si7021_Temperature < 0){
//    memcpy(&Si7021_Temperature, &SensorsOutRawData.Si7021outRawData.temperature, 4);
//  }
  
//  if (Si7021_Humidity < 0){
//    memcpy(&Si7021_Humidity, &SensorsOutRawData.Si7021outRawData.humidity, 4);
//  }
  
  
    if (Si7021_Humidity < 0){
      Si7021_Humidity = 0;
    }
    
    if (Si7021_Humidity > 100){
      Si7021_Humidity = 100;
    }
  
    DEBUG_SENSOR("Si7021: B4 NTC NTC_temp_1 = %2f, NTC NTC_temp_2 = %2f, RawTemperature=%2f\r\n",NTC_temp_1, NTC_temp_2, Si7021_Temperature_Raw);
    //Si7021_Temperature = Si7021_Temperature - (NTC_temp - Si7021_Temperature);
    DEBUG_SENSOR("Si7021: Humidity=%2f, Temperature=%2f\r\n",Si7021_Humidity, Si7021_Temperature);
    
    

    

    memcpy(&SensorsOutRawData.Si7021outRawData.humidity, &Si7021_Humidity,4);
    
#ifdef BROAN_BRNC011
    if (LED_Control_data.Btn1_OnOff == GPIO_PIN_RESET){
      Si7021_Temperature += 0.5f;
    }
    memcpy(&SensorsOutRawData.Si7021outRawData.temperature, &Si7021_Temperature,4);
#endif
    
#ifdef BROAN_BRNC021
    memcpy(&SensorsOutRawData.Si7021outRawData.temperature, &Si7021_Temperature,4);
#endif
    
    
    if (Si7021_Humidity > 50){
      RH_MarkF = 2.0f*Si7021_Humidity - 100.0f;
    }else{
      RH_MarkF = -2.0f*Si7021_Humidity + 100.0f;
    }
    
    if (RH_MarkF > 100){
      RH_MarkF = 100;
    }
    
    if (RH_MarkF < 0){
      RH_MarkF = 0;
    }
    RH_Mark = (uint8_t)RH_MarkF;
//  }
  
//  CurrStatus = STATE_GETSCD30;
}
#endif


//Manual Override Feature Code//

bool device_isValidDeviceConfig(uint8_t _DeviceConfig)
{
	if (_DeviceConfig == 0) {
		return false;
	}
	if(_DeviceConfig > DEVICECONFIG_TABLE_CNT) {
		return false;
	}
	return true;
}

static const DeviceConfigOutput_t* device_getDeviceConfigOutput(uint8_t _DeviceConfig, uint8_t _FanAlgoState)
{
	if (_DeviceConfig == 0) {
		return NULL;
	}
	if(_DeviceConfig > DEVICECONFIG_TABLE_CNT) {
		return NULL;
	}

	if(DeviceConfigTable[_DeviceConfig].bFanAlgoCmd) {
		if (_FanAlgoState >= DeviceConfigTable[_DeviceConfig].size) {
			//FanAlgoState exceeds
			return NULL;
		}
		return &DeviceConfigTable[_DeviceConfig].pOutput[_FanAlgoState];
	} else {
		//no FanAlgo command
		return &DeviceConfigTable[_DeviceConfig].pOutput[0];
	}
}

static uint8_t device_getOutput1Config_internal(uint8_t _DeviceConfig, uint8_t _FanAlgoState)
{
	const DeviceConfigOutput_t* pOutput;
	pOutput = device_getDeviceConfigOutput(_DeviceConfig, _FanAlgoState);
	if (!pOutput) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	if (pOutput->Output1 >= NUM_OF_DEVICECONFIG_OUTPUT) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	return pOutput->Output1;
}

static uint8_t device_getOutput2Config_internal(uint8_t _DeviceConfig, uint8_t _FanAlgoState)
{
	const DeviceConfigOutput_t* pOutput;
	pOutput = device_getDeviceConfigOutput(_DeviceConfig, _FanAlgoState);
	if (!pOutput) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	if (pOutput->Output2 >= NUM_OF_DEVICECONFIG_OUTPUT) {
		return DEVICECONFIG_OUTPUT_IGNORE;
	}
	return pOutput->Output2;
}

//input = DeviceConfig, Bttn1RealState, Bttn2RealState
//output = FanRealState
uint8_t device_calcFanRealState(uint8_t _DeviceConfig, uint8_t _Bttn1RealState, uint8_t _Bttn2RealState)
{
	//check DeviceConfig
	if(!device_isValidDeviceConfig(_DeviceConfig)) {
		return 0;
	}
	//check FanAlgo command
	if(!DeviceConfigTable[_DeviceConfig].bFanAlgoCmd) {
		return 0;
	}

	uint8_t i;
	const DeviceConfigOutput_t *pOutput;
	uint8_t state = 0;
	for(i = 0; i < DeviceConfigTable[_DeviceConfig].size; i++) {
		bool bResult_Output1 = false;
		bool bResult_Output2 = false;
		pOutput = &DeviceConfigTable[_DeviceConfig].pOutput[i];
		if (!pOutput) {
			continue;
		}
		if((pOutput->Output1 == DEVICECONFIG_OUTPUT_DISABLE)
			||(pOutput->Output1 == DEVICECONFIG_OUTPUT_IGNORE))
		{
			bResult_Output1 = true;
		}
		if((pOutput->Output2 == DEVICECONFIG_OUTPUT_DISABLE)
			||(pOutput->Output2 == DEVICECONFIG_OUTPUT_IGNORE))
		{
			bResult_Output2 = true;
		}
		if(!bResult_Output1)
		{
			if(pOutput->Output1 == DEVICECONFIG_OUTPUT_OFF && _Bttn1RealState == 0) {
				bResult_Output1 = true;
			}
			if(pOutput->Output1 == DEVICECONFIG_OUTPUT_ON && _Bttn1RealState) {
				bResult_Output1 = true;
			}
		}
		if(!bResult_Output2)
		{
			if(pOutput->Output2 == DEVICECONFIG_OUTPUT_OFF && _Bttn2RealState == 0) {
				bResult_Output2 = true;
			}
			if(pOutput->Output2 == DEVICECONFIG_OUTPUT_ON && _Bttn2RealState) {
				bResult_Output2 = true;
			}
		}

		if(bResult_Output1 && bResult_Output2)
		{
			state = i;
			break;
		}
	}
	return state;
}

//input = FanAlgoState, FanRealState
//output = MOverride
uint8_t device_calcMOverride(uint8_t _DeviceConfig, uint8_t _FanAlgoState, uint8_t _FanRealState)
{
	//check DeviceConfig
	if(!device_isValidDeviceConfig(_DeviceConfig)) {
		return 0;
	}
	//check FanAlgo command
	if(!DeviceConfigTable[_DeviceConfig].bFanAlgoCmd) {
		return 0;
	}

	if(_FanAlgoState == _FanRealState) {
		return 0;
	} else {
		return 1;
	}
}


bool device_isAlgoActive(void)
{
	return bAlgoActive;
}

uint8_t device_getOutput1Config(void)
{
	return output1_config;
}

uint8_t device_getOutput2Config(void)
{
	return output2_config;
}

void device_updateResult(void)
{
        if (Switch_config_setting != 7){
            LED_Control_data.Fan_App_Status_Value = device_calcFanRealState(Switch_config_setting, LED_Control_data.Btn1_OnOff, LED_Control_data.Btn2_OnOff);
            LED_Control_data.Fan_App_Status_Value_B4 = LED_Control_data.Fan_App_Status_Value;
        }
	LED_Control_data.Manual_on_OnOff = device_calcMOverride(Switch_config_setting, LED_Control_data.Fan_AlgoApp_State_value, LED_Control_data.Fan_App_Status_Value);
	if ((LED_Control_data.Fan_AlgoApp_State_value > 0) && (LED_Control_data.Manual_on_OnOff == 0)) {
		bAlgoActive = true;
	} else {
		bAlgoActive = false;
	}
	output1_config = device_getOutput1Config_internal(Switch_config_setting, LED_Control_data.Fan_AlgoApp_State_value);
	output2_config = device_getOutput2Config_internal(Switch_config_setting, LED_Control_data.Fan_AlgoApp_State_value);
        //        ESP_Cmd_Send_En = 1;
//        LED_Control_data.FanRealState_IsChange = 1; 
//        vTaskDelay(5);
        ESP_Cmd_Send_En = 1;
        LED_Control_data.FanRealState_IsChange = 1; 
        vTaskDelay(ESP_CMD_SEND_DELAY_T);
        ESP_Cmd_Send_En = 1;
        LED_Control_data.Manual_on_IsChange = 1;
        vTaskDelay(ESP_CMD_SEND_DELAY_T);
        ESP_Cmd_Send_En = 1;
        LED_Control_data.Btn1_RealState_isChange = 1;
        vTaskDelay(ESP_CMD_SEND_DELAY_T);
#ifdef BROAN_BRNC021
        ESP_Cmd_Send_En = 1;
        LED_Control_data.Btn2_RealState_isChange = 1;
        vTaskDelay(ESP_CMD_SEND_DELAY_T);
#endif
        //DEBUG_DRIVER("Fan_AlgoApp_State_value %d, Fan_App_Status_Value = %d, Manual_on_OnOff = %d, output1_config = %d, output2_config = %d, bAlgoActive = %d \r\n", LED_Control_data.Fan_AlgoApp_State_value, LED_Control_data.Fan_App_Status_Value, LED_Control_data.Manual_on_OnOff, output1_config, output2_config, bAlgoActive);

        
}

//Manual Override Feature Code//


void STM_Data_Init(void){
  ModuleInitState.SCD30 = 1;
  ModuleInitState.Si7021 = 1;
  ModuleInitState.SPS30 = 1;
  ModuleInitState.ZMOD4410= 1;
  HAL_GPIO_WritePin(ZMOD_RES_GPIO_Port, GPIO_PIN_8, GPIO_PIN_RESET);
            TIM3->CCR2 = 0;     //Red
            TIM3->CCR3 = 0;     //Green
            TIM3->CCR4 = 0;     //Blue
  IWDG_ResetClearCounter_Flag1=0;
  IWDG_ResetClearCounter_Flag2=0;
  just_StartUP_canSend = true;
  
  LED_Control_data.IAQ_Intensity_value = Get_Flash_IAQintensityData();
  if ((LED_Control_data.IAQ_Intensity_value < 1) || (LED_Control_data.IAQ_Intensity_value > 10)){
    LED_Control_data.IAQ_Intensity_value = IAQ_INTENSITY_DEFAULT;
    SetSTM_FlashUserData(FLASH_SET_IAQ_INTENSITY_FLAG_POS, &LED_Control_data.IAQ_Intensity_value);
  }
  DEBUG_DRIVER("Get Flash IAQ instensity = %d \r\n", LED_Control_data.IAQ_Intensity_value);
  
  Switch_config_setting = Get_Flash_SWconfigData();
  if ((Switch_config_setting < 1) || (Switch_config_setting > 7)){
    Switch_config_setting = SW_CONFIG_DEFAULT;
    SetSTM_FlashUserData(FLASH_SET_SW_CONFIG_FLAG_POS, &Switch_config_setting);
  }  
  DEBUG_DRIVER("Get Flash SW Config = %d \r\n", Switch_config_setting);
  
  
  
  
  UART1_reinit_Cnt = 0;
  
  
  
}
#define DATA_64                 ((uint64_t)0x1234567912345679)
#define DATA_32                 ((uint32_t)0x12345678)

void Store_UserData(uint32_t Addr, uint8_t Data){
  uint64_t Data64[2];
  uint8_t Data8[16];
uint32_t FirstPage = 0, NbOfPages = 0;
uint32_t PageError = 0;
  static FLASH_EraseInitTypeDef EraseInitStruct = {0};
  
  
  if (Addr < STORE_END_BYTE){
    Data64[0] = *(__IO uint64_t *)FLASH_USER_START_ADDR;
    Data64[1] = *(__IO uint64_t *)(FLASH_USER_START_ADDR + 8);
    
//    DEBUG_DRIVER("org Data: Data64[0][1]: %X  %X\r\n", Data64[0], Data64[1]);
    
    memcpy(Data8, Data64, sizeof(Data8));
    
    Data8[Addr] = Data;
    
    memset(Data8, 2, sizeof(Data8));
    memcpy(Data64, Data8, sizeof(Data8));
//    DEBUG_DRIVER("ready write Data: Data64[0][1]: %X  %X\r\n", Data64[0], Data64[1]);
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();
    
    FirstPage = GetPage(FLASH_USER_START_ADDR);
    NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page        = FirstPage;
    EraseInitStruct.NbPages     = NbOfPages;
    HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    
    if (Addr < 8){
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_START_ADDR, Data64[0]);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_START_ADDR + 8, Data64[1]);
    }
    else if (Addr >= 8 && Addr < 16){
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_START_ADDR + 8, Data64[1]);
    }
    HAL_FLASH_Lock();
    
    Data64[0] = *(__IO uint64_t *)FLASH_USER_START_ADDR;
    Data64[1] = *(__IO uint64_t *)(FLASH_USER_START_ADDR + 8);
    
//    DEBUG_DRIVER("After write Data: Data64[0][1]: %X  %X\r\n", Data64[0], Data64[1]);
    
  }
}

uint8_t Get_UserData(uint32_t Addr){
  uint64_t Data64[2];
  uint8_t Data8[16];
  uint32_t ReadAddr;
  ReadAddr = FLASH_USER_START_ADDR;
    if (Addr < STORE_END_BYTE){
      Data64[0] = *(__IO uint64_t *)ReadAddr;
      ReadAddr = FLASH_USER_START_ADDR + 8;
      Data64[1] = *(__IO uint64_t *)ReadAddr;
      
      memcpy(Data8, Data64, sizeof(Data8));
      DEBUG_DRIVER("Get_UserData: ");
      for (uint8_t i = 0 ; i < sizeof(Data8); i++){
        DEBUG_DRIVER("%X ", Data8[i]);
      }
      DEBUG_DRIVER("\r\n");
      return Data8[Addr];
    }else{
      return 255;
    }
    
}

void GetSTM_SignData(uint8_t *Data){
  uint8_t i = 0;
  uint64_t Data64[8];
  for (i = 0; i < 8; i++){
    Data64[i] = *(__IO uint64_t *)(STM_SIGN_DATA_ADDR + (i * 8));
  }
  
  memcpy(Data, Data64, 64);
  
  
  
}

uint8_t Get_Flash_IAQintensityData(void){
  uint8_t Data8[8];
  GetSTM_FlashUserData(&Data8[0]);
  DEBUG_DRIVER("Get Flash Data8[%02X][%02X][%02X][%02X][%02X][%02X][%02X][%02X]\r\n", Data8[0], Data8[1], Data8[2], Data8[3], Data8[4], Data8[5], Data8[6], Data8[7]);
  return Data8[FLASH_SET_IAQ_INTENSITY_FLAG_POS];
  
}

uint8_t Get_Flash_SWconfigData(void){
  uint8_t Data8[8];
  GetSTM_FlashUserData(&Data8[0]);
  DEBUG_DRIVER("Get Flash Data8[%02X][%02X][%02X][%02X][%02X][%02X][%02X][%02X]\r\n", Data8[0], Data8[1], Data8[2], Data8[3], Data8[4], Data8[5], Data8[6], Data8[7]);
  return Data8[FLASH_SET_SW_CONFIG_FLAG_POS];
  
}

void GetSTM_FlashUserData(uint8_t *Data){
  uint64_t Data64;
  
  Data64 = *(__IO uint64_t *)(STM_FLASH_USER_DATA_ADDR);
  
  memcpy(Data, &Data64, sizeof(Data64));
}

uint32_t Erase_FlashUserData(uint32_t StartSector)
{
  FLASH_EraseInitTypeDef EraseInitStruct = {0};
  uint32_t ret;

  uint32_t FirstPage = 0, NbOfPages = 0, PageError = 0;

  HAL_FLASH_Unlock();
  FirstPage = GetPage(StartSector);
  NbOfPages = 1;
//  printf("FirstPage = %d, NbOfPages = %d \r\n", FirstPage, NbOfPages);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
    ret = 1;
  }else{
    ret = 0;
  }
  
  HAL_FLASH_Lock();
  
  
  
  
  return ret;
}

void SetSTM_FlashUserData(uint8_t FlashStoreType, uint8_t *Data){
  uint64_t Data64;
  uint8_t Data8[8];
  
    memset(Data8, 0, sizeof(Data8));
    GetSTM_FlashUserData(&Data8[0]);
    memcpy(&Data64, Data8, sizeof(Data8));
    if (Data64 != 0xFFFFFFFFFFFFFFFF){
      Erase_FlashUserData(STM_FLASH_USER_DATA_ADDR);
    }
    
    HAL_FLASH_Unlock();
    switch(FlashStoreType){
    case FLASH_SET_IAQ_INTENSITY_FLAG_POS:
     memcpy(&Data8[FLASH_SET_IAQ_INTENSITY_FLAG_POS], Data, 1);
     break;
    case FLASH_SET_SW_CONFIG_FLAG_POS:
      memcpy(&Data8[FLASH_SET_SW_CONFIG_FLAG_POS], Data, 1);
      break;
     
    };
    memcpy(&Data64, Data8, sizeof(Data8));
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, STM_FLASH_USER_DATA_ADDR , Data64);
    HAL_FLASH_Lock();
}

void Button_event(uint8_t buttonID, uint8_t event)
{

	if (buttonID == BTN_ID_1)
	{
		if (event == BTN_EVENT_SHORT_RELEASED){
                  DEBUG_DRIVER("Button 1  short press !!\r\n");
                  Button1_SingleClick = 1;
		}else if (event == BTN_EVENT_LONG_RELEASED) {
                  DEBUG_DRIVER("Button 1  Long Press !!\r\n");
                  isButton1_LongPress = 1;
                }else if (event == BTN_EVENT_LONG_PRESSED){
                  DEBUG_DRIVER("Button 1  Long Press Hold!!\r\n");
                  isButton1_LongPress_Hold = 1;
                }
                GPIO_Cont = 1;
	}
	else if (buttonID == BTN_ID_2)
	{
          if (event == BTN_EVENT_SHORT_RELEASED){
            DEBUG_DRIVER("Button 2  short press !!\r\n");
            Button2_SingleClick = 1;
          }else if(event == BTN_EVENT_LONG_RELEASED) {
            DEBUG_DRIVER("Button 2  long press !!\r\n");
            isButton2_LongPress = 1;
          }
          GPIO_Cont = 1;
	}
	else if(buttonID == BTN_ID_BOTH)
	{
		if (event == BTN_EVENT_LONG_PRESSED) {
			DEBUG_DRIVER("Both Button long pressed !!\r\n");
			isLongDualPress_Hold = 1;
                        GPIO_Cont = 1;
		} else if (event == BTN_EVENT_LONG_RELEASED) {
			DEBUG_DRIVER("Both Button long released !!\r\n");
		}
	}
}

void StartKeyTask(void const * argument){
	while (1)
	{
		//--------------------------------------------------//button 1
		Btn1Status_curr = HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin);
		
		if (Btn1Status_curr == 0)
		{
			if (Btn1Status_B4 != Btn1Status_curr){	//detect falling
				Btn1_detectFall_Cnt = 1;
				Btn1Status_B4 = Btn1Status_curr;
			}
			if (Btn1_detectFall_Cnt == 1){
				but1PCnt ++;
			}
			if (DualBtn_Detected == 0) {
//                            if (but1PCnt % 100 == 0){
//                                    printf("Detect Btn1 press down %lu s \r\n", but1PCnt/100);
//                            }
				if (but1PCnt == LongPressDelayCnt) {
					Button_event(BTN_ID_1, BTN_EVENT_LONG_PRESSED);
				}
			}
		}
		else
		{
			
			if (Btn1Status_B4 != Btn1Status_curr){	//detect rising
				Btn1_detectFall_Cnt = 0;
				Btn1Status_B4 = Btn1Status_curr;
				if (DualBtn_Detected == 0){
					if (but1PCnt >= ShortPressDelayCnt && but1PCnt < LongPressDelayCnt){		//detect short press
						Button_event(BTN_ID_1, BTN_EVENT_SHORT_RELEASED);
					}else if (but1PCnt >= LongPressDelayCnt){		//detect hold (long) press
						Button_event(BTN_ID_1, BTN_EVENT_LONG_RELEASED);
					}
					but1PCnt = 0;
				}
			}
		}
		//--------------------------------------------------//button 2
#ifdef UICONT_BRNC021_BUTTON
                if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
                  Btn2Status_curr = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
                }else if (HW_VER == BRNC_ES3_HWVER){
                  Btn2Status_curr = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
                }
                
                
		if (Btn2Status_curr == 0)
		{
			if (Btn2Status_B4 != Btn2Status_curr) {	//detect falling
				Btn2_detectFall_Cnt = 1;
				Btn2Status_B4 = Btn2Status_curr;
			}
			if (Btn2_detectFall_Cnt == 1) {
				but2PCnt ++;
			}
			if (DualBtn_Detected == 0) {
				if (but2PCnt == LongPressDelayCnt) {
					Button_event(BTN_ID_2, BTN_EVENT_LONG_PRESSED);
				}
			}
		}
		else
		{
			
			if (Btn2Status_B4 != Btn2Status_curr){	//detect rising
				Btn2_detectFall_Cnt = 0;
				Btn2Status_B4 = Btn2Status_curr;
				if (DualBtn_Detected == 0){
					if (but2PCnt >= ShortPressDelayCnt && but2PCnt < LongPressDelayCnt) {		//detect short press
						Button_event(BTN_ID_2, BTN_EVENT_SHORT_RELEASED);
					} else if (but2PCnt >= LongPressDelayCnt){		//detect hold (long) press
						Button_event(BTN_ID_2, BTN_EVENT_LONG_RELEASED);
					}
					but2PCnt = 0;
				}
			}
		}
#endif
		if (Btn1_detectFall_Cnt == 1 && Btn2_detectFall_Cnt == 1){
			DualBtn_Detected = 1;
			DualBtnCnt++;
//			if (DualBtnCnt % 100 == 0){
//				printf("Detect Dual press down %lu s \r\n", DualBtnCnt/100);
//			}
			if (DualBtnCnt == LongPressDelayCnt) {
				Button_event(BTN_ID_BOTH, BTN_EVENT_LONG_PRESSED);
			}
		}else if (Btn1_detectFall_Cnt == 0 && Btn2_detectFall_Cnt == 0 && DualBtn_Detected == 1){
			if (DualBtnCnt >= ShortPressDelayCnt && DualBtnCnt < LongPressDelayCnt){
				Button_event(BTN_ID_BOTH, BTN_EVENT_SHORT_RELEASED);
			}else if (DualBtnCnt >= LongPressDelayCnt){		//detect hold (long) press
				Button_event(BTN_ID_BOTH, BTN_EVENT_LONG_RELEASED);
			}
			DualBtn_Detected = 0;
			DualBtnCnt = 0;
			but1PCnt = 0;
			but2PCnt = 0;
		}

		
		vTaskDelay(KeyTaskDelay);//10ms
	}
}


static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;;
}


void Reinit_I2C1(bool Trigger9Pulse){
//  HAL_GPIO_WritePin(GPIOB, SPS_PWR_Pin, GPIO_PIN_RESET);
//  HAL_Delay(20);
//  HAL_GPIO_WritePin(GPIOB, SPS_PWR_Pin, GPIO_PIN_SET);
  uint8_t i = 0;
//  I2CDirectReturnErrCntNo++;
  if (I2C1_rstCnt_TH > I2C_MakeRstCnt || I2C1_rstCnt_CO2> I2C_MakeRstCnt || I2C1_rstCnt_PM25 > I2C_MakeRstCnt){
    NVIC_SystemReset();
  }

    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(5);
    
  if (Trigger9Pulse == true){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2C1_SCL_Pin | I2C1_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStruct);
    HAL_Delay(1);
    HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin, GPIO_PIN_SET);
    for (i = 0; i < I2C_RECOVER_NUM_CLOCKS; i++){
      HAL_Delay(1);
      HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_DeInit(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin);
  }
    MX_I2C1_Init();
    HAL_Delay(5);

  

  
//  DEBUG_DRIVER("**********************************Trigger I2C error cnt = %llu, I2C1 Reinit cnt = %llu, I2C OK cnt = %llu\r\n", TriggerI2CErrCntNo, I2CDirectReturnErrCntNo, I2C1_OkCnt_CO2);
//  InitSPS30();
//  HAL_Delay(10);
//  rst_Si7021();
}

void Reinit_I2C2(void){
  uint8_t i = 0;
  if (I2C2_rstCnt_TVOC > I2C_MakeRstCnt){
    NVIC_SystemReset();
  }
  HAL_I2C_DeInit(&hi2c2);
  HAL_Delay(5);
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = I2C2_SCL_Pin | I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStruct);
  HAL_Delay(1);
  HAL_GPIO_WritePin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin, GPIO_PIN_SET);
  for (i = 0; i < I2C_RECOVER_NUM_CLOCKS; i++){
    HAL_Delay(1);
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);
  }
  HAL_GPIO_DeInit(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin);
  
  
  MX_I2C2_Init();
  HAL_Delay(5);
}



/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 * ref: https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
    if you wish to just from main program to system memory at anytime, some important steps needs to be performed first:

    *Find system memory location for specific STM32 in AN2606
    *Set RCC to default values (the same as on startup) [Internal clock, no PLL, etc.)
    *Disable SysTick interrupt and reset it to default
    *Disable all interrupts
    *Map system memory to 0x00000000 location
    *Set jump location to memory location + 4 bytes offset
    *Set main stack pointer to value stored at system memory location address
    *Call virtual function assigned before
 */
void JumpToBootloader(void) {
	void (*SysMemBootJump)(void);
	
	/**
	 * Step: Set system memory address. 
	 *       
	 *       For STM32F429, system memory is on 0x1FFF 0000
	 *       For other families, check AN2606 document table 110 with descriptions of memory addresses 
	 */
	volatile uint32_t addr = 0x1FFF0000;
	
	/**
	 * Step: Disable RCC, set it to default (after reset) settings
	 *       Internal clock, no PLL, etc.
	 */
#if defined(USE_HAL_DRIVER)
	HAL_RCC_DeInit();
#endif /* defined(USE_HAL_DRIVER) */
#if defined(USE_STDPERIPH_DRIVER)
	RCC_DeInit();
#endif /* defined(USE_STDPERIPH_DRIVER) */
	
	/**
	 * Step: Disable systick timer and reset it to default values
	 */
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	/**
	 * Step: Disable all interrupts
	 */
	__disable_irq();
	
	/**
	 * Step: Remap system memory to address 0x0000 0000 in address space
	 *       For each family registers may be different. 
	 *       Check reference manual for each family.
	 *
	 *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
	 *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
	 *       For others, check family reference manual
	 */
	//Remap by hand... {
#if defined(STM32F4)
	SYSCFG->MEMRMP = 0x01;
#endif
#if defined(STM32F0)
	SYSCFG->CFGR1 = 0x01;
#endif
	//} ...or if you use HAL drivers
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
	
	/**
	 * Step: Set jump memory location for system memory
	 *       Use address with 4 bytes offset which specifies jump location where program starts
	 */
	SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
	
	/**
	 * Step: Set main stack pointer.
	 *       This step must be done last otherwise local variables in this function
	 *       don't have proper value since stack pointer is located on different position
	 *
	 *       Set direct address location which specifies stack pointer in SRAM location
	 */
	__set_MSP(*(uint32_t *)addr);
	
	/**
	 * Step: Actually call our function to jump to set location
	 *       This will start system memory execution
	 */
	SysMemBootJump();
	
	/**
	 * Step: Connect USB<->UART converter to dedicated USART pins and test
	 *       and test with bootloader works with STM32 Flash Loader Demonstrator software
	 */
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
