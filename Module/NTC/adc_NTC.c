#include "adc_NTC.h"
#include "main.h"
#include "define.h"
#include "cmsis_os.h"
#include "STM32g0xx_hal_iwdg.h"
#include "stm32g0xx_hal_gpio.h"


#define ADC1_DR_Address    ((u32)0x40012400+0x4c)

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
uint32_t adcData[8];

__IO uint16_t ADC_ConvertedValue;		// ADC1 get value by MDA to SRAM

float ADC_ConvertedValueLocal;   		// real value	
uint16_t adcx;


float NTC_MRURATA_NCP18XH103F03RB_cal(uint16_t reading);

/*
void T_Adc_Init(void)  //ADC通道初始化
{
	ADC_InitTypeDef ADC_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能GPIOA,ADC1通道时钟
  
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //分频因子6时钟为72M/6=12MHz

   	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值
 
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//开启连续转换模式，即不停地进行ADC转换	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器

	ADC_TempSensorVrefintCmd(ENABLE); //开启内部温度传感器
	
 
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1

	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的复位寄存器

        while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待

	ADC_StartCalibration(ADC1);	 //

	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
}
*/

#if defined STM32_ADC_HAL_FUNC
    
/*
 * function ：ADC1_GPIO_Config
 * desc  ：enable ADC1 & DMA1 clock，PC.00
 */
static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure PA.00  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;          // NTC 1 & 2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				// PC1,输入时不用设置速率

        
        
}

/* function：ADC1_Mode_Config
 * desc  ：config ADC1 mode MDA
 */
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  //内存地址固定
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	DMA_Cmd(DMA1_Channel2, ENABLE);

	/* ADC1 configuration */
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 //禁止扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	 	//要转换的通道数目1
	ADC_Init(ADC1, &ADC_InitStructure);
	
	
//	RCC_ADCCLKConfig(RCC_PCLK2_Div8);   //config ADC clock，PCLK2->8，=9Hz
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //分频因子6时钟为72M/6=12MHz	
	
	/*config ADC1 channel 11 for 55.	5 sampple period，serial=1 */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}

uint16_t Temp_getAdc(uint8_t ch)   
{
 
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道3,第一个转换,采样时间为239.5周期	  			    
 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	
	return ADC_GetConversionValue(ADC1);		//返回最近一次ADC1规则组的转换结果
}

uint16_t Temp_getAdcAverage(uint8_t ch, uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Temp_getAdc(ch);
		Delay_ms(5);
	}
	
	return temp_val/times;
} 	   

void Temp_getTemperature(void)
{
    float temp;
    float temperature;
    
    adcx=Temp_getAdcAverage(ADC_CH_TEMP,10);
    temp=(float)adcx*(3.3/4096);
    temperature=temp;//保存温度传感器的电压值								  
    temperature=(1.43-temperature)/0.0043+25;		//计算出当前温度值	 
  
    DEBUG_DRIVER("\r\n get sample = 0x%04X \r\n", adcx); 
    DEBUG_DRIVER("\r\n converted temperature = %f C \r\n",temperature); 
	  
//	ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*3.3; // 读取转换的AD值	
//	printf("\r\n get sample = 0x%04X \r\n", ADC_ConvertedValue); 
//	printf("\r\n value after conversion = %f V \r\n",ADC_ConvertedValueLocal); 
}
#else

static void ADC1_GPIO_Config(void)
{
/*  
	GPIO_InitTypeDef GPIO_InitStruct;
		
	/// Configure PA.00  as analog input ///
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_0 | GPIO_PIN_1;          // NTC 1 & 2
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				// PC1,输入时不用设置速率
*/
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = GPIO_PIN_0 | GPIO_PIN_1,          // NTC 1 & 2,
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
    };
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
    ADC1_GPIO_Config();
    
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE BEGIN ADC1_Init 2 */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
//  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }    
  /* USER CODE END ADC1_Init 2 */

}

#if defined DMA_TEST_REF
/** 
* Enable DMA controller clock
*/
void MX_DMA_Init(void) 
{
/* DMA controller clock enable */
    __DMA2_CLK_ENABLE();
   

    /* DMA interrupt init */
    /* Sets the priority grouping field */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void Temp_getADCAvgValue(void)
{
// Start ADC DMA 
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, 8);
}
#endif

// ADC DMA interrupt handler
#if DEFINE_ADC_CALLBACK1
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef * hadc)
{
    printf("ADC conversion done.\n");
    HAL_ADC_Stop_DMA(hadc);
    HAL_ADC_Stop(hadc);
}
#else
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)  
{ 
  uint16_t ADC_raw;
  float Vdd;
      
  if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) 
  { 
    ADC_raw = HAL_ADC_GetValue(hadc); 
    Vdd = 3300 * (*VREFINT_CAL_ADDR)/ADC_raw; 
  } 
  
/*
 if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)) 
  {
  ADC_raw[index] = HAL_ADC_GetValue(hadc); 
  index++;
  }
if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS))  
 {
 index=0;
 Vdd = 3300 * (*VREFINT_CAL_ADDR) / ADC_raw[2];
 temperature = (((int32_t)ADC_raw[1] * Vdd/3300)- (int32_t) *TEMP30_CAL_ADDR );
 temperature = temperature * (int32_t)(110 - 30);
 temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
 temperature = temperature + 30; Vin = Vdd*ADC_raw[0]/4095;
 }
*/  
}
#endif

// Define variable to hold the 8 ADC values
// Why 32 bits if the resolution is 12b? 

void NTC_getTempValue(void)
{
  /* USER CODE BEGIN WHILE */
  /* USER CODE END WHILE */
 
  /* USER CODE BEGIN 3 */
 
      volatile uint16_t rawValue1, rawValue2;
      volatile float temp1, temp2;
 

      
#if defined SENSOR_NTC1
      
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 10);
      rawValue1 = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 10);
      rawValue2 = HAL_ADC_GetValue(&hadc1);
      
  #if MURATA_NTC
         //*** this eq is applied to (-40~125C) accuracy at 0.087C ***
        //  x=C/2n - 0.5, where C is adc value
        //  T=25 + x(C1 + x(x2 + xC3))/(1+ x(c4 + xc5))
        temp1 = NTC_MRURATA_NCP18XH103F03RB_cal(rawValue1);

  #else      
        temp1 = ((float)rawValue1) / 4096 * 3300;
        temp1 = ((temp1 - 760.0) / 2.5) + 25;
  #endif
      NTC_temp1 = temp1;
      DEBUG_DRIVER("\r\n NTC1 get sample = 0x%04X \r\n", rawValue1); 
      DEBUG_DRIVER("\r\n NTC1 converted temperature = %f C \r\n",NTC_temp1);
#endif    
      
      
#if defined SENSOR_NTC2
      
            

      
  #if MURATA_NTC
         //*** this eq is applied to (-40~125C) accuracy at 0.087C ***
        //  x=C/2n - 0.5, where C is adc value
        //  T=25 + x(C1 + x(x2 + xC3))/(1+ x(c4 + xc5))
        temp2 = NTC_MRURATA_NCP18XH103F03RB_cal(rawValue2);

  #else      
        temp2 = ((float)rawValue2) / 4096 * 3300;
        temp2 = ((temp2 - 760.0) / 2.5) + 25;
  #endif
      
      DEBUG_DRIVER("\r\n NTC2 get sample = 0x%04X \r\n", rawValue2); 
      DEBUG_DRIVER("\r\n NTC2 converted temperature = %f C \r\n",temp2);
#endif    
      
      
      HAL_ADC_Stop(&hadc1);
      
      
//      HAL_Delay(1000); //Display updates every second 
}

float NTC_MRURATA_NCP18XH103F03RB_cal(uint16_t reading){
  float temp;
  //reference: http://www..mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/ntc-thermistors/example-code-equations
#ifdef BROAN_BRNC011
      temp = (((float)reading)/4096) - 0.5;                      //12 bit
#elif defined(BROAN_BRNC021)
      temp = (((float)reading)/4096) - 0.5;                      //12 bit
#endif
      temp = 25 + temp*(-108.21 + temp*(24.118 +temp*216.38))/(1 + temp*(0.03895 + temp*(-3.1804)));
      
      return temp;
}

void Temp_getTemperataure_test(void)
{
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */
 
  /* USER CODE BEGIN 3 */
 
      volatile uint16_t rawValue;
      volatile float temp;
 

#if defined SENSOR_NTC1
      
      HAL_ADC_Start(&hadc1);      
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      rawValue = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Stop(&hadc1);
      
      temp = ((float)rawValue) / 4095 * 3300;
      temp = ((temp - 760.0) / 2.5) + 25;

      DEBUG_DRIVER("\r\n NTC1 get sample = 0x%04X \r\n", rawValue); 
      DEBUG_DRIVER("\r\n NTC1 converted temperature = %f C \r\n",temp);
#endif    
//      HAL_Delay(1000); //Display updates every second 
  }
}  


float GetNTCTemp1(void){
  return NTC_temp1;
}

#endif
