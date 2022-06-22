#ifndef _ADC_NTC_H_
#define _ADC_NTC_H_
	
//#include "stm32f10x.h"
#include "stm32g0xx.h"
#include <math.h>
//#include "CommonUse.h"

//#include <stdint.h>
 		 
#define ADC_CH_TEMP  			ADC_Channel_16 // channel=16
#define MURATA_NTC                      1
//NTC model MUrata NCP18XH103F03RB




static float NTC_temp1;
static float NTC_temp2;
static float NTC_temp3;

static float Temp_backup = 0;

static uint16_t NTC1_rawADCvalue;
static uint16_t NTC2_rawADCvalue;
static uint16_t NTC3_rawADCvalue;

static float OutDelta_data;

void ADC1_Init(void);
float GetNTCTemp1(void);
float GetNTCTemp2(void);
float GetNTCTemp3(void);

uint16_t GetNTC1_rawADCvalue(void);
uint16_t GetNTC2_rawADCvalue(void);
uint16_t GetNTC3_rawADCvalue(void);

float GetOutDelta_data(void);

#if defined STM32_ADC_HAL

void T_Adc_Init(void); 					
uint16_t NTC_getTemperature(void);  			// get temperature value
uint16_t NTC_getAdc(uint8_t ch);
uint16_t NTC_getAdcAverage(uint8_t ch, uint8_t times);	//get the channel value, 10 times average 
#else
void MX_ADC1_Init(void);
void NTC_getTempValue(uint8_t inTask);
#endif

float CalTemperature(float RawTemperature, float NTC1temp, float NTC2temp, uint8_t IAQvalue, GPIO_PinState Btn1_OnOff, GPIO_PinState Btn2_OnOff, uint8_t IAQ_Intensity_value);
float CalHumidity(float RawHumidity, float RawTemp, float RealTemp, float NTC3temp, uint8_t IAQ_Intensity_value, GPIO_PinState Btn1_OnOff, GPIO_PinState Btn2_OnOff);



#endif 
