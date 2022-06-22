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

void ADC1_Init(void);
float GetNTCTemp1(void);
#if defined STM32_ADC_HAL

void T_Adc_Init(void); 					
uint16_t NTC_getTemperature(void);  			// get temperature value
uint16_t NTC_getAdc(uint8_t ch);
uint16_t NTC_getAdcAverage(uint8_t ch, uint8_t times);	//get the channel value, 10 times average 
#else
void MX_ADC1_Init(void);
void NTC_getTempValue(void);
#endif



#endif 
