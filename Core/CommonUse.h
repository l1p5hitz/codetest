#ifndef __COMMONUSE_H
#define __COMMONUSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>	//add for printf()
#include "define.h"
#include "stdint.h"

#define DEBUG_ENABLE_UART3_
#define DEBUG_SYS_INFO_

#define DEBUG_MP_MODE_
#define DEBUG_DRIVER_
//#define DEBUG_UART_
#define DEBUG_SENSOR_


  

  
  
#ifdef DEBUG_DRIVER_
  #define DEBUG_DRIVER printf
#else
  #define DEBUG_DRIVER printNull
#endif

#ifdef DEBUG_UART_
  #define DEBUG_UART printf
#else
  #define DEBUG_UART printNull
#endif

#ifdef DEBUG_SENSOR_
  #define DEBUG_SENSOR printf
#else
  #define DEBUG_SENSOR printNull
#endif

#ifdef DEBUG_MP_MODE_
  #define DEBUG_MP_MODE printf
#else
  #define DEBUG_MP_MODE printNull
#endif

#ifdef DEBUG_SYS_INFO_
  #define DEBUG_SYS_INFO printf
#else
  #define DEBUG_SYS_INFO printNull
#endif
  
  
#define BUTTON_DEBOUNCE_DELAY 100
#define SINGLE_PRESS_TIME 1200//499
#define LONG_PRESS_TIME 5000//1200//500
#define LONG_PRESS_MAX_TIME 15000
#define EXTENDED_PRESS_TIME 15000//1000

#define PWM_MAX 100
#define IAQ_PWM_MAX 1000
#define LED_RUN_FREQ_2p0HZ 2500
#define LED_RUN_FREQ_1p0HZ 5000
#define LED_RUN_FREQ_0p5HZ 10000
  
  
#ifdef BROAN_BRNC011
  #define IAQ_INTENSITY_DEFAULT 5
#endif
  
#ifdef BROAN_BRNC021
  #define IAQ_INTENSITY_DEFAULT 5
#endif
  
#define SW_CONFIG_DEFAULT 2

typedef enum
{
  ON9_FADE_UP_1	        = 0	,
  ON9_FADE_UP_2	        = 30	,
  ON9_FADE_UP_3	        = 60	,
  ON9_FADE_UP_4	        = 90	,
  ON9_FADE_UP_5	        = 120	,
  ON9_FADE_UP_6	        = 150	,
  ON9_FADE_UP_7	        = 180	,
  ON9_FADE_UP_8	        = 210	,
  ON9_FADE_UP_9	        = 240	,
  ON9_FADE_UP_10	= 270	,
  ON9_FADE_STABLE	= 300	,
  ON9_FADE_DN_1	        =1730	,
  ON9_FADE_DN_2	        =1760	,
  ON9_FADE_DN_3	        =1790	,
  ON9_FADE_DN_4	        =1820	,
  ON9_FADE_DN_5	        =1850	,
  ON9_FADE_DN_6	        =1880	,
  ON9_FADE_DN_7	        =1910	,
  ON9_FADE_DN_8	        =1940	,
  ON9_FADE_DN_9	        =1970	,
  ON9_FADE_DN_10	=2000	,
  ON9_FADE_END          =2015   ,

  OFF9_IAQ_FADE_UP_1	=	0	,
  OFF9_IAQ_FADE_UP_2	=	30	,
  OFF9_IAQ_FADE_UP_3	=	60	,
  OFF9_IAQ_FADE_UP_4	=	90	,
  OFF9_IAQ_FADE_UP_5	=	120	,
  OFF9_IAQ_FADE_UP_6	=	150	,
  OFF9_IAQ_FADE_UP_7	=	180	,
  OFF9_IAQ_FADE_UP_8	=	210	,
  OFF9_IAQ_FADE_UP_9	=	240	,
  OFF9_IAQ_FADE_UP_10	=	270	,
  OFF9_IAQ_FADE_STABLE	=	300	,
  OFF9_IAQ_FADE_DN_10	=	2730	,
  OFF9_IAQ_FADE_DN_9	=	2760	,
  OFF9_IAQ_FADE_DN_8	=	2790	,
  OFF9_IAQ_FADE_DN_7	=	2820	,
  OFF9_IAQ_FADE_DN_6	=	2850	,
  OFF9_IAQ_FADE_DN_5	=	2880	,
  OFF9_IAQ_FADE_DN_4	=	2910	,
  OFF9_IAQ_FADE_DN_3	=	2940	,
  OFF9_IAQ_FADE_DN_2	=	2970	,
  OFF9_IAQ_FADE_DN_1	=	3000	,
  OFF9_BLUE_FADE_UP_1	=	3015	,
  OFF9_BLUE_FADE_UP_2	=	3030	,
  OFF9_BLUE_FADE_UP_3	=	3045	,
  OFF9_BLUE_FADE_UP_4	=	3060	,
  OFF9_BLUE_FADE_UP_5	=	3075	,
  OFF9_BLUE_FADE_UP_6	=	3090	,
  OFF9_BLUE_FADE_UP_7	=	3105	,
  OFF9_BLUE_FADE_UP_8	=	3120	,
  OFF9_BLUE_FADE_UP_9	=	3135	,
  OFF9_BLUE_FADE_UP_10	=	3150	,
  OFF9_BLUE_FADE_STABLE	=	3165	,
  OFF9_BLUE_FADE_DN_10	=	3865	,
  OFF9_BLUE_FADE_DN_9	=	3880	,
  OFF9_BLUE_FADE_DN_8	=	3895	,
  OFF9_BLUE_FADE_DN_7	=	3910	,
  OFF9_BLUE_FADE_DN_6	=	3925	,
  OFF9_BLUE_FADE_DN_5	=	3940	,
  OFF9_BLUE_FADE_DN_4	=	3955	,
  OFF9_BLUE_FADE_DN_3	=	3970	,
  OFF9_BLUE_FADE_DN_2	=	3985	,
  OFF9_BLUE_FADE_DN_1	=	4000	,


}FADING_TIMMING_T_;


 
    
void printNull(const char *data, ...);
    

#define ESP_CMD_SEND_DELAY_T 5
    
    
    
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
