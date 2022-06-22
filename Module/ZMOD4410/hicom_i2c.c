/*******************************************************************************
 * Copyright (c) 2020 Renesas Electronics Corporation
 * All Rights Reserved.
 *
 * This code is proprietary to Renesas, and is license pursuant to the terms and
 * conditions that may be accessed at:
 * https://www.idt.com/document/msc/idt-software-license-terms-gas-sensor-software
 *
 ******************************************************************************/

/**
 * @file    hicom_i2c.c
 * @brief   HiCom specific function definitions
 * @version 2.0.0
 * @author Renesas Electronics Corporation
 */
#include "define.h"
#include "main.h"
#include "hicom_i2c.h"
#include "stm32g0xx_hal.h"
#include <stm32g0xx_hal_i2c.h>

extern uint8_t HW_VER;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

//#define ZMOD4410_i2c_ES3 hi2c1

//#define ZMOD4410_i2c_ES4 hi2c2

//#ifdef BRNC_ES3
//  extern I2C_HandleTypeDef hi2c1;
//  #define ZMOD4410_i2c hi2c1
//#else
//  extern I2C_HandleTypeDef hi2c2;
//  #define ZMOD4410_i2c hi2c2
//#endif
//static hicom_handle_t *hicom_handle;

//void set_hicom_handle(hicom_handle_t *hc_handle)
//{
//   hicom_handle = hc_handle;
//}

void hicom_sleep(uint32_t ms)
{
//    Sleep(ms);
  HAL_Delay(ms);
}

zmod4xxx_err hicom_i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf,
                      uint8_t len)
{
	/*
    hicom_status_t hc_status;
    uint8_t control_buf[2];
    char error_str[512];

    control_buf[0] = (i2c_addr << 1) | 1;
    control_buf[1] = reg_addr;

    hc_status =
        I2C_Read(*hicom_handle, (PWriteControlByteBuffer)control_buf, 2, TRUE,
                 20, BLOCK_READ_TYPE, (PReadDataByteBuffer)buf, len);
    if (FTC_SUCCESS != hc_status) {
        return hc_status;
    };
    return 0;
	*/
#ifdef BROAN_BRNC011
  if (HW_VER == BRNC_ES3_HWVER){
    return HAL_I2C_Mem_Read(&hi2c1, i2c_addr, reg_addr, 1, buf, len, SensorI2C_Timeout);
  }else if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
    return HAL_I2C_Mem_Read(&hi2c2, i2c_addr, reg_addr, 1, buf, len, SensorI2C_Timeout);
  }
#else
  return HAL_I2C_Mem_Read(&hi2c2, i2c_addr, reg_addr, 1, buf, len, SensorI2C_Timeout);
#endif

}

zmod4xxx_err hicom_i2c_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *buf,
                       uint8_t len)
{
	/*
    hicom_status_t hc_status;
    uint8_t control_buf[2];
    FTC_PAGE_WRITE_DATA page_write_data = { 0, 0 };
    char error_str[512];

    control_buf[0] = i2c_addr << 1;
    control_buf[1] = reg_addr;

    hc_status =
        I2C_Write(*hicom_handle, (PWriteControlByteBuffer)control_buf, 2, TRUE,
                  20, TRUE, PAGE_WRITE_TYPE, (PWriteDataByteBuffer)buf, len,
                  TRUE, 20, &page_write_data);

    if (FTC_SUCCESS != hc_status) {
        return hc_status;
    };
    return 0;
	*/
#ifdef BROAN_BRNC011
  if (HW_VER == BRNC_ES3_HWVER){
    return HAL_I2C_Mem_Write(&hi2c1, i2c_addr, reg_addr, 1, buf, len, SensorI2C_Timeout);
  }else if (HW_VER == BRNC_ES4_HWVER || HW_VER == BRNC_ES5_HWVER){
    return HAL_I2C_Mem_Write(&hi2c2, i2c_addr, reg_addr, 1, buf, len, SensorI2C_Timeout);
  }
#else
  return HAL_I2C_Mem_Write(&hi2c2, i2c_addr, reg_addr, 1, buf, len, SensorI2C_Timeout);
#endif
//    return HAL_I2C_Mem_Write(&ZMOD4410_i2c , i2c_addr, reg_addr, 1, (uint8_t*)buf, len, SensorI2C_Timeout);  
//	return HAL_I2C_Master_Transmit(&ZMOD4410_i2c, i2c_addr, buf, len, SensorI2C_Timeout);
}
