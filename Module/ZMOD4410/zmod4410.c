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
 * @file    main.c
 * @brief   This is an example for the ZMOD4410 gas sensor using the IAQ 2nd Gen library.
 * @version 2.0.0
 * @author Renesas Electronics Corporation
 **/
#include "zmod4410.h"
//#include "main.h"
#include "stm32g0xx_hal.h"      //nyc
#include "zmod4410_config_iaq2.h"
#include "CommonUse.h"
#include "hicom_i2c.h"

//#include "zmod4410_config_iaq2.h"
//#include "zmod4xxx.h"
//#include "zmod4xxx_cleaning.h"
//#include "zmod4xxx_hal.h"
//#include "iaq_2nd_gen.h"

extern zmod4xxx_dev_t zmod4410_dev;
extern uint8_t zmod4410_EN;
extern algorithm_version iaq_2nd_gen_ver;
//extern uint8_t zmod4410_RDY;
//

    iaq_2nd_gen_handle_t algo_handle;
    iaq_2nd_gen_results_t algo_results;
uint8_t prod_data[ZMOD4410_PROD_DATA_LEN];
int8_t InitZMOD4410(void)
{
    int8_t ret;
//    zmod4xxx_dev_t dev;
    zmod4410_EN = 1;
    /* Sensor target variables */

    
    /* HAL */
//    ret = init_hardware(&dev);
//    if (ret) {
//        DEBUG_SENSOR("Error %d during initialize hardware, exiting program!\r\n", ret);
//        return ret;
//    }

    /* Sensor Related Data */
    memset(&zmod4410_dev, 0, sizeof(zmod4410_dev));
  zmod4410_dev.read = hicom_i2c_read;
  zmod4410_dev.write = hicom_i2c_write;
  zmod4410_dev.delay_ms = hicom_sleep;
    zmod4410_dev.i2c_addr = ZMOD4410_I2C_ADDR;
    zmod4410_dev.pid = ZMOD4410_PID;
    zmod4410_dev.init_conf = &zmod_sensor_type[INIT];
    zmod4410_dev.meas_conf = &zmod_sensor_type[MEASUREMENT];
    zmod4410_dev.prod_data = prod_data;

    
    
    
    HAL_Delay(100);
    HAL_GPIO_WritePin(ZMOD_RES_GPIO_Port, ZMOD_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(ZMOD_RES_GPIO_Port, ZMOD_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    ret = zmod4xxx_read_sensor_info(&zmod4410_dev);
    if (ret) {
        DEBUG_SENSOR("Error %d during reading sensor information, exiting program!\r\n",
               ret);
        return ret;
    }
    
    DEBUG_SENSOR("ZMOD4410 Lib ver: %u.%u.%u \r\n", iaq_2nd_gen_ver.major, iaq_2nd_gen_ver.minor, iaq_2nd_gen_ver.patch);

    /* This function starts the cleaning procedure. It's
    * recommended to be executed after product assembly. This
    * helps to clean the metal oxide surface from assembly.
    * IMPORTANT NOTE: The cleaning procedure can be run only once
    * during the modules lifetime and takes 10 minutes. */

    //ret = zmod4xxx_cleaning_run(&zmod4410_dev);
    //if (ret) {
    //    DEBUG_SENSOR("Error %d during cleaning procedure, exiting program!\r\n", ret);
    //    return ret;
    //}

    /* Preperation of sensor */
    ret = zmod4xxx_prepare_sensor(&zmod4410_dev);
    if (ret) {
        DEBUG_SENSOR("Error %d during preparation of the sensor, exiting program!\r\n",
               ret);
        return ret;
    }

    /* One time initialization of the algorithm */
    ret = init_iaq_2nd_gen(&algo_handle);
    if (ret) {
        DEBUG_SENSOR("Error %d when initializing algorithm, exiting program!\r\n", ret);
        return ret;
    }

    ret = zmod4xxx_start_measurement(&zmod4410_dev);
    if (ret) {
        DEBUG_SENSOR("Error %d when starting measurement, exiting program!\r\n", ret);
        return ret;
    }
//
    DEBUG_SENSOR("Evaluate measurements in a loop. Press any key to quit.\r\n\r\n");
//    do {
//        do {
//            ret = zmod4xxx_read_status(&dev, &zmod4xxx_status);
//            if (ret) {
//                DEBUG_SENSOR(
//                    "Error %d during read of sensor status, exiting program!\r\n",
//                    ret);
//                return ret;
//            }
//            s_step_new = (zmod4xxx_status & STATUS_LAST_SEQ_STEP_MASK);
//            if (s_step_new != s_step_last) {
//                if (s_step_new == ((dev.meas_conf->s.len / 2) - 1)) {
//                    eoc_flag = 1;
//                }
//            }
//            s_step_last = s_step_new;
//            dev.delay_ms(50);
//        } while (!eoc_flag);
//
//        eoc_flag = 0;
//
//        ret = zmod4xxx_read_adc_result(&dev, adc_result);
//        if (ret) {
//            DEBUG_SENSOR("Error %d during read of ADC results, exiting program!\r\n",
//                   ret);
//            return ret;
//        }
//
//        /* calculate the algorithm */
//        ret = calc_iaq_2nd_gen(&algo_handle, &dev, adc_result, &algo_results);
//        if ((ret != IAQ_2ND_GEN_OK) && (ret != IAQ_2ND_GEN_STABILIZATION)) {
//            DEBUG_SENSOR("Error %d when calculating algorithm, exiting program!\r\n",
//                   ret);
//            return ret;
//        } else {
//            DEBUG_SENSOR("*********** Measurements ***********\r\n");
//            for (int i = 0; i < 13; i++) {
//                DEBUG_SENSOR(" Rmox[%d] = ", i);
//                DEBUG_SENSOR("%.3f kOhm\r\n", algo_results.rmox[i] / 1e3);
//            }
//            DEBUG_SENSOR(" log_Rcda = %5.3f logOhm\r\n", algo_results.log_rcda);
//            DEBUG_SENSOR(" EtOH = %6.3f ppm\r\n", algo_results.etoh);
//            DEBUG_SENSOR(" TVOC = %6.3f mg/m^3\r\n", algo_results.tvoc);
//            DEBUG_SENSOR(" eCO2 = %4.0f ppm\r\n", algo_results.eco2);
//            DEBUG_SENSOR(" IAQ  = %4.1f\r\n", algo_results.iaq);
//            if (ret == IAQ_2ND_GEN_STABILIZATION) {
//                DEBUG_SENSOR("Warmup!\r\n");
//            } else {
//                DEBUG_SENSOR("Valid!\r\n");
//            }
//            DEBUG_SENSOR("************************************\r\n");
//        }
//
//        /* wait 1.99 seconds before starting the next measurement */
//        dev.delay_ms(1990);
//
//        /* start a new measurement before result calculation */
//        ret = zmod4xxx_start_measurement(&dev);
//        if (ret) {
//            DEBUG_SENSOR("Error %d when starting measurement, exiting program!\r\n",
//                   ret);
//            return ret;
//        }
//        dev.delay_ms(50);
//
//    } while (!is_key_pressed());
//    
//    
    return HAL_OK;
//
////exit:
////    ret = deinit_hardware();
////    if (ret) {
////        DEBUG_SENSOR("Error %d during deinitialize hardware, exiting program\r\n", ret);
////        return ret;
////    }
////    return 0;
}


int8_t cont_run(zmod4xxx_dev_t* dev, uint32_t *TVOCdata, uint32_t *r_mox_data, uint32_t *IAQdata, uint32_t *eCO2_data)
{
    uint8_t zmod4xxx_status;
    uint8_t ret;
    uint8_t adc_result[32] = { 0 };
    float Temp_rmox = 0;

    /* Polling related variables -  if interrupt is used, they can be deleted */
    
    uint8_t eoc_flag = 0;
    uint8_t s_step_last = 0;
    uint8_t s_step_new = 0;
    
        do {
            ret = zmod4xxx_read_status(dev, &zmod4xxx_status);
            if (ret) {
                DEBUG_SENSOR(
                    "Error %d during read of sensor status, exiting program!\r\n",
                    ret);
                return ret;
            }
            s_step_new = (zmod4xxx_status & STATUS_LAST_SEQ_STEP_MASK);
            if (s_step_new != s_step_last) {
                if (s_step_new == ((dev->meas_conf->s.len / 2) - 1)) {
                    eoc_flag = 20;
                }
            }
            s_step_last = s_step_new;
            dev->delay_ms(50);
            eoc_flag++;
        } while (eoc_flag<20);

        eoc_flag = 0;

    /* INSTEAD OF POLLING THE INTERRUPT CAN BE USED FOR OTHER HW */
    /* wait until readout result is possible */


        
        
        ret = zmod4xxx_read_adc_result(dev, adc_result);
        if (ret) {
            DEBUG_SENSOR("Error %d during read of ADC results, exiting program!\r\n",
                   ret);
//            return ret;
        }

        /* calculate the algorithm */
        ret = calc_iaq_2nd_gen(&algo_handle, dev, adc_result, &algo_results);
        if ((ret != IAQ_2ND_GEN_OK) && (ret != IAQ_2ND_GEN_STABILIZATION)) {
            DEBUG_SENSOR("Error %d when calculating algorithm, exiting program!\r\n",
                   ret);
            return ret;
        } else {
//            DEBUG_SENSOR("*********** Measurements ***********\r\n");
//             DEBUG_SENSOR(" Rmox (kOhm)\r\n");
//            for (int i = 0; i < 13; i++) {
//                DEBUG_SENSOR("[%d]\t", i);
//            }
//            DEBUG_SENSOR("\r\n");
//            for (int i = 0; i < 13; i++) {
//                DEBUG_SENSOR("%.3f\t", algo_results.rmox[i] / 1e3);
//            }
//            DEBUG_SENSOR("\r\n");
            Temp_rmox = algo_results.rmox[4] / 1e3;
            memcpy(TVOCdata, &algo_results.tvoc, 4);
            memcpy(r_mox_data, &Temp_rmox, 4);
            memcpy(IAQdata, &algo_results.iaq, 4);
            memcpy(eCO2_data, &algo_results.eco2, 4);
            
            DEBUG_SENSOR("rmox[4] %.3f\t", algo_results.rmox[4] / 1e3);
            DEBUG_SENSOR("rmox[6] %.3f\r\n", algo_results.rmox[6] / 1e3);
            DEBUG_SENSOR(" log_Rcda = %5.3f logOhm\r\n", algo_results.log_rcda);
            DEBUG_SENSOR(" EtOH = %6.3f ppm\r\n", algo_results.etoh);
            DEBUG_SENSOR(" TVOC = %6.3f mg/m^3\r\n", algo_results.tvoc);
            DEBUG_SENSOR(" eCO2 = %4.0f ppm\r\n", algo_results.eco2);
            DEBUG_SENSOR(" IAQ  = %4.1f\r\n", algo_results.iaq);
            
            if (ret == IAQ_2ND_GEN_STABILIZATION) {
                DEBUG_SENSOR("Warmup!\r\n");
            } else {
                DEBUG_SENSOR("Valid!\r\n");
            }
//            DEBUG_SENSOR("************************************\r\n");
        }

        /* wait 1.99 seconds before starting the next measurement */
        dev->delay_ms(1990);

        /* start a new measurement before result calculation */
        ret = zmod4xxx_start_measurement(dev);
        if (ret) {
            DEBUG_SENSOR("Error %d when starting measurement, exiting program!\r\n",
                   ret);
            return ret;
        }
        dev->delay_ms(50);
  return ZMOD4XXX_OK;
}