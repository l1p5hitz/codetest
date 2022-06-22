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


/*
include lib
$PROJ_DIR$\..\Drivers\Module\ZMOD4410\lib\lib_zmod4xxx_cleaning.a
$PROJ_DIR$\..\Drivers\Module\ZMOD4410\lib\lib_iaq_2nd_gen.a
*/

#ifndef _ZMOD4410_H
#define _ZMOD4410_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
//  

#include "zmod4xxx.h"
#include "zmod4xxx_cleaning.h"
//#include "zmod4xxx_hal.h"
#include "iaq_2nd_gen.h"

//int zmod4410_main();
int8_t InitZMOD4410(void);
int8_t cont_run(zmod4xxx_dev_t* dev, uint32_t *TVOCdata, uint32_t *r_mox_data, uint32_t *IAQdata, uint32_t *eCO2_data);

#ifdef __cplusplus
}
#endif

#endif // _ZMOD4XXX_H