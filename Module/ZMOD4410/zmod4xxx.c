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
 * @file   zmod4xxx.c
 * @brief  zmod4xxx-API functions
 * @version 2.0.0
 * @author Renesas Electronics Corporation
 */

#include "zmod4xxx.h"

zmod4xxx_err zmod4xxx_read_status(zmod4xxx_dev_t *dev, uint8_t *status)
{
    zmod4xxx_err ret;
    uint8_t st;

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_STATUS, &st, 1);
    if (0 != ret) {
        return ERROR_I2C;
    }
    *status = st;
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_read_sensor_info(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret = 0;
    uint8_t status = 0;
    uint8_t data[ZMOD4XXX_LEN_PID];
    uint16_t product_id;
    uint8_t cmd = 0;
    uint16_t i = 0;

    do {
        ret = dev->write(dev->i2c_addr, ZMOD4XXX_ADDR_CMD, &cmd, 1);
        if (ret) {
            return ERROR_I2C;
        }
        ret = zmod4xxx_read_status(dev, &status);
        if (ret) {
            return ret;
        }
        i++;
        dev->delay_ms(200);
    } while ((0x00 != (status & 0x80)) && (i < 1000));

    if (1000 <= i) {
        return ERROR_GAS_TIMEOUT;
    }

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_PID, data, ZMOD4XXX_LEN_PID);
    if (ret) {
        return ERROR_I2C;
    }
    product_id = ((data[0] * 256) + data[1]);

    if (dev->pid != product_id) {
        return ERROR_SENSOR_UNSUPPORTED;
    }

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_CONF, dev->config,
                    ZMOD4XXX_LEN_CONF);
    if (ret) {
        return ERROR_I2C;
    }

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_PROD_DATA, dev->prod_data,
                    dev->meas_conf->prod_data_len);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_read_tracking_number(zmod4xxx_dev_t *dev,
                                           uint8_t *track_num)
{
    zmod4xxx_err ret;

    ret = dev->read(dev->i2c_addr, ZMOD4XXX_ADDR_TRACKING, track_num,
                    ZMOD4XXX_LEN_TRACKING);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_calc_factor(zmod4xxx_dev_t *dev)
{
    int16_t hspi[HSP_MAX];
    int16_t hspm[HSP_MAX];
    float hspf;
    uint8_t i;

    for (i = 0; i < dev->init_conf->h.len; i = i + 2) {
        hspi[i / 2] =
            ((dev->init_conf->h.data[i] << 8) + dev->init_conf->h.data[i + 1]);
        hspf = (-((float)dev->config[2] * 256.0 + dev->config[3]) *
                ((dev->config[4] + 640.0) * (dev->config[5] + hspi[i / 2]) -
                 512000.0)) /
               12288000.0;

        dev->init_conf->h.data[i] = (uint8_t)((uint16_t)hspf >> 8);
        dev->init_conf->h.data[i + 1] = (uint8_t)((uint16_t)hspf & 0x00FF);
    }

    for (i = 0; i < dev->meas_conf->h.len; i = i + 2) {
        hspm[i / 2] =
            ((dev->meas_conf->h.data[i] << 8) + dev->meas_conf->h.data[i + 1]);
        hspf = (-((float)dev->config[2] * 256.0 + dev->config[3]) *
                ((dev->config[4] + 640.0) * (dev->config[5] + hspm[i / 2]) -
                 512000.0)) /
               12288000.0;

        dev->meas_conf->h.data[i] = (uint8_t)((uint16_t)hspf >> 8);
        dev->meas_conf->h.data[i + 1] = (uint8_t)((uint16_t)hspf & 0x00FF);
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_init_sensor(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret = 0;
    uint8_t data_r[RSLT_MAX];
    uint8_t zmod4xxx_status;

    ret = dev->read(dev->i2c_addr, 0xB7, data_r, 1);
    if (ret) {
        return ERROR_I2C;
    }

    ret = dev->write(dev->i2c_addr, dev->init_conf->h.addr,
                     dev->init_conf->h.data, dev->init_conf->h.len);
    if (ret) {
        return ERROR_I2C;
    }
    ret = dev->write(dev->i2c_addr, dev->init_conf->d.addr,
                     dev->init_conf->d.data, dev->init_conf->d.len);
    if (ret) {
        return ERROR_I2C;
    }
    ret = dev->write(dev->i2c_addr, dev->init_conf->m.addr,
                     dev->init_conf->m.data, dev->init_conf->m.len);
    if (ret) {
        return ERROR_I2C;
    }
    ret = dev->write(dev->i2c_addr, dev->init_conf->s.addr,
                     dev->init_conf->s.data, dev->init_conf->s.len);
    if (ret) {
        return ERROR_I2C;
    }

    ret =
        dev->write(dev->i2c_addr, ZMOD4XXX_ADDR_CMD, &dev->init_conf->start, 1);
    if (ret) {
        return ERROR_I2C;
    }
    /* This section can be change with interrupt for microcontrollers */
    do {
        ret = zmod4xxx_read_status(dev, &zmod4xxx_status);
        if (ret) {
            return ret;
        }
        dev->delay_ms(50);
    } while (zmod4xxx_status & STATUS_SEQUENCER_RUNNING_MASK);

    ret = dev->read(dev->i2c_addr, dev->init_conf->r.addr, data_r,
                    dev->init_conf->r.len);
    if (ret) {
        return ERROR_I2C;
    }

    dev->mox_lr = (uint16_t)(data_r[0] << 8) | data_r[1];
    dev->mox_er = (uint16_t)(data_r[2] << 8) | data_r[3];
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_init_measurement(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret;

    ret = dev->write(dev->i2c_addr, dev->meas_conf->h.addr,
                     dev->meas_conf->h.data, dev->meas_conf->h.len);
    if (ret) {
        return ERROR_I2C;
    }
    ret = dev->write(dev->i2c_addr, dev->meas_conf->d.addr,
                     dev->meas_conf->d.data, dev->meas_conf->d.len);
    if (ret) {
        return ERROR_I2C;
    }
    ret = dev->write(dev->i2c_addr, dev->meas_conf->m.addr,
                     dev->meas_conf->m.data, dev->meas_conf->m.len);
    if (ret) {
        return ERROR_I2C;
    }
    ret = dev->write(dev->i2c_addr, dev->meas_conf->s.addr,
                     dev->meas_conf->s.data, dev->meas_conf->s.len);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_start_measurement(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret;

    ret =
        dev->write(dev->i2c_addr, ZMOD4XXX_ADDR_CMD, &dev->meas_conf->start, 1);
    if (ret) {
        return ERROR_I2C;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_read_adc_result(zmod4xxx_dev_t *dev, uint8_t *adc_result)
{
    zmod4xxx_err ret = 0;
    uint8_t data;

    ret = dev->read(dev->i2c_addr, dev->meas_conf->r.addr, adc_result,
                    dev->meas_conf->r.len);
    if (ret) {
        return ERROR_I2C;
    }

    ret = dev->read(dev->i2c_addr, 0xB7, &data, 1);
    if (ret) {
        return ERROR_I2C;
    }

    if (0 != data) {
        if (STATUS_ACCESS_CONFLICT_MASK & data) {
            return ERROR_ACCESS_CONFLICT;
        } else if (STATUS_POR_EVENT_MASK & data) {
            return ERROR_POR_EVENT;
        }
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_calc_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result,
                                float *rmox)
{
    uint8_t i;
    uint16_t adc_value = 0;
    float *p = rmox;
    float rmox_local = 0;

    for (i = 0; i < dev->meas_conf->r.len; i = i + 2) {
        adc_value = (((uint16_t)(adc_result[i])) << 8);
        adc_value |= (adc_result[i + 1]);
        if (0.0 > (adc_value - dev->mox_lr)) {
            *p = 1e-3;
            p++;
        } else if (0.0 >= (dev->mox_er - adc_value)) {
            *p = 10e9;
            p++;
        } else {
            rmox_local = dev->config[0] * 1e3 *
                         (float)(adc_value - dev->mox_lr) /
                         (float)(dev->mox_er - adc_value);
            *p = rmox_local;
            p++;
        }
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_prepare_sensor(zmod4xxx_dev_t *dev)
{
    zmod4xxx_err ret;

    ret = zmod4xxx_calc_factor(dev);
    if (ret) {
        return ret;
    }
    ret = zmod4xxx_init_sensor(dev);
    if (ret) {
        return ret;
    }
    dev->delay_ms(50);
    ret = zmod4xxx_init_measurement(dev);
    if (ret) {
        return ret;
    }
    return ZMOD4XXX_OK;
}

zmod4xxx_err zmod4xxx_read_rmox(zmod4xxx_dev_t *dev, uint8_t *adc_result,
                                float *rmox)
{
    zmod4xxx_err ret;
    ret = zmod4xxx_read_adc_result(dev, adc_result);
    if (ret) {
        return ret;
    }
    dev->delay_ms(50);
    ret = zmod4xxx_calc_rmox(dev, adc_result, rmox);
    if (ret) {
        return ret;
    }
    return ZMOD4XXX_OK;
}

