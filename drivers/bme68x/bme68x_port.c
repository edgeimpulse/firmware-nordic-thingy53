/**
 * Copyright (C) 2021 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <device.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include "bme68x.h"
#include "bme68x_port.h"

LOG_MODULE_REGISTER(common);

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;
const struct device *i2c_dev;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct i2c_msg msgs[2];
    int ret;

    /* Send the address to read */
    msgs[0].buf = &reg_addr;
    msgs[0].len = 1U;
    msgs[0].flags = I2C_MSG_WRITE;

    /* Read from device. RESTART as neededm and STOP after this. */
    msgs[1].buf = reg_data;
    msgs[1].len = len;
    msgs[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

    ret = i2c_transfer(i2c_dev, &msgs[0], 2, BME68X_I2C_ADDR_LOW);
    if(ret) {
        LOG_ERR("i2c read failed (%d)", ret);
    }

    return ret;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    struct i2c_msg msgs[1];
    int ret;
    uint8_t temp_buf[64] = {0};

    temp_buf[0] = reg_addr;
    memcpy(&temp_buf[1], reg_data, len);

    /* Send the address to read */
    msgs[0].buf = temp_buf;
    msgs[0].len = len + 1;
    msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    ret = i2c_transfer(i2c_dev, &msgs[0], 1, BME68X_I2C_ADDR_LOW);
    if(ret) {
        LOG_ERR("i2c write error (%d)", ret);
    }

    return ret;
}

/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    k_sleep(K_USEC(period));
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            LOG_ERR("API name [%s]  Error [%d] : Null pointer", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            LOG_ERR("API name [%s]  Error [%d] : Communication failure", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            LOG_ERR("API name [%s]  Error [%d] : Incorrect length parameter", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            LOG_ERR("API name [%s]  Error [%d] : Device not found", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            LOG_ERR("API name [%s]  Error [%d] : Self test error", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            LOG_ERR("API name [%s]  Warning [%d] : No new data found", api_name, rslt);
            break;
        default:
            LOG_ERR("API name [%s]  Error [%d] : Unknown error code", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    if (bme != NULL)
    {
        i2c_dev = device_get_binding("I2C_1");
        if (!i2c_dev) {
            LOG_ERR("I2C Device not found!");
            return -1;
        }

        /* Bus configuration : I2C */
        if (intf != BME68X_I2C_INTF)
        {
            LOG_ERR("Interface (%d) not supported!", intf);
            return BME68X_E_DEV_NOT_FOUND;
        }
        dev_addr = BME68X_I2C_ADDR_LOW;
        bme->read = bme68x_i2c_read;
        bme->write = bme68x_i2c_write;
        bme->intf = BME68X_I2C_INTF;
        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        return BME68X_E_NULL_PTR;
    }

    return BME68X_OK;
}
