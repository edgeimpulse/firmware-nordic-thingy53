/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAenvironmentES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_environment_sensor.h"
#include "bme68x/bme68x.h"
#include "bme68x/bme68x_port.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr.h>
#include <logging/log.h>
#include <cstdint>

#define LOG_MODULE_NAME ei_environment
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static float sample[ENVIRONMENT_N_AXIS_SAMPLED];
static bool environment_init = false;
static struct bme68x_dev bme;
static struct bme68x_conf conf;
static struct bme68x_heatr_conf heatr_conf;

/* private functions */
bool ei_environment_fetch_sample(void);

/**
 * @brief      Setup environmentneetometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_environment_init(void)
{
    int8_t rslt;

    rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    if(rslt != BME68X_OK) {
        bme68x_check_rslt("bme68x_interface_init", rslt);
        return false;
    }

    rslt = bme68x_init(&bme);
    if(rslt != BME68X_OK) {
        bme68x_check_rslt("bme68x_init", rslt);
        return false;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    if(rslt != BME68X_OK) {
        bme68x_check_rslt("bme68x_set_conf", rslt);
        return false;
    }

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 320;
    heatr_conf.heatr_dur = 197;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    if(rslt != BME68X_OK) {
        bme68x_check_rslt("bme68x_set_heatr_conf", rslt);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(environment_sensor) == false) {
        ei_printf("ERR: failed to register Environmental sensor!\n");
        return false;
    }

    environment_init = true;
    
    return true;
}

bool ei_environment_fetch_sample(void)
{
    int8_t rslt;
    struct bme68x_data data;
    uint32_t del_period;
    uint8_t n_fields;

    if(!environment_init) {
        return false;
    }

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    if(rslt != BME68X_OK) {
        bme68x_check_rslt("bme68x_set_op_mode", rslt);
        return false;
    }

    /* Calculate delay period in microseconds */
    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    if(n_fields == 0) {
        bme68x_check_rslt("bme68x_get_data", rslt);
        return false;
    }

    sample[0] = data.temperature;
    /* converet Pa to kPa */
    sample[1] = data.pressure / 1000.0f;
    sample[2] = data.humidity;
    /* converet Ohm to MOhm */
    sample[3] = data.gas_resistance / 1000000.0f;

    return true;
}

float *ei_fusion_environment_read_data(int n_samples)
{
    if(ei_environment_fetch_sample() == false) {
        sample[0] = 0.0f;
        sample[1] = 0.0f;
        sample[2] = 0.0f;
        sample[3] = 0.0f;
    }

    return sample;
}
