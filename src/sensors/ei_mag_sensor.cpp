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
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_mag_sensor.h"
#include "ei_device_thingy53.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <cstdint>

LOG_MODULE_REGISTER(mag_sensor);

static const struct device *const mag_dev = DEVICE_DT_GET(DT_ALIAS(magn0));
static float mag_data[MAG_AXIS_SAMPLED];
static bool mag_init = false;

/**
 * @brief      Setup accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_mag_init(void)
{

    if (!device_is_ready(mag_dev)) {
        LOG_ERR("sensor: device %s not ready.\n", mag_dev->name);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(mag_sensor) == false) {
        ei_printf("ERR: failed to register Inertial sensor!\n");
        return false;
    }

    mag_init = true;
    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_mag_fetch_sample(void)
{
    struct sensor_value raw_sample[MAG_AXIS_SAMPLED];

    if(mag_init == false) {
        return false;
    }

    if (sensor_sample_fetch(mag_dev) < 0) {
        LOG_ERR("fetch error");
        return false;
    }

    if (sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, &raw_sample[0]) < 0) {
        LOG_ERR("channel get error");
        return false;
    }
    
    mag_data[0] = (float)sensor_value_to_double(&raw_sample[0]) * 100.0f;
    mag_data[1] = (float)sensor_value_to_double(&raw_sample[1]) * 100.0f;
    mag_data[2] = (float)sensor_value_to_double(&raw_sample[2]) * 100.0f;

    return true;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* pointer to internal array with magnetometer data
 */
float *ei_fusion_mag_read_data(int n_samples)
{
    if(ei_mag_fetch_sample() == false) {        
        for(int i = 0; i < MAG_AXIS_SAMPLED; i++) {
            mag_data[i] = 0.0f;
        }
    }

    return mag_data;
}
