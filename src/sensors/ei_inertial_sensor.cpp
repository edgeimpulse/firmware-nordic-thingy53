/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include "ei_inertial_sensor.h"
#include "ei_device_thingy53.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <cstdint>

LOG_MODULE_REGISTER(acc_sensor);

static const struct device *const accel_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
static float acc_data[ACC_AXIS_SAMPLED];
static bool inertial_init = false;

/**
 * @brief      Setup accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_acc_init(void)
{
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("sensor: device %s not ready.\n", accel_dev->name);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(accelerometer_sensor) == false) {
        ei_printf("ERR: failed to register Accelerometer sensor!\n");
        return false;
    }

    inertial_init = true;
    return true;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
static bool ei_acc_fetch_sample(void)
{
    struct sensor_value raw_sample[ACC_AXIS_SAMPLED];

    if(inertial_init == false) {
        return false;
    }

    if (sensor_sample_fetch(accel_dev) < 0) {
        LOG_ERR("Sample fetch error");
        return false;
    }

    if (sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, raw_sample) < 0) {
        LOG_ERR("Failed to get accel readings\n");
        return false;
    }

    acc_data[0] = (float)sensor_value_to_double(&raw_sample[0]);
    acc_data[1] = (float)sensor_value_to_double(&raw_sample[1]);
    acc_data[2] = (float)sensor_value_to_double(&raw_sample[2]);

    return true;
}

/**
 * @brief
 *
 * @return float* pointer to internal array with accel data
 */
float *ei_fusion_acc_read_data(int n_samples)
{
    if(ei_acc_fetch_sample() == false) {
        for(int i = 0; i < ACC_AXIS_SAMPLED; i++) {
            acc_data[i] = 0.0f;
        }
    }

    return acc_data;
}
