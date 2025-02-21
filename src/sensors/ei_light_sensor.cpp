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
#include "ei_light_sensor.h"
#include "ei_device_thingy53.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <cstdint>

#define LOG_MODULE_NAME ei_light
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static const struct device *const light_dev = DEVICE_DT_GET(DT_ALIAS(light0));
static float sample[LIGHTSENSOR_VALUES_IN_SAMPLE] = {0};

bool ei_lightsensor_init(void)
{
    if (!device_is_ready(light_dev)) {
        LOG_ERR("sensor: device %s not ready.\n", light_dev->name);
        return false;
    }

    if(ei_add_sensor_to_fusion_list(light_sensor) == false) {
        ei_printf("ERR: failed to register Light sensor!\n");
        return false;
    }

    return true;
}

float *ei_fusion_light_read_data(int n_samples)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    if(dev->get_state() != eiStateIdle) {
        LOG_WRN("Disabling LED singalling due to light interferences!");
        dev->set_state(eiStateIdle);
        ei_sleep(500);
    }

    int ret;
    struct sensor_value raw_sample[LIGHTSENSOR_VALUES_IN_SAMPLE] = {0};

    ret = sensor_sample_fetch_chan(light_dev, SENSOR_CHAN_ALL);
    if (ret != 0) {
        LOG_ERR("Sample fetch error (%d)", ret);
        return sample;
    }

    ret = sensor_channel_get(light_dev, SENSOR_CHAN_RED, &raw_sample[0]);
    if (ret != 0) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_RED (%d)", ret);
    }

    ret = sensor_channel_get(light_dev, SENSOR_CHAN_GREEN, &raw_sample[1]);
    if (ret != 0) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_GREEN (%d)", ret);
    }

    ret = sensor_channel_get(light_dev, SENSOR_CHAN_BLUE, &raw_sample[2]);
    if (ret != 0) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_BLUE (%d)", ret);
    }

    ret = sensor_channel_get(light_dev, SENSOR_CHAN_IR, &raw_sample[3]);
    if (ret != 0) {
        LOG_ERR("sensor_channel_get failed on SENSOR_CHAN_IR (%d)", ret);
    }

    for(int i=0; i<LIGHTSENSOR_VALUES_IN_SAMPLE; i++) {
        sample[i] = (float)sensor_value_to_double(&raw_sample[i]);
    }

    return sample;
}
