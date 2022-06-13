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
#include "ei_light_sensor.h"
#include "ei_device_thingy53.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <cstdint>

#define LOG_MODULE_NAME ei_light
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

const struct device *light_dev;
static float sample[LIGHTSENSOR_VALUES_IN_SAMPLE] = {0};

bool ei_lightsensor_init(void)
{
	light_dev = device_get_binding("BH1749");
	if (!light_dev) {
		LOG_ERR("Devicetree has no BH1749 node");
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
