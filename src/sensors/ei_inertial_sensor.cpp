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
#include "ei_inertial_sensor.h"
#include "ei_device_thingy53.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <cstdint>

LOG_MODULE_REGISTER(inertial_sensor);

static void acc_timer_handler(struct k_timer *dummy);
static void acc_work_handler(struct k_work *work);

sampler_callback inertial_cb_sampler;
static const struct device *accel_dev;
static const struct device *mag_dev;
static float imu_data[INERTIAL_AXIS_SAMPLED];
static bool inertial_init = false;
K_TIMER_DEFINE(acc_timer, acc_timer_handler, NULL);
K_WORK_DEFINE(acc_work, acc_work_handler);

/**
 * @brief      Setup accelerometer convert value
 *
 * @return     false if communinication error occured
 */
bool ei_inertial_init(void)
{
	accel_dev = device_get_binding("ADXL362");
	if (accel_dev == NULL) {
		LOG_ERR("Device get binding device");
		return false;
	}

	mag_dev = device_get_binding("BMM150");
	if (mag_dev == NULL) {
		LOG_ERR("Device get binding device");
		return false;
	}

    if(ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
        ei_printf("ERR: failed to register Inertial sensor!\n");
        return false;
    }

    inertial_init = true;
    return true;
}

static bool ei_inertial_fetch_sample(void)
{
    struct sensor_value raw_sample[INERTIAL_AXIS_SAMPLED];

	if(inertial_init == false) {
		return false;
	}

    if (sensor_sample_fetch(accel_dev) < 0) {
        LOG_ERR("Sample fetch error");
        return false;
    }

    if (sensor_sample_fetch(mag_dev) < 0) {
        LOG_ERR("fetch error");
        return false;
    }

    if (sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, raw_sample) < 0) {
        LOG_ERR("Failed to get accel readings\n");
        return false;
    }

    if (sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, &raw_sample[3]) < 0) {
        LOG_ERR("channel get error");
        return false;
    }
    
    imu_data[0] = (float)sensor_value_to_double(&raw_sample[0]);
    imu_data[1] = (float)sensor_value_to_double(&raw_sample[1]);
    imu_data[2] = (float)sensor_value_to_double(&raw_sample[2]);
    imu_data[3] = (float)sensor_value_to_double(&raw_sample[3]) * 100.0f;
    imu_data[4] = (float)sensor_value_to_double(&raw_sample[4]) * 100.0f;
    imu_data[5] = (float)sensor_value_to_double(&raw_sample[5]) * 100.0f;

    return true;
}

static void acc_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&acc_work);
}

static void acc_work_handler(struct k_work *work)
{
	if(ei_inertial_fetch_sample() == false) {
        imu_data[0] = 0.0f;
        imu_data[1] = 0.0f;
        imu_data[2] = 0.0f;
	}

    if(inertial_cb_sampler((const void *)&imu_data[0], SIZEOF_ACCEL_AXIS_SAMPLED) == true) {
        k_timer_stop(&acc_timer);
    }
}

bool ei_accel_sample_start(sampler_callback callsampler, float sample_interval_ms)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    inertial_cb_sampler = callsampler;

    k_timer_start(&acc_timer, K_MSEC(sample_interval_ms), K_MSEC(sample_interval_ms));

    dev->set_sample_interval_ms(sample_interval_ms);

    dev->set_state(eiStateSampling);

    return true;
}

bool ei_accel_setup_data_sampling(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    if (dev->get_sample_interval_ms() < 10.0f ) {
        dev->set_sample_interval_ms(10.0f);
    }

    sensor_aq_payload_info payload = {
        // Unique device ID (optional), set this to e.g. MAC address or device EUI **if** your device has one
        dev->get_device_id().c_str(),
        // Device type (required), use the same device type for similar devices
        dev->get_device_type().c_str(),
        // How often new data is sampled in ms. (100Hz = every 10 ms.)
        dev->get_sample_interval_ms(),
        // The axes which you'll use. The units field needs to comply to SenML units (see https://www.iana.org/assignments/senml/senml.xhtml)
        { { "accX", "m/s2" }, { "accY", "m/s2" }, { "accZ", "m/s2" }, },
    };

    dev->set_state(eiStateErasingFlash);
    ei_sampler_start_sampling(&payload, &ei_accel_sample_start, SIZEOF_ACCEL_AXIS_SAMPLED);
    dev->set_state(eiStateIdle);

    return true;
}

/**
 * @brief 
 * 
 * @return float* pointer to internal array with accel and magnetometer data
 */

float *ei_fusion_inertial_read_data(int n_samples)
{
	if(ei_inertial_fetch_sample() == false) {
        for(int i = 0; i < INERTIAL_AXIS_SAMPLED; i++) {
            imu_data[i] = 0.0f;
        }
	}

    return imu_data;
}

