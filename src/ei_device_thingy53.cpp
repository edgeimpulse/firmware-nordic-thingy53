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
#include "ei_device_thingy53.h"
#include "flash_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_memory.h"
#include "sensors/ei_microphone.h"
#include "sensors/ei_inertial_sensor.h"
#include <bluetooth/addr.h>
#include <bluetooth/bluetooth.h>
#include <dk_buttons_and_leds.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(ei_device_thingy53);

#define LED_RED     DK_LED1
#define LED_GREEN   DK_LED2
#define LED_BLUE    DK_LED3

using namespace std;

static void led_timer_handler(struct k_timer *dummy);
static void led_work_handler(struct k_work *work);
static void sampler_timer_handler(struct k_timer *dummy);
static void sampler_work_handler(struct k_work *work);

K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
K_WORK_DEFINE(led_work, led_work_handler);
K_TIMER_DEFINE(sampler_timer, sampler_timer_handler, NULL);
K_WORK_DEFINE(sampler_work, sampler_work_handler);

static void led_work_handler(struct k_work *work)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    EiState state =dev->get_state();
    static uint8_t animation = 0;
    static uint8_t blink;

    switch(state)
    {
        case eiStateErasingFlash:
            dk_set_led(LED_RED, blink++ % 2);
            break;
        case eiStateSampling:
            dk_set_led(LED_BLUE, blink++ % 2);
            break;
        case eiStateUploading:
            dk_set_led(LED_GREEN, blink++ % 2);
            break;
        case eiStateFinished:
            blink = 0;
            if(animation == 0) {
                animation = 10;
            }
            break;
        default:
            break;
    }

    if(animation == 0) {
        return;
    }

    switch(animation) {
        case 10:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 0);
            dk_set_led(LED_BLUE, 0);
            break;
        case 9:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 0);
            dk_set_led(LED_BLUE, 1);
            break;
        case 8:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 1);
            dk_set_led(LED_BLUE, 0);
            break;
        case 7:
            dk_set_led(LED_RED, 1);
            dk_set_led(LED_GREEN, 0);
            dk_set_led(LED_BLUE, 0);
            break;
        case 6:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 0);
            dk_set_led(LED_BLUE, 1);
            break;
        case 5:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 1);
            dk_set_led(LED_BLUE, 0);
            break;
        case 4:
            dk_set_led(LED_RED, 1);
            dk_set_led(LED_GREEN, 0);
            dk_set_led(LED_BLUE, 0);
            break;
        case 3:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 0);
            dk_set_led(LED_BLUE, 1);
            break;
        case 2:
            dk_set_led(LED_RED, 0);
            dk_set_led(LED_GREEN, 1);
            dk_set_led(LED_BLUE, 0);
            break;
        case 1:
            dev->set_state(eiStateIdle);
            break;
    }
    animation--;
}

static void led_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&led_work);
}

static void sampler_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&sampler_work);
}

static void sampler_work_handler(struct k_work *work)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());

    dev->sample_read_callback();
}

EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDeviceThingy53 dev(&memory);

    return &dev;
}

EiDeviceThingy53::EiDeviceThingy53(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    load_config();

    int err = 0;

    err = dk_leds_init();
    if (err) {
        LOG_ERR("Cannot init LEDs (err: %d)", err);
    }

    device_type = "THINGY53     ";

    standalone_sensor_list[0].name = "Accelerometer";
    standalone_sensor_list[0].frequencies[0] = 20.0f;
    standalone_sensor_list[0].frequencies[1] = 62.5f;
    standalone_sensor_list[0].frequencies[2] = 100.0f;
    standalone_sensor_list[0].start_sampling_cb = &ei_accel_setup_data_sampling;
    standalone_sensor_list[0].max_sample_length_s = mem->get_available_sample_bytes() / (20 * sizeof(SIZEOF_ACCEL_AXIS_SAMPLED));
    standalone_sensor_list[1].name = "Microphone";
    standalone_sensor_list[1].frequencies[0] = 16000.0f;
    standalone_sensor_list[1].start_sampling_cb = &ei_microphone_sample_start;
    standalone_sensor_list[1].max_sample_length_s = mem->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t));
}

EiDeviceThingy53::~EiDeviceThingy53()
{

}

void EiDeviceThingy53::init_device_id(void)
{
    bt_addr_le_t addr;
    size_t id_count = 1;
    char temp[18];

    bt_id_get(&addr, &id_count);

    snprintf(temp, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
        addr.a.val[5], addr.a.val[4], addr.a.val[3],
        addr.a.val[2], addr.a.val[1], addr.a.val[0]);

    LOG_INF("Setting ID = %s", log_strdup(temp));

    device_id = string(temp);
    mac_address = string(temp);
    save_config();
}

void EiDeviceThingy53::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

string EiDeviceThingy53::get_mac_address(void)
{
    return mac_address;
}

bool EiDeviceThingy53::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    this->sample_read_callback = sample_read_cb;
 
    k_timer_start(&sampler_timer, K_MSEC(sample_interval_ms), K_MSEC(sample_interval_ms));

    return true;
}

bool EiDeviceThingy53::stop_sample_thread(void)
{
    k_timer_stop(&sampler_timer);

    return true;
}

void EiDeviceThingy53::set_state(EiState state)
{
    this->state = state;

    dk_set_led(LED_RED, 0);
    dk_set_led(LED_GREEN, 0);
    dk_set_led(LED_BLUE, 0);

    switch(state) {
        case eiStateErasingFlash:
        case eiStateSampling:
        case eiStateUploading:
        case eiStateFinished:
            k_timer_start(&led_timer, K_MSEC(250), K_MSEC(250));
            break;
        case eiStateIdle:
        default:
            k_timer_stop(&led_timer);
            break;
    }
}

EiState EiDeviceThingy53::get_state(void)
{
    return this->state;
}

void EiDeviceThingy53::set_serial_channel(serial_channel_t chan)
{
    last_channel = chan;
}

serial_channel_t EiDeviceThingy53::get_serial_channel(void)
{
    return last_channel;
}

bool EiDeviceThingy53::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list = this->standalone_sensor_list;
    *sensor_list_size = this->standalone_sensor_num;

    return true;
}

uint32_t EiDeviceThingy53::get_data_output_baudrate(void)
{
    return 921600;
}
