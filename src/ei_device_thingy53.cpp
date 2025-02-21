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
#include "ei_device_thingy53.h"
#include "flash_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_memory.h"
#include "sensors/ei_microphone.h"
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

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

#if MULTI_FREQ_ENABLED == 1

    if (dev->get_fusioning() == 1) {
        dev->sample_read_callback();
    }
    else {
        uint8_t flag = 0;
        uint8_t i = 0;

        dev->actual_timer += dev->get_sample_interval();  /* update actual time */

        for (i = 0; i < dev->get_fusioning(); i++){
            if (((uint32_t)(dev->actual_timer % (uint32_t)dev->multi_sample_interval.at(i))) == 0) {   /* check if period of sensor is a multiple of actual time*/
                flag |= (1<<i);                                                                     /* if so, time to sample it! */
            }
        }

        if (dev->sample_multi_read_callback != nullptr){
            dev->sample_multi_read_callback(flag);
        }

    }

#else
    dev->sample_read_callback();
#endif
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
    standalone_sensor_list[0].name = "Microphone";
    standalone_sensor_list[0].frequencies[0] = 16000.0f;
    standalone_sensor_list[0].start_sampling_cb = &ei_microphone_sample_start;
    standalone_sensor_list[0].max_sample_length_s = mem->get_available_sample_bytes() / (16000 * sizeof(microphone_sample_t));
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

    LOG_INF("Setting ID = %s", temp);

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
#if MULTI_FREQ_ENABLED == 1
    this->actual_timer = 0;
    this->fusioning = 1;
#endif

    k_timer_start(&sampler_timer, K_MSEC(sample_interval_ms), K_MSEC(sample_interval_ms));

    return true;
}

bool EiDeviceThingy53::stop_sample_thread(void)
{
    k_timer_stop(&sampler_timer);

#if MULTI_FREQ_ENABLED == 1
    this->actual_timer = 0;
    this->fusioning = 0;
#endif

    return true;
}

#if MULTI_FREQ_ENABLED == 1
bool EiDeviceThingy53::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;

    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;

    k_timer_start(&sampler_timer, K_MSEC(this->sample_interval), K_MSEC(this->sample_interval));

    return true;
}
#endif

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


int EiDeviceThingy53::set_wifi_config(const char *ssid, const char *password, const int security)
{
    LOG_INF("Setting WiFi config");
    wifi_ssid = (std::string)ssid;
    wifi_password = (std::string)password;
    wifi_security = (EiWiFiSecurity)security;

    LOG_INF("wifi: save_config");
    save_config();

    return 0;
}
int EiDeviceThingy53::get_wifi_config(char *ssid, char *password, int *security)
{
    // EiDeviceInfo::memory = mem;
    LOG_INF("Getting WiFi config");
    load_config();

    LOG_INF("config loded");
    LOG_INF("wifi_ssdi: %s", wifi_ssid.c_str());
    LOG_INF("wifi_password: %s", wifi_password.c_str());
    LOG_INF("wifi_security: %d", wifi_security);
    // LOG_INF("config not empty");
    strcpy(ssid, wifi_ssid.c_str());
    strcpy(password, wifi_password.c_str());
    *security = wifi_security;

    return 0;
}

/**
 * @brief Overrides ei_classifier_porting version
 * Uses direct uart out to avoid printing of additional \r\r
 * @param format
 * @param ...
 */
void ei_printf(const char *format, ...)
{
    extern const struct device *uart;
    static char print_buf[256] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    uart_fifo_fill(uart, (const uint8_t *)print_buf, r);
}