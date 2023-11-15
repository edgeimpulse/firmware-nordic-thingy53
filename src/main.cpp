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

#include "ei_at_handlers.h"
#include "ei_device_thingy53.h"
#include "ble/ei_ble_com.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "inference/ei_run_impulse.h"
#include "sensors/ei_environment_sensor.h"
#include "sensors/ei_inertial_sensor.h"
#include "sensors/ei_light_sensor.h"
#include "sensors/ei_microphone.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <nrfx_clock.h>
#include <zephyr/kernel.h>

#define LOG_MODULE_NAME main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

const struct device *uart;

void ei_putchar(char c)
{
    // it is ~30% faster than printf
    uart_fifo_fill(uart, (const uint8_t*)&c, 1);
}

int main(void)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    uint8_t rcv_char;
	int ret = 0;
    ATServer *at;

    /* output of printf is output immediately without buffering */ 
    setvbuf(stdout, NULL, _IONBF, 0);

    /* Switch CPU core clock to 128 MHz */
    nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

    /* Initialize board uart */
	uart = device_get_binding("CDC_ACM_0");
	if (!uart) {
        LOG_ERR("Failed to init CDC_ACM_0\n");
	}

    /* Setup the inertial sensor */
    if(ei_inertial_init() == false) {
        LOG_ERR("Inerial sensor communication error occured");
    }

    /* Setup the light sensor */
    if(ei_lightsensor_init() == false) {
        LOG_ERR("Light sensor communication error occured");
    }

    /* Setup the env sensor */
    if(ei_environment_init() == false) {
        LOG_ERR("Environment sensor communication error occured");
    }

    /* Setup the microphone sensor */
    if (ei_microphone_init() == false) {
        LOG_ERR("Microphone init failed!");
    }

    // init BLE stack
    ei_ble_com_init();

    // we have to do this later because the BLE stack is not available on boot
    dev->init_device_id();

    dev->set_state(eiStateFinished);

    ei_printf("Hello from Edge Impulse\r\n"
              "Compiled on %s %s\r\n", __DATE__, __TIME__);

    at = ei_at_init();
    at->print_prompt();

    while(1) {
        while(uart_fifo_read(uart, &rcv_char, 1) == 1) {
            if(is_inference_running() && rcv_char == 'b') {
                ei_stop_impulse();
                at->print_prompt();
                continue;
            }
            dev->set_serial_channel(UART);
            at->handle(rcv_char);
        }
        ei_sleep(1);
    }
}
