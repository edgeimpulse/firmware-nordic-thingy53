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

#ifndef EI_DEVICE_NORDIC_NRF5X_H
#define EI_DEVICE_NORDIC_NRF5X_H

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include <cstdint>

typedef enum {
    UART = 0,
    BLE
} serial_channel_t;

class EiDeviceThingy53 : public EiDeviceInfo
{
private:
    // EiDeviceThingy53() = delete;
    std::string mac_address = "00:11:22:33:44:55:66";
    EiState state;
    static const int standalone_sensor_num = 2;
    ei_device_sensor_t standalone_sensor_list[standalone_sensor_num];
    serial_channel_t last_channel;

public:	
	EiDeviceThingy53(void);
    EiDeviceThingy53(EiDeviceMemory* mem);
    ~EiDeviceThingy53();

    std::string get_mac_address(void);

    void init_device_id(void);
    void clear_config(void);

    void set_state(EiState state) override;
    EiState get_state(void);

    void set_serial_channel(serial_channel_t chan);
    serial_channel_t get_serial_channel(void);

    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;
    void (*sample_read_callback)(void);

    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size) override;
    uint32_t get_data_output_baudrate(void) override;
};

#endif /* EI_DEVICE_NORDIC_NRF5X_H */
