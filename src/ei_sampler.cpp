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
#include "ei_sampler.h"
#include "ei_device_thingy53.h"
#include "firmware-sdk/ei_device_memory.h"
#include "firmware-sdk/ei_config_types.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ble/ble_nus.h"
#include "ble/ei_ble_com.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <cstdint>
#include <cstdlib>
#include "wifi/ei_ws_client.h"

#define LOG_MODULE_NAME ei_sampler
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

/* Forward declarations ---------------------------------------------------- */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static time_t ei_time(time_t *t);
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);
static bool create_header(sensor_aq_payload_info *payload);

/* Private variables ------------------------------------------------------- */
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t sample_buffer_size;
static uint32_t headerOffset = 0;
static uint8_t write_word_buf[4] __attribute__((aligned(4)));
static int write_addr = 0;
EI_SENSOR_AQ_STREAM stream;

static unsigned char ei_mic_ctx_buffer[1024] __attribute__((aligned(4)));
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_ctx ei_sampler_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    &ei_time,
};

/**
 * @brief      Write sample data to FLASH
 * @details    Write size is always 4 bytes to keep alignment
 *
 * @param[in]  buffer     The buffer
 * @param[in]  size       The size
 * @param[in]  count      The count
 * @param      EI_SENSOR_AQ_STREAM file pointer (not used)
 *
 * @return     number of bytes handled
 */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *)
{
    EiDeviceMemory* mem = EiDeviceInfo::get_device()->get_memory();

    for (size_t i = 0; i < count; i++) {
        write_word_buf[write_addr & 0x3] = *((char *)buffer + i);

        if ((++write_addr & 0x03) == 0x00) {
            mem->write_sample_data(write_word_buf, (write_addr - 4) + headerOffset, 4);
        }
    }

    return count;
}

/**
 * @brief      File handle seed function. Not used
 */
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin)
{
    return 0;
}

/**
 * @brief      File handle time function. Not used
 */
static time_t ei_time(time_t *t)
{
    time_t cur_time = 4564867;
    if (t) *(t) = cur_time;
    return cur_time;
}

/**
 * @brief      Write out remaining data in word buffer to FLASH.
 *             And append CBOR end character.
 */
static void ei_write_last_data(void)
{
    EiDeviceMemory* mem = EiDeviceInfo::get_device()->get_memory();
    uint8_t fill = ((uint8_t)write_addr & 0x03);
    uint8_t insert_end_address = 0;

    if (fill != 0x00) {
        for (uint8_t i = fill; i < 4; i++) {
            write_word_buf[i] = 0xFF;
        }

        mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
        insert_end_address = 4;
    }

    /* Write appending word for end character */
    for (uint8_t i = 0; i < 4; i++) {
        write_word_buf[i] = 0xFF;
    }
    mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset + insert_end_address, 4);
}

bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_memory();
    char * json_string = NULL;
    sensor_aq_payload_info *payload = (sensor_aq_payload_info *)v_ptr_payload;
    // used for optimizing memory comsumpton
    const char *str_sample_settings = "Sampling settings:";
    const char *str_interval = "\tInterval:";
    const char *str_length = "\tLength:";
    const char *str_name = "\tName:";
    const char *str_hmac_key = "\tHMAC Key:";
    const char *str_file_name = "\tFile name:";
    const char *str_unknown_serial_channel = "Unknown serial channel";
    const char *str_failed_to_allocate_page = "Failed to allocate a page buffer to write the hash";

    if(dev->get_serial_channel() == BLE) {
        LOG_INF("%s", str_sample_settings);
        LOG_INF("%s %.5f ms.", str_interval, dev->get_sample_interval_ms());
        LOG_INF("%s %lu ms.", str_length, dev->get_sample_length_ms());
        LOG_INF("%s %s", str_name, dev->get_sample_label().c_str());
        LOG_INF("%s %s", str_hmac_key, dev->get_sample_hmac_key().c_str());
        LOG_INF("%s %s", str_file_name, dev->get_sample_label().c_str());
    }
    else if(dev->get_serial_channel() == UART) {
        LOG_INF("UART COM: Sampling settings: UART communication");
        ei_printf("%s\n", str_sample_settings);
        ei_printf("%s %.5f ms.\n", str_interval, dev->get_sample_interval_ms());
        ei_printf("%s %lu ms.\n", str_length, dev->get_sample_length_ms());
        ei_printf("%s %s\n", str_name, dev->get_sample_label().c_str());
        ei_printf("%s %s\n", str_hmac_key, dev->get_sample_hmac_key().c_str());
        ei_printf("%s %s\n", str_file_name, dev->get_sample_label().c_str());
    }
#ifdef CONFIG_WIFI_NRF700X
    else if(dev->get_serial_channel() == WIFI) {
        LOG_INF("%s", str_sample_settings);
        LOG_INF("%s %.5f ms.", str_interval, dev->get_sample_interval_ms());
        LOG_INF("%s %lu ms.", str_length, dev->get_sample_length_ms());
        LOG_INF("%s %s", str_name, dev->get_sample_label().c_str());
        LOG_INF("%s %s", str_hmac_key, dev->get_sample_hmac_key().c_str());
        LOG_INF("%s %s", str_file_name, dev->get_sample_label().c_str());
        LOG_DBG("WiFi COM: Sample request responce...");
        if(ei_ws_get_connection_status()) {
            ei_ws_send_msg(TxMsgType::SampleStartMsg);
        }
    }
#endif
    else {
        LOG_ERR("%s", str_unknown_serial_channel);
        return false;
    }

    samples_required = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());
    sample_buffer_size = (samples_required * sample_size) * 2;
    current_sample = 0;

    // Minimum delay of 2000 ms for daemon
    uint32_t delay_time_ms = ((sample_buffer_size / mem->block_size) + 1) * mem->block_erase_time;
    if(dev->get_serial_channel() == BLE) {
        LOG_DBG("BLE COM: Starting in %d ms... (or until all flash was erased)", delay_time_ms < 2000 ? 2000 : delay_time_ms);
    }
    else if(dev->get_serial_channel() == UART) {
        LOG_DBG("UART COM: Starting in %d ms...(or until all flash was erased)", delay_time_ms < 2000 ? 2000 : delay_time_ms);
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);
    }
    else if(dev->get_serial_channel() == WIFI) {
        LOG_DBG("WIFI COM: Starting in %d ms... (or until all flash was erased)", delay_time_ms < 2000 ? 2000 : delay_time_ms);
    }
    else{
        LOG_ERR("%s", str_unknown_serial_channel);
    }

    dev->set_state(eiStateErasingFlash);

    if(mem->erase_sample_data(0, sample_buffer_size) != (sample_buffer_size)) {
        if(dev->get_serial_channel() == UART){
            LOG_ERR("UART COM: Failed to erase samples memory");
            ei_printf("ERR: Failed to erase samples memory\n");
        }
        LOG_ERR("Failed to erase samples memory");
        return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
        ei_sleep(2000 - delay_time_ms);
    }

    if (create_header(payload) == false) {
        LOG_ERR("Failed to create header");
        return false;
    }

    if (ei_sample_start(&sample_data_callback, dev->get_sample_interval_ms()) == false) {
        LOG_ERR("Failed to start sampling");
        return false;
    }

    if(dev->get_serial_channel() == BLE) {
        LOG_DBG("BLE COM: Sampling...");
        /*send responce: sampleStarted true*/
        json_string = ei_ble_com_response_sample("sampleStarted", true, NULL);
        ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
        // cJSON_FreeString(json_string);
        k_free(json_string);
    }
    else if(dev->get_serial_channel() == UART) {
        LOG_DBG("UART COM: Sampling...");
        ei_printf("Sampling...\n");
    }
#ifdef CONFIG_WIFI_NRF700X
    else if(dev->get_serial_channel() == WIFI) {
        LOG_DBG("WiFi COM: Sampling...");
        if(ei_ws_get_connection_status()) {
            ei_ws_send_msg(TxMsgType::SampleStartedMsg);
        }
    }
#endif
    else {
        LOG_ERR("%s", str_unknown_serial_channel);
        return false;
    }

    dev->set_state(eiStateSampling);

    while (current_sample < samples_required) {
        ei_sleep(10);
    }
    dev->stop_sample_thread();

    ei_write_last_data();
    write_addr++;

    uint8_t final_byte[] = {0xff};
    int ctx_err = ei_sampler_ctx.signature_ctx->update(ei_sampler_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        LOG_ERR("Failed to update data signature");
        return ctx_err;
    }

    // finish the signing
    ctx_err =
        ei_sampler_ctx.signature_ctx->finish(ei_sampler_ctx.signature_ctx, ei_sampler_ctx.hash_buffer.buffer);

    // load the first page in flash...
    uint8_t *page_buffer = (uint8_t *)ei_malloc(mem->block_size);
    if (!page_buffer) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("%s\n", str_failed_to_allocate_page);
        }
        else if(dev->get_serial_channel() == UART) {
            ei_printf("%s\n", str_failed_to_allocate_page);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("%s\n", str_failed_to_allocate_page);
        }
#endif
        else {
        LOG_ERR("%s", str_unknown_serial_channel);
        }
        return false;
    }

    uint32_t j = mem->read_sample_data(page_buffer, 0, mem->block_size);
    if (j != mem->block_size) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("Failed to read first page (%d)", j);
        }
        else if(dev->get_serial_channel() == UART) {
            LOG_ERR("UART COM: Failed to read first page (%d)", j);
            ei_printf("Failed to read first page (%d)\n", j);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("Failed to read first page (%d)", j);
        }
#endif
        else {
        LOG_ERR("%s", str_unknown_serial_channel);
        }
        ei_free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_sampler_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent
    // in hex) thus only loop over the first half of the bytes as the signature_ctx has written to
    // those
    for (size_t hash_ix = 0; hash_ix < (ei_sampler_ctx.hash_buffer.size / 2); hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by
        // newlib-nano we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_sampler_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_sampler_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    j = mem->erase_sample_data(0, mem->block_size);
    if (j != mem->block_size) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("Failed to erase first page (%d)", j);
        }
        else if(dev->get_serial_channel() == UART){
            LOG_ERR("UART COM: Failed to erase first page (%d)", j);
            ei_printf("Failed to erase first page (%d)\n", j);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("Failed to erase first page (%d)", j);
        }
#endif
        else {
        LOG_ERR("%s", str_unknown_serial_channel);
        }
        ei_free(page_buffer);
        return false;
    }

    j = mem->write_sample_data(page_buffer, 0, mem->block_size);

    ei_free(page_buffer);

    if (j != mem->block_size) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("Failed to write first page with updated hash (%d)", j);
        }
        else if(dev->get_serial_channel() == UART) {
            LOG_ERR("UART COM: Failed to write first page with updated hash (%d)", j);
            ei_printf("Failed to write first page with updated hash (%d)\n", j);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("Failed to write first page with updated hash (%d)", j);
        }
#endif
        else {
        LOG_ERR("%s", str_unknown_serial_channel);
        }
        return false;
    }

    uint32_t my_size = (uint32_t)write_addr + headerOffset;

    if(dev->get_serial_channel() == BLE) {
        LOG_INF("need to upload over BLE");
        /*send responce: sampleUpload true*/
        json_string = ei_ble_com_response_sample("sampleUploading", true, NULL);
        ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
        // cJSON_FreeString(json_string);
        k_free(json_string);
    }
    else if (dev->get_serial_channel() == UART) {
        LOG_DBG("UART COM: need to upload over UART");
        LOG_DBG("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.", my_size);
        ei_printf("Done sampling, total bytes collected: %lu\n", samples_required);
        ei_printf("[1/1] Uploading file to Edge Impulse...\n");
        ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", my_size);
        ei_printf("OK\n");
    }
#ifdef CONFIG_WIFI_NRF700X
    else if(dev->get_serial_channel() == WIFI) {
        if(ei_ws_get_connection_status()) {
            LOG_DBG("Used buffer, from=0, to=%u.\n", my_size);
            ei_ws_send_msg(TxMsgType::SampleUploadingMsg);
            ei_ws_send_sample(0, my_size);
            ei_ws_send_msg(TxMsgType::SampleFinishedMsg);
        }
        else {
            LOG_ERR("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%u.\n", write_addr + headerOffset);
        }
    }
#endif

    return true;
}

/**
 * @brief      Create and write the CBOR header to FLASH
 *
 * @param      payload  The payload
 *
 * @return     True on success
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    sensor_aq_init_none_context(&ei_mic_signing_ctx);//sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    int tr = sensor_aq_init(&ei_sampler_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_sampler_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_sampler_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    tr = mem->write_sample_data((uint8_t*)ei_sampler_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (tr != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    ei_sampler_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

/**
 * @brief      Write samples to FLASH in CBOR format
 *
 * @param[in]  sample_buf  The sample buffer
 * @param[in]  byteLenght  The byte lenght
 *
 * @return     true if all required samples are received. Caller should stop sampling,
 */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    sensor_aq_add_data(&ei_sampler_ctx, (float *)sample_buf, byteLenght / sizeof(float));

    if (++current_sample > samples_required) {
        return true;
    }
    else {
        return false;
    }
}

uint32_t ei_sampler_get_data_size(void)
{
    return write_addr + headerOffset;
}
