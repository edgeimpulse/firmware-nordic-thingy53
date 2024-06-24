/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Permission is hereby granted, ei_free of charge, to any person obtaining a copy
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
#include "ei_microphone.h"
#include "ei_device_thingy53.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"
#include "edge-impulse-sdk/CMSIS/DSP/Include/dsp/support_functions.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ble/ble_nus.h"
#include "ble/ei_ble_com.h"
#include "wifi/ei_ws_client.h"
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <nrfx_pdm.h>
#include <zephyr/kernel.h>
#include "edge-impulse-sdk/dsp/numpy.hpp"

LOG_MODULE_REGISTER(ei_microphone, LOG_LEVEL_DBG);

/* Audio sampling config */
#define AUDIO_SAMPLING_FREQUENCY        16000
#define AUDIO_SAMPLES_PER_MS            (AUDIO_SAMPLING_FREQUENCY / 1000)
#define AUDIO_DSP_SAMPLE_LENGTH_MS      16
#define AUDIO_DSP_SAMPLE_RESOLUTION     (sizeof(short))
#define AUDIO_DSP_SAMPLE_BUFFER_SIZE    (AUDIO_SAMPLES_PER_MS * AUDIO_DSP_SAMPLE_LENGTH_MS * AUDIO_DSP_SAMPLE_RESOLUTION) //4096
#define PDM_CLK_PIN                     41 // 32+9 = p1.09
#define PDM_DIN_PIN                     27 // 32+6 = p0.27

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static bool setup_nrf_pdm(nrfx_pdm_event_handler_t  event_handler);

struct pcm_stream_cfg mic_streams = {
	.pcm_rate = AUDIO_SAMPLING_FREQUENCY,
	// .pcm_width = CHAN_SIZE,
	// .block_size = PCM_BLK_SIZE,
	// .mem_slab = &rx_mem_slab,
};

struct dmic_cfg cfg = {
	.streams = &mic_streams,
};

static const struct device *dmic;
static inference_t inference;
static uint32_t required_samples_size;
static uint32_t headerOffset;
static int16_t pdm_buffer_temp[2][AUDIO_DSP_SAMPLE_BUFFER_SIZE] = {0};
static int16_t *buffer_ready;
static uint32_t current_sample;
K_SEM_DEFINE(data_ready, 0, 1);

static unsigned char ei_mic_ctx_buffer[1024] __attribute__((aligned(4)));
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void*, size_t size, size_t count, EI_SENSOR_AQ_STREAM*)
{
    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM*, long int offset, int origin)
{
    return 0;
}

static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))
    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory* mem = dev->get_memory();
    sensor_aq_init_none_context(&ei_mic_signing_ctx);//sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix >= 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);
    // and update the signature
    ret = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)(ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (ret != 0) {
        ei_printf("Failed to update signature from header (%d)\n", ret);
        return false;
    }
    end_of_header_ix += ref_size;

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

static void ingestion_samples_process(void)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory* mem = dev->get_memory();

    mem->write_sample_data((uint8_t*)buffer_ready, headerOffset + current_sample, sizeof(pdm_buffer_temp)/2);

    ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)buffer_ready, sizeof(pdm_buffer_temp)/2);

    current_sample += sizeof(pdm_buffer_temp)/2;
}

static void ingestion_samples_callback(nrfx_pdm_evt_t const * p_evt)
{
    nrfx_err_t err = NRFX_SUCCESS;
    static uint8_t buf_toggle = 0;

    if(p_evt->error != 0){
        ei_printf("ERR: PDM handler error ocured\n");
        ei_printf("ERR: ingestion_samples_callback error: %d, %d  \n", p_evt->error, p_evt->buffer_requested);
        return;
    }

    if(true == p_evt->buffer_requested){
        buf_toggle ^= 1;
        err = nrfx_pdm_buffer_set(pdm_buffer_temp[buf_toggle], AUDIO_DSP_SAMPLE_BUFFER_SIZE);
        if(err != NRFX_SUCCESS){
            ei_printf("ERR: PDM buffer init error: %d\n", err);
        }
    }

    if(p_evt->buffer_released != NULL) {
        k_sem_give(&data_ready);
        buffer_ready = p_evt->buffer_released;
    }
}

bool ei_microphone_sample_start(void)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    EiDeviceMemory* mem = dev->get_memory();
    uint32_t nrfx_err;
    char * json_string = NULL;
    nrfx_err_t err;
    int ret;
    uint32_t required_samples;
    uint8_t *page_buffer;
    // used for optimizing memory comsumpton
    const char *str_sample_settings = "Sampling settings:";
    const char *str_interval = "\tInterval:";
    const char *str_length = "\tLength:";
    const char *str_name = "\tName:";
    const char *str_hmac_key = "\tHMAC Key:";
    const char *str_file_name = "\tFile name:";
    const char *str_error_mic_init = "Error microphone init";
    const char *str_error_mic_start = "Error microphone srart";
    const char *str_unknown_serial_channel = "Unknown serial channel";
    const char *str_failed_to_finish_signature = "Failed to finish signature";
    const char *str_failed_to_allocate_page = "Failed to allocate a page buffer to write the hash";
    const char *failed_to_read_first_page = "Failed to read first page";
    const char *failed_to_erase_first_page = "Failed to erase first page";
    const char *failed_to_write_first_page = "Failed to write first page with updated hash";


    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    if(dev->get_serial_channel() == BLE) {
        LOG_INF("%s", str_sample_settings);
        LOG_INF("%s %.5f ms.", str_interval, dev->get_sample_interval_ms());
        LOG_INF("%s %lu ms.", str_length, dev->get_sample_length_ms());
        LOG_INF("%s %s", str_name, dev->get_sample_label().c_str());
        LOG_INF("%s %s", str_hmac_key, dev->get_sample_hmac_key().c_str());
        LOG_INF("%s %s", str_file_name, dev->get_sample_label().c_str());
    }
    else if(dev->get_serial_channel() == UART) {
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
    }

    required_samples = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(required_samples & 1) {
        required_samples++;
    }

    required_samples_size = required_samples * sizeof(microphone_sample_t);
    current_sample = 0;

    // Minimum delay of 2000 ms for daemon
    uint32_t delay_time_ms = ((required_samples_size / mem->block_size) + 1) * mem->block_erase_time;
    if(dev->get_serial_channel() == UART) {
        ei_printf("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);
    }
#ifdef CONFIG_WIFI_NRF700X
    else if(dev->get_serial_channel() == WIFI) {
        LOG_INF("Starting in %lu ms... (or until all flash was erased)\n", delay_time_ms < 2000 ? 2000 : delay_time_ms);
    }
#endif
    else {
        LOG_ERR("%s", str_unknown_serial_channel);
    }


    dev->set_state(eiStateErasingFlash);

    // enable mic driver
    nrfx_pdm_uninit();
    if(!setup_nrf_pdm(ingestion_samples_callback)) {
        LOG_ERR("%s", str_error_mic_init);

        ei_printf("%s\n", str_error_mic_init);
        return false;
    }

    // start sampling now to fix the issue with initial 'click noise'
    err = nrfx_pdm_start();
    if(err != NRFX_SUCCESS){
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("%s", str_error_mic_start);
        }
        else if(dev->get_serial_channel() == UART) {
            ei_printf("%s\n", str_error_mic_start);
        }
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("%s\n", str_error_mic_start);
        }
        else {
            LOG_ERR("%s", str_unknown_serial_channel);
        }
        return false;
    }

    if(mem->erase_sample_data(0, required_samples_size) != (required_samples_size)) {
        LOG_ERR("Failed to erase memory (required %d B)", required_samples_size);
        //return false;
    }

    // if erasing took less than 2 seconds, wait additional time
    if(delay_time_ms < 2000) {
        ei_sleep(2000 - delay_time_ms);
    }

    /* create and write header into memory */
    if (create_header(&payload) == false) {
        return false;
    }

    if(dev->get_serial_channel() == BLE) {
        char *json_string;
        LOG_INF("Sampling...");
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

    while (current_sample < required_samples_size) {
        k_sem_take(&data_ready, K_FOREVER);
        ingestion_samples_process();
    };

    nrfx_err = nrfx_pdm_stop();
    if(nrfx_err != NRFX_SUCCESS) {
        ei_printf("ERR: PDM Could not stop PDM sampling, error = %d", nrfx_err);
    }

    ret = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ret != 0) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("%s (%d)", str_failed_to_finish_signature, ret);
        }
        else if(dev->get_serial_channel() == UART) {
            ei_printf("%s (%d)\n", str_failed_to_finish_signature, ret);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("%s (%d)\n", str_failed_to_finish_signature, ret);
        }
#endif
        else {
            LOG_ERR("%s", str_unknown_serial_channel);
        }
        return false;
    }

    // load the first page in flash...
    page_buffer = (uint8_t*)ei_malloc(mem->block_size);
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
        ei_free(page_buffer);
        return false;
    }

    ret = mem->read_sample_data(page_buffer, 0, mem->block_size);
    if (ret != mem->block_size) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("%s (read %d, requersted %d)\n", failed_to_read_first_page, ret, mem->block_size);
        }
        else if(dev->get_serial_channel() == UART) {
            ei_printf("%s (read %d, requersted %d)\n", failed_to_read_first_page, ret, mem->block_size);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("%s (read %d, requersted %d)\n", failed_to_read_first_page, ret, mem->block_size);
        }
#endif
        else {
            LOG_ERR("%s", str_unknown_serial_channel);
        }
        ei_free(page_buffer);
        return false;
    }

    // update the hash
    uint8_t *hash = ei_mic_ctx.hash_buffer.buffer;
    // we have allocated twice as much for this data (because we also want to be able to represent in hex)
    // thus only loop over the first half of the bytes as the signature_ctx has written to those
    for (size_t hash_ix = 0; hash_ix < ei_mic_ctx.hash_buffer.size / 2; hash_ix++) {
        // this might seem convoluted, but snprintf() with %02x is not always supported e.g. by newlib-nano
        // we encode as hex... first ASCII char encodes top 4 bytes
        uint8_t first = (hash[hash_ix] >> 4) & 0xf;
        // second encodes lower 4 bytes
        uint8_t second = hash[hash_ix] & 0xf;

        // if 0..9 -> '0' (48) + value, if >10, then use 'a' (97) - 10 + value
        char first_c = first >= 10 ? 87 + first : 48 + first;
        char second_c = second >= 10 ? 87 + second : 48 + second;

        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 0] = first_c;
        page_buffer[ei_mic_ctx.signature_index + (hash_ix * 2) + 1] = second_c;
    }

    ret = mem->erase_sample_data(0, mem->block_size);
    if (ret != mem->block_size) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("%s (read %d, requested %d)", failed_to_erase_first_page,ret, mem->block_size);
        }
        else if(dev->get_serial_channel() == UART){
            ei_printf("%s (read %d, requested %d)\n", failed_to_erase_first_page, ret, mem->block_size);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI){
            LOG_ERR("%s (read %d, requested %d)\n", failed_to_erase_first_page, ret, mem->block_size);
        }
#endif
        else {
            LOG_ERR("%s", str_unknown_serial_channel);
        }
        free(page_buffer);
        return false;
    }

    ret = mem->write_sample_data(page_buffer, 0, mem->block_size);
    free(page_buffer);
    if (ret != mem->block_size) {
        if(dev->get_serial_channel() == BLE) {
            LOG_ERR("%s (read %d, requested %d)", failed_to_write_first_page, ret, mem->block_size);
        }
        else if(dev->get_serial_channel() == UART) {
            ei_printf("%s (read %d, requested %d)\n", failed_to_write_first_page, ret, mem->block_size);
        }
#ifdef CONFIG_WIFI_NRF700X
        else if(dev->get_serial_channel() == WIFI) {
            LOG_ERR("%s (read %d, requested %d)\n", failed_to_write_first_page, ret, mem->block_size);
        }
#endif
        else {
            LOG_ERR("%s", str_unknown_serial_channel);
        }
        return false;
    }

    uint32_t my_size = required_samples_size + headerOffset;

    if(dev->get_serial_channel() == BLE) {
        LOG_INF("need to upload over BLE");
        /*send responce: sampleUpload true*/
        json_string = ei_ble_com_response_sample("sampleUploading", true, NULL);
        ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
        // cJSON_FreeString(json_string);
        k_free(json_string);
    }
    else if(dev->get_serial_channel() == UART) {
        ei_printf("Done sampling, total bytes collected: %u\n", required_samples_size);
        ei_printf("[1/1] Uploading file to Edge Impulse...\n");
        ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=%lu, to=%lu.\n", 0, my_size);
        ei_printf("OK\n");
    }
#ifdef CONFIG_WIFI_NRF700X
    else if(dev->get_serial_channel() == WIFI) {
        if(ei_ws_get_connection_status()) {
            LOG_DBG("Used buffer, from=0, to=%u.\n", my_size);
            ei_ws_send_msg(TxMsgType::SampleUploadingMsg);
            ei_ws_send_sample(0, my_size, false);
            ei_ws_send_msg(TxMsgType::SampleFinishedMsg);
        }
        else {
            LOG_ERR("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%u.\n", my_size);
        }
    }
#endif
    return true;
}

static void audio_buffer_inference_callback(void *buffer, uint32_t n_bytes)
{
    int16_t *samples = (int16_t *)buffer;

    for(uint32_t i = 0; i < (n_bytes >> 1); i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = samples[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void inference_samples_callback(nrfx_pdm_evt_t const * p_evt)
{
    nrfx_err_t err = NRFX_SUCCESS;
    static uint8_t buf_toggle = 0;

    if(p_evt->error != 0){
        ei_printf("PDM handler error ocured\n");
        ei_printf("pdm_data_handler error: %d, %d  \n", p_evt->error, p_evt->buffer_requested);
        return;
    }
    if(true == p_evt->buffer_requested){
        buf_toggle ^= 1;
        err = nrfx_pdm_buffer_set(pdm_buffer_temp[buf_toggle], AUDIO_DSP_SAMPLE_BUFFER_SIZE);
        if(err != NRFX_SUCCESS){
            ei_printf("PDM buffer init error: %d\n", err);
        }
    }
    if(p_evt->buffer_released != NULL){
        // audio_buffer_inference_callback(&pdm_buffer_temp[buf_toggle], sizeof(pdm_buffer_temp)/2);
        audio_buffer_inference_callback(p_evt->buffer_released, sizeof(pdm_buffer_temp)/2);
    }
}

int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr)
{
    ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    inference.buf_ready = 0;

    return 0;
}

bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    nrfx_err_t err;

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(microphone_sample_t));
    if(inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    nrfx_pdm_uninit();
    if(!setup_nrf_pdm(inference_samples_callback)) {
        return false;
    }

    err = nrfx_pdm_start();
    if(err != NRFX_SUCCESS){
        return false;
    }

    return true;
}

bool ei_microphone_inference_is_recording(void)
{
    return inference.buf_ready == 0;
}

void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
}

bool ei_microphone_inference_end(void)
{
    nrfx_err_t err;

    err = nrfx_pdm_stop();
    if(err != NRFX_SUCCESS) {
        ei_printf("ERR: PDM Could not stop PDM sampling, error = %d", err);
    }

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    // ei_free(sample_buffer);

    return true;
}

static bool setup_nrf_pdm(nrfx_pdm_event_handler_t  event_handler)
{
    nrfx_err_t err;

    /* PDM driver configuration */
    nrfx_pdm_config_t config_pdm = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DIN_PIN);
    config_pdm.clock_freq = NRF_PDM_FREQ_1280K;
    config_pdm.ratio = NRF_PDM_RATIO_80X;
    config_pdm.edge = NRF_PDM_EDGE_LEFTRISING;
    config_pdm.gain_l = NRF_PDM_GAIN_MAXIMUM;
    config_pdm.gain_r = NRF_PDM_GAIN_MAXIMUM;

    IRQ_DIRECT_CONNECT(PDM0_IRQn, 6, nrfx_pdm_irq_handler, 0);

    err = nrfx_pdm_init(&config_pdm, event_handler);
    if(err != NRFX_SUCCESS){
        LOG_ERR("Failed to init PDM");
        return false;
    }
    else{
        LOG_DBG("PDM init ok");
        return true;
    }
}

bool ei_microphone_init(void)
{
    int ret;

    dmic = device_get_binding("VM3011");
	if (dmic == NULL)
	{
		LOG_ERR("Failed to get dmic device binding");
		return false;
	}

    ret = dmic_trigger(dmic, DMIC_TRIGGER_RESET);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to reset DMIC", ret);
		return false;
	}

	ret = dmic_configure(dmic, &cfg);
	if (ret) {
		LOG_ERR("DMIC configure error, error %d", ret);
        return false;
	}

    LOG_DBG("Microphone init ok");
    return true;
}

uint32_t ei_microphone_get_data_size(void)
{
    return (required_samples_size + headerOffset);
}
