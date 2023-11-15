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

#include "ei_ble_com.h"
#include <string>
#include <cstring>
#include <vector>
#include <zephyr/kernel.h>
#include "cJSON.h"
#include "ei_device_thingy53.h"
#include "ei_sampler.h"
#include "sensors/ei_microphone.h"
#include "inference/ei_run_impulse.h"
#include "ble_nus.h"
#include "firmware-sdk/ei_config_types.h"
#include "firmware-sdk/ei_fusion.h"
#include "firmware-sdk/at_base64_lib.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ei_ble_com, LOG_LEVEL_DBG);

#define BLE_CONNECTION 1
//TODO: need to use one from ei_config.h
#define EDGE_IMPULSE_MAX_FREQUENCIES_L (5)
#define EI_DATA_TOKEN "@@EIDATA@@"

using namespace std;

// the send_buffer_size is determiden by buffer in read_sample_buffer() at ei_device_nordic_nrf53.cpp
// we need 4*(n/3) rounded up to multiply of 4 (where n=513)
static const int send_buffer_size = 684;
static cJSON_Hooks _cjson_hooks;
static uint8_t send_buffer[send_buffer_size];
static uint32_t send_buffer_wr_idx;

static void *malloc_fn_hook(size_t sz) { return k_malloc(sz); }
static void free_fn_hook(void *p_ptr) { k_free(p_ptr); }
static void ei_ble_encode_send(uint8_t *buffer, size_t size);
void ble_nus_append_buffer(char c);
static char* ei_inference_start_ack(bool success);
static char* ei_inference_stop_ack(bool success);

static bool read_sample_buffer(size_t begin, size_t length, void(*data_fn)(uint8_t*, size_t))
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();
    EiDeviceMemory *mem = dev->get_memory();
    const int buffer_size = 513;
    uint8_t *buffer;
    size_t pos = begin;
    size_t bytes_left = length;
    bool retVal;

    buffer = (uint8_t*)ei_malloc(buffer_size);
    if(buffer == nullptr) {
        LOG_ERR("Failed to allocate buffer for sending data");
        return false;
    }

    dev->set_state(eiStateUploading);

    while (1) {
        size_t bytes_to_read = buffer_size;
        if (bytes_to_read > bytes_left) {
            bytes_to_read = bytes_left;
        }
        if (bytes_to_read == 0) {
            retVal = true;
            break;
        }

        if(mem->read_sample_data(buffer, pos, bytes_to_read) != bytes_to_read) {
            LOG_ERR("Failed to read samples from memory");
            retVal = false;
            break;
        }
        data_fn(buffer, bytes_to_read);

        pos += bytes_to_read;
        bytes_left -= bytes_to_read;
    }

    dev->set_state(eiStateFinished);

    ei_free(buffer);
    return retVal;
}

// {
//     "type": "http",
//     "address": "https://ingestion.edgeimpulse.com/api/training/data",
//     "method": "POST",
//     "headers": {
//         "x-api-key": "ei_12389211",
//         "x-label": "wave",
//         "x-file-name": "wave.cbor"
//         "x-disallow-duplicates": "0"
//     },
//     "body": "base64 representation of the payload"
// }
char * ei_ble_com_sample_upload(char * path, const char * api_key, char * label, const char * disallow_duplicates, const char* content_type) {
    char *string = NULL;
    const uint8_t buf_size = 128;
    char buf[buf_size];
    cJSON *headers_o = NULL;

    cJSON *handshake = cJSON_CreateObject();

    if(NULL == cJSON_AddStringToObject(handshake, "type", "http")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    snprintf(buf, buf_size, "https://ingestion.edgeimpulse.com%s", path);
    if(NULL == cJSON_AddStringToObject(handshake, "address", buf)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(handshake, "method", "POST")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    headers_o = cJSON_AddObjectToObject(handshake, "headers");
    if(NULL == headers_o){
        LOG_ERR("error: cJSON_AddObjectToObject");
    }

    if(NULL == cJSON_AddStringToObject(headers_o, "x-api-key", api_key)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(headers_o, "x-label", label)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    snprintf(buf, buf_size, "%s.cbor", label);
    if(NULL == cJSON_AddStringToObject(headers_o, "x-file-name", buf)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(headers_o, "Content-Type", content_type)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(headers_o, "x-disallow-duplicates", disallow_duplicates)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(handshake, "body", EI_DATA_TOKEN)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    string = cJSON_PrintUnformatted(handshake);
    if (string == NULL)
    {
        LOG_WRN("Failed to print monitor.\n");
    }

    cJSON_Delete(handshake);
    return string;
}

char * ei_ble_com_response_sample (const char * sample_name, bool sample, uint8_t * error)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    char *string = NULL;
    cJSON *message = NULL;
    cJSON *sample_l = NULL;

   cJSON *handshake = cJSON_CreateObject();

    if(NULL == cJSON_AddStringToObject(handshake, "type", "ws")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(handshake, "direction", "tx")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(handshake, "address", dev->get_management_url().c_str())){
        LOG_ERR("error: cJSON_AddStringToObject");
    }    
    
    message = cJSON_AddObjectToObject(handshake, "message");
    if(NULL == message){
        LOG_ERR("error: cJSON_AddObjectToObject");
    }

    sample_l = cJSON_AddBoolToObject(message, sample_name, (const cJSON_bool)sample);
    if(NULL == sample_l){
        LOG_ERR("error: cJSON_AddBoolToObject");
    }

    string = cJSON_PrintUnformatted(handshake);
    if (string == NULL)
    {
        LOG_WRN("Failed to print monitor.\n");
    }

    cJSON_Delete(handshake);
    return string;
}

char* ei_ble_connect_handshake(void)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    const ei_device_sensor_t *list;
    size_t list_size;
    char *string = NULL;
    cJSON *message = NULL;
    cJSON *hello = NULL;
    cJSON *sensors = NULL;
    cJSON *frequencies = NULL;
    cJSON *handshake = cJSON_CreateObject();

    if(NULL == cJSON_AddStringToObject(handshake, "type", "ws")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(handshake, "direction", "tx")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(handshake, "address", dev->get_management_url().c_str())){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    message = cJSON_AddObjectToObject(handshake, "message");
    if(NULL == message){
        LOG_ERR("error: cJSON_AddObjectToObject");
    }

    hello = cJSON_AddObjectToObject(message, "hello");
    if(NULL == hello){
        LOG_ERR("error: cJSON_AddObjectToObject");
    }

    if(NULL == cJSON_AddNumberToObject(hello, "version", 3)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(hello, "apiKey", dev->get_upload_api_key().c_str())){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(hello, "deviceId", dev->get_device_id().c_str())){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(hello, "deviceType", dev->get_device_type().c_str())){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(hello, "fw_version", CONFIG_MCUBOOT_IMAGE_VERSION)){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(NULL == cJSON_AddStringToObject(hello, "connection", "ip")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    sensors = cJSON_AddArrayToObject(hello, "sensors");
    if(NULL == sensors){
        LOG_ERR("error: cJSON_AddArrayToObject");
    }

    if (dev->get_sensor_list((const ei_device_sensor_t **)&list, &list_size) == false) {
        LOG_WRN("Failed to get sensor list");
        return NULL;
    }

    //add all sensor data
    for(size_t i = 0; i < list_size; i++){
        cJSON *sensor = cJSON_CreateObject();
        if(NULL == cJSON_AddStringToObject(sensor, "name", list[i].name)){
            LOG_ERR("error: cJSON_AddArrayToObject");
        }
        if(NULL == cJSON_AddNumberToObject(sensor, "maxSampleLengthS", (const int)list[i].max_sample_length_s)) {
            LOG_ERR("error: cJSON_AddNumberToObject");
        }
        frequencies = cJSON_AddArrayToObject(sensor, "frequencies");
        if(frequencies == NULL) {
            LOG_ERR("error: cJSON_AddArrayToObject");
        }
        for (int j = 0; j < EDGE_IMPULSE_MAX_FREQUENCIES_L; j++){
            if (list[i].frequencies[j] != 0.0f){
                cJSON_AddItemToArray(frequencies, cJSON_CreateNumber((const double) list[i].frequencies[j]));
            }
        }
        cJSON_AddItemToArray(sensors, sensor);
    }
    /************************************************************/
    const vector<fused_sensors_t> sens_list = ei_get_sensor_fusion_list();
    for (auto it = sens_list.begin(); it != sens_list.end(); it++) {
        cJSON *sensor = cJSON_CreateObject();
        cJSON *frequencies = NULL;
        if(NULL == cJSON_AddStringToObject(sensor, "name", it->name.c_str())){
            LOG_ERR("error: cJSON_AddArrayToObject");
        } 
        if(NULL == cJSON_AddNumberToObject(sensor, "maxSampleLengthS", (unsigned int)it->max_sample_length)) {
            LOG_ERR("error: cJSON_AddNumberToObject");
        }
        frequencies = cJSON_AddArrayToObject(sensor, "frequencies");
        if(frequencies == NULL) {
            LOG_ERR("error: cJSON_AddArrayToObject");
        }
        else {
            for (auto freq = it->frequencies.begin(); freq != it->frequencies.end(); freq++) {
                cJSON_AddItemToArray(frequencies, cJSON_CreateNumber((const double)*freq));
            }
        }
        cJSON_AddItemToArray(sensors, sensor);
    }

    if(NULL == cJSON_AddFalseToObject(hello, "supportsSnapshotStreaming")){
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    string = cJSON_PrintUnformatted(handshake);
    if (string == NULL)
    {
        LOG_WRN("Failed to print monitor.\n");
    }

    cJSON_Delete(handshake);
    return string;
}

static void ei_ble_configure(const char* apiKey, const char* address)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    char *json_string = NULL;

    dev->set_upload_api_key(string(apiKey));
    dev->set_upload_path(string(address));
    json_string = ei_ble_connect_handshake();
    if(!json_string) {
        LOG_ERR("Failed to create handshake response!");
    }
    ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
    // cJSON_FreeString(json_string);
    k_free(json_string);
}

static void sample_start(char *sensor_name)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    const ei_device_sensor_t *list;
    size_t list_size;

    if (dev->get_sensor_list((const ei_device_sensor_t **)&list, &list_size) == false) {
        LOG_ERR("Failed to get sensor list");
        return;
    }

    for (size_t ix = 0; ix < list_size; ix++) {
        if (strcmp(list[ix].name, sensor_name) == 0) {
            bool r = list[ix].start_sampling_cb();
            if (!r) {
                LOG_ERR("Failed to start sampling\n");
            }
            return;
        }
    }

    if (ei_connect_fusion_list(sensor_name, SENSOR_FORMAT)) {
        int ret = ei_fusion_setup_data_sampling();
        if (!ret) {
            LOG_ERR("Failed to start sampling");
        }
        return;
    }

    LOG_ERR("Failed to find sensor '%s' in the sensor list", sensor_name);
}

static void ei_ble_sample(const cJSON *sample_config)
{
    EiDeviceInfo* dev = EiDeviceInfo::get_device();
    cJSON *label_o = NULL;
    cJSON *length_o = NULL;
    cJSON *hmacKey_o = NULL;
    cJSON *interval_o = NULL;
    cJSON *sensor_o = NULL;
    cJSON *path_o = NULL;
    float interval;
    uint32_t length;
    char * json_string = NULL;
    char* pos_token;
    const uint8_t content_type_size = 32;
    char content_type[content_type_size];
    uint32_t samples_size;

    LOG_INF("%s", __FUNCTION__);

    interval_o = cJSON_GetObjectItem(sample_config, "interval");
    if (cJSON_IsNumber(interval_o)) {
        interval = (float)interval_o->valuedouble;
        LOG_INF("interval: %d (us)", (uint32_t)(interval*1000));

    }
    else{
        LOG_WRN("issue with parameter interval");
        return;
    }

    length_o = cJSON_GetObjectItemCaseSensitive(sample_config, "length");
    if (cJSON_IsNumber(length_o)) {
        length = (uint32_t)length_o->valuedouble;
        LOG_INF("length_o: %d", length);

    }
    else{
        LOG_WRN("issue with parameter length");
        return;
    }

    hmacKey_o = cJSON_GetObjectItemCaseSensitive(sample_config, "hmacKey");
    if (cJSON_IsString(hmacKey_o) && (hmacKey_o != NULL)) {
        LOG_INF("hmackey: %s", cJSON_GetStringValue(hmacKey_o));
    }
    else{
        LOG_WRN("issue with parameter hmackey");
        return;
    }

    label_o = cJSON_GetObjectItemCaseSensitive(sample_config, "label");
    if (cJSON_IsString(label_o) && (label_o != NULL)) {
        LOG_INF("label %s", cJSON_GetStringValue(label_o));
    }
    else{
        LOG_WRN("issue with parameter label");
        return;
    }

    path_o = cJSON_GetObjectItemCaseSensitive(sample_config, "path");
    if (cJSON_IsString(path_o) && (path_o != NULL)) {
        LOG_INF("path %s", cJSON_GetStringValue(path_o));
    }
    else{
        LOG_WRN("issue with parameter path");
        return;
    }

    LOG_INF("sample info data colledted");
    dev->set_sample_label(cJSON_GetStringValue(label_o));
    dev->set_sample_interval_ms(interval);
    dev->set_sample_length_ms(length);
    dev->set_sample_hmac_key(cJSON_GetStringValue(hmacKey_o));

    sensor_o = cJSON_GetObjectItemCaseSensitive(sample_config, "sensor");
    if (cJSON_IsString(sensor_o) && (sensor_o != NULL)) {
        LOG_INF("sensor %s", cJSON_GetStringValue(sensor_o));
    }
    else{
        LOG_WRN("issue with parameter sensor");
        return;
    }

    ei_sleep(10);
    /*send responce: sample true*/
    json_string = ei_ble_com_response_sample("sample", true, NULL);
    ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
    // cJSON_FreeString(json_string);
    k_free(json_string);

    sample_start(cJSON_GetStringValue(sensor_o));

    if(strstr(sensor_o->valuestring, "Microphone")){
        strncpy(content_type, "application/octet-stream", content_type_size);
        samples_size = ei_microphone_get_data_size();
    }
    else {
        strncpy(content_type, "application/cbor", content_type_size);
        samples_size = ei_sampler_get_data_size();
    }

    /*send response: sample send*/
    json_string = ei_ble_com_sample_upload(cJSON_GetStringValue(path_o), dev->get_upload_api_key().c_str(), cJSON_GetStringValue(label_o), "1", content_type);

    // find special token to inject data
    pos_token = strstr(json_string, EI_DATA_TOKEN);
    if(pos_token == NULL) {
        LOG_ERR("Missing token in sample JSON!");
        // cJSON_FreeString(json_string);
        k_free(json_string);
        return;
    }
    // caluclate length of the first part unitl token, reuse length variable
    length =  (uint32_t)(pos_token - json_string);
    // send head of the json
    ble_nus_send_data((uint8_t*)json_string, length);

    // send data (encoded in base64)
    read_sample_buffer(0, samples_size, ei_ble_encode_send);

    // send end of the json
    length = strlen(pos_token) - strlen(EI_DATA_TOKEN);
    ble_nus_send_data((uint8_t*)(pos_token + strlen(EI_DATA_TOKEN)), length);

    // free memory alocated for json string
    // cJSON_FreeString(json_string);
    k_free(json_string);

    json_string = ei_ble_com_response_sample("sampleFinished", true, NULL);
    ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
    // cJSON_FreeString(json_string);
    k_free(json_string);

    LOG_INF("%s: OK\n", __FUNCTION__);

}

void ei_ble_message_rcv(const char * const responce)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());
    cJSON *type = NULL;
    cJSON *direction = NULL;
    cJSON *message = NULL;
    cJSON *hello = NULL;
    cJSON *api_key = NULL;
    cJSON *url = NULL;
    char* json_string = NULL;

    LOG_INF("\n");
    LOG_INF("*******************************************************");

    dev->set_serial_channel(BLE);

    cJSON *response_json = cJSON_Parse(responce);
    if (response_json == NULL) {
        LOG_WRN("NO Valid JSON");
        return;
    }

    direction = cJSON_GetObjectItemCaseSensitive(response_json, "direction");
    if (cJSON_IsString(direction) && (direction->valuestring != NULL))
    {
        if (strcmp(direction->valuestring, "rx")){
            goto cleanup;
        }
    }
    LOG_INF("RX message received");

    type = cJSON_GetObjectItemCaseSensitive(response_json, "type");
    if (cJSON_IsString(type) && (type->valuestring != NULL))
    {
        if (!strcmp(type->valuestring, "ws")){
            LOG_INF("WS type message received");
            message =  cJSON_GetObjectItem(response_json, "message");
            if (NULL == message) {
                LOG_WRN("Message obj problem");
            }
            else if (cJSON_HasObjectItem(message, "sample")) {
                LOG_INF("Message Sample");
                ei_ble_sample(cJSON_GetObjectItem(message, "sample"));
            }
            else if (cJSON_HasObjectItem(message, "hello")) {
                LOG_INF("Hello message received");
                hello = cJSON_GetObjectItem(message, "hello");
                if(cJSON_IsTrue(hello)){
                    LOG_INF("Connection established");
                }
            }
            else {
                LOG_WRN("Unsupported ws message type");
            }
        }
        else if (!strcmp(type->valuestring, "configure")) {
            LOG_INF("configure type message received");
            message =  cJSON_GetObjectItem(response_json, "message");
            if (NULL == message) {
                LOG_ERR("Message obj problem");
                goto cleanup;
            }
            else if (cJSON_HasObjectItem(message, "apiKey")) {
                api_key = cJSON_GetObjectItemCaseSensitive(message, "apiKey");
                url = cJSON_GetObjectItemCaseSensitive(message, "address");
                if(!cJSON_IsString(api_key) || api_key->valuestring == NULL) {
                    LOG_ERR("apiKey incorrect!");
                    goto cleanup;
                }
                if(!cJSON_IsString(url) || url->valuestring == NULL) {
                    LOG_ERR("URL incorrect!");
                    goto cleanup;
                }
                ei_ble_configure(api_key->valuestring, url->valuestring);
            }
            else {
                LOG_ERR("Missing apiKey in configure");
            }
        }
        else if (!strcmp(type->valuestring, "start-inferencing")) {
            LOG_DBG("Received start inferencing");
            ei_start_impulse(false, false);
            //TODO: check if inferencing started
            json_string = ei_inference_start_ack(true);
            ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
            // cJSON_FreeString(json_string);
            k_free(json_string);
        }
        else if (!strcmp(type->valuestring, "stop-inferencing")) {
            LOG_DBG("Received stop inferencing");
            ei_stop_impulse();
            //TODO: check if inferencing stopped
            json_string = ei_inference_stop_ack(true);
            ble_nus_send_data((uint8_t*)json_string, strlen((const char *)json_string));
            // cJSON_FreeString(json_string);
            k_free(json_string);
        }
        else {
            LOG_WRN("Unsupported msg type: %s", type->valuestring);
        }
    }
    else {
        LOG_WRN("No type ");
    }

    LOG_INF("*******************************************************\n\n");

cleanup:
    if(response_json != NULL) {
        LOG_INF("Deleting: response_json");
        cJSON_Delete(response_json);
    }
}

void ei_ble_com_init(void)
{
    /* It should be easier to all cJSON_Init from cJSON_os
     * but the linker is not see that function, even
     * it is present in libcjson.a
     */
	_cjson_hooks.malloc_fn = malloc_fn_hook;
	_cjson_hooks.free_fn = free_fn_hook;

	cJSON_InitHooks(&_cjson_hooks);
    LOG_INF("cJSON init done");

    ble_nus_init();
    LOG_INF("BLE NUS init done");
}

static void ei_ble_encode_send(uint8_t *buffer, size_t size)
{
    // we are getting a buffer from read_sample_buffer (ei_device_nordic_nrf53.cpp)
    // which is 513B or less, it will be encoded into 684B base64 string
    // passed byte-by-byte through ble_nus_append_buffer to send_buffer
    base64_encode((const char*)buffer, size, ble_nus_append_buffer);

    // send_buffer is global
    ble_nus_send_data(send_buffer, send_buffer_wr_idx);

    // incremented in ble_nus_append_buffer
    send_buffer_wr_idx = 0;
}

/* callback for base64_encode */
void ble_nus_append_buffer(char c)
{
    send_buffer[send_buffer_wr_idx++] = (uint8_t)c;
}

static char* ei_inference_start_ack(bool success)
{
    cJSON *ack = cJSON_CreateObject();
    char *string = NULL;

    if(!cJSON_AddStringToObject(ack, "type", "start-inferencing-response")) {
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(!cJSON_AddBoolToObject(ack, "ok", success)) {
        LOG_ERR("error: cJSON_AddBoolToObject");
    }

    if(!success) {
        //TODO: add more descriptive errors
        if(!cJSON_AddStringToObject(ack, "error", "failed to start inference")) {
            LOG_ERR("error: cJSON_AddBoolToObject");
        }        
    }

    string = cJSON_PrintUnformatted(ack);
    if (string == NULL)
    {
        LOG_ERR("Failed to print monitor.\n");
    }

    cJSON_Delete(ack);

    return string;
}

static char* ei_inference_stop_ack(bool success)
{
    cJSON *ack = cJSON_CreateObject();
    char *string = NULL;

    if(!cJSON_AddStringToObject(ack, "type", "stop-inferencing-response")) {
        LOG_ERR("error: cJSON_AddStringToObject");
    }

    if(!cJSON_AddBoolToObject(ack, "ok", success)) {
        LOG_ERR("error: cJSON_AddBoolToObject");
    }

    if(!success) {
        //TODO: add more descriptive errors
        if(!cJSON_AddStringToObject(ack, "error", "failed to stop inference")) {
            LOG_ERR("error: cJSON_AddBoolToObject");
        }        
    }

    string = cJSON_PrintUnformatted(ack);
    if (string == NULL)
    {
        LOG_ERR("Failed to print monitor.\n");
    }

    cJSON_Delete(ack);

    return string;
}
