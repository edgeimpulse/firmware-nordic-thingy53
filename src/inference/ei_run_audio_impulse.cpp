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

#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "sensors/ei_microphone.h"
#include "ei_device_thingy53.h"
#include "ble/ble_nus.h"
#include "cJSON.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(run_impulse);

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static int print_results;
static uint16_t samples_per_inference;
static inference_state_t state = INFERENCE_STOPPED;
static bool continuous_mode = false;
static bool debug_mode = false;
static float samples_circ_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int samples_wr_index = 0;
static EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());

static inline inference_state_t set_thread_state(inference_state_t new_state)
{
    if(state != INFERENCE_STOPPED) {
        state = new_state;
    }

    return state;
}

static void process_results(ei_impulse_result_t* result)
{
    char *string = NULL;

    if(dev->get_serial_channel() == UART) {
        display_results(&ei_default_impulse, result);
    }
    else {
        cJSON *response = cJSON_CreateObject();
        cJSON *results = NULL;

        cJSON_AddStringToObject(response, "type", "inference-results");

        results = cJSON_AddArrayToObject(response, "classification");

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            cJSON *res = cJSON_CreateObject();
            cJSON_AddStringToObject(res, "label", result->classification[ix].label);
            cJSON_AddNumberToObject(res, "value", result->classification[ix].value);
            cJSON_AddItemToArray(results, res);
        }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
        cJSON_AddNumberToObject(response, "anomaly", result->anomaly);
#endif

        string = cJSON_PrintUnformatted(response);
        if (string == NULL) {
            LOG_ERR("Failed to print monitor.\n");
        }

        cJSON_Delete(response);

        ble_nus_send_data((uint8_t*)string, strlen((const char *)string));

        // cJSON_FreeString(string);
        k_free(string);
    }
}

void ei_inference_thread(void* param1, void* param2, void* param3)
{
    while(1) {
        switch(state) {
            case INFERENCE_STOPPED:
                // nothing to do
                ei_sleep(5);
                continue;
            case INFERENCE_WAITING:
                ei_sleep(2000);
                if(set_thread_state(INFERENCE_SAMPLING) == INFERENCE_STOPPED) {
                    // if someone stopped inference during delay, go to thread loop iteration
                    continue;
                }
                if(dev->get_serial_channel() == UART) {
                    ei_printf("Recording\n");
                }
                ei_microphone_inference_reset_buffers();
                dev->set_state(eiStateSampling);
                continue;
            case INFERENCE_SAMPLING:
                // wait for data to be collected through callback
                if (ei_microphone_inference_is_recording()) {
                    ei_sleep(1);
                    continue;
                }
                state = INFERENCE_DATA_READY;
                continue;
            // nothing to do, just continue to inference provcessing below
            case INFERENCE_DATA_READY:
            default:
                break;
        }

        signal_t signal;

        signal.total_length = continuous_mode ? EI_CLASSIFIER_SLICE_SIZE : EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &ei_microphone_inference_get_data;

        // run the impulse: DSP, neural network and the Anomaly algorithm
        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR ei_error;
        if(continuous_mode == true) {
            ei_error = run_classifier_continuous(&signal, &result, debug_mode);
        }
        else {
            ei_error = run_classifier(&signal, &result, debug_mode);
        }
        if (ei_error != EI_IMPULSE_OK) {
            ei_printf("Failed to run impulse (%d)", ei_error);
            set_thread_state(INFERENCE_STOPPED);
            continue;
        }

        if(continuous_mode == true) {
            if(++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1)) {
                process_results(&result);
                print_results = 0;
            }
        }
        else {
            process_results(&result);
        }

        if(continuous_mode == true) {
            set_thread_state(INFERENCE_SAMPLING);
        }
        else {
            ei_printf("Starting inferencing in 2 seconds...\n");
            set_thread_state(INFERENCE_WAITING);
        }
    }
}

void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    continuous_mode = continuous;
    debug_mode = debug;

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.04fms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %.02f ms.\n", (float)(EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS));
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));
    ei_printf("Starting inferencing, press 'b' to break\n");

    dev->set_sample_length_ms(EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS);
    dev->set_sample_interval_ms(EI_CLASSIFIER_INTERVAL_MS);

    if (continuous == true) {
        samples_per_inference = EI_CLASSIFIER_SLICE_SIZE * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // In order to have meaningful classification results, continuous inference has to run over
        // the complete model window. So the first iterations will print out garbage.
        // We now use a fixed length moving average filter of half the slices per model window and
        // only print when we run the complete maf buffer to prevent printing the same classification multiple times.
        print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        run_classifier_init();
        state = INFERENCE_SAMPLING;
    }
    else {
        samples_per_inference = EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // it's time to prepare for sampling
        ei_printf("Starting inferencing in 2 seconds...\n");
        state = INFERENCE_WAITING;
    }

    if (ei_microphone_inference_start(continuous_mode ? EI_CLASSIFIER_SLICE_SIZE : EI_CLASSIFIER_RAW_SAMPLE_COUNT, EI_CLASSIFIER_INTERVAL_MS) == false) {
        ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        state = INFERENCE_STOPPED;
        return;
    }
}

void ei_stop_impulse(void)
{
    if(state != INFERENCE_STOPPED) {
        set_thread_state(INFERENCE_STOPPED);
        ei_printf("Inferencing stopped by user\r\n");
        dev->set_state(eiStateFinished);
        /* reset samples buffer */
        samples_wr_index = 0;
        run_classifier_deinit();
    }
}

bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

K_THREAD_DEFINE(inference_thread_id, CONFIG_EI_INFERENCE_THREAD_STACK,
                ei_inference_thread, NULL, NULL, NULL,
                CONFIG_EI_INFERENCE_THREAD_PRIO, 0, 0);

#endif /* defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_MICROPHONE */
