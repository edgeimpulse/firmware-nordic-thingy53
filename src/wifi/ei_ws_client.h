/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
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

#ifndef EI_WS_CLIENT_H
#define EI_WS_CLIENT_H

#include "firmware-sdk/ei_device_info_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HelloMsg,
    SampleStartMsg,
    SampleFailedMsg,
    SampleStartedMsg,
    SampleProcessingMsg,
    SampleUploadingMsg,
    SampleFinishedMsg,
    SnapshotFrameMsg,
} TxMsgType;

/**
 * @brief      Send a message to remote management servie
 * @param[in]  msg_type The message type
 * @param[in]  data     Optional additional data
 *                      For SampleFailedMsg, this is the error message
 *                      For SnapshotFrameMsg, this is the snapshot frame (base 64 encoded)
 * @return     True if message was sent successfully, false otherwise
*/
bool ei_ws_send_msg(TxMsgType msg_type, const char* data = nullptr);

/**
 * @brief      Start the websocket client thread
 * @param[in]  dev  Pointer to the device info object
 * @param[in]  handler  Callback function to call when a sample start message is received
*/
void ei_ws_client_start(EiDeviceInfo *dev, bool (*handler)(const char **, const int));

/**
 * @brief      Stop the websocket client thread
*/
void ei_ws_client_stop(void);

/**
 * @brief      Get connection status
 * @return     True if connected, false otherwise
*/
bool ei_ws_get_connection_status(void);

/**
 * @brief      Send a sample to remote management service from internal memory
 * @param[in]  address  Address of the sample in internal memory
 * @param[in]  length   Length of the sample
 * @return     True if sample was sent successfully, false otherwise
*/
bool ei_ws_send_sample(size_t address, size_t length, bool cbor = true);

#ifdef __cplusplus
};
#endif

#endif /* EI_WS_CLIENT_H */