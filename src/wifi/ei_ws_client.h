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