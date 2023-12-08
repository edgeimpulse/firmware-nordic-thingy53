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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ei_ws_client, CONFIG_REMOTE_INGESTION_LOG_LEVEL);

#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "ei_ws_client.h"
#include "firmware-sdk/remote-mgmt.h"
#include "ei_device_thingy53.h"
#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/websocket.h>
#include <climits>
#include <string>
#include <errno.h>

#define REMOTE_MGMT_PORT "80"
#define INGESTION_PORT "80"

using namespace std;

typedef struct {
    uint32_t samples_addr;
} http_priv_data_t;

static int remote_mgmt_socket = -1;
static int remote_mgmt_http_socket = -1;
static int ingestion_socket = -1;
static bool is_connected = false;
static struct k_thread ws_read_thread_data;
static EiDeviceInfo *device;
static struct addrinfo *ingestion_addrinfo;
bool (*sample_start_handler)(const char **, const int);
void ws_ping_work_handler(struct k_work *work);
void ws_ping_timer_handler(struct k_timer *dummy);

K_THREAD_STACK_DEFINE(ws_read_stack, 8192);
K_WORK_DEFINE(ws_ping_work, ws_ping_work_handler);
K_TIMER_DEFINE(ws_ping_timer, ws_ping_timer_handler, NULL);

bool ws_sample_start(const char **argv, int n)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    const ei_device_sensor_t *sensor_list;
    size_t sensor_list_size;

    dev->get_sensor_list((const ei_device_sensor_t **)&sensor_list, &sensor_list_size);

    for (size_t ix = 0; ix < sensor_list_size; ix++) {
        if (strcmp(sensor_list[ix].name, argv[0]) == 0) {
            if (!sensor_list[ix].start_sampling_cb()) {
                LOG_ERR("Failed to start sampling\n");
                dev->set_state(eiStateIdle);
            }
            else {
                dev->set_state(eiStateFinished);
            }
            return true;
        }
    }

    if (ei_connect_fusion_list(argv[0], SENSOR_FORMAT)) {
        if (!ei_fusion_setup_data_sampling()) {
            LOG_ERR("Failed to start sensor fusion sampling\n");
            dev->set_state(eiStateIdle);
        }
        else {
            dev->set_state(eiStateFinished);
        }
    }
    else {
        LOG_ERR("Failed to find sensor '%s' in the sensor list\n", argv[0]);
    }

    return true;
}


void ws_ping_work_handler(struct k_work *work)
{
    int ret;

    if(remote_mgmt_socket < 0) {
        LOG_WRN("Remote Management service not connected!");
        return;
    }

    LOG_DBG("Ping!");
    ret = websocket_send_msg(remote_mgmt_socket, NULL, 0, WEBSOCKET_OPCODE_PING, true, true, 100);
    if (ret < 0) {
        LOG_ERR("Failed to send ping! (%d)", ret);
    }
}

void ws_ping_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&ws_ping_work);
}

static string get_domain_from_url(const string &url)
{
    size_t pos = url.find("://");

    if (pos != string::npos) {
        pos += 3;
    }
    else {
        pos = 0;
    }
    size_t end_pos = url.find("/", pos);

    return (end_pos != string::npos) ? url.substr(pos, end_pos - pos) : url.substr(pos);
}

static bool server_connect(void)
{
	int32_t timeout = 3 * MSEC_PER_SEC;
	struct addrinfo hints;
	struct addrinfo *remote_mgmt_addrinfo;
	int ret;
	struct websocket_request req;
	uint8_t temp_recv_buf_ipv4[512];
    char peer_addr[INET6_ADDRSTRLEN];
    string domain_name;

    if(ingestion_addrinfo != NULL) {
        free(ingestion_addrinfo);
    }

    domain_name = get_domain_from_url(device->get_management_url());
	LOG_DBG("Resolving address: %s", domain_name.c_str());
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	while((ret = zsock_getaddrinfo(domain_name.c_str(), REMOTE_MGMT_PORT, &hints, &remote_mgmt_addrinfo)) != 0) {
        LOG_ERR("Unable to resolve address (%d), retrying...", ret);
        k_sleep(K_SECONDS(1));
    }
    inet_ntop(remote_mgmt_addrinfo->ai_family, &((struct sockaddr_in *)(remote_mgmt_addrinfo->ai_addr))->sin_addr, peer_addr, INET6_ADDRSTRLEN);
    LOG_DBG("Resolved %s (%s)", peer_addr, net_family2str(remote_mgmt_addrinfo->ai_family));
	LOG_DBG("Resolving address OK");

    domain_name = get_domain_from_url(device->get_upload_host());
	LOG_DBG("Resolving address: %s", domain_name.c_str());
	while((ret = zsock_getaddrinfo(domain_name.c_str(), INGESTION_PORT, &hints, &ingestion_addrinfo)) != 0) {
		LOG_ERR("Unable to resolve address (%d), retrying...", ret);
        k_sleep(K_SECONDS(1));
    }
    inet_ntop(ingestion_addrinfo->ai_family, &((struct sockaddr_in *)(ingestion_addrinfo->ai_addr))->sin_addr, peer_addr, INET6_ADDRSTRLEN);
    LOG_DBG("Resolved %s (%s)", peer_addr, net_family2str(ingestion_addrinfo->ai_family));
	LOG_DBG("Resolving address OK");

	LOG_DBG("Connecting to remote management server...");
    while((remote_mgmt_http_socket = zsock_socket(remote_mgmt_addrinfo->ai_family, remote_mgmt_addrinfo->ai_socktype, remote_mgmt_addrinfo->ai_protocol)) == -1) {
        LOG_ERR("Failed to create socket (%d), retrying...", errno);
        k_sleep(K_SECONDS(1));
    }

	while((ret = zsock_connect(remote_mgmt_http_socket, remote_mgmt_addrinfo->ai_addr, remote_mgmt_addrinfo->ai_addrlen)) < 0) {
		LOG_ERR("Cannot connect to remote management service (%d), retrying...", errno);
        k_sleep(K_SECONDS(1));
    }
	LOG_DBG("Connecting to remote management server... OK");

	LOG_DBG("Establishing websocket connection to remote management server...");
	memset(&req, 0, sizeof(req));
    domain_name = get_domain_from_url(device->get_management_url());
	req.host = domain_name.c_str();
	req.url = "/";
	req.cb = nullptr;
	req.tmp_buf = temp_recv_buf_ipv4;
	req.tmp_buf_len = sizeof(temp_recv_buf_ipv4);

	while((remote_mgmt_socket = websocket_connect(remote_mgmt_http_socket, &req, timeout, nullptr)) < 0) {
		LOG_ERR("Cannot connect to %s:%s (%d), retrying...", device->get_management_url().c_str(), REMOTE_MGMT_PORT, remote_mgmt_socket);
        k_sleep(K_SECONDS(1));
        //TODO: in case of timeouting this operation, close the http socket
		// close(remote_mgmt_http_socket);
    }
	LOG_DBG("Establishing websocket connection to remote management server... OK");

    if(remote_mgmt_addrinfo != NULL) {
        free(remote_mgmt_addrinfo);
    }

	return true;
}

static void parse_received_msg(const uint8_t *buf, size_t buf_len)
{
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());

    auto decoded_message = decode_message(buf, buf_len, device);
    dev->set_serial_channel(WIFI);

    if (decoded_message->getType() == MessageType::DecoderErrorType) {
        auto msg = static_cast<DecoderError*>(decoded_message.get());
        LOG_DBG("Error decoding message (%d): %s", msg->err_code, msg->err_message.c_str());
    } else if (decoded_message->getType() == MessageType::HelloResponseType) {
        auto msg = static_cast<HelloResponse*>(decoded_message.get());
        LOG_DBG("Hello response: %s", msg->status ? "OK" : "ERROR");
        is_connected = msg->status;
    } else if (decoded_message->getType() == MessageType::ErrorResponseType) {
        auto msg = static_cast<ErrorResponse*>(decoded_message.get());
        LOG_DBG("Error response: %s", msg->err_message.c_str());
    } else if (decoded_message->getType() == MessageType::SampleRequestType) {
        auto msg = static_cast<SampleRequest*>(decoded_message.get());
        LOG_DBG("Sample request: %s", msg->sensor.c_str());
        const char* sensor = msg->sensor.c_str();
        if(sample_start_handler) {
            sample_start_handler(&sensor, 1);
        }
    } else if (decoded_message->getType() == MessageType::StreamingStartRequestType) {
        // auto msg = static_cast<StreamingStartRequest*>(decoded_message.get());
        LOG_DBG("Streaming Start request");
        // not supported yet
    } else if (decoded_message->getType() == MessageType::StreamingStopRequestType) {
        // auto msg = static_cast<StreamingStopRequest*>(decoded_message.get());
        LOG_DBG("Streaming Stop request");
        // not supported yet
    }
    else {
        LOG_WRN("Unknown message type!");
    }
}

void ws_read_handler(void *arg1, void *arg2, void *arg3)
{
    uint64_t remaining;
    uint32_t message_type;
    uint8_t buf[512];
    uint32_t total_read;
    int read_pos;
    int ret;

	if(!server_connect()) {
		LOG_ERR("Server connection failed!");
		k_sleep(K_FOREVER);
	}

    // send hello
    ei_ws_send_msg(TxMsgType::HelloMsg);

    while(true) {
        remaining = ULLONG_MAX;
        read_pos = 0;
        total_read = 0;

        while (remaining > 0) {
            ret = websocket_recv_msg(remote_mgmt_socket, buf + read_pos,
                        sizeof(buf) - read_pos,
                        &message_type,
                        &remaining,
                        SYS_FOREVER_MS);
            if (ret < 0) {
                if (ret == -EAGAIN) {
                    k_sleep(K_MSEC(50));
                    continue;
                }

                LOG_DBG("connection closed while waiting (%d/%d)", ret, errno);
                //TODO: reconnect in case of websocket closed
                is_connected = false;
                k_sleep(K_SECONDS(1));
                continue;
            }

            read_pos += ret;
            total_read += ret;
        }

        if (remaining != 0) {
            LOG_ERR("data recv failure (remaining %" PRId64 ")", remaining);
            // TODO: reconnect in case of websocket closed
            continue;
        }

        if(message_type & WEBSOCKET_FLAG_PING) {
            LOG_DBG("PING received, sending PONG");
            websocket_send_msg(remote_mgmt_socket, buf, total_read, WEBSOCKET_OPCODE_PONG, true, true, SYS_FOREVER_MS);
            continue;
        }
        else if((message_type & WEBSOCKET_FLAG_BINARY) && (message_type & WEBSOCKET_FLAG_FINAL)) {
            parse_received_msg(buf, total_read);
            continue;
        }
        else {
            LOG_DBG("recv %d bytes (type: 0x%02x)", total_read, message_type);
            LOG_HEXDUMP_DBG(buf, total_read, "received ws buf");
        }
    }
}

bool ei_ws_send_msg(TxMsgType msg_type, const char* data)
{
    const size_t tx_msg_len = 1024;
    uint8_t tx_msg_buf[tx_msg_len];
    int ret;
    uint32_t act_msg_len = 0;
    string msg_name;

    switch (msg_type) {
        case TxMsgType::HelloMsg:
            act_msg_len = get_hello_msg(tx_msg_buf, tx_msg_len, device);
            msg_name = "Hello";
        break;
        case TxMsgType::SampleStartMsg:
            act_msg_len = get_sample_start_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Start";
        break;
        case TxMsgType::SampleFailedMsg:
            act_msg_len = get_sample_failed_msg(tx_msg_buf, tx_msg_len, data);
            msg_name = "Sample Failed";
        break;
        case TxMsgType::SampleStartedMsg:
            act_msg_len = get_sample_started_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Started";
        break;
        case TxMsgType::SampleProcessingMsg:
            act_msg_len = get_sample_processing_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Processing";
        break;
        case TxMsgType::SampleUploadingMsg:
            act_msg_len = get_sample_uploading_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Uploading";
        break;
        case TxMsgType::SampleFinishedMsg:
            act_msg_len = get_sample_finished_msg(tx_msg_buf, tx_msg_len);
            msg_name = "Sample Finished";
        break;
        case TxMsgType::SnapshotFrameMsg:
            act_msg_len = get_snapshot_frame_msg(tx_msg_buf, tx_msg_len, data);
            msg_name = "Snapshot Frame";
        break;
    }

    ret = websocket_send_msg(remote_mgmt_socket, (const uint8_t*)tx_msg_buf, act_msg_len, WEBSOCKET_OPCODE_DATA_BINARY,
                          true, true, SYS_FOREVER_MS);
    if(ret < 0) {
        LOG_ERR("Failed to send %s message! (%d)", msg_name.c_str(), ret);
        return false;
    }

    LOG_DBG("Message %s len = %d", msg_name.c_str(), act_msg_len);
    LOG_HEXDUMP_DBG(tx_msg_buf, act_msg_len, msg_name.c_str());

    return true;
}

void ei_ws_client_start(EiDeviceInfo *dev, bool (*handler)(const char **, const int))
{
    device = dev;

    sample_start_handler = &ws_sample_start;

    k_timer_start(&ws_ping_timer, K_SECONDS(15), K_SECONDS(15));

    k_thread_create(&ws_read_thread_data, ws_read_stack,
                    K_THREAD_STACK_SIZEOF(ws_read_stack),
                    ws_read_handler,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);
}

void ei_ws_client_stop(void)
{
    k_timer_stop(&ws_ping_timer);
    k_thread_abort(&ws_read_thread_data);
    zsock_close(remote_mgmt_socket);
    zsock_close(remote_mgmt_http_socket);
}

bool ei_ws_get_connection_status(void)
{
    return is_connected;
}

static void response_cb(struct http_response *rsp, enum http_final_call final_data, void *user_data)
{
    int ret;
	if (final_data == HTTP_DATA_MORE) {
		LOG_DBG("Partial data received (%zd bytes)", rsp->data_len);
	} else if (final_data == HTTP_DATA_FINAL) {
		LOG_DBG("All the data received (%zd bytes)", rsp->data_len);
        ret = zsock_close(ingestion_socket);
        if(ret < 0) {
            LOG_ERR("Failed to close ingestion socket! (%d)", errno);
        }
	}

    LOG_HEXDUMP_DBG(rsp->recv_buf, rsp->recv_buf_len, "rx http buf");
	LOG_DBG("Response status %s", rsp->http_status);
}

int ingestion_payload_cb(int sock, struct http_request *req, void *user_data)
{
    EiDeviceMemory *memory = device->get_memory();
    uint8_t buf[256];
    http_priv_data_t *priv_data = (http_priv_data_t *)user_data;
    size_t bytes_sent = 0;
    LOG_DBG("Bytes to send: %d", req->payload_len);

    while(bytes_sent < req->payload_len) {
        size_t len = MIN(sizeof(buf), req->payload_len - bytes_sent);
        memory->read_sample_data(buf, priv_data->samples_addr + bytes_sent, len);

        LOG_DBG("Sending %d bytes from %d (%d)", len, priv_data->samples_addr + bytes_sent, bytes_sent);
        (void)send(sock, buf, len, 0);
        bytes_sent += len;
    }

    return bytes_sent;
}

bool ei_ws_send_sample(size_t address, size_t length, bool cbor)
{
    int ret;
    char api_key_header[128];
    char label_header[128];
    char content_type[128];
    char label_x[128] = "";
    static uint8_t temp_recv_buf_ipv4[512];
    int32_t timeout = 3 * MSEC_PER_SEC;
    string host;
    struct http_request req;
    http_priv_data_t priv_data;

	LOG_DBG("Connecting to ingestion service...");
	ingestion_socket = zsock_socket(ingestion_addrinfo->ai_family, ingestion_addrinfo->ai_socktype, ingestion_addrinfo->ai_protocol);
    if(ingestion_socket < 0) {
        LOG_ERR("Failed to create socket! (%d)", errno);
        return false;
    }
	ret = zsock_connect(ingestion_socket, ingestion_addrinfo->ai_addr, ingestion_addrinfo->ai_addrlen);
	if (ret < 0) {
		LOG_ERR("Cannot create HTTP connection. ret = %d", ret);
		return false;
	}
	LOG_DBG("Connecting to ingestion service... OK");

    snprintf(api_key_header, sizeof(api_key_header), "x-api-key: %s\r\n", device->get_upload_api_key().c_str());
    snprintf(label_header, sizeof(label_header), "x-file-name: %s\r\n", device->get_sample_label().c_str());

    if (cbor) {
        strcpy(content_type, "Content-type: application/cbor\r\n");
    }
    else {
        strcpy(content_type, "Content-type: application/octet-stream\r\n");
        sprintf(label_x, "x-label: %s\r\n", device->get_sample_label().c_str());
    }

    const char *extra_headers[] = {
        api_key_header,
        label_header,
        // "x-disallow-duplicates\r\n",
        content_type,
        label_x,
        NULL
    };

    LOG_DBG("Samples len = %d", length);
    //LOG_HEXDUMP_DBG(buffer, length, "Sample");

    memset(&req, 0, sizeof(req));

    host = get_domain_from_url(device->get_upload_host());

    req.method = HTTP_POST;
    req.url = device->get_upload_path().c_str();
    req.host = host.c_str();
    req.protocol = "HTTP/1.1";
    req.optional_headers = extra_headers;
    req.payload_cb = ingestion_payload_cb;
    req.payload_len = length;
    req.response = response_cb;
    req.recv_buf = temp_recv_buf_ipv4;
    req.recv_buf_len = sizeof(temp_recv_buf_ipv4);

    priv_data.samples_addr = address;

    ret = http_client_req(ingestion_socket, &req, timeout, &priv_data);
    if(ret <0) {
        LOG_ERR("Failed to send sample! (%d)", ret);
        // k_free(buffer);
        return false;
    }

    return true;
}