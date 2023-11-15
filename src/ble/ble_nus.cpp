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

#include "ble_nus.h"
#include "nus.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include "ei_ble_com.h"
#include "inference/ei_run_impulse.h"
#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME ble_nus
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

//maximum number of received characters in json
#define RECEIVE_BUF_MAX 5000

static uint8_t prv_receive_buf[RECEIVE_BUF_MAX] = {0x00};

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

/* Advertising Manufacturer data */
/* Manufacturer data company ID -> Nordic Semi */
#define DEV_COMPANY_ID 0x0059 

//TODO: currently needed because of the central ble device, will be removed later
#define NUS_MAX_SEND_DECREES 0

static struct k_work send_queued;
static uint32_t buffer_end = 0;
static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

struct bt_le_adv_param nus_ble_param = {
    .id = BT_ID_DEFAULT,
    .sid = 0,
    .secondary_max_skip = 0,
    .options = BT_LE_ADV_OPT_CONNECTABLE,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};

struct packet {
    uint16_t company_id;
    uint8_t  ei_unique_id;
    uint8_t  data[6];
};

struct manufacturer_data {
    union {
        uint8_t buffer[sizeof(struct packet)];
        struct packet packet;
    };
};

struct manufacturer_data manu_data = {0};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manu_data.buffer, sizeof(manu_data.buffer)-1),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

void ble_write_fifo (char * buf, uint32_t len);

static void send_queued_fn(struct k_work *w)
{
    // send_data
    char * send_temp_array = ei_ble_connect_handshake();
    ble_write_fifo(send_temp_array, strlen((const char *)send_temp_array));
    k_free(send_temp_array);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);

    current_conn = bt_conn_ref(conn);

    buffer_end = 0;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("Disconnected: %s (reason %u)", addr, reason);

    //stop inferencing if running on disconnect
    if (is_inference_running()) {
        ei_stop_impulse();
    }

    if (auth_conn) {
        bt_conn_unref(auth_conn);
        auth_conn = NULL;
    }

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        buffer_end = 0;
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = connected,
    .disconnected = disconnected,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
              uint16_t len)
{
    char addr[BT_ADDR_LE_STR_LEN] = {0};

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

    if(buffer_end + len > RECEIVE_BUF_MAX) {
        LOG_ERR("pv_receive_buffer overflow!");
        return;
    }

    memcpy(&prv_receive_buf[buffer_end], data, len);
    buffer_end += len;

    if (prv_receive_buf[buffer_end-1] == '\r' || prv_receive_buf[buffer_end-1] == '\n') {
        LOG_INF("Complete message received!");
        ei_ble_message_rcv((const char *)prv_receive_buf);
        //TODO: do we realy need to clear the memory?    
        memset(prv_receive_buf, 0x00, RECEIVE_BUF_MAX);
        buffer_end = 0;
    }
    else {
        LOG_INF("Message not complete");
    }
}

static void bt_nus_cb_send_en(enum bt_nus_send_status evt)
{
    if (BT_NUS_SEND_STATUS_ENABLED == evt) {
        LOG_INF("BT_NUS_SEND_STATUS_ENABLED");
        k_work_submit(&send_queued);
        LOG_INF("connection response sent");
    }
    else if (BT_NUS_SEND_STATUS_DISABLED == evt) {
        LOG_INF("BT_NUS_SEND_STATUS_DISABLED");
    }
    else {
        LOG_WRN("no nus event");
    }
}

static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb,
    .send_enabled = bt_nus_cb_send_en,
};

void error(void)
{
    LOG_ERR("BT error init");
    while (true) {
        /* Spin for ever */
    }
}

/* This function swaps and modifies the content of inp
 *
 * Returns: a pointer to the output value (inp) */
void * swapbytes(void *inp, size_t len)
{
    unsigned int i;
    unsigned char *in=(unsigned char *)inp,tmp;

    for(i=0;i<len/2;i++) {
        tmp=*(in+i);
        *(in+i)=*(in+len-i-1);
        *(in+len-i-1)=tmp;
    }

    return inp;
}

void ble_nus_init(void)
{
    struct bt_le_oob oob;
    int err = 0;

    k_work_init(&send_queued, send_queued_fn);

    bt_conn_cb_register(&conn_callbacks);

    err = bt_enable(NULL);
    if (err) {
        error();
    }

    LOG_INF("Bluetooth initialized");

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    /* Use identity address as device ID in the Manufacturer information */
	if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
		LOG_ERR("Identity Address unavailable\n");
	} else {
        manu_data.packet.company_id = DEV_COMPANY_ID;
        manu_data.packet.ei_unique_id = 0xE1;
		memcpy(manu_data.packet.data, oob.addr.a.val, 6);
        swapbytes(manu_data.packet.data, 6);
        LOG_HEXDUMP_INF(manu_data.packet.data, 6, "address");
	}

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("Failed to initialize UART service (err: %d)", err);
        return;
    }

    err = bt_le_adv_start(&nus_ble_param, ad, ARRAY_SIZE(ad), sd,
                  ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }
}

void ble_nus_send_data(uint8_t* buffer, uint32_t size)
{
    ble_write_fifo((char*)buffer, (uint32_t)size);
}

void ble_write_fifo (char * send_buf, uint32_t len)
{
    uint32_t buf_count = 0;
    uint32_t buf_size = 0;

    uint32_t l_nus_max_send_len = bt_nus_get_mtu(current_conn);
    uint32_t ble_nus_mtu_size = l_nus_max_send_len - NUS_MAX_SEND_DECREES;

    if (0 == len || NULL == send_buf) {
        LOG_WRN("%s: wrong parameters", __FUNCTION__);
        return;
    }

    if (0 != len % ble_nus_mtu_size) {
        buf_count = (len / ble_nus_mtu_size) + 1;
    }
    else {
        buf_count = len / ble_nus_mtu_size;
    }

    for (uint32_t i = 0; i < buf_count; i++){
        if ((buf_count-1) == i && (0 != len % ble_nus_mtu_size)){
            buf_size = len % ble_nus_mtu_size;
        }
        else {
            buf_size = ble_nus_mtu_size;
        }
        //LOG_INF("mtu: %d, send buf: %d, size: %d", ble_nus_mtu_size, i, buf_size);
        if (bt_nus_send(current_conn, (uint8_t *)&send_buf[i * ble_nus_mtu_size], buf_size)) {
            LOG_WRN("Failed to send data over BLE connection");
        }
    }
}
