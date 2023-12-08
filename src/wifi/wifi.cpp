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
LOG_MODULE_REGISTER(ei_wifi, LOG_LEVEL_DBG);

#include "wifi.h"
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/init.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>

#include <nrfx_clock.h>
#include <zephyr/device.h>
#include <zephyr/net/net_config.h>

#include "net_private.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_device_thingy53.h"

#define WIFI_CHANNEL_ANY 255


#define WIFI_SHELL_MGMT_EVENTS (NET_EVENT_WIFI_SCAN_RESULT |		\
                NET_EVENT_WIFI_SCAN_DONE |		\
                NET_EVENT_WIFI_CONNECT_RESULT |		\
                NET_EVENT_WIFI_DISCONNECT_RESULT |  \
                NET_EVENT_WIFI_TWT |		\
                NET_EVENT_WIFI_RAW_SCAN_RESULT)

#define print(sh, level, fmt, ...)		\
    do {								\
            ei_printf(fmt, ##__VA_ARGS__);	\
    } while (false)

static struct {
    //const struct shell *sh;

    union {
        struct {

            uint8_t connecting		: 1;
            uint8_t disconnecting	: 1;
            uint8_t _unused		: 6;
        };
        uint8_t all;
    };
} context;

static uint32_t scan_result;
static uint8_t scan_running;
static bool dhcp_configured = false;
static bool wifi_connected = false;
static bool timed_out = false;

static struct net_mgmt_event_callback wifi_shell_mgmt_cb;
static struct net_mgmt_event_callback net_shell_mgmt_cb;

static void timer_timeout(struct k_timer *timer) {
    timed_out = true;
    LOG_DBG("Timer timed out");
}
K_TIMER_DEFINE(timeout_timer, timer_timeout, NULL);

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_scan_result *entry =
        (const struct wifi_scan_result *)cb->info;

    scan_result++;

    ei_printf("SSID: %s, Security: %s (%d),RSSI: %d dBm\n",
          entry->ssid,
          wifi_security_txt(entry->security), entry->security,
          entry->rssi);
}

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status =
        (const struct wifi_status *)cb->info;

    if (status->status) {
        LOG_ERR("Scan request failed (%d)", status->status);
    }

    scan_result = 0U;
    scan_running = 0;
}

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status =
        (const struct wifi_status *) cb->info;

    if (status->status) {
        LOG_DBG("Connection request failed (%d)", status->status);
    } else {
        LOG_DBG("Connected");
        wifi_connected = true;
    }

    context.connecting = false;
}

static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_status *status =
        (const struct wifi_status *) cb->info;

    if (context.disconnecting) {
        LOG_DBG("Disconnection request %s (%d)",
              status->status ? "failed" : "done",
              status->status);
        context.disconnecting = false;
        wifi_connected = false;
    } else {
        LOG_DBG("Disconnected");
    }
}

static void handle_wifi_twt_event(struct net_mgmt_event_callback *cb)
{
    // const struct wifi_twt_params *resp =
    //     (const struct wifi_twt_params *)cb->info;

    // if (resp->resp_status == WIFI_TWT_RESP_RECEIVED) {
    // 	print(NULL, NULL, "TWT response: %s\n",
    // 	      wifi_twt_setup_cmd2str[resp->setup_cmd]);
    // 	print_twt_params(resp->dialog_token,
    // 			 resp->flow_id,
    // 			 resp->negotiation_type,
    // 			 resp->setup.responder,
    // 			 resp->setup.implicit,
    // 			 resp->setup.announce,
    // 			 resp->setup.trigger,
    // 			 resp->setup.twt_wake_interval,
    // 			 resp->setup.twt_interval);
    // } else {
    // 	print(NULL, NULL, "TWT response timed out\n");
    // }
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                    uint32_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event) {
    case NET_EVENT_WIFI_SCAN_RESULT:
        handle_wifi_scan_result(cb);
        break;
    case NET_EVENT_WIFI_SCAN_DONE:
        handle_wifi_scan_done(cb);
        break;
    case NET_EVENT_WIFI_CONNECT_RESULT:
        handle_wifi_connect_result(cb);
        break;
    case NET_EVENT_WIFI_DISCONNECT_RESULT:
        handle_wifi_disconnect_result(cb);
        break;
    case NET_EVENT_WIFI_TWT:
        handle_wifi_twt_event(cb);
        break;
// #ifdef CONFIG_WIFI_MGMT_RAW_SCAN_RESULTS
// 	case NET_EVENT_WIFI_RAW_SCAN_RESULT:
// 		handle_wifi_raw_scan_result(cb);
// 		break;
// #endif /* CONFIG_WIFI_MGMT_RAW_SCAN_RESULTS */
    default:
        break;
    }
}

static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint32_t mgmt_event, struct net_if *iface)
{
	if(mgmt_event != NET_EVENT_IPV4_DHCP_BOUND) {
		LOG_WRN("Unhandled event: 0x%x", mgmt_event);
		return;
	}
	const struct net_if_dhcpv4 *dhcpv4 = (net_if_dhcpv4*)cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];

	/* Get DHCP info from struct net_if_dhcpv4 and print */
	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

	LOG_INF("DHCP IP address: %s", dhcp_info);
	dhcp_configured = true;
}

int cmd_wifi_scan(void)
{
    struct net_if *iface = net_if_get_default();

    // context.sh = NULL;

    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0)) {
        LOG_ERR( "Scan request failed");
        scan_running = 1;
        return -ENOEXEC;
    }
    LOG_INF("Scan requested");

    scan_running = 1;
    return 0;
}

void cmd_wifi_scan_done(void)
{
    while(scan_running) ei_sleep(100);
}

int cmd_wifi_connect(const char *ssid, const char *psk, int security)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_connect_req_params cnx_params = { 0 };
    EiDeviceThingy53 *dev = static_cast<EiDeviceThingy53*>(EiDeviceInfo::get_device());

    if (ssid == NULL) {
        return -1;
    }

    cnx_params.timeout = SYS_FOREVER_MS;

    cnx_params.ssid = (uint8_t*)ssid;
    cnx_params.ssid_length = strlen((char*)ssid);
    cnx_params.channel = WIFI_CHANNEL_ANY;

    if (psk != NULL) {
        cnx_params.psk = (uint8_t*)psk;
        cnx_params.psk_length = strlen((const char *)psk);
        cnx_params.security = (wifi_security_type)security;
        cnx_params.mfp = WIFI_MFP_OPTIONAL;
    } else {
        cnx_params.security = WIFI_SECURITY_TYPE_NONE;
    }

    LOG_DBG("Connecting to %s", ssid);
	context.connecting = true;


    LOG_DBG("cnx_params.ssid: %s, cnx_params.psk: %s, cnx_params.security: %d", cnx_params.ssid, cnx_params.psk, cnx_params.security);

    dev->set_wifi_config(ssid, psk, security);

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
		     &cnx_params, sizeof(struct wifi_connect_req_params))) {
		LOG_DBG("Connection request failed");
		context.connecting = false;

		return -ENOEXEC;
	}

	LOG_DBG("Connection requested");
    return 0;
}

int cmd_wifi_disconnect(void)
{
	struct net_if *iface = net_if_get_default();
	int status;

	context.disconnecting = true;

	status = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);

	if (status) {
		context.disconnecting = false;

		if (status == -EALREADY) {
			LOG_INF("Already disconnected");
		} else {
			LOG_ERR("Disconnect request failed");
			return -ENOEXEC;
		}
	} else {
		LOG_INF("Disconnect requested");
	}

	return 0;
}

int cmd_wifi_connecting(void)
{
    // Reset the timed_out flag
    timed_out = false;

        // Start the timer
    k_timer_start(&timeout_timer, K_SECONDS(30), K_NO_WAIT);
    LOG_DBG("Waiting for connection to be established");
    while(context.connecting && !timed_out) ei_sleep(100);

    // Stop the timer
    k_timer_stop(&timeout_timer);

    if (timed_out) {
        LOG_ERR("Connecting timed out");
        return -ETIMEDOUT;
    } else {
        LOG_INF("Connecting established");
        return 0;
    }
}

int cmd_dhcp_configured(void)
{
    // Reset the timed_out flag
    timed_out = false;

    // Start the timer
    k_timer_start(&timeout_timer, K_SECONDS(30), K_NO_WAIT);
    LOG_DBG("Waiting for DHCP to be configured");
    while(!dhcp_configured && !timed_out) ei_sleep(500);


    // Stop the timer
    k_timer_stop(&timeout_timer);

    if (timed_out) {
        LOG_ERR("DHCP configuration timed out");
        return -ETIMEDOUT;
    } else {
        LOG_WRN("DHCP configuration established");
        return 0;
    }
}

bool cmd_wifi_connected(void)
{
    return wifi_connected;
}

static int wifi_shell_init(void)
{

    // context.sh = NULL;
    context.all = 0U;
    scan_result = 0U;
    scan_running = 0;

    net_mgmt_init_event_callback(&wifi_shell_mgmt_cb,
                     wifi_mgmt_event_handler,
                     WIFI_SHELL_MGMT_EVENTS);
    net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);

    net_mgmt_init_event_callback(&net_shell_mgmt_cb,
                    net_mgmt_event_handler, 
                    NET_EVENT_IPV4_DHCP_BOUND);
	net_mgmt_add_event_callback(&net_shell_mgmt_cb);
    

    return 0;
}

SYS_INIT(wifi_shell_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);