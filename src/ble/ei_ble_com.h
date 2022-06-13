#ifndef EI_BLE_COM_H
#define EI_BLE_COM_H

/* Include ----------------------------------------------------------------- */
// #include "ei_device_info.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <cstdio>

#include "cJSON.h"
#include "cJSON_os.h"


void ei_ble_com_init(void);
char * ei_ble_connect_handshake (void);
void ei_ble_message_rcv(const char * responce);
char * ei_ble_com_response_sample (const char * sample_name, bool sample, uint8_t * error);

#endif /* EI_BLE_COM_H */
