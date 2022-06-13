#ifndef BLE_NUS_H
#define BLE_NUS_H

#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>
#include <device.h>
#include <soc.h>
#include <settings/settings.h>
#include <stdio.h>

void ble_nus_init(void);
void ble_nus_send_data(uint8_t* buffer, uint32_t size);

#endif /* BLE_NUS_H */
