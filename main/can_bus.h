#pragma once 

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define CAN_MAX_DATA_LEN 8

typedef struct {
    uint32_t id; 
    uint8_t len; 
    uint8_t data[CAN_MAX_DATA_LEN];
} can_msg_t; 

typedef struct {
    int      tx_gpio;
    int      rx_gpio;
    uint32_t bitrate;
    bool     loopback;   // ← this semicolon is probably missing
} can_bus_config_t;      // ← check this name too: can_bus_config_t

esp_err_t can_bus_init(const can_bus_config_t *cfg);

esp_err_t can_bus_send(uint32_t id, const uint8_t *data, uint8_t len, int timeout_ms);

bool can_bus_receive(can_msg_t *msg, TickType_t wait); 

void can_bus_print_status(void); 

