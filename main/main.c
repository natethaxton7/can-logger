#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_bus.h"

/* ---------- Settings: change per board ---------- */
#define IS_SENDER      1       // 1 = board A (sender), 0 = board B (receiver)
#define LOOPBACK_TEST  1       // 1 = one-board test with GPIO4 jumpered to GPIO5
/* ------------------------------------------------ */

#define CAN_TX_GPIO    4
#define CAN_RX_GPIO    5
#define CAN_BITRATE    500000  // 500 kbit/s, the usual OBD-II speed
#define TEST_ID        0x123

static const char *TAG = "main";

// Prints every received frame and counts gaps in the counter (dropped frames)
static void rx_task(void *arg)
{
    can_msg_t msg;
    uint32_t expected = 0, received = 0, gaps = 0;
    bool first = true;

    while (1) {
        if (!can_bus_receive(&msg, portMAX_DELAY)) {
            continue;
        }
        received++;

        uint32_t counter = 0;
        if (msg.id == TEST_ID && msg.len >= 4) {
            memcpy(&counter, msg.data, sizeof(counter));
            if (!first && counter != expected) {
                gaps++;
            }
            expected = counter + 1;
            first = false;
        }

        ESP_LOGI(TAG, "RX id=0x%03lX len=%u counter=%lu received=%lu gaps=%lu",
                 (unsigned long)msg.id, msg.len, (unsigned long)counter,
                 (unsigned long)received, (unsigned long)gaps);
    }
}

// Sends a frame with an increasing counter every 100 ms
static void tx_task(void *arg)
{
    uint8_t data[8] = {0};
    uint32_t counter = 0;

    while (1) {
        memcpy(data, &counter, sizeof(counter));   // bytes 0-3 = counter
        data[7] = 0xAA;                            // marker byte

        esp_err_t err = can_bus_send(TEST_ID, data, sizeof(data), 100);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "TX #%lu sent", (unsigned long)counter);
        } else {
            ESP_LOGW(TAG, "TX #%lu failed: %s", (unsigned long)counter, esp_err_to_name(err));
            can_bus_print_status();
        }

        counter++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    can_bus_config_t cfg = {
        .tx_gpio  = CAN_TX_GPIO,
        .rx_gpio  = CAN_RX_GPIO,
        .bitrate  = CAN_BITRATE,
        .loopback = LOOPBACK_TEST,
    };
    ESP_ERROR_CHECK(can_bus_init(&cfg));

    xTaskCreate(rx_task, "rx_task", 4096, NULL, 5, NULL);
#if IS_SENDER
    xTaskCreate(tx_task, "tx_task", 4096, NULL, 4, NULL);
#endif
}