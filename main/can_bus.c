#include <string.h> 
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "can_bus.h"

static const char *TAG = "can_bus";
static twai_node_handle_t s_node = NULL;      // handle to the CAN controller
static QueueHandle_t      s_rx_queue = NULL;
static uint8_t      s_tx_buf[CAN_MAX_DATA_LEN];   // ← needs the [CAN_MAX_DATA_LEN]
static twai_frame_t s_tx_frame = {
    .buffer = s_tx_buf,                           // ← comma, not semicolon
};    
static volatile bool s_last_tx_ok = false;   // set by the TX-done interrupt                                            // ← closing brace + semicolon
// Runs in INTERRUPT context when a frame finishes transmitting



static bool on_tx_done(twai_node_handle_t handle,
                       const twai_tx_done_event_data_t *edata,
                       void *user_ctx)
{
    s_last_tx_ok = edata->is_tx_success;
    return false;
}

static bool on_rx_done(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *edata,
                       void *user_ctx)
{
    uint8_t buf[CAN_MAX_DATA_LEN];
    twai_frame_t frame = {
        .buffer = buf,
        .buffer_len = sizeof(buf),
    };
    BaseType_t woken = pdFALSE;

    if (twai_node_receive_from_isr(handle, &frame) == ESP_OK) {
        can_msg_t msg = {
            .id  = frame.header.id,
            .len = frame.header.dlc > CAN_MAX_DATA_LEN ? CAN_MAX_DATA_LEN : frame.header.dlc,
        };
        memcpy(msg.data, buf, msg.len);
        xQueueSendFromISR(s_rx_queue, &msg, &woken);
    }
    return woken == pdTRUE;   // tells FreeRTOS to switch to the waiting task right away
}

esp_err_t can_bus_init(const can_bus_config_t *cfg)
{
    s_rx_queue = xQueueCreate(32, sizeof(can_msg_t));
    if (s_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = cfg->tx_gpio,
        .io_cfg.rx = cfg->rx_gpio,
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,      // not used
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,   // not used
        .bit_timing.bitrate = cfg->bitrate,
        .tx_queue_depth = 8,
        .fail_retry_cnt = 3,                       // retry a failed frame 3 times
        .flags.enable_self_test = cfg->loopback,
        .flags.enable_loopback  = cfg->loopback,
    };

    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        return err;
    }

    // Callbacks must be registered BEFORE the node is enabled
        twai_event_callbacks_t cbs = {
        .on_rx_done = on_rx_done,
        .on_tx_done = on_tx_done,
    };

    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err != ESP_OK) {
        return err;
    }

    err = twai_node_enable(s_node);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "started: %lu bit/s, TX=GPIO%d, RX=GPIO%d%s",
                 (unsigned long)cfg->bitrate, cfg->tx_gpio, cfg->rx_gpio,
                 cfg->loopback ? " (LOOPBACK SELF TEST)" : "");
    }
    return err;
}

esp_err_t can_bus_send(uint32_t id, const uint8_t *data, uint8_t len, int timeout_ms)
{
    if (len > CAN_MAX_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    // Make sure the previous frame is finished before we reuse the shared buffer
    esp_err_t err = twai_node_transmit_wait_all_done(s_node, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    memcpy(s_tx_buf, data, len);
    s_tx_frame.header.id = id;        // standard 11-bit ID
    s_tx_frame.buffer_len = len;

        s_last_tx_ok = false;
    err = twai_node_transmit(s_node, &s_tx_frame, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    err = twai_node_transmit_wait_all_done(s_node, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    return s_last_tx_ok ? ESP_OK : ESP_FAIL;
}

bool can_bus_receive(can_msg_t *msg, TickType_t wait)
{
    return xQueueReceive(s_rx_queue, msg, wait) == pdTRUE;
}

void can_bus_print_status(void)
{
    twai_node_status_t st = {0};
    twai_node_record_t rec = {0};
    twai_node_get_info(s_node, &st, &rec);

    ESP_LOGW(TAG, "state=%d TEC=%u REC=%u bus_errors=%lu",
             st.state, st.tx_error_count, st.rx_error_count,
             (unsigned long)rec.bus_err_num);

    if (st.state == TWAI_ERROR_BUS_OFF) {
        ESP_LOGW(TAG, "bus-off: starting recovery");
        twai_node_recover(s_node);
    }
}

